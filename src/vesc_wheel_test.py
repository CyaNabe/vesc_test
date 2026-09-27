#!/usr/bin/env python3
"""
VESC 4-Wheel Drive Motor Test Script
4輪足回り用 VESC モーターテストスクリプト

キーボード操作:
  'w'          : 前進 (Forward)
  's'          : 後進 (Reverse)
  Space / 'x'  : 停止 (Stop / 速度0)
  'q'          : 終了 (安全に速度0を送って終了)
"""

from __future__ import annotations

import argparse
import errno
import os
import select
import socket
import struct
import sys
import termios
import time
import tty
from typing import Any, Dict, List, Optional, Tuple

# ==============================================================================
# 設定パラメータ（実機環境・モーター構成に合わせてここを変更してください）
# ==============================================================================

# 1. CANインターフェース設定
CAN_CHANNEL: str = "can0"       # SocketCAN インターフェース名 (例: "can0", "vcan0")
CAN_BITRATE: int = 1000000      # CAN 通信速度 (1 Mbps)

# 2. 指令速度設定 (機械角 RPM)
TARGET_RPM: float = 500.0       # 前進/後進時の目標速度 [RPM] (正の値で指定)

# 3. モーターの極対数 (pole pairs)
#    VESCへのRPM指令は ERPM (電気角RPM) で送信されます。
#    ERPM = 機械角RPM * 極対数 (POLE_PAIRS)
#    ※ rox2026プロジェクトでは14極モーター (極対数: 7) が使用されていました。
POLE_PAIRS: int = 7

# 4. 足回り4輪のモーターIDおよび回転方向設定
#    - "name": モーターの配置名称
#    - "id": VESCのCAN ID (0〜255)
#    - "direction": 前進時の回転方向 (+1: 正転, -1: 逆転)
#      ※ 車体が旋回せず直進するように、左右で回転方向の符号を逆に設定しています。
#      ※ 実機のモーター取り付け向きやギア構成に合わせて 1 または -1 を調整してください。
MOTOR_CONFIGS: List[Dict[str, Any]] = [
    {"name": "Front-Left  (左前)", "id": 0x31, "direction":  1},
    {"name": "Rear-Left   (左後)", "id": 0x32, "direction":  1},
    {"name": "Front-Right (右前)", "id": 0x33, "direction": -1},
    {"name": "Rear-Right  (右後)", "id": 0x34, "direction": -1},
]

# 5. 通信・安全パラメータ
COMMAND_RATE_HZ: float = 40.0       # CAN指令送信周期 [Hz] (40Hz = 25ms周期)
FEEDBACK_TIMEOUT_SEC: float = 0.5   # フィードバック途絶時の切断判定時間 [秒]
ENABLE_DISCONNECT_STOP: bool = True # 接続切れ（STATUS途絶）時に速度0を送信して緊急停止するか
REQUIRE_INITIAL_FEEDBACK: bool = False # 起動直後にSTATUS未受信でも走行を許可するか (False: 受信後の途絶のみ検知)

# 'w' や 's' が押されていないときの自動停止タイムアウト [秒]
# キーを押し続けている間だけ走行し、キーを離す（入力が途絶える）と自動的に速度0が送信されます。
# ※ OSのキーリピート遅延（通常250〜350ms）をカバーするため、デフォルトは 0.35 秒に設定しています。
KEY_TIMEOUT_SEC: float = 0.35

# ==============================================================================
# VESC CAN プロトコル定義 (rox2026互換)
# ==============================================================================
PACKET_SET_DUTY = 0
PACKET_SET_CURRENT = 1
PACKET_SET_CURRENT_BRAKE = 2
PACKET_SET_RPM = 3
PACKET_STATUS = 9          # Status 1: ERPM, Current, Duty
PACKET_STATUS_4 = 16       # Status 4: FET Temp, Motor Temp, Current In
PACKET_STATUS_5 = 27       # Status 5: Tachometer, Vin

# Linux SocketCAN 定数
CAN_EFF_FLAG = 0x80000000  # 拡張フレーム (29-bit Extended ID) フラグ
CAN_EFF_MASK = 0x1FFFFFFF  # 拡張フレーム ID マスク
CAN_FRAME_FMT = "=IB3x8s"   # struct can_frame: can_id(4B), dlc(1B), pad(3B), data(8B)


def build_can_id(packet_id: int, controller_id: int) -> int:
    """VESCの拡張CAN IDを生成: (packet_id << 8) | controller_id"""
    return ((packet_id & 0xFF) << 8) | (controller_id & 0xFF)


class VescCanBus:
    """SocketCAN を使用した VESC CAN 通信クラス"""

    def __init__(self, channel: str = CAN_CHANNEL, dry_run: bool = False):
        self.channel = channel
        self.dry_run = dry_run
        self.sock: Optional[socket.socket] = None

        if not self.dry_run:
            try:
                self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
                self.sock.bind((self.channel,))
                self.sock.setblocking(False)
            except OSError as exc:
                raise RuntimeError(
                    f"CANインターフェース '{self.channel}' を開けませんでした: {exc}\n"
                    f"※ 実機CANがセットアップされていない場合は '--dry-run' を指定してテスト実行できます。"
                ) from exc

    def send_rpm(self, controller_id: int, erpm: int) -> bool:
        """指定したESCへERPM指令 (PACKET_SET_RPM = 3) を送信"""
        arb_id = build_can_id(PACKET_SET_RPM, controller_id)
        data = int(erpm).to_bytes(4, byteorder="big", signed=True)

        if self.dry_run:
            return True

        if self.sock is None:
            return False

        can_id_eff = (arb_id & CAN_EFF_MASK) | CAN_EFF_FLAG
        padded_data = data.ljust(8, b"\x00")
        frame = struct.pack(CAN_FRAME_FMT, can_id_eff, len(data), padded_data)
        try:
            self.sock.send(frame)
            return True
        except OSError as exc:
            if exc.errno in (errno.ENOBUFS, errno.EAGAIN):
                return False
            raise

    def send_zero(self, controller_id: int) -> bool:
        """指定したESCへ速度0を送信"""
        return self.send_rpm(controller_id, 0)

    def recv_statuses(self) -> List[Tuple[int, int, float, float]]:
        """
        バッファに溜まっているSTATUSフレームを受信してパースする
        戻り値: [(controller_id, erpm, current_a, duty), ...]
        """
        results = []
        if self.dry_run or self.sock is None:
            return results

        while True:
            try:
                frame_bytes = self.sock.recv(16)
                if len(frame_bytes) < 16:
                    break
                can_id_raw, dlc, data = struct.unpack(CAN_FRAME_FMT, frame_bytes)
                if not (can_id_raw & CAN_EFF_FLAG):
                    continue

                arb_id = can_id_raw & CAN_EFF_MASK
                packet_id = (arb_id >> 8) & 0xFF
                controller_id = arb_id & 0xFF

                if packet_id == PACKET_STATUS and dlc >= 8:
                    erpm = struct.unpack(">i", data[0:4])[0]
                    raw_current = struct.unpack(">h", data[4:6])[0]
                    raw_duty = struct.unpack(">h", data[6:8])[0]
                    current_a = raw_current / 10.0
                    duty = raw_duty / 1000.0
                    results.append((controller_id, erpm, current_a, duty))
            except BlockingIOError:
                break
            except OSError as exc:
                if exc.errno == errno.EAGAIN:
                    break
                raise
        return results

    def close(self) -> None:
        """ソケットを閉じる"""
        if self.sock is not None:
            try:
                self.sock.close()
            except Exception:
                pass
            self.sock = None


class MotorState:
    """各モーターの動作状態およびフィードバック保持用"""

    def __init__(self, name: str, controller_id: int, direction: int):
        self.name = name
        self.id = controller_id
        self.direction = direction  # +1 または -1
        self.target_rpm: float = 0.0
        self.command_erpm: int = 0
        self.measured_erpm: Optional[int] = None
        self.measured_rpm: Optional[float] = None
        self.measured_current: Optional[float] = None
        self.measured_duty: Optional[float] = None
        self.last_status_time: Optional[float] = None
        self.status_received_ever: bool = False

    def update_feedback(self, erpm: int, current_a: float, duty: float, now: float, pole_pairs: int) -> None:
        self.measured_erpm = erpm
        self.measured_rpm = erpm / float(pole_pairs) if pole_pairs > 0 else 0.0
        self.measured_current = current_a
        self.measured_duty = duty
        self.last_status_time = now
        self.status_received_ever = True

    def is_connected(self, now: float, timeout_sec: float) -> bool:
        if self.last_status_time is None:
            return False
        return (now - self.last_status_time) <= timeout_sec


class NonBlockingKeyboard:
    """非ブロッキングで標準入力から1文字読み取るコンテキストマネージャ"""

    def __init__(self):
        self.old_settings = None
        self.is_tty = sys.stdin.isatty()
        if self.is_tty:
            self.fd = sys.stdin.fileno()

    def __enter__(self):
        if self.is_tty:
            try:
                self.old_settings = termios.tcgetattr(self.fd)
                tty.setcbreak(self.fd)
            except Exception:
                self.is_tty = False
        return self

    def __exit__(self, exc_type, exc_val, exc_tb):
        if self.is_tty and self.old_settings is not None:
            try:
                termios.tcsetattr(self.fd, termios.TCSADRAIN, self.old_settings)
            except Exception:
                pass

    def read_key(self) -> Optional[str]:
        if self.is_tty:
            rlist, _, _ = select.select([self.fd], [], [], 0.0)
            if rlist:
                char = sys.stdin.read(1)
                # エスケープシーケンス等の追加文字を消化
                while True:
                    extra, _, _ = select.select([self.fd], [], [], 0.0)
                    if extra:
                        sys.stdin.read(1)
                    else:
                        break
                return char
            return None
        else:
            # パイプやスクリプト入力の場合
            rlist, _, _ = select.select([sys.stdin], [], [], 0.0)
            if rlist:
                char = sys.stdin.read(1)
                if char == "":  # EOF
                    return "q"
                if char in ("\n", "\r"):
                    return None
                return char
            return None


def run_wheel_test(
    channel: str = CAN_CHANNEL,
    target_rpm: float = TARGET_RPM,
    pole_pairs: int = POLE_PAIRS,
    key_timeout: float = KEY_TIMEOUT_SEC,
    dry_run: bool = False,
) -> None:
    """足回り4輪VESCテストのメイン実行関数"""
    print("=" * 70)
    print(" VESC 4-Wheel Drive Motor Test (4輪足回り速度制御テスト)")
    print("=" * 70)
    print(f" CAN Interface : {channel} {'(DRY-RUN / シミュレーションモード)' if dry_run else ''}")
    print(f" Target Speed  : {target_rpm:.1f} mechanical RPM")
    print(f" Pole Pairs    : {pole_pairs} (ERPM = RPM * {pole_pairs})")
    print(f" Deadman Timeout: {key_timeout:.2f} s (w/s を離すと自動で速度0になります)")
    print(" Motors:")
    for m in MOTOR_CONFIGS:
        dir_label = "正転 (+1)" if m["direction"] > 0 else "反転 (-1)"
        print(f"   - {m['name']:<18} ID: 0x{m['id']:02X} ({m['id']:3d}) | 前進時: {dir_label}")
    print("-" * 70)
    print(" 操作方法:")
    print("   [w] (押し続け) : 前進 (Forward)")
    print("   [s] (押し続け) : 後進 (Reverse)")
    print("   ※ キーが押されていないときは自動的に速度0が送られます")
    print("   [Space] / [x]  : 停止 (Stop / 速度0)")
    print("   [q]            : 終了 (全モーター停止後、終了)")
    print("=" * 70)

    # モーターステート初期化
    motors: Dict[int, MotorState] = {
        cfg["id"]: MotorState(cfg["name"], cfg["id"], cfg["direction"])
        for cfg in MOTOR_CONFIGS
    }

    # CAN初期化
    can_bus = VescCanBus(channel=channel, dry_run=dry_run)

    state: str = "STOP"  # "STOP", "FORWARD", "REVERSE"
    last_logged_state: Optional[str] = None
    last_log_time: float = 0.0
    last_key_time: float = 0.0  # 起動時は未入力状態
    last_ui_update: float = 0.0
    disconnect_warning: Optional[str] = None
    period_sec = 1.0 / COMMAND_RATE_HZ

    def send_zero_all() -> None:
        """全モーターに速度0を送信"""
        for m in motors.values():
            m.target_rpm = 0.0
            m.command_erpm = 0
            can_bus.send_zero(m.id)

    try:
        # 開始時は確実に速度0を送信
        send_zero_all()

        with NonBlockingKeyboard() as kbd:
            while True:
                loop_start = time.monotonic()
                now = loop_start

                # 1. キーボード入力の取得
                key = kbd.read_key()
                if key is not None:
                    disconnect_warning = None  # キー操作で警告リセット
                    if key.lower() == "w":
                        state = "FORWARD"
                        last_key_time = now
                    elif key.lower() == "s":
                        state = "REVERSE"
                        last_key_time = now
                    elif key in (" ", "x", "X"):
                        state = "STOP"
                        last_key_time = 0.0
                    elif key.lower() == "q":
                        print("\n[INFO] 'q' が押されました。安全に停止して終了します...")
                        break

                # 'w' や 's' が押されていない場合（キー入力タイムアウト）は速度0に復帰
                if state in ("FORWARD", "REVERSE"):
                    if (now - last_key_time) > key_timeout:
                        state = "STOP"

                # 2. CAN受信処理 (STATUSフレーム)
                statuses = can_bus.recv_statuses()
                for c_id, erpm, cur, duty in statuses:
                    if c_id in motors:
                        motors[c_id].update_feedback(erpm, cur, duty, now, pole_pairs)

                # 3. 接続監視 (走行中のSTATUS途絶チェック)
                if ENABLE_DISCONNECT_STOP and not dry_run and state in ("FORWARD", "REVERSE"):
                    lost_ids = []
                    for m in motors.values():
                        if REQUIRE_INITIAL_FEEDBACK:
                            if not m.is_connected(now, FEEDBACK_TIMEOUT_SEC):
                                lost_ids.append(f"0x{m.id:02X} ({m.name})")
                        else:
                            if m.status_received_ever and not m.is_connected(now, FEEDBACK_TIMEOUT_SEC):
                                lost_ids.append(f"0x{m.id:02X} ({m.name})")

                    if lost_ids:
                        state = "STOP"
                        disconnect_warning = f"通信切断検知: {', '.join(lost_ids)} からの応答途絶！速度0を送信して停止しました。"
                        send_zero_all()

                # 4. 指令送信
                for m in motors.values():
                    if state == "FORWARD":
                        m.target_rpm = target_rpm * m.direction
                    elif state == "REVERSE":
                        m.target_rpm = -target_rpm * m.direction
                    else:  # STOP
                        m.target_rpm = 0.0

                    m.command_erpm = int(round(m.target_rpm * pole_pairs))
                    can_bus.send_rpm(m.id, m.command_erpm)

                # 5. コンソールUI更新 (約10Hzで更新)
                if now - last_ui_update >= 0.1:
                    last_ui_update = now
                    state_color = {
                        "FORWARD": "\033[32m[ FORWARD  (前進) ]\033[0m",
                        "REVERSE": "\033[33m[ REVERSE  (後進) ]\033[0m",
                        "STOP":    "\033[34m[ STOPPED  (停止) ]\033[0m",
                    }.get(state, f"[{state}]")

                    lines = [
                        f"\r\033[KState: {state_color}  |  Base Target: {target_rpm:.0f} RPM (ERPM: {int(target_rpm * pole_pairs)})",
                        "----------------------------------------------------------------------",
                        " Motor Position      | CAN ID | Dir | Cmd RPM | Real RPM | Current | Link ",
                        "----------------------------------------------------------------------",
                    ]
                    for m in motors.values():
                        cmd_str = f"{m.target_rpm:+7.0f}"
                        if m.measured_rpm is not None:
                            real_str = f"{m.measured_rpm:+7.0f}"
                        else:
                            real_str = "    ---"
                        if m.measured_current is not None:
                            cur_str = f"{m.measured_current:+5.1f} A"
                        else:
                            cur_str = "  --- A"

                        is_link_ok = dry_run or m.is_connected(now, FEEDBACK_TIMEOUT_SEC)
                        link_str = "\033[32m  OK  \033[0m" if is_link_ok else "\033[31m NO-RX\033[0m"

                        lines.append(
                            f" {m.name:<19} | 0x{m.id:02X}   | {m.direction:+2d}  | {cmd_str} | {real_str}  | {cur_str} | {link_str}"
                        )
                    lines.append("----------------------------------------------------------------------")
                    if disconnect_warning:
                        lines.append(f"\033[31;1m[WARNING] {disconnect_warning}\033[0m")
                    else:
                        lines.append(" [w: 前進(長押し) | s: 後進(長押し) | 離すと停止 | q: 終了]")

                    if sys.stdout.isatty():
                        # ターミナルへ一括出力 (カーソルを戻して上書き)
                        text = "\n".join(lines)
                        num_lines = len(lines)
                        sys.stdout.write(text + f"\033[{num_lines - 1}A\r")
                        sys.stdout.flush()
                    else:
                        # 非TTY環境 (テスト/パイプ/ログ出力用)
                        if state != last_logged_state or (now - last_log_time) >= 1.0:
                            last_logged_state = state
                            last_log_time = now
                            summary = " ".join(f"id=0x{m.id:02X}:{m.target_rpm:+.0f}rpm" for m in motors.values())
                            print(f"[State: {state:<7}] {summary}")

                # ループ周期スリープ
                elapsed = time.monotonic() - loop_start
                sleep_time = period_sec - elapsed
                if sleep_time > 0:
                    time.sleep(sleep_time)

    except KeyboardInterrupt:
        print("\n\n[INFO] Ctrl+C が検出されました。安全に停止します...")
    finally:
        # 終了時は確実に速度0を複数回送信してモーターを停止させる
        print("\n[INFO] 全モーターへ速度0 (STOP) を送信中...")
        for _ in range(5):
            send_zero_all()
            time.sleep(0.01)
        can_bus.close()
        print("[INFO] 停止完了。安全に終了しました。\n")


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="VESC 4-Wheel Drive Motor Speed Test Script",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "--channel",
        default=CAN_CHANNEL,
        help="SocketCAN インターフェース名 (例: can0, vcan0)",
    )
    parser.add_argument(
        "--rpm",
        type=float,
        default=TARGET_RPM,
        help="目標機械回転数 [RPM]",
    )
    parser.add_argument(
        "--pole-pairs",
        type=int,
        default=POLE_PAIRS,
        help="モーターの極対数 (ERPM = RPM * pole_pairs)",
    )
    parser.add_argument(
        "--key-timeout",
        type=float,
        default=KEY_TIMEOUT_SEC,
        help="キー入力途絶時の自動停止タイムアウト [秒]",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="CANソケットを開かず、コンソール表示とロジックのみをテストするモード",
    )
    return parser.parse_args()


if __name__ == "__main__":
    args = parse_arguments()
    run_wheel_test(
        channel=args.channel,
        target_rpm=args.rpm,
        pole_pairs=args.pole_pairs,
        key_timeout=args.key_timeout,
        dry_run=args.dry_run,
    )
