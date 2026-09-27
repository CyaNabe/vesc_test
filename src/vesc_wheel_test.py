#!/usr/bin/env python3
"""
VESC 4-Wheel Drive Motor Test (Minimal & Direct)
4輪足回り VESC モーター速度制御テスト (最小構成版)

キーボード操作:
  'w' 押し続け : 前進 (直進)
  's' 押し続け : 後進 (直進)
  キーを離す   : 自動停止 (速度0)
  'q' / Ctrl+C : 終了 (全モーター停止)
"""

from __future__ import annotations

import csv
import math
import os
import select
import struct
import sys
import termios
import time
import tty
from typing import Optional, Tuple

# ==============================================================================
# 設定パラメータ（ここを変更してください）
# ==============================================================================
CAN_CHANNEL = "can0"    # SocketCAN インターフェース名
BITRATE = 1000000       # CAN 通信速度 (1 Mbps)

TARGET_RPM = 500.0      # 目標速度 [機械角 RPM] (w/s でこの速度を送る)
POLE_PAIRS = 7          # 極対数 (rox2026は14極モーター -> 極対数 7, ERPM = RPM * 7)
WHEEL_RADIUS = 0.075    # 車輪半径 [m] (加速度計算用: 75mm)

# 4つの足回りモーター設定 (IDと前進時の回転方向)
# 車体が直進するように、左右で回転方向の符号を逆に設定しています
MOTORS = [
    {"name": "FL (左前)", "id": 0x1, "dir":  1},
    {"name": "RL (左後)", "id": 0x2, "dir":  1},
    {"name": "FR (右前)", "id": 0x0, "dir": -1},
    {"name": "RR (右後)", "id": 0x3, "dir": -1},
]

SEND_PERIOD_SEC = 0.025 # CAN送信周期 [秒] (40 Hz = 25 ms)
KEY_TIMEOUT_SEC = 0.35  # キーを離したと判定して速度0にする時間 [秒]
LOG_FILE = "logs/vesc_test.csv" # 走行データCSV保存先 (速度・加速度解析用)


# ==============================================================================
# CAN 送受信ハンドラ (rox2026互換: python-can優先, socketcanフォールバック)
# ==============================================================================
class CanHandler:
    def __init__(self, channel: str, bitrate: int, dry_run: bool = False):
        self.channel = channel
        self.dry_run = dry_run
        self.use_python_can = False
        self.bus = None
        self.sock = None

        if self.dry_run:
            print(f"[CAN] DRY-RUN (シミュレーション) モードで起動しました (実CAN送信なし)")
            return

        # 1. まず python-can (rox2026標準) を試行
        try:
            import can
            self.bus = can.interface.Bus(channel=channel, interface="socketcan", bitrate=bitrate)
            self.use_python_can = True
            print(f"[CAN] '{channel}' を python-can 経由で開きました (bitrate={bitrate})")
            return
        except Exception:
            pass

        # 2. python-can が未インストールの場合は標準 socket(AF_CAN) を試行
        try:
            import socket
            self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.sock.bind((channel,))
            self.sock.settimeout(0.0)
            print(f"[CAN] '{channel}' を標準 socket(AF_CAN) 経由で開きました")
        except OSError as e_sock:
            print(f"\n[CAN ERROR] CANインターフェース '{channel}' を開けませんでした ({e_sock})")
            print("【確認事項】 実機でSocketCANが起動しているか確認してください:")
            print("   sudo ip link set can0 txqueuelen 1000")
            print("   sudo ip link set can0 up type can bitrate 1000000 restart-ms 100")
            print("※ 実機がない環境で動作確認する場合は '--dry-run' を指定してください:")
            print("   python3 src/vesc_wheel_test.py --dry-run\n")
            sys.exit(1)

    def send_rpm(self, controller_id: int, erpm: int) -> bool:
        """VESCへ PACKET_SET_RPM = 3 (Extended ID: 0x300 | id) を送信"""
        arb_id = (3 << 8) | (controller_id & 0xFF)
        data = int(erpm).to_bytes(4, byteorder="big", signed=True)

        if self.dry_run:
            return True

        if self.use_python_can:
            import can
            msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=True)
            try:
                self.bus.send(msg, timeout=0.02)
                return True
            except Exception as e:
                print(f"\n[CAN TX ERROR] 0x{controller_id:02X}: {e}")
                return False
        else:
            # Linux can_frame (Extended: CAN_EFF_FLAG = 0x80000000)
            frame = struct.pack("=IB3x8s", arb_id | 0x80000000, 4, data.ljust(8, b"\x00"))
            try:
                self.sock.send(frame)
                return True
            except OSError as e:
                print(f"\n[CAN TX ERROR] 0x{controller_id:02X}: {e}")
                return False

    def recv(self) -> Optional[Tuple[int, bool, bytes]]:
        """受信バッファから1フレーム取得: (arb_id, is_extended, data_bytes)"""
        if self.dry_run:
            return None
        if self.use_python_can:
            msg = self.bus.recv(timeout=0.0)
            if msg is None:
                return None
            return msg.arbitration_id, msg.is_extended_id, bytes(msg.data)
        else:
            try:
                raw = self.sock.recv(16)
                if len(raw) < 16:
                    return None
                can_id, dlc, data = struct.unpack("=IB3x8s", raw)
                is_ext = bool(can_id & 0x80000000)
                arb_id = (can_id & 0x1FFFFFFF) if is_ext else (can_id & 0x7FF)
                return arb_id, is_ext, data[:dlc]
            except (BlockingIOError, OSError):
                return None

    def close(self):
        if self.bus:
            self.bus.shutdown()
        if self.sock:
            self.sock.close()


# ==============================================================================
# メイン処理 (最小構成)
# ==============================================================================
def main():
    print("=" * 70)
    print(" VESC 4-Wheel Drive Motor Test (Minimal)")
    print("=" * 70)
    print(f" CAN Channel : {CAN_CHANNEL}")
    print(f" Target Speed: {TARGET_RPM:.0f} RPM (ERPM: {int(TARGET_RPM * POLE_PAIRS)})")
    print(" Motors:")
    for m in MOTORS:
        dir_str = "正転(+1)" if m["dir"] > 0 else "反転(-1)"
        print(f"   - {m['name']:<12} ID: 0x{m['id']:02X} ({m['id']:3d}) | {dir_str}")
    print("-" * 70)
    print(" 操作: 'w' (前進), 's' (後進), 離すと停止(速度0), 'q' (終了)")
    print("=" * 70)

    dry_run = "--dry-run" in sys.argv
    channel = CAN_CHANNEL
    for i, arg in enumerate(sys.argv):
        if arg == "--channel" and i + 1 < len(sys.argv):
            channel = sys.argv[i + 1]

    # ログファイル準備
    os.makedirs(os.path.dirname(LOG_FILE) or ".", exist_ok=True)
    f_log = open(LOG_FILE, "w", newline="", encoding="utf-8")
    writer = csv.writer(f_log)
    header = ["elapsed_sec", "state", "target_rpm"]
    for m in MOTORS:
        header.extend([f"m_{m['id']:02x}_cmd", f"m_{m['id']:02x}_real", f"m_{m['id']:02x}_cur", f"m_{m['id']:02x}_accel"])
    writer.writerow(header)

    # CAN初期化
    can = CanHandler(channel, BITRATE, dry_run=dry_run)

    # モーター状態辞書 (実測RPM, 電流, 加速度)
    motor_state = {
        m["id"]: {
            "real_rpm": 0.0,
            "current": 0.0,
            "last_rpm": 0.0,
            "last_time": 0.0,
            "accel": 0.0,
        }
        for m in MOTORS
    }

    # 端末入力設定 (cbreak)
    old_term = None
    if sys.stdin.isatty():
        old_term = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())

    def send_zero():
        for m in MOTORS:
            can.send_rpm(m["id"], 0)

    state = "STOP"
    last_key_time = 0.0
    start_time = time.monotonic()
    last_print_time = 0.0

    try:
        send_zero()

        while True:
            t_now = time.monotonic()
            t_elapsed = t_now - start_time

            # 1. キー入力判定
            r, _, _ = select.select([sys.stdin], [], [], 0.0)
            if r:
                char = sys.stdin.read(1)
                if not char or char.lower() == "q":
                    print("\n[INFO] 終了します...")
                    break
                elif char.lower() == "w":
                    state = "FORWARD"
                    last_key_time = t_now
                elif char.lower() == "s":
                    state = "REVERSE"
                    last_key_time = t_now
                elif char in (" ", "x", "X"):
                    state = "STOP"
                    last_key_time = 0.0

            # キーが押されていない (タイムアウト) なら速度0
            if state in ("FORWARD", "REVERSE"):
                if (t_now - last_key_time) > KEY_TIMEOUT_SEC:
                    state = "STOP"

            # 2. CAN受信処理 (VESC STATUS: 0x900 | id のみ受信)
            while True:
                pkt = can.recv()
                if pkt is None:
                    break
                arb_id, is_ext, data = pkt
                if is_ext:
                    packet_id = (arb_id >> 8) & 0xFF
                    c_id = arb_id & 0xFF
                    # PACKET_STATUS = 9 (ERPM, Current, Duty)
                    if packet_id == 9 and len(data) >= 8 and c_id in motor_state:
                        erpm = struct.unpack(">i", data[0:4])[0]
                        cur_raw = struct.unpack(">h", data[4:6])[0]
                        real_rpm = erpm / float(POLE_PAIRS)
                        
                        # 加速度計算 (実測RPMの時間微分から車輪並進加速度 a = r * d(omega)/dt を計算)
                        st = motor_state[c_id]
                        if st["last_time"] > 0:
                            dt = t_now - st["last_time"]
                            if dt > 0.005:
                                d_rpm = real_rpm - st["last_rpm"]
                                raw_accel = WHEEL_RADIUS * (d_rpm * (2.0 * math.pi / 60.0)) / dt
                                st["accel"] = 0.7 * st["accel"] + 0.3 * raw_accel
                        st["last_rpm"] = real_rpm
                        st["last_time"] = t_now
                        st["real_rpm"] = real_rpm
                        st["current"] = cur_raw / 10.0

            # 3. 指令送信 (4輪へ同時にSET_RPM送信)
            cmd_rpms = {}
            for m in MOTORS:
                if state == "FORWARD":
                    cmd = TARGET_RPM * m["dir"]
                elif state == "REVERSE":
                    cmd = -TARGET_RPM * m["dir"]
                else:
                    cmd = 0.0
                cmd_rpms[m["id"]] = cmd
                erpm = int(round(cmd * POLE_PAIRS))
                can.send_rpm(m["id"], erpm)

            # 4. CSVログ保存
            row = [f"{t_elapsed:.3f}", state, f"{TARGET_RPM:.0f}"]
            for m in MOTORS:
                st = motor_state[m["id"]]
                row.extend([f"{cmd_rpms[m['id']]:.0f}", f"{st['real_rpm']:.0f}", f"{st['current']:.1f}", f"{st['accel']:.2f}"])
            writer.writerow(row)

            # 5. コンソール出力 (10Hz更新, 1行で送信状況を表示)
            if (t_now - last_print_time) >= 0.1:
                last_print_time = t_now
                summary = " | ".join(
                    f"{m['name']}: {cmd_rpms[m['id']]:+5.0f} (実{motor_state[m['id']]['real_rpm']:+5.0f})"
                    for m in MOTORS
                )
                sys.stdout.write(f"\r[{state:<7}] {summary}  ")
                sys.stdout.flush()

            time.sleep(SEND_PERIOD_SEC)

    except KeyboardInterrupt:
        print("\n[INFO] Ctrl+C が検出されました。")
    finally:
        print("\n[INFO] 全モーターへ速度0を送信中...")
        for _ in range(5):
            send_zero()
            time.sleep(0.01)
        can.close()
        f_log.close()
        if old_term is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_term)
        print(f"[INFO] 走行データを保存しました: {LOG_FILE}")
        print("[INFO] 終了しました。\n")


if __name__ == "__main__":
    main()
