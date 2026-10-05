#!/usr/bin/env python3
"""
VESC 4-Wheel Drive Motor Speed Test (Clear Logging & Verified Transmission)
4輪足回り VESC モーター速度制御テスト (確実送信 & 詳細ログ版)

操作:
  'w' 押し続け : 前進 (直進)
  's' 押し続け : 後進 (直進)
  離すと自動停止 (速度0)
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
# 設定パラメータ（実機に合わせてここを変更してください）
# ==============================================================================
CAN_CHANNEL = "can0"    # SocketCAN インターフェース名 (例: can0)
BITRATE = 1000000        # CAN 通信速度 (500 kbps)

TARGET_RPM = 5000.0     # 目標速度 [機械角 RPM] (w/s でこの速度を送る。--rpm 引数でも変更可能)
POLE_PAIRS = 7          # 極対数 (14極モーター -> 7, ERPM = RPM * 7。直接ERPM指定したい場合は 1)
WHEEL_RADIUS = 0.03    # 車輪半径 [m] (加速度計算用: 75mm)

# 4つの足回りモーター設定 (IDと前進時の回転方向)
# 車体が直進するように、左右で回転方向の符号を逆に設定 (+1: 正転, -1: 逆転)
MOTORS = [
    {"name": "FL (左前)", "id": 0x1, "dir":  1},
    {"name": "RL (左後)", "id": 0x2, "dir":  1},
    {"name": "FR (右前)", "id": 0x0, "dir": -1},
    {"name": "RR (右後)", "id": 0x3, "dir": -1},
]

SEND_PERIOD_SEC = 0.01  # CAN送信周期 [秒] (50 Hz = 20 ms, VESC公式ドキュメント推奨)
KEY_TIMEOUT_SEC = 0.05  # キーを離したと判定して速度0にする時間 [秒]

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
LOG_DIR = os.path.join(PROJECT_DIR, "logs")
LOG_FILE = os.path.join(LOG_DIR, "vesc_test.csv") # 走行データCSV保存先
CONSOLE_LOG_FILE = os.path.join(LOG_DIR, "terminal_run.log") # ターミナル表示の自動保存先


# ==============================================================================
# ターミナル表示 & ファイル自動保存 (TeeLogger)
# ==============================================================================
class DualLogger:
    """標準出力/標準エラー出力を端末画面とログファイルの両方に同時書き込みするクラス"""
    def __init__(self, file_handle, stream):
        self.stream = stream
        self.file_handle = file_handle

    def write(self, message):
        self.stream.write(message)
        self.stream.flush()
        try:
            self.file_handle.write(message)
            self.file_handle.flush()
        except Exception:
            pass

    def flush(self):
        self.stream.flush()
        try:
            self.file_handle.flush()
        except Exception:
            pass

    def fileno(self):
        return self.stream.fileno()

    def isatty(self):
        return self.stream.isatty()


# ==============================================================================
# CAN 送受信ハンドラ (詳細エラー出力 & 確実送信 & ENOBUFS対策)
# ==============================================================================
class CanManager:
    def __init__(self, channel: str, bitrate: int, dry_run: bool = False):
        self.channel = channel
        self.dry_run = dry_run
        self.mode = "none"
        self.bus = None
        self.sock = None
        self.consecutive_errors = 0
        self.last_error_time = 0.0
        self.has_printed_enobufs_guide = False

        if self.dry_run:
            print(f"[CAN INIT] DRY-RUN (シミュレーション) モードで起動しました (実CAN送信なし)")
            return

        # 1. まず python-can (rox2026標準) を試行
        try:
            import can
            print(f"[CAN INIT] python-can を検出。インターフェース '{channel}' (bitrate={bitrate}) をオープン中...")
            self.bus = can.interface.Bus(channel=channel, interface="socketcan", bitrate=bitrate)
            self.mode = "python-can"
            print(f"[CAN INIT] 成功: python-can (socketcan) 経由で '{channel}' をオープンしました。")
            return
        except ImportError:
            print("[CAN INIT] python-can が未検出のため、標準 socket(AF_CAN) を試行します...")
        except Exception as e:
            print(f"[CAN INIT] python-can オープン失敗 ({type(e).__name__}: {e})。標準 socket(AF_CAN) を試行します...")

        # 2. 標準 socket(AF_CAN) を試行
        try:
            import socket
            print(f"[CAN INIT] socket.AF_CAN で '{channel}' をオープン中...")
            self.sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
            self.sock.bind((channel,))
            self.sock.settimeout(0.0)
            self.mode = "socketcan"
            print(f"[CAN INIT] 成功: socket.AF_CAN 経由で '{channel}' をオープンしました。")
        except Exception as e:
            print("\n" + "=" * 70)
            print(f"[CAN INIT 致命的エラー] CANインターフェース '{channel}' を開けませんでした！")
            print(f"エラー詳細: {type(e).__name__}: {e}")
            print("=" * 70)
            print("【確認と復旧手順】")
            print(f" 1. CANインターフェースが存在し、UPになっているか確認:")
            print(f"    ip link show {channel}")
            print(f" 2. UPになっていない場合は以下を実行:")
            print(f"    sudo ip link set {channel} txqueuelen 1000")
            print(f"    sudo ip link set {channel} up type can bitrate {bitrate} restart-ms 100")
            print(f" 3. CAN未接続環境で動作確認する場合は:")
            print(f"    python3 src/vesc_wheel_test.py --dry-run")
            print("=" * 70 + "\n")
            sys.exit(1)

    def send_rpm(self, controller_id: int, erpm: int, label: str = "") -> bool:
        """VESCへ PACKET_SET_RPM = 3 (Extended ID: (3 << 8) | controller_id) を送信"""
        arb_id = (3 << 8) | (controller_id & 0xFF)
        data = int(erpm).to_bytes(4, byteorder="big", signed=True)
        hex_data = " ".join(f"{b:02X}" for b in data)

        if self.dry_run:
            print(f"  [TX-DRY] 0x{controller_id:02X} ({label:<8}) | arb=0x{arb_id:03X} EXT | ERPM={erpm:+6d} | data=[{hex_data}]")
            return True

        tx_ok = False
        err_obj = None

        if self.mode == "python-can":
            import can
            msg = can.Message(arbitration_id=arb_id, data=data, is_extended_id=True)
            try:
                self.bus.send(msg, timeout=0.05)
                tx_ok = True
            except Exception as e:
                err_obj = e
        elif self.mode == "socketcan":
            can_id = arb_id | 0x80000000  # CAN_EFF_FLAG
            frame = struct.pack("=IB3x8s", can_id, 4, data.ljust(8, b"\x00"))
            try:
                self.sock.send(frame)
                tx_ok = True
            except Exception as e:
                err_obj = e

        if tx_ok:
            if self.consecutive_errors > 0:
                print(f"\n[CAN TX 復旧] CANパケット送信が正常に復帰しました (直前のエラー回数: {self.consecutive_errors}回)\n")
                self.consecutive_errors = 0
                self.has_printed_enobufs_guide = False
            return True
        else:
            self.consecutive_errors += 1
            now = time.monotonic()
            err_str = str(err_obj)
            is_enobufs = "105" in err_str or "No buffer space" in err_str or "ENOBUFS" in err_str

            # ENOBUFS の初回発生時に原因と対処手順を大きく表示
            if is_enobufs and not self.has_printed_enobufs_guide:
                self.has_printed_enobufs_guide = True
                print("\n" + "!" * 70)
                print("[CAN TX 致命的エラー: ENOBUFS (Error Code 105: No buffer space available)]")
                print("【原因】CANコントローラの送信バッファが満杯です！")
                print("  CANバス上にACK（受信確認）を返す相手機器（VESC）が1台もいないため、")
                print("  ハードウェアが再送を繰り返し、OSの送信キューが一瞬で詰まっています。")
                print("【確認・対処チェックリスト】")
                print("  1. VESCの主電源はONになっていますか？（LED点灯を確認）")
                print("  2. CAN_H と CAN_L の配線は正しいですか？（極性の逆接・断線・接触不良）")
                print("  3. CANバスの両端に 120Ω の終端抵抗はありますか？")
                print(f"  4. 通信速度（bitrate）は一致していますか？（VESC側設定と {BITRATE} bps）")
                print("  5. SocketCANの送信キュー拡張 & 自動再起動コマンドを実行してください:")
                print(f"       sudo ip link set {self.channel} down")
                print(f"       sudo ip link set {self.channel} txqueuelen 1000")
                print(f"       sudo ip link set {self.channel} up type can bitrate {BITRATE} restart-ms 100")
                print("!" * 70 + "\n")

            # ログの画面埋め尽くしを防ぐため、エラー表示は2秒に1回、または初回のみに制限
            if self.consecutive_errors == 1 or (now - self.last_error_time) >= 2.0:
                print(f"[CAN TX エラー] 0x{controller_id:02X} ({label}) 送信失敗: {type(err_obj).__name__}: {err_obj} (累積失敗: {self.consecutive_errors}回)")
                self.last_error_time = now

            return False

    def recv(self) -> Optional[Tuple[int, bool, bytes]]:
        """受信バッファから1フレーム取得: (arb_id, is_extended, data_bytes)"""
        if self.dry_run:
            return None
        if self.mode == "python-can":
            try:
                msg = self.bus.recv(timeout=0.0)
                if msg is None:
                    return None
                return msg.arbitration_id, msg.is_extended_id, bytes(msg.data)
            except Exception:
                return None
        elif self.mode == "socketcan":
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
        return None

    def close(self):
        if self.bus:
            try:
                self.bus.shutdown()
            except Exception:
                pass
        if self.sock:
            try:
                self.sock.close()
            except Exception:
                pass


# ==============================================================================
# メイン処理
# ==============================================================================
def main():
    # 端末出力ログの自動ファイル記録を開始
    os.makedirs(os.path.dirname(CONSOLE_LOG_FILE) or ".", exist_ok=True)
    f_console = open(CONSOLE_LOG_FILE, "w", encoding="utf-8", buffering=1)
    orig_stdout = sys.stdout
    orig_stderr = sys.stderr
    sys.stdout = DualLogger(f_console, orig_stdout)
    sys.stderr = DualLogger(f_console, orig_stderr)

    dry_run = "--dry-run" in sys.argv
    channel = CAN_CHANNEL
    target_rpm = TARGET_RPM
    for i, arg in enumerate(sys.argv):
        if arg == "--channel" and i + 1 < len(sys.argv):
            channel = sys.argv[i + 1]
        elif arg == "--rpm" and i + 1 < len(sys.argv):
            try:
                target_rpm = float(sys.argv[i + 1])
            except ValueError:
                pass

    print("=" * 70)
    print(" VESC 4-Wheel Drive Motor Test (確実送信 & 詳細ログ版)")
    print("=" * 70)
    print(f" CAN Channel : {channel}")
    print(f" Target Speed: {target_rpm:.0f} RPM (ERPM: {int(target_rpm * POLE_PAIRS)})")
    print(f" CSV Log     : {LOG_FILE} (走行データCSV)")
    print(f" Terminal Log: {CONSOLE_LOG_FILE} (ターミナル出力の自動保存先)")
    print(" Motors:")
    for m in MOTORS:
        dir_str = "正転(+1)" if m["dir"] > 0 else "反転(-1)"
        print(f"   - {m['name']:<12} ID: 0x{m['id']:02X} ({m['id']:3d}) | {dir_str}")
    print("-" * 70)
    print(" 操作方法:")
    print("   [w] 押し続け : 前進 (直進)")
    print("   [s] 押し続け : 後進 (直進)")
    print("   キーを離す   : 自動停止 (速度0送信)")
    print("   [q] / Ctrl+C : 終了 (全モーター停止)")
    print("=" * 70)

    # CAN初期化
    can = CanManager(channel, BITRATE, dry_run=dry_run)

    # 1. 起動時疎通テスト送信 (candumpで即座に確認できるように全モーターへ0を送信)
    print("\n[疎通テスト] 起動確認のため、全モーターへ速度0パケットを送信します...")
    all_tx_ok = True
    for m in MOTORS:
        arb_id = (3 << 8) | m["id"]
        ok = can.send_rpm(m["id"], 0, m["name"])
        status_str = "OK" if ok else "NG"
        print(f"   -> [TX] 0x{m['id']:02X} ({m['name']}) arb=0x{arb_id:03X} EXT | ERPM=0 | 結果: {status_str}")
        if not ok:
            all_tx_ok = False
    if all_tx_ok:
        print("[疎通テスト] 全モーターへのパケット送信に成功しました。(candumpで確認できます)\n")
    else:
        print("[疎通テスト 警告] 一部のパケット送信に失敗しました。CANバス配線や状態を確認してください。\n")

    # ログファイル準備
    os.makedirs(os.path.dirname(LOG_FILE) or ".", exist_ok=True)
    f_log = open(LOG_FILE, "w", newline="", encoding="utf-8")
    writer = csv.writer(f_log)
    header = ["elapsed_sec", "state", "target_rpm"]
    for m in MOTORS:
        header.extend([f"m_{m['id']:02x}_cmd", f"m_{m['id']:02x}_real", f"m_{m['id']:02x}_cur", f"m_{m['id']:02x}_accel"])
    writer.writerow(header)

    # モーター状態辞書
    motor_state = {
        m["id"]: {
            "real_rpm": 0.0,
            "current": 0.0,
            "last_rpm": 0.0,
            "last_time": 0.0,
            "accel": 0.0,
            "rx_count": 0,
        }
        for m in MOTORS
    }

    # 端末入力設定 (cbreak: キー入力を即座に検知)
    old_term = None
    if sys.stdin.isatty():
        old_term = termios.tcgetattr(sys.stdin.fileno())
        tty.setcbreak(sys.stdin.fileno())

    def send_all_rpms(speed_rpm: float):
        for m in MOTORS:
            cmd = speed_rpm * m["dir"]
            erpm = int(round(cmd * POLE_PAIRS))
            can.send_rpm(m["id"], erpm, m["name"])

    state = "STOP"
    prev_state = None
    last_key_time = 0.0
    start_time = time.monotonic()
    last_log_print_time = 0.0

    print("キー入力待機中... ('w'=前進, 's'=後進, 'q'=終了)")

    try:
        while True:
            t_now = time.monotonic()
            t_elapsed = t_now - start_time

            # 1. キーボード入力検知
            r, _, _ = select.select([sys.stdin], [], [], 0.0)
            if r:
                char = sys.stdin.read(1)
                if not char or char.lower() == "q":
                    print("\n[USER] 'q' が押されました。テストを終了します。")
                    break
                elif char.lower() == "w":
                    if state != "FORWARD":
                        print(f"\n[KEY EVENT] 'w' 検知 -> 【前進開始】 目標速度: +{target_rpm:.0f} RPM")
                    state = "FORWARD"
                    last_key_time = t_now
                elif char.lower() == "s":
                    if state != "REVERSE":
                        print(f"\n[KEY EVENT] 's' 検知 -> 【後進開始】 目標速度: -{target_rpm:.0f} RPM")
                    state = "REVERSE"
                    last_key_time = t_now
                elif char in (" ", "x", "X"):
                    if state != "STOP":
                        print(f"\n[KEY EVENT] 停止キー検知 -> 【停止】 速度0")
                    state = "STOP"
                    last_key_time = 0.0

            # 2. デッドマンタイムアウト (キーを離したら自動停止)
            if state in ("FORWARD", "REVERSE"):
                if (t_now - last_key_time) > KEY_TIMEOUT_SEC:
                    print(f"\n[DEADMAN] キー入力途絶 (離した) -> 【自動停止】 速度0")
                    state = "STOP"

            # 3. CAN受信処理 (VESC STATUSフレーム: 0x900 | id)
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
                        
                        st = motor_state[c_id]
                        st["rx_count"] += 1
                        if st["rx_count"] == 1:
                            print(f"\n[CAN RX 応答検知] モーター 0x{c_id:02X} から初回STATUS受信成功！ (実測RPM={real_rpm:+.0f})")

                        # 加速度計算 (実測RPMの時間微分: a = r * d(omega)/dt)
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

            # 4. 指令送信 (4輪へ同時にSET_RPM送信)
            cmd_rpms = {}
            for m in MOTORS:
                if state == "FORWARD":
                    cmd = target_rpm * m["dir"]
                elif state == "REVERSE":
                    cmd = -target_rpm * m["dir"]
                else:
                    cmd = 0.0
                cmd_rpms[m["id"]] = cmd
                erpm = int(round(cmd * POLE_PAIRS))
                can.send_rpm(m["id"], erpm, m["name"])

            # 5. CSVログ保存
            row = [f"{t_elapsed:.3f}", state, f"{target_rpm:.0f}"]
            for m in MOTORS:
                st = motor_state[m["id"]]
                row.extend([f"{cmd_rpms[m['id']]:.0f}", f"{st['real_rpm']:.0f}", f"{st['current']:.1f}", f"{st['accel']:.2f}"])
            writer.writerow(row)

            # 6. 状態出力 (状態変化時、または0.5秒おきに1行ログ出力)
            if state != prev_state or (t_now - last_log_print_time) >= 0.5:
                prev_state = state
                last_log_print_time = t_now
                status_parts = []
                for m in MOTORS:
                    st = motor_state[m["id"]]
                    status_parts.append(f"{m['name']}: 指令{cmd_rpms[m['id']]:+5.0f} / 実測{st['real_rpm']:+5.0f}rpm ({st['accel']:+5.2f}m/s²)")
                print(f"[{t_elapsed:5.1f}s][{state:<7}] " + " | ".join(status_parts))

            time.sleep(SEND_PERIOD_SEC)

    except KeyboardInterrupt:
        print("\n[USER] Ctrl+C が検出されました。")
    finally:
        print("\n" + "=" * 70)
        print("[終了処理] 全モーターへ速度0 (STOP) を送信中...")
        for _ in range(5):
            send_all_rpms(0.0)
            time.sleep(0.02)
        can.close()
        f_log.close()
        if old_term is not None:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, old_term)
        print(f"[終了処理] 走行データCSV保存完了   : {LOG_FILE}")
        print(f"[終了処理] ターミナル出力ログ保存完了: {CONSOLE_LOG_FILE}")
        print("[終了処理] モーターを停止し、安全に終了しました。")
        print("=" * 70 + "\n")

        # 端末出力を元に戻してファイルクローズ
        sys.stdout = orig_stdout
        sys.stderr = orig_stderr
        f_console.close()


if __name__ == "__main__":
    main()
