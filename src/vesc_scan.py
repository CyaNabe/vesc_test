#!/usr/bin/env python3
"""
VESC ID 自動スキャン & 疎通診断スクリプト (500 kbps 対応)
CANバス上のVESCモーターの存在と、実際のController IDを自動検出します。
"""

import sys
import time
import struct

try:
    import can
except ImportError:
    print("[ERROR] python-can が必要です: pip3 install python-can")
    sys.exit(1)

CHANNEL = "can0"
BITRATE = 500000
HOST_ID = 0xFD
PACKET_STATUS = 9
PACKET_PING = 17
PACKET_PONG = 18

def main():
    print("=" * 65)
    print(f" VESC CAN スキャナー ({CHANNEL} @ {BITRATE//1000} kbps)")
    print("=" * 65)
    print(f"[1/2] パッシブ受信テスト (2秒間、VESCからの自発パケットを待機)...")

    try:
        bus = can.interface.Bus(channel=CHANNEL, interface="socketcan", bitrate=BITRATE)
    except Exception as e:
        print(f"[ERROR] CANオープン失敗: {e}")
        print("以下を実行してください:")
        print(f"  sudo ip link set {CHANNEL} down")
        print(f"  sudo ip link set {CHANNEL} txqueuelen 1000")
        print(f"  sudo ip link set {CHANNEL} up type can bitrate {BITRATE} loopback off restart-ms 100")
        sys.exit(1)

    detected_ids = set()
    t_end = time.monotonic() + 2.0
    while time.monotonic() < t_end:
        msg = bus.recv(timeout=0.05)
        if msg is None:
            continue
        if msg.is_extended_id:
            pkt_id = (msg.arbitration_id >> 8) & 0xFF
            c_id = msg.arbitration_id & 0xFF
            if pkt_id == PACKET_STATUS:
                detected_ids.add(c_id)
                erpm = struct.unpack(">i", msg.data[0:4])[0] if len(msg.data) >= 4 else 0
                print(f"  -> [STATUS受信] VESC ID: 0x{c_id:02X} ({c_id:3d}) | 実測ERPM={erpm}")
            else:
                print(f"  -> [EXTパケット受信] arb=0x{msg.arbitration_id:08X} (packet_id={pkt_id}, controller_id={c_id})")

    print("\n[2/2] アクティブ PING スキャン (ID 0x00 〜 0x20 へ PING 送信)...")
    pong_ids = set()
    for target_id in range(0, 33):
        tx_arb = (PACKET_PING << 8) | target_id
        ping_msg = can.Message(arbitration_id=tx_arb, data=bytes([HOST_ID]), is_extended_id=True)
        try:
            bus.send(ping_msg, timeout=0.02)
        except Exception:
            pass

        # PONG返答待機 (0.03秒)
        t_wait = time.monotonic() + 0.03
        while time.monotonic() < t_wait:
            rx = bus.recv(timeout=0.005)
            if rx and rx.is_extended_id:
                pkt_id = (rx.arbitration_id >> 8) & 0xFF
                dest_id = rx.arbitration_id & 0xFF
                if pkt_id == PACKET_PONG and dest_id == HOST_ID:
                    responder_id = rx.data[0] if len(rx.data) >= 1 else target_id
                    pong_ids.add(responder_id)
                    print(f"  ★ [PONG受信!] VESC ID: 0x{responder_id:02X} ({responder_id:3d}) から応答がありました！")

    bus.shutdown()

    all_found = detected_ids | pong_ids
    print("\n" + "=" * 65)
    print("【スキャン結果サマリー】")
    if all_found:
        print(f"  検出された VESC ID: {[f'0x{i:02X}({i})' for i in sorted(all_found)]}")
        print("  -> vesc_wheel_test.py の MOTORS 配列の id を上記に合わせてください。")
    else:
        print("  VESC からの応答が一切検出されませんでした。")
        print("  【原因と対策】")
        print("  1. loopback が on になっていると物理バスに送受信されません:")
        print("     sudo ip link set can0 type can loopback off")
        print("  2. VESC の電源(LED点灯) / CAN_H・CAN_L 配線 / 終端抵抗(120Ω) を確認してください。")
    print("=" * 65 + "\n")

if __name__ == "__main__":
    main()
