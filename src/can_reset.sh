#!/bin/bash
# ==============================================================================
# SocketCAN (can0) リセット & 診断スクリプト
# ==============================================================================

CHANNEL="${1:-can0}"
BITRATE="${2:-500000}"

echo "======================================================================"
echo " CAN インターフェース再設定 & 診断: ${CHANNEL} (${BITRATE} bps)"
echo "======================================================================"

echo "[1/2] can0 をリセットし、txqueuelen を 1000 に設定中..."
sudo ip link set "${CHANNEL}" down 2>/dev/null
sudo ip link set "${CHANNEL}" txqueuelen 1000
sudo ip link set "${CHANNEL}" up type can bitrate "${BITRATE}" loopback off restart-ms 100

echo ""
echo "[2/2] can0 のステータス・エラー統計:"
ip -details -statistics link show "${CHANNEL}"
echo "======================================================================"
echo " state が 'ERROR-ACTIVE' または 'UP' になっていることを確認してください。"
echo " 'BUS-OFF' や 'STOPPED' の場合はハードウェア結線や電源を確認してください。"
echo "======================================================================"
