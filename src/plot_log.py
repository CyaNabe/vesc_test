#!/usr/bin/env python3
"""
VESC 4-Wheel Drive Test Log Plotter (Minimal)
4輪足回り走行ログ解析・グラフ可視化スクリプト (最小構成版)

使用法:
  python3 src/plot_log.py               # logs/vesc_test.csv を読み込んでグラフ保存
  python3 src/plot_log.py [csv_path]    # 指定したCSVを読み込んでグラフ保存
"""

from __future__ import annotations

import csv
import os
import sys

import matplotlib
if "DISPLAY" not in os.environ and "WAYLAND_DISPLAY" not in os.environ:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# 日本語フォント設定
matplotlib.rcParams["font.family"] = ["IPAexGothic", "Noto Sans CJK JP", "IPAGothic", "DejaVu Sans"]
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
PROJECT_DIR = os.path.dirname(SCRIPT_DIR)
DEFAULT_LOG = os.path.join(PROJECT_DIR, "logs", "vesc_test.csv")
OUTPUT_PLOT = os.path.join(PROJECT_DIR, "logs", "vesc_test_plot.png")


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_LOG
    if not os.path.exists(csv_path):
        print(f"[ERROR] ログファイルが見つかりません: {csv_path}")
        print("まずは 'python3 src/vesc_wheel_test.py' を実行してください。")
        sys.exit(1)

    print(f"[INFO] ログ読み込み中: {csv_path}")

    # CSV読み込み
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.reader(f)
        header = next(reader)
        cols = {h.strip(): [] for h in header}
        for row in reader:
            if not row or len(row) != len(header):
                continue
            for h, v in zip(header, row):
                h_c = h.strip()
                v_c = v.strip()
                cols[h_c].append(v_c if h_c == "state" else float(v_c))

    t = np.array(cols["elapsed_sec"])
    if len(t) == 0:
        print("[WARNING] データが空です。")
        sys.exit(0)

    # モータープレフィックス検出 (m_31_, m_32_, ...)
    prefixes = sorted(list(set(col[:4] for col in cols if col.startswith("m_"))))
    colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]
    names = {"m_31": "FL (左前 0x31)", "m_32": "RL (左後 0x32)", "m_33": "FR (右前 0x33)", "m_34": "RR (右後 0x34)"}

    # サマリー表示
    print("=" * 60)
    print(f" 記録時間: {t[-1] - t[0]:.2f} 秒 (サンプル数: {len(t)})")
    for pref in prefixes:
        real_col = f"{pref}_real"
        accel_col = f"{pref}_accel"
        cur_col = f"{pref}_cur"
        name = names.get(pref, pref)
        max_rpm = np.max(np.abs(cols[real_col])) if real_col in cols else 0.0
        max_accel = np.max(np.abs(cols[accel_col])) if accel_col in cols else 0.0
        max_cur = np.max(np.abs(cols[cur_col])) if cur_col in cols else 0.0
        print(f" {name:<16}: 最高速度 {max_rpm:6.0f} RPM | 最大加速度 {max_accel:5.2f} m/s² | 最大電流 {max_cur:5.1f} A")
    print("=" * 60)

    # グラフ作成 (3段: 速度、車輪加速度、電流)
    fig, axes = plt.subplots(3, 1, figsize=(10, 9), sharex=True)
    fig.suptitle(f"VESC 4-Wheel Drive Analysis ({os.path.basename(csv_path)})", fontsize=13, fontweight="bold")

    # 1. 速度 (Target vs Real)
    ax1 = axes[0]
    for i, pref in enumerate(prefixes):
        cmd_k = f"{pref}_cmd"
        real_k = f"{pref}_real"
        c = colors[i % len(colors)]
        label = names.get(pref, pref)
        if cmd_k in cols:
            ax1.plot(t, cols[cmd_k], "--", color=c, alpha=0.5, label=f"{label} Cmd")
        if real_k in cols:
            ax1.plot(t, cols[real_k], "-", color=c, linewidth=1.5, label=f"{label} Real")
    ax1.axhline(0, color="gray", linestyle=":", linewidth=0.8)
    ax1.set_ylabel("Speed [RPM]", fontweight="bold")
    ax1.set_title("1. Motor Rotational Speed", loc="left", fontsize=11)
    ax1.grid(True, linestyle="--", alpha=0.5)
    ax1.legend(loc="upper right", ncol=4, fontsize=8)

    # 2. 車輪並進加速度 [m/s²] (実測RPM微分から算出)
    ax2 = axes[1]
    for i, pref in enumerate(prefixes):
        acc_k = f"{pref}_accel"
        c = colors[i % len(colors)]
        label = names.get(pref, pref)
        if acc_k in cols:
            ax2.plot(t, cols[acc_k], "-", color=c, linewidth=1.2, label=f"{label} Wheel Accel")
    ax2.axhline(0, color="gray", linestyle=":", linewidth=0.8)
    ax2.set_ylabel("Accel [m/s²]", fontweight="bold")
    ax2.set_title("2. Wheel Acceleration (from Motor RPM derivative)", loc="left", fontsize=11)
    ax2.grid(True, linestyle="--", alpha=0.5)
    ax2.legend(loc="upper right", ncol=4, fontsize=8)

    # 3. モーター電流 [A]
    ax3 = axes[2]
    for i, pref in enumerate(prefixes):
        cur_k = f"{pref}_cur"
        c = colors[i % len(colors)]
        label = names.get(pref, pref)
        if cur_k in cols:
            ax3.plot(t, cols[cur_k], "-", color=c, linewidth=1.3, label=f"{label} Current")
    ax3.axhline(0, color="gray", linestyle=":", linewidth=0.8)
    ax3.set_ylabel("Current [A]", fontweight="bold")
    ax3.set_xlabel("Time [s]", fontweight="bold")
    ax3.set_title("3. Motor Current", loc="left", fontsize=11)
    ax3.grid(True, linestyle="--", alpha=0.5)
    ax3.legend(loc="upper right", ncol=4, fontsize=8)

    plt.tight_layout()
    plt.savefig(OUTPUT_PLOT, dpi=130)
    plt.close(fig)
    print(f"[SUCCESS] グラフを保存しました: {OUTPUT_PLOT}\n")


if __name__ == "__main__":
    main()
