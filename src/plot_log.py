#!/usr/bin/env python3
"""
VESC & IMU Test Log Analysis and Plotting Script
4輪足回りVESC走行ログ解析・グラフ可視化スクリプト

CSVログファイルを読み込み、速度・加速度・電流・IMUデータの時系列グラフを生成してPNG保存します。

使用例:
  python3 src/plot_log.py                      # logs/ 内の最新ログを自動解析
  python3 src/plot_log.py logs/vesc_test_....csv # 指定ファイルを解析
  python3 src/plot_log.py --show               # GUIウィンドウでグラフ表示 (X11/Wayland環境)
"""

from __future__ import annotations

import argparse
import csv
import glob
import os
import sys
from typing import Any, Dict, List, Optional, Tuple

import matplotlib
# GUI環境がない（ヘッドレス/SSH環境）でも画像出力できるよう非対話バックエンドを設定
if "DISPLAY" not in os.environ and "WAYLAND_DISPLAY" not in os.environ:
    matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

# 日本語フォント設定 (利用可能な日本語フォントを優先設定)
matplotlib.rcParams["font.family"] = ["IPAexGothic", "Noto Sans CJK JP", "IPAGothic", "DejaVu Sans"]
matplotlib.rcParams["axes.unicode_minus"] = False


def find_latest_log(log_dir: str = "logs") -> Optional[str]:
    """指定ディレクトリ内の最新のCSVログファイルを探す"""
    pattern = os.path.join(log_dir, "vesc_test_*.csv")
    files = glob.glob(pattern)
    if not files:
        # ディレクトリ内の全CSVを探索
        pattern = os.path.join(log_dir, "*.csv")
        files = glob.glob(pattern)
    if not files:
        return None
    files.sort(key=os.path.getmtime, reverse=True)
    return files[0]


def load_log_data(filepath: str) -> Dict[str, Any]:
    """CSVログファイルをパースして辞書形式に変換"""
    if not os.path.exists(filepath):
        raise FileNotFoundError(f"ログファイルが見つかりません: {filepath}")

    with open(filepath, "r", encoding="utf-8") as f:
        reader = csv.reader(f)
        try:
            header = next(reader)
        except StopIteration:
            raise ValueError(f"ファイルが空です: {filepath}")

        # 各列のリストを初期化
        columns: Dict[str, List[Any]] = {col.strip(): [] for col in header}

        for row in reader:
            if not row or len(row) != len(header):
                continue
            for col_name, val in zip(header, row):
                col_clean = col_name.strip()
                val_clean = val.strip()
                # 数値変換の試行
                if col_clean in ("state",):
                    columns[col_clean].append(val_clean)
                else:
                    try:
                        columns[col_clean].append(float(val_clean))
                    except ValueError:
                        columns[col_clean].append(val_clean)

    # numpy配列へ変換
    data: Dict[str, Any] = {}
    for k, v in columns.items():
        if k == "state":
            data[k] = np.array(v, dtype=object)
        else:
            data[k] = np.array(v, dtype=float)

    data["_header"] = header
    data["_filepath"] = filepath
    return data


def print_summary_statistics(data: Dict[str, Any]) -> None:
    """走行データの要約統計量をコンソールに表示"""
    elapsed = data.get("elapsed_sec")
    if elapsed is None or len(elapsed) == 0:
        print("[WARNING] 有効なデータポイントがありません。")
        return

    duration = elapsed[-1] - elapsed[0]
    num_samples = len(elapsed)
    avg_rate = num_samples / duration if duration > 0 else 0.0

    print("=" * 70)
    print(" 走行ログ解析サマリー (Summary Statistics)")
    print("=" * 70)
    print(f" 対象ファイル    : {data['_filepath']}")
    print(f" 総記録時間      : {duration:.2f} 秒 (サンプル数: {num_samples}, 平均 {avg_rate:.1f} Hz)")

    # モーターごとの列を検出 (m_XX_...)
    header = data["_header"]
    motor_ids = sorted(list(set(col.split("_")[1] for col in header if col.startswith("m_"))))

    print("\n [モーター別 統計]")
    print(" Motor ID | Max Cmd RPM | Max Real RPM | Max Accel (m/s²) | Max Current (A)")
    print("-" * 70)
    for m_id in motor_ids:
        cmd_col = f"m_{m_id}_cmd_rpm"
        real_col = f"m_{m_id}_real_rpm"
        accel_col = f"m_{m_id}_accel_m_s2"
        cur_col = f"m_{m_id}_current_a"

        max_cmd = np.max(np.abs(data[cmd_col])) if cmd_col in data else 0.0
        max_real = np.max(np.abs(data[real_col])) if real_col in data else 0.0
        max_accel = np.max(np.abs(data[accel_col])) if accel_col in data else 0.0
        max_cur = np.max(np.abs(data[cur_col])) if cur_col in data else 0.0

        print(f"   0x{m_id:<5} | {max_cmd:11.1f} | {max_real:12.1f} | {max_accel:16.2f} | {max_cur:14.2f}")

    # IMU 統計 (有効なデータがある場合)
    if "imu_ax_m_s2" in data and np.any(data["imu_ax_m_s2"] != 0.0):
        print("\n [STM32 IMU 加速度 / ジャイロ 統計]")
        ax = data["imu_ax_m_s2"]
        ay = data["imu_ay_m_s2"]
        az = data["imu_az_m_s2"]
        gz = data.get("imu_gz_deg_s", np.zeros_like(ax))
        print(f"   並進加速度 X (前後): 最大 {np.max(ax):+.2f} m/s², 最小 {np.min(ax):+.2f} m/s²")
        print(f"   並進加速度 Y (左右): 最大 {np.max(ay):+.2f} m/s², 最小 {np.min(ay):+.2f} m/s²")
        print(f"   並進加速度 Z (上下): 最大 {np.max(az):+.2f} m/s², 最小 {np.min(az):+.2f} m/s²")
        print(f"   ヨー角速度 Z (旋回): 最大 {np.max(gz):+.1f} deg/s, 最小 {np.min(gz):+.1f} deg/s")
    else:
        print("\n [STM32 IMU] 受信データなし (車輪速度の微分加速度のみ使用)")
    print("=" * 70)


def plot_test_log(data: Dict[str, Any], output_path: Optional[str] = None, show_window: bool = False) -> str:
    """4段サブプロットでログデータを可視化"""
    t = data["elapsed_sec"]
    states = data.get("state")

    # モーターID検出
    header = data["_header"]
    motor_ids = sorted(list(set(col.split("_")[1] for col in header if col.startswith("m_"))))
    motor_colors = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728"]
    color_map = {m_id: motor_colors[i % len(motor_colors)] for i, m_id in enumerate(motor_ids)}

    # モーター配置ラベル (デフォルト対応)
    name_map = {
        "31": "FL (0x31)",
        "32": "RL (0x32)",
        "33": "FR (0x33)",
        "34": "RR (0x34)",
    }

    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
    fig.suptitle(f"VESC 4-Wheel Drive Test Analysis\nFile: {os.path.basename(data['_filepath'])}", fontsize=14, fontweight="bold")

    # 状態領域の背景ハイライト関数
    def highlight_states(ax):
        if states is None or len(states) == 0:
            return
        # 連続した状態区間を探す
        curr_state = states[0]
        start_idx = 0
        for i in range(1, len(states)):
            if states[i] != curr_state or i == len(states) - 1:
                t0 = t[start_idx]
                t1 = t[i]
                if curr_state == "FORWARD":
                    ax.axvspan(t0, t1, color="#2ca02c", alpha=0.12, label="FORWARD" if start_idx == 0 else "")
                elif curr_state == "REVERSE":
                    ax.axvspan(t0, t1, color="#ff7f0e", alpha=0.12, label="REVERSE" if start_idx == 0 else "")
                curr_state = states[i]
                start_idx = i

    # --------------------------------------------------------------------------
    # Subplot 1: 回転速度 (Target vs Real RPM)
    # --------------------------------------------------------------------------
    ax1 = axes[0]
    highlight_states(ax1)
    for m_id in motor_ids:
        cmd_col = f"m_{m_id}_cmd_rpm"
        real_col = f"m_{m_id}_real_rpm"
        label_base = name_map.get(m_id, f"ID 0x{m_id}")
        color = color_map[m_id]

        if cmd_col in data:
            ax1.plot(t, data[cmd_col], linestyle="--", alpha=0.6, color=color, label=f"{label_base} Cmd")
        if real_col in data:
            ax1.plot(t, data[real_col], linestyle="-", linewidth=1.5, color=color, label=f"{label_base} Real")

    ax1.axhline(0, color="gray", linestyle=":", linewidth=0.8)
    ax1.set_ylabel("Speed [RPM]", fontsize=11, fontweight="bold")
    ax1.set_title("1. Motor Rotational Speed (Target vs Measured)", fontsize=11, loc="left")
    ax1.grid(True, linestyle="--", alpha=0.6)
    ax1.legend(loc="upper right", ncol=4, fontsize=9)

    # --------------------------------------------------------------------------
    # Subplot 2: 加速度応答 (Wheel Linear Accel & IMU Accel [m/s²])
    # --------------------------------------------------------------------------
    ax2 = axes[1]
    highlight_states(ax2)
    # 各車輪の微分加速度
    for m_id in motor_ids:
        accel_col = f"m_{m_id}_accel_m_s2"
        label_base = name_map.get(m_id, f"ID 0x{m_id}")
        color = color_map[m_id]
        if accel_col in data:
            ax2.plot(t, data[accel_col], linewidth=1.2, alpha=0.8, color=color, label=f"{label_base} Wheel Accel")

    # STM32 IMU 加速度 (データが存在する場合)
    has_imu_accel = "imu_ax_m_s2" in data and np.any(data["imu_ax_m_s2"] != 0.0)
    if has_imu_accel:
        ax2.plot(t, data["imu_ax_m_s2"], color="#9467bd", linewidth=2.0, label="IMU Accel-X (前後)")
        ax2.plot(t, data["imu_ay_m_s2"], color="#8c564b", linewidth=1.5, linestyle="-.", label="IMU Accel-Y (左右)")

    ax2.axhline(0, color="gray", linestyle=":", linewidth=0.8)
    ax2.set_ylabel("Accel [m/s²]", fontsize=11, fontweight="bold")
    ax2.set_title("2. Linear Acceleration (Wheel Derivative & STM32 IMU)", fontsize=11, loc="left")
    ax2.grid(True, linestyle="--", alpha=0.6)
    ax2.legend(loc="upper right", ncol=3, fontsize=9)

    # --------------------------------------------------------------------------
    # Subplot 3: モーター電流 (Current [A])
    # --------------------------------------------------------------------------
    ax3 = axes[2]
    highlight_states(ax3)
    for m_id in motor_ids:
        cur_col = f"m_{m_id}_current_a"
        label_base = name_map.get(m_id, f"ID 0x{m_id}")
        color = color_map[m_id]
        if cur_col in data:
            ax3.plot(t, data[cur_col], linewidth=1.5, color=color, label=f"{label_base} Current")

    ax3.axhline(0, color="gray", linestyle=":", linewidth=0.8)
    ax3.set_ylabel("Current [A]", fontsize=11, fontweight="bold")
    ax3.set_title("3. Motor Current Consumption", fontsize=11, loc="left")
    ax3.grid(True, linestyle="--", alpha=0.6)
    ax3.legend(loc="upper right", ncol=4, fontsize=9)

    # --------------------------------------------------------------------------
    # Subplot 4: デューティ比 [%] & IMU ヨー角速度 [deg/s]
    # --------------------------------------------------------------------------
    ax4 = axes[3]
    highlight_states(ax4)
    for m_id in motor_ids:
        duty_col = f"m_{m_id}_duty"
        label_base = name_map.get(m_id, f"ID 0x{m_id}")
        color = color_map[m_id]
        if duty_col in data:
            ax4.plot(t, data[duty_col] * 100.0, linewidth=1.3, color=color, label=f"{label_base} Duty")

    ax4.axhline(0, color="gray", linestyle=":", linewidth=0.8)
    ax4.set_ylabel("Duty Cycle [%]", fontsize=11, fontweight="bold")
    ax4.set_xlabel("Elapsed Time [s]", fontsize=11, fontweight="bold")
    ax4.set_title("4. Output Duty Cycle & Yaw Rate (Straight-line Stability)", fontsize=11, loc="left")
    ax4.grid(True, linestyle="--", alpha=0.6)

    # ヨー角速度の2軸目プロット (直進性確認)
    has_imu_gyro = "imu_gz_deg_s" in data and np.any(data["imu_gz_deg_s"] != 0.0)
    if has_imu_gyro:
        ax4_right = ax4.twinx()
        line_gz = ax4_right.plot(t, data["imu_gz_deg_s"], color="magenta", linewidth=1.8, linestyle="--", label="IMU Gyro-Z (Yaw Rate)")
        ax4_right.set_ylabel("Yaw Rate [deg/s]", color="magenta", fontsize=11, fontweight="bold")
        ax4_right.tick_params(axis="y", labelcolor="magenta")
        # 凡例統合
        lines1, labels1 = ax4.get_legend_handles_labels()
        lines2, labels2 = ax4_right.get_legend_handles_labels()
        ax4.legend(lines1 + lines2, labels1 + labels2, loc="upper right", ncol=5, fontsize=9)
    else:
        ax4.legend(loc="upper right", ncol=4, fontsize=9)

    plt.tight_layout()

    # 保存処理
    if output_path is None:
        base_name = os.path.splitext(os.path.basename(data["_filepath"]))[0]
        output_dir = os.path.dirname(data["_filepath"]) or "."
        output_path = os.path.join(output_dir, f"plot_{base_name}.png")

    plt.savefig(output_path, dpi=150)
    print(f"\n[SUCCESS] グラフ画像を保存しました: {output_path}")

    if show_window:
        try:
            plt.show()
        except Exception as exc:
            print(f"[NOTE] ウィンドウ表示スキップ: {exc}")

    plt.close(fig)
    return output_path


def main() -> None:
    parser = argparse.ArgumentParser(
        description="VESC & IMU 4-Wheel Drive Test Log Plotter and Analyzer",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        "logfile",
        nargs="?",
        default=None,
        help="解析対象のCSVログファイルパス (未指定時は logs/ 内の最新ファイルを自動選択)",
    )
    parser.add_argument(
        "--output", "-o",
        default=None,
        help="出力するグラフ画像 (PNG) の保存パス (未指定時は plot_<csv名>.png)",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="GUIウィンドウでグラフを表示する",
    )
    args = parser.parse_args()

    filepath = args.logfile
    if filepath is None:
        filepath = find_latest_log()
        if filepath is None:
            print("[ERROR] logs/ ディレクトリ内にCSVログファイルが見つかりません。")
            print("まずは 'python3 src/vesc_wheel_test.py' を実行してテスト走行を行ってください。")
            sys.exit(1)
        print(f"[INFO] 最新のログファイルを自動選択しました: {filepath}")

    data = load_log_data(filepath)
    print_summary_statistics(data)
    plot_test_log(data, output_path=args.output, show_window=args.show)


if __name__ == "__main__":
    main()
