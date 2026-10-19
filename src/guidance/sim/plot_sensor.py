#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""传感器墙对比图：同一场景 / 同一导引律，只换传感器逼真度。

读取 pn_sim 生成的 sensor_ideal.csv / sensor_semi.csv / sensor_full.csv，
画 4 张子图：λ̇(t) / |a|(t) / R(t) / 制导模式(含丢帧)。

用法：
    python plot_sensor.py          # 需先跑 ./bin/pn_sim
"""

import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

SERIES = [
    ("sensor_ideal.csv", "IDEAL（真值）", "#1D9E75"),
    ("sensor_semi.csv", "SEMI_REAL（投影+噪声+size 测距）", "#BA7517"),
    ("sensor_full.csv", "FULL_REAL（再加 50ms 延迟 + 5% 丢帧）", "#A32D2D"),
]

MODE_NAMES = {0: "PN", 1: "APN", 2: "HOLD", 3: "SEARCH"}


def load(path):
    with open(path, newline="", encoding="utf-8") as f:
        r = csv.DictReader(f)
        cols = {k: [] for k in r.fieldnames}
        for row in r:
            for k in r.fieldnames:
                cols[k].append(float(row[k]))
    return cols


def main():
    here = os.path.dirname(os.path.abspath(__file__))
    fig, axes = plt.subplots(2, 2, figsize=(13, 8.5))
    ax1, ax2, ax3, ax4 = axes.ravel()

    any_data = False
    for name, label, color in SERIES:
        p = os.path.join(here, name)
        if not os.path.exists(p):
            print("skip (not found):", name)
            continue
        any_data = True
        d = load(p)
        ax1.plot(d["t"], d["lam_dot"], color=color, lw=1.2, label=label)
        ax2.plot(d["t"], d["a_norm"], color=color, lw=1.2, label=label)
        # 测距误差 = 估计 R − 真值 R（两者同一时刻；size 反推在远距离误差大）
        err = [e - r for e, r in zip(d["R_est"], d["R_upd"])]
        ax3.plot(d["t"], err, color=color, lw=1.1, label=label)
        ax4.plot(d["t"], d["mode"], color=color, lw=1.0, drawstyle="steps-post", label=label)

    if not any_data:
        print("没有找到 sensor_*.csv，请先运行 ./bin/pn_sim")
        return

    ax1.set_xlabel("t (s)")
    ax1.set_ylabel(r"$\dot{\lambda}$ (rad/s)")
    ax1.set_title(r"视线角速率 $\dot{\lambda}(t)$：传感器噪声直接进来")
    ax1.set_yscale("symlog", linthresh=0.01)
    ax1.grid(alpha=0.3, which="both")
    ax1.legend(fontsize=8)

    ax2.set_xlabel("t (s)")
    ax2.set_ylabel(r"$|a|$ (m/s$^2$)")
    ax2.set_title("制导指令加速度：噪声把指令抬高数倍")
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8)

    ax3.axhline(0.0, color="#888888", lw=1.0)
    ax3.set_xlabel("t (s)")
    ax3.set_ylabel(r"$R_{est} - R$ (m)")
    ax3.set_title(r"测距误差 $R_{est}-R$：size 反推远距离差 + 延迟滞后(≈3 m@50 ms)")
    ax3.grid(alpha=0.3)
    ax3.legend(fontsize=8)

    ax4.set_xlabel("t (s)")
    ax4.set_ylabel("mode")
    ax4.set_yticks(list(MODE_NAMES.keys()))
    ax4.set_yticklabels(list(MODE_NAMES.values()))
    ax4.set_title("制导模式（3=SEARCH 即目标丢失/丢帧）")
    ax4.grid(alpha=0.3)
    ax4.legend(fontsize=8)

    fig.tight_layout()
    out = os.path.join(here, "sensor_compare_plot.png")
    fig.savefig(out, dpi=130)
    print("saved:", out)


if __name__ == "__main__":
    main()
