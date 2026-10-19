#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""对比图：不同导引律 / 导航比下的 λ̇(t) 与 |a|(t)。

用来直观展示「PPN 有效导航比 N_eff = N·V_M/V_c」这条结论：
同一迎头场景（V_M/V_c ≈ 0.5）下，
    PPN N=3  → N_eff=1.5 < 2 → λ̇ 发散
    PPN N=4  → N_eff=2.0 = 2 → λ̇ 持平（临界）
    PPN N=6  → N_eff=3.0 > 2 → λ̇ 收敛
    TPN N=4  → N_eff=4.0 > 2 → λ̇ 快速收敛

用法（需先由 pn_sim 生成各 CSV）：
    python plot_compare.py
默认读取同目录下：ppn3_log.csv ppn4_log.csv ppn6_log.csv tpn4_log.csv
"""

import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

# (文件, 图例, 颜色)
SERIES = [
    ("ppn3_log.csv", r"PPN N=3  ($N_{eff}=1.5$)", "#A32D2D"),
    ("ppn4_log.csv", r"PPN N=4  ($N_{eff}=2.0$，临界)", "#BA7517"),
    ("ppn6_log.csv", r"PPN N=6  ($N_{eff}=3.0$)", "#185FA5"),
    ("tpn4_log.csv", r"TPN N=4  ($N_{eff}=4.0$)", "#1D9E75"),
]


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
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 4.8))

    for name, label, color in SERIES:
        p = os.path.join(here, name)
        if not os.path.exists(p):
            print("skip (not found):", name)
            continue
        d = load(p)
        ax1.plot(d["t"], d["lam_dot"], color=color, lw=1.8, label=label)
        ax2.plot(d["t"], d["a_norm"], color=color, lw=1.8, label=label)

    ax1.set_xlabel("t (s)")
    ax1.set_ylabel(r"$\dot{\lambda}$ (rad/s)")
    ax1.set_title(r"视线角速率 $\dot{\lambda}(t)$：PPN 的 $N_{eff}$ 判据")
    ax1.set_ylim(0, 0.09)
    ax1.grid(alpha=0.3)
    ax1.legend(fontsize=9)

    ax2.set_xlabel("t (s)")
    ax2.set_ylabel(r"$|a|$ (m/s$^2$)")
    ax2.set_title("制导指令加速度：发散 vs 收敛")
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=9)

    fig.tight_layout()
    out = os.path.join(here, "pn_compare_plot.png")
    fig.savefig(out, dpi=130)
    print("saved:", out)


if __name__ == "__main__":
    main()
