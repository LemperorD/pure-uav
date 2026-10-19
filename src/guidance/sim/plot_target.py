#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""飞镖目标模型图：弹道 / 速度衰减 / 加速度。

读取 pn_sim 导出的 target_*.csv（目标真值弹道，只依赖 t，与拦截过程无关），
以及 pn_sim_log.csv（含无人机轨迹，用于把拦截过程叠到 3D 图上）。

四张子图：
  ① 3D 弹道：无人机 + 飞镖 + 命中点
  ② 侧视 x-z：抛物线弧，以及阻力如何缩短射程
  ③ 目标速率 |v_T|(t)：阻力导致单调衰减
  ④ 目标加速度 |a_T|(t)：重力恒定 9.81，阻力随 v² 叠加 → a_T 时变（APN 要用）

用法：
    python plot_target.py          # 需先跑一次 ./bin/pn_sim
"""

import csv
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  (注册 3d projection)

plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False

G = 9.81

# 三条对比曲线：基准 / 只加重力 / 重力+阻力
SERIES = [
    ("target_STRAIGHT.csv",  "STRAIGHT（无重力无阻力）", "#888888", "--"),
    ("target_BALLISTIC.csv", "BALLISTIC（只受重力）",     "#534AB7", "-"),
    ("target_DART.csv",      "DART（重力+气动阻力）",     "#A32D2D", "-"),
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

    fig = plt.figure(figsize=(14, 9))
    ax3d = fig.add_subplot(2, 2, 1, projection="3d")
    ax2 = fig.add_subplot(2, 2, 2)
    ax3 = fig.add_subplot(2, 2, 3)
    ax4 = fig.add_subplot(2, 2, 4)

    have = {}
    for name, label, color, ls in SERIES:
        p = os.path.join(here, name)
        if not os.path.exists(p):
            print("skip (not found):", name)
            continue
        d = load(p)
        have[name] = d

        # ① 3D
        ax3d.plot(d["px"], d["py"], d["pz"], color=color, lw=1.6, ls=ls, label=label)
        ax3d.scatter(d["px"][0], d["py"][0], d["pz"][0],
                     color=color, marker="o", s=28, depthshade=False)
        ax3d.scatter(d["px"][-1], d["py"][-1], d["pz"][-1],
                     color=color, marker="x", s=34, depthshade=False)

        # ② 侧视 x-z
        ax2.plot(d["px"], d["pz"], color=color, lw=1.6, ls=ls, label=label)

        # ③ 速率
        spd = [ (vx**2 + vy**2 + vz**2) ** 0.5 for vx, vy, vz in zip(d["vx"], d["vy"], d["vz"]) ]
        ax3.plot(d["t"], spd, color=color, lw=1.5, ls=ls, label=label)

        # ④ 加速度模长（STRAIGHT 恒为 0，画进来会把量程撑开 → 单独说明，不画）
        if "STRAIGHT" not in name:
            acc = [ (ax**2 + ay**2 + az**2) ** 0.5 for ax, ay, az in zip(d["ax"], d["ay"], d["az"]) ]
            ax4.plot(d["t"], acc, color=color, lw=1.5, ls=ls, label=label)

    # 把无人机轨迹 + 命中点叠到 3D / 侧视图上
    log = os.path.join(here, "pn_sim_log.csv")
    if os.path.exists(log):
        d = load(log)
        ax3d.plot(d["pMx"], d["pMy"], d["pMz"], color="#1D9E75", lw=1.8, label="无人机（PPN N=4）")
        ax2.plot(d["pMx"], d["pMz"], color="#1D9E75", lw=1.8, label="无人机（PPN N=4）")
        i = min(range(len(d["R"])), key=lambda k: d["R"][k])
        ax3d.scatter([d["pTx"][i]], [d["pTy"][i]], [d["pTz"][i]],
                     color="#111111", marker="*", s=140, depthshade=False, label="最近点")
        ax2.scatter([d["pTx"][i]], [d["pTz"][i]], color="#111111", marker="*", s=140, label="最近点")
        ax2.annotate(f"最近距离 {d['R'][i]:.3f} m",
                     xy=(d["pTx"][i], d["pTz"][i]), xytext=(d["pTx"][i] + 3, d["pTz"][i] + 3),
                     fontsize=9,
                     arrowprops=dict(arrowstyle="->", color="#111111", lw=1.0))

    if not have:
        print("没有找到 target_*.csv，请先运行 ./bin/pn_sim")
        return

    ax3d.set_xlabel("x 东 (m)")
    ax3d.set_ylabel("y 北 (m)")
    ax3d.set_zlabel("z 天 (m)")
    ax3d.set_title("3D 弹道：飞镖抛物线 vs 无人机拦截")
    ax3d.legend(fontsize=8, loc="upper right")
    ax3d.view_init(elev=18, azim=-72)
    # STRAIGHT（无重力）会一路爬到 24 m 出框 —— 正好说明"重力决定了弹道"
    ax3d.set_zlim(0, 12)

    ax2.set_xlabel("x 东 (m)")
    ax2.set_ylabel("z 天 (m)")
    ax2.set_title("侧视 x–z：抛物线弧（阻力使射程缩短）")
    ax2.grid(alpha=0.3)
    ax2.legend(fontsize=8, loc="upper left")
    ax2.set_ylim(0, 12)
    ax2.text(0.98, 0.94, "STRAIGHT（无重力）已飞出上边界（z→24 m）",
             transform=ax2.transAxes, fontsize=8, color="#888888", ha="right")

    ax3.set_xlabel("t (s)")
    ax3.set_ylabel(r"$|v_T|$ (m/s)")
    ax3.set_title(r"目标速率 $|v_T|(t)$：阻力 → 单调衰减")
    ax3.grid(alpha=0.3)
    ax3.legend(fontsize=8)

    ax4.axhline(G, color="#534AB7", lw=0.9, ls=":", alpha=0.8)
    ax4.annotate("g = 9.81", xy=(0.02, G), xycoords=("axes fraction", "data"),
                 fontsize=8, color="#534AB7", va="bottom")
    ax4.set_xlabel("t (s)")
    ax4.set_ylabel(r"$|a_T|$ (m/s$^2$)")
    ax4.set_title(r"目标加速度 $|a_T|(t)$：重力恒定 + 阻力 ∝$v^2$ → 时变（APN 的输入）")
    ax4.grid(alpha=0.3)
    ax4.legend(fontsize=8, loc="lower right")
    ax4.text(0.02, 0.06, "STRAIGHT: $|a_T|\\equiv 0$（未画）",
             transform=ax4.transAxes, fontsize=8, color="#888888")

    fig.tight_layout()
    out = os.path.join(here, "target_model_plot.png")
    fig.savefig(out, dpi=130)
    print("saved:", out)


if __name__ == "__main__":
    main()
