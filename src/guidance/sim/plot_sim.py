#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""读取 pn_sim 输出的 CSV，画 3D 弹道 + R / λ̇ / 过载 曲线。

用法：
    python plot_sim.py [csv路径] [输出png路径]
默认：
    pn_sim_log.csv  ->  pn_sim_plot.png
"""

import csv
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401  注册 3D 投影

# 中文显示（Windows 上通常有微软雅黑）
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False


def load(path):
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        data = {k: [] for k in reader.fieldnames}
        for row in reader:
            for k in reader.fieldnames:
                data[k].append(float(row[k]))
    return data


def main():
    csv_path = sys.argv[1] if len(sys.argv) > 1 else "pn_sim_log.csv"
    out_png = sys.argv[2] if len(sys.argv) > 2 else "pn_sim_plot.png"

    d = load(csv_path)
    fig = plt.figure(figsize=(13, 9))

    # 1) 3D 弹道
    ax = fig.add_subplot(2, 2, 1, projection="3d")
    ax.plot(d["pMx"], d["pMy"], d["pMz"], color="#185FA5", lw=2, label="无人机")
    ax.plot(d["pTx"], d["pTy"], d["pTz"], color="#A32D2D", lw=2, label="目标")
    ax.scatter(d["pMx"][0], d["pMy"][0], d["pMz"][0], color="#185FA5", marker="o")
    ax.scatter(d["pTx"][0], d["pTy"][0], d["pTz"][0], color="#A32D2D", marker="o")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_zlabel("z (m)")
    ax.set_title("3D 弹道")
    ax.legend(fontsize=9)

    # 2) 相对距离
    ax2 = fig.add_subplot(2, 2, 2)
    ax2.plot(d["t"], d["R"], color="#185FA5")
    ax2.set_xlabel("t (s)")
    ax2.set_ylabel("R (m)")
    ax2.set_title("相对距离 R(t)")
    ax2.grid(alpha=0.3)

    # 3) 视线角速率（用 mathtext 画 λ̇，避免中文字体缺希腊字母）
    ax3 = fig.add_subplot(2, 2, 3)
    ax3.plot(d["t"], d["lam_dot"], color="#534AB7")
    ax3.set_xlabel("t (s)")
    ax3.set_ylabel(r"$\dot{\lambda}$ (rad/s)")
    ax3.set_title(r"视线角速率 $\dot{\lambda}(t)$")
    # 末端 R→0 时 (r×v_rel)/R² 会出现飞越尖峰（几何使然，制导已由 HOLD 冻结），
    # 用 symlog 才不会把整条曲线压成贴地直线。
    ax3.set_yscale("symlog", linthresh=0.01)
    ax3.grid(alpha=0.3, which="both")
    # 标注末端飞越尖峰
    lam = d["lam_dot"]
    if lam and max(lam) > 5 * (sorted(lam)[len(lam) // 2] + 1e-9):
        i_pk = lam.index(max(lam))
        ax3.annotate(
            "末端 R→0 飞越尖峰\n（几何使然；HOLD 已冻结指令）",
            xy=(d["t"][i_pk], lam[i_pk]),
            xytext=(0.30, 0.55),
            textcoords="axes fraction",
            fontsize=8,
            color="#A32D2D",
            arrowprops=dict(arrowstyle="->", color="#A32D2D", lw=1),
        )

    # 4) 指令加速度（从 0 起画，否则自动缩放会压成一条几乎水平的线）
    ax4 = fig.add_subplot(2, 2, 4)
    ax4.plot(d["t"], d["a_norm"], color="#854F0B")
    ax4.set_xlabel("t (s)")
    ax4.set_ylabel(r"$|a|$ (m/s$^2$)")
    ax4.set_title("制导指令加速度")
    ax4.set_ylim(0, max(d["a_norm"]) * 1.1 if d["a_norm"] else 1.0)
    ax4.grid(alpha=0.3)

    fig.tight_layout()
    fig.savefig(out_png, dpi=130)
    print("saved:", out_png)


if __name__ == "__main__":
    main()
