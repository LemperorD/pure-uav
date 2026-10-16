#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""场景复现 + 拦截过程可视化（Python 绘图；**核心算法仍是 C++**）

【这一层做什么】
  · 把 `scene_model.Scene`（塔 / 发射场 / 弹丸 / 无人机 / 场地）**画出来**；
  · 驱动 `sim_phase_b_offline` 跑闭环仿真（真 C++ 制导核 + 真 C++ TargetModel），
    再把轨迹画成多视图（3D / 俯视 / 侧视 / 时序）。
  · 本文件 **不含任何制导 / 动力学算法**，只做"取数 → 画图"。

【产物】（默认写到 --out 目录）
  viz_scene_layout.png      静态场景复现：3D + 俯视 + 侧视（含弹丸标称弹道）
  viz_intercept.png         拦截全过程：3D + 俯视 + 侧视 + 4 张时序曲线
  viz_intercept_anim.gif    拦截过程动画（可选，--anim；需 pillow）
  traj_law<k>.csv           每条制导律的轨迹原始数据（由仿真脚本导出）

用法：
    python viz_scene.py                          # 默认场景 + APN
    python viz_scene.py --laws 0,2               # 同时对比 PPN / APN
    python viz_scene.py --scene scene.json       # 从场景文件加载
    python viz_scene.py --anim                   # 额外产出 GIF 动画
    python viz_scene.py --preset phase_b         # 换成阶段 B 预设场景
"""
from __future__ import annotations

import argparse
import csv
import math
import os
import sys

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyArrowPatch, Rectangle
from mpl_toolkits.mplot3d.art3d import Poly3DCollection, Line3DCollection

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import scene_model as SM                                  # noqa: E402
import sim_phase_b_offline as SIM                         # noqa: E402

# --------------------------------------------------------------------------
#  中文字体（Windows / WSL 都能命中）
# --------------------------------------------------------------------------
plt.rcParams["font.sans-serif"] = ["Microsoft YaHei", "SimHei", "Noto Sans CJK SC",
                                   "WenQuanYi Zen Hei", "DejaVu Sans"]
plt.rcParams["axes.unicode_minus"] = False
plt.rcParams["figure.dpi"] = 110

# --------------------------------------------------------------------------
#  配色
# --------------------------------------------------------------------------
C = {
    "tower":      "#8d6e63",
    "tower_top":  "#a1887f",
    "ring":       "#d32f2f",
    "wall":       "#9e9e9e",
    "ground":     "#f0f0f0",
    "box":        "#2e7d32",
    "drone":      "#1565c0",
    "drone_cru":  "#90caf9",
    "proj":       "#e53935",
    "proj_nom":   "#ffab91",
    "hit":        "#f9a825",
    "launch":     "#6a1b9a",
    "tower_pt":   "#4e342e",
}
LAW_COLORS = {0: "#e53935", 1: "#fb8c00", 2: "#1565c0"}
LAW_NAMES = {0: "PPN", 1: "TPN", 2: "APN"}


# ==========================================================================
#  一、取数：跑仿真 / 取标称弹道
# ==========================================================================
def open_core():
    """载入真 C++ 制导核（shim），返回 (lib, handle)。"""
    lib = SIM.load_lib(SIM.find_lib(None))
    return lib


def configure_core(L, sc):
    h = L.pgu_create()
    L.pgu_config_target(h, SIM.arr(SIM.target_cfg(sc)))
    L.pgu_config_guidance(h, SIM.arr(SIM.guidance_cfg(sc)))
    L.pgu_config_sensor(h, SIM.arr(SIM.sensor_cfg()))
    return h


def run_case(L, scene, law, csv_path):
    """跑一条制导律的闭环仿真，返回 (result_dict, sc)。"""
    sc = scene.to_sim_dict()
    sc["law"] = law
    sc["tower"] = scene.tower
    h = configure_core(L, sc)
    res = SIM.run(L, h, sc, csv_path=csv_path)
    L.pgu_destroy(h)
    return res, sc


def nominal_projectile(L, sc, dt=0.005):
    """弹丸的**标称弹道**（不拦截时的自由飞行），用于"若不拦截会怎样"。"""
    h = configure_core(L, sc)
    T = L.pgu_target_flight_time(h)
    tgt = SIM.arr([0.0] * 9)
    ts, pts = [], []
    t = 0.0
    while t <= T + 1e-9:
        L.pgu_target_state(h, t, tgt)
        ts.append(t)
        pts.append((tgt[0], tgt[1], tgt[2]))
        t += dt
    L.pgu_destroy(h)
    return np.array(ts), np.array(pts)


def read_traj(path):
    """读仿真导出的 CSV → dict of np.ndarray。"""
    cols = {}
    with open(path, newline="", encoding="utf-8") as fp:
        rd = csv.DictReader(fp)
        for name in rd.fieldnames:
            cols[name] = []
        for row in rd:
            for k, v in row.items():
                cols[k].append(v)
    out = {}
    for k, v in cols.items():
        if k == "phase":
            out[k] = np.array(v, dtype=object)
        else:
            out[k] = np.array([float(x) for x in v], dtype=float)
    return out


# ==========================================================================
#  二、几何绘制原语
# ==========================================================================
def box_faces(center, size):
    """长方体的 6 个面（每个面 4 个顶点），供 Poly3DCollection 用。"""
    cx, cy, cz = center
    sx, sy, sz = size
    x0, x1 = cx - sx / 2, cx + sx / 2
    y0, y1 = cy - sy / 2, cy + sy / 2
    z0, z1 = cz - sz / 2, cz + sz / 2
    v = [(x0, y0, z0), (x1, y0, z0), (x1, y1, z0), (x0, y1, z0),
         (x0, y0, z1), (x1, y0, z1), (x1, y1, z1), (x0, y1, z1)]
    idx = [(0, 1, 2, 3), (4, 5, 6, 7), (0, 1, 5, 4),
           (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    return [[v[i] for i in f] for f in idx]


def draw_box_3d(ax, center, size, color, alpha=0.22, edge=None, lw=0.8):
    faces = box_faces(center, size)
    pc = Poly3DCollection(faces, facecolors=color, edgecolors=edge or color,
                          linewidths=lw, alpha=alpha)
    ax.add_collection3d(pc)


def draw_cylinder_3d(ax, center, radius, height, color, alpha=0.55, n=48):
    cx, cy, cz = center
    th = np.linspace(0.0, 2.0 * np.pi, n)
    zz = np.array([cz, cz + height])
    TH, ZZ = np.meshgrid(th, zz)
    X = cx + radius * np.cos(TH)
    Y = cy + radius * np.sin(TH)
    ax.plot_surface(X, Y, ZZ, color=color, alpha=alpha,
                    linewidth=0, antialiased=True, shade=True)
    # 顶面圆盘
    r = np.linspace(0, radius, 8)
    R, T2 = np.meshgrid(r, th)
    ax.plot_surface(cx + R * np.cos(T2), cy + R * np.sin(T2),
                    np.full_like(R, cz + height), color=C["tower_top"],
                    alpha=alpha, linewidth=0, shade=False)


def draw_tower_2d(ax, tower, show_ring=True):
    cx, cy = tower.center[0], tower.center[1]
    ax.add_patch(Circle((cx, cy), tower.radius, facecolor=C["tower"],
                        edgecolor="black", lw=1.0, alpha=0.85, zorder=5))
    if show_ring:
        ax.add_patch(Circle((cx, cy), tower.radius + tower.safety_margin,
                            facecolor="none", edgecolor=C["ring"],
                            lw=1.4, ls="--", zorder=5))
        ax.add_patch(Circle((cx, cy), tower.radius + tower.safety_margin,
                            facecolor=C["ring"], alpha=0.06, zorder=1))


def draw_field_2d(ax, fld):
    for c, s in fld.obstacles:
        ax.add_patch(Rectangle((c[0] - s[0] / 2, c[1] - s[1] / 2), s[0], s[1],
                               facecolor=C["wall"], edgecolor="#616161",
                               lw=0.8, alpha=0.55, zorder=2))
    b0, b1 = fld.bounds_min, fld.bounds_max
    ax.add_patch(Rectangle((b0[0], b0[1]), b1[0] - b0[0], b1[1] - b0[1],
                           facecolor="none", edgecolor=C["box"],
                           lw=1.2, ls=":", zorder=3))


def draw_field_3d(ax, fld):
    for c, s in fld.obstacles_above_ground():
        draw_box_3d(ax, c, s, C["wall"], alpha=0.16, edge="#757575", lw=0.6)
    b0, b1 = fld.bounds_min, fld.bounds_max
    cx = [(b0[0] + b1[0]) / 2, (b0[1] + b1[1]) / 2, (b0[2] + b1[2]) / 2]
    sz = (b1[0] - b0[0], b1[1] - b0[1], b1[2] - b0[2])
    draw_box_3d(ax, cx, sz, "none", alpha=0.0, edge=C["box"], lw=1.2)
    # 地面
    gs = fld.ground_size / 2
    ax.add_collection3d(Poly3DCollection(
        [[(-gs, -gs, 0), (gs, -gs, 0), (gs, gs, 0), (-gs, gs, 0)]],
        facecolors=C["ground"], edgecolors="#bdbdbd", linewidths=0.5, alpha=0.5))


def set_axes_equal_3d(ax, lims):
    ax.set_xlim(lims[0]); ax.set_ylim(lims[1]); ax.set_zlim(lims[2])
    ax.set_box_aspect((lims[0][1] - lims[0][0],
                       lims[1][1] - lims[1][0],
                       lims[2][1] - lims[2][0]))


def arrow2d(ax, p0, p1, color, lw=1.8, ls="-", label=None, alpha=1.0):
    ax.add_patch(FancyArrowPatch((p0[0], p0[1]), (p1[0], p1[1]),
                                 arrowstyle="-|>", mutation_scale=14,
                                 color=color, lw=lw, ls=ls, alpha=alpha,
                                 zorder=8, label=label))


def v3txt(v, nd=1):
    return f"({v[0]:.{nd}f}, {v[1]:.{nd}f}, {v[2]:.{nd}f})"


def add_zoom(host, cases, tower, center, half, kind):
    """在 host 轴内嵌一个「交会区放大」子图。

    kind='xy' → 俯视；kind='xz' → 侧视。中心取真实最近点，窗口 ±half。
    """
    axz = host.inset_axes([0.03, 0.05, 0.36, 0.36])
    axz.set_facecolor("#fffdf5")
    law0, res0, tr0 = cases[0]
    if kind == "xy":
        draw_tower_2d(axz, tower, show_ring=True)
        for law, res, tr in cases:
            axz.plot(tr["pMx"], tr["pMy"], color=LAW_COLORS.get(law, "k"), lw=1.4)
        axz.plot(tr0["pTx"], tr0["pTy"], color=C["proj"], lw=1.4, ls="--")
        axz.set_xlim(center[0] - half, center[0] + half)
        axz.set_ylim(center[1] - half, center[1] + half)
        axz.set_aspect("equal")
        axz.set_xlabel("x (m)", fontsize=7); axz.set_ylabel("y (m)", fontsize=7)
    else:
        for law, res, tr in cases:
            axz.plot(tr["pMx"], tr["pMz"], color=LAW_COLORS.get(law, "k"), lw=1.4)
        axz.plot(tr0["pTx"], tr0["pTz"], color=C["proj"], lw=1.4, ls="--")
        axz.set_xlim(center[0] - half, center[0] + half)
        axz.set_ylim(center[2] - half, center[2] + half)
        axz.set_xlabel("x (m)", fontsize=7); axz.set_ylabel("z (m)", fontsize=7)
    if res0["p_M_at_min"]:
        p = res0["p_M_at_min"]
        xy = (p[0], p[1]) if kind == "xy" else (p[0], p[2])
        axz.scatter(*xy, color=C["hit"], s=90, marker="*", zorder=12,
                    edgecolor="k", lw=0.4)
    axz.tick_params(labelsize=6.5)
    axz.set_title("交会区放大", fontsize=7.5, pad=2)
    for sp in axz.spines.values():
        sp.set_color(C["hit"]); sp.set_linewidth(1.2)
    return axz


# ==========================================================================
#  三、图 1：静态场景复现
# ==========================================================================
def fig_layout(scene, nom_pts, nom_t, tev, out_path):
    tower, fld, proj, drn, lch = (scene.tower, scene.field_, scene.projectile,
                                  scene.drone, scene.launcher)
    fig = plt.figure(figsize=(19, 6.4))
    fig.suptitle(f"场景复现 · {scene.name}   （ENU：x=东，y=北，z=上）",
                 fontsize=15, fontweight="bold")

    lim = ((-24, 16), (-13, 8), (0, 22))

    # ---------- (a) 3D ----------
    ax = fig.add_subplot(1, 3, 1, projection="3d")
    draw_field_3d(ax, fld)
    draw_cylinder_3d(ax, tower.center, tower.radius, tower.height, C["tower"])
    # 塔保护圈（地面投影）
    th = np.linspace(0, 2 * np.pi, 80)
    rr = tower.radius + tower.safety_margin
    ax.plot(tower.center[0] + rr * np.cos(th), tower.center[1] + rr * np.sin(th),
            np.zeros_like(th), color=C["ring"], lw=1.6, ls="--")
    # 标称弹道
    ax.plot(nom_pts[:, 0], nom_pts[:, 1], nom_pts[:, 2],
            color=C["proj"], lw=2.2, ls="--", label="弹丸标称弹道（不拦截）")
    # 发射点 / 无人机起点
    ax.scatter(*lch.position, color="#4527a0", s=110, marker="s",
               label="敌方发射场", depthshade=False)
    ax.plot([lch.position[0], proj.launch_point[0]],
            [lch.position[1], proj.launch_point[1]],
            [lch.position[2], proj.launch_point[2]],
            color=C["launch"], lw=1.0, ls=":", alpha=0.7)
    ax.scatter(*proj.launch_point, color=C["launch"], s=70, marker="^",
               label="弹丸发射点", depthshade=False)
    ax.scatter(*drn.position, color=C["drone"], s=70, marker="o",
               label="无人机起点", depthshade=False)
    v = drn.velocity
    ax.quiver(*drn.position, v[0] * 1.2, v[1] * 1.2, v[2] * 1.2,
              color=C["drone"], lw=2.0, arrow_length_ratio=0.28)
    lv = proj.launch_velocity()
    ax.quiver(*proj.launch_point, lv[0] * 0.5, lv[1] * 0.5, lv[2] * 0.5,
              color=C["proj"], lw=2.0, arrow_length_ratio=0.28)
    set_axes_equal_3d(ax, lim)
    ax.set_xlabel("x 东 (m)"); ax.set_ylabel("y 北 (m)"); ax.set_zlabel("z 上 (m)")
    ax.set_title("(a) 3D 透视", fontsize=11)
    ax.legend(fontsize=7.5, loc="upper left")
    ax.view_init(elev=22, azim=-62)

    # ---------- (b) 俯视 ----------
    ax = fig.add_subplot(1, 3, 2)
    draw_field_2d(ax, fld)
    draw_tower_2d(ax, tower)
    ax.plot(nom_pts[:, 0], nom_pts[:, 1], color=C["proj"], lw=2.0, ls="--",
            label="弹丸标称弹道")
    ax.plot([lch.position[0], proj.launch_point[0]],
            [lch.position[1], proj.launch_point[1]], color=C["launch"], lw=1.0,
            ls=":", alpha=0.7)
    ax.scatter(*lch.position[:2], color="#4527a0", s=110, marker="s", zorder=9,
               label="敌方发射场")
    ax.scatter(*proj.launch_point[:2], color=C["launch"], s=80, marker="^",
               zorder=9, label="弹丸发射点")
    ax.scatter(*drn.position[:2], color=C["drone"], s=80, marker="o",
               zorder=9, label="无人机起点")
    arrow2d(ax, drn.position[:2], (drn.position[0] + drn.velocity[0] * 1.6,
                                   drn.position[1] + drn.velocity[1] * 1.6),
            C["drone"])
    arrow2d(ax, proj.launch_point[:2],
            (proj.launch_point[0] + lv[0] * 0.8, proj.launch_point[1] + lv[1] * 0.8),
            C["proj"])
    if tev:
        ax.scatter(tev["p_at"][0], tev["p_at"][1], color=C["ring"], s=90,
                   marker="X", zorder=10,
                   label=f"若不拦截：最近距塔 {tev['d_min']:.2f} m")
    ax.set_aspect("equal")
    ax.set_xlim(-25, 17); ax.set_ylim(-14, 9)
    ax.set_xlabel("x 东 (m)"); ax.set_ylabel("y 北 (m)")
    ax.set_title("(b) 俯视 XY（虚线=塔保护圈；点线=安全盒；灰=walls.sdf 墙体）",
                 fontsize=10)
    ax.grid(alpha=0.25)
    ax.legend(fontsize=7.5, loc="lower left", framealpha=0.9)

    # ---------- (c) 侧视 ----------
    ax = fig.add_subplot(1, 3, 3)
    for c, s in fld.obstacles_above_ground():
        ax.add_patch(Rectangle((c[0] - s[0] / 2, c[2] - s[2] / 2), s[0], s[2],
                               facecolor=C["wall"], edgecolor="#616161",
                               lw=0.6, alpha=0.4, zorder=1))
    ax.add_patch(Rectangle((tower.center[0] - tower.radius, tower.center[2]),
                           tower.radius * 2, tower.height,
                           facecolor=C["tower"], edgecolor="black", zorder=5))
    ax.plot(nom_pts[:, 0], nom_pts[:, 2], color=C["proj"], lw=2.0, ls="--",
            label="弹丸标称弹道")
    ax.plot([lch.position[0], proj.launch_point[0]],
            [lch.position[2], proj.launch_point[2]], color=C["launch"], lw=1.0,
            ls=":", alpha=0.7)
    ax.scatter(lch.position[0], lch.position[2], color="#4527a0", s=110,
               marker="s", zorder=9, label="敌方发射场")
    ax.scatter(proj.launch_point[0], proj.launch_point[2], color=C["launch"],
               s=80, marker="^", zorder=9, label="弹丸发射点")
    ax.scatter(drn.position[0], drn.position[2], color=C["drone"], s=80,
               marker="o", zorder=9, label="无人机起点")
    ax.axhline(0, color="#424242", lw=1.2)
    ax.set_xlim(-25, 17); ax.set_ylim(-1, 22)
    ax.set_xlabel("x 东 (m)"); ax.set_ylabel("z 上 (m)")
    ax.set_title("(c) 侧视 XZ（高度剖面）", fontsize=10)
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8, loc="upper right")

    fig.tight_layout(rect=(0, 0, 1, 0.94))
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)
    print(f"  ✓ {out_path}")


# ==========================================================================
#  四、图 2：拦截全过程
# ==========================================================================
def fig_intercept(scene, cases, nom_pts, tev, out_path):
    """cases: [(law, res, traj_dict), ...]"""
    tower, fld = scene.tower, scene.field_
    fig = plt.figure(figsize=(19, 10.5))
    fig.suptitle(f"拦截全过程 · {scene.name}   （真 C++ 制导核 + 真 DART 弹丸模型）",
                 fontsize=15, fontweight="bold")
    lim = ((-18, 16), (-13, 8), (0, 22))

    law0, res0, tr0 = cases[0]

    # ---------- (a) 3D ----------
    ax = fig.add_subplot(2, 3, 1, projection="3d")
    draw_field_3d(ax, fld)
    draw_cylinder_3d(ax, tower.center, tower.radius, tower.height, C["tower"])
    ax.plot(nom_pts[:, 0], nom_pts[:, 1], nom_pts[:, 2], color=C["proj_nom"],
            lw=1.6, ls=":", label="弹丸标称弹道（不拦截）")
    for law, res, tr in cases:
        ax.plot(tr["pMx"], tr["pMy"], tr["pMz"], color=LAW_COLORS.get(law, "k"),
                lw=2.0, label=f"无人机 {LAW_NAMES.get(law, law)}")
    ax.plot(tr0["pTx"], tr0["pTy"], tr0["pTz"], color=C["proj"], lw=2.0,
            ls="--", label="弹丸（被拦截）")
    if res0["p_M_at_min"]:
        ax.scatter(*res0["p_M_at_min"], color=C["hit"], s=130, marker="*",
                   zorder=12, depthshade=False, label="最近点/拦截点")
    set_axes_equal_3d(ax, lim)
    ax.set_xlabel("x 东 (m)"); ax.set_ylabel("y 北 (m)"); ax.set_zlabel("z 上 (m)")
    ax.set_title("(a) 3D 透视", fontsize=11)
    ax.legend(fontsize=7.5, loc="upper left")
    ax.view_init(elev=22, azim=-62)

    # ---------- (b) 俯视 ----------
    ax = fig.add_subplot(2, 3, 2)
    draw_field_2d(ax, fld)
    draw_tower_2d(ax, tower)
    ax.plot(nom_pts[:, 0], nom_pts[:, 1], color=C["proj_nom"], lw=1.4, ls=":")
    for law, res, tr in cases:
        ax.plot(tr["pMx"], tr["pMy"], color=LAW_COLORS.get(law, "k"), lw=2.0,
                label=f"无人机 {LAW_NAMES.get(law, law)}（脱靶 {res['min_R']:.3f} m）")
    ax.plot(tr0["pTx"], tr0["pTy"], color=C["proj"], lw=2.0, ls="--", label="弹丸")
    if res0["p_M_at_min"]:
        ax.scatter(res0["p_M_at_min"][0], res0["p_M_at_min"][1], color=C["hit"],
                   s=140, marker="*", zorder=12, edgecolor="k", lw=0.5)
    ax.set_aspect("equal")
    ax.set_xlim(-19, 17); ax.set_ylim(-14, 9)
    ax.set_xlabel("x 东 (m)"); ax.set_ylabel("y 北 (m)")
    ax.set_title("(b) 俯视 XY", fontsize=11)
    ax.grid(alpha=0.25)
    ax.legend(fontsize=7.5, loc="lower right")
    if res0["p_M_at_min"]:
        add_zoom(ax, cases, tower, res0["p_M_at_min"], 4.0, "xy")

    # ---------- (c) 侧视 ----------
    ax = fig.add_subplot(2, 3, 3)
    for c, s in fld.obstacles_above_ground():
        ax.add_patch(Rectangle((c[0] - s[0] / 2, c[2] - s[2] / 2), s[0], s[2],
                               facecolor=C["wall"], edgecolor="#616161",
                               lw=0.6, alpha=0.4, zorder=1))
    ax.add_patch(Rectangle((tower.center[0] - tower.radius, tower.center[2]),
                           tower.radius * 2, tower.height,
                           facecolor=C["tower"], edgecolor="black", zorder=5))
    ax.plot(nom_pts[:, 0], nom_pts[:, 2], color=C["proj_nom"], lw=1.4, ls=":")
    for law, res, tr in cases:
        ax.plot(tr["pMx"], tr["pMz"], color=LAW_COLORS.get(law, "k"), lw=2.0,
                label=f"无人机 {LAW_NAMES.get(law, law)}")
    ax.plot(tr0["pTx"], tr0["pTz"], color=C["proj"], lw=2.0, ls="--", label="弹丸")
    if res0["p_M_at_min"]:
        ax.scatter(res0["p_M_at_min"][0], res0["p_M_at_min"][2], color=C["hit"],
                   s=140, marker="*", zorder=12, edgecolor="k", lw=0.5)
    ax.axhline(0, color="#424242", lw=1.2)
    ax.set_xlim(-19, 17); ax.set_ylim(-1, 22)
    ax.set_xlabel("x 东 (m)"); ax.set_ylabel("z 上 (m)")
    ax.set_title("(c) 侧视 XZ", fontsize=11)
    ax.grid(alpha=0.25)
    ax.legend(fontsize=7.5, loc="upper right")
    if res0["p_M_at_min"]:
        add_zoom(ax, cases, tower, res0["p_M_at_min"], 4.0, "xz")

    # ---------- (d) R(t) ----------
    ax = fig.add_subplot(2, 3, 4)
    for law, res, tr in cases:
        ax.plot(tr["t"], tr["R_true"], color=LAW_COLORS.get(law, "k"), lw=2.0,
                label=f"{LAW_NAMES.get(law, law)}（最近 {res['min_R']:.3f} m）")
    ax.axhline(0.5, color="#616161", ls="--", lw=1.0, label="命中球 0.5 m")
    ax.set_yscale("log")
    ax.set_xlabel("t (s)"); ax.set_ylabel("|R| 相对距离 (m, 对数)")
    ax.set_title("(d) 交会距离 R(t)", fontsize=11)
    ax.grid(alpha=0.25, which="both")
    ax.legend(fontsize=8)

    # ---------- (e) 指令加速度 ----------
    ax = fig.add_subplot(2, 3, 5)
    for law, res, tr in cases:
        ax.plot(tr["t"], tr["a_norm"], color=LAW_COLORS.get(law, "k"), lw=1.8,
                label=f"{LAW_NAMES.get(law, law)}（峰值 {res['peak_a_cmd']:.1f}）")
    ax.axhline(scene.drone.a_max, color="#c62828", ls="--", lw=1.0,
               label=f"a_max = {scene.drone.a_max:.0f} m/s²")
    ax.set_xlabel("t (s)"); ax.set_ylabel("|a_cmd| (m/s²)")
    ax.set_title("(e) 制导指令加速度", fontsize=11)
    ax.grid(alpha=0.25)
    ax.legend(fontsize=8)

    # ---------- (f) 速度 & t_go ----------
    ax = fig.add_subplot(2, 3, 6)
    for law, res, tr in cases:
        spd = np.sqrt(tr["vMx"] ** 2 + tr["vMy"] ** 2 + tr["vMz"] ** 2)
        ax.plot(tr["t"], spd, color=LAW_COLORS.get(law, "k"), lw=1.8,
                label=f"|v_M| {LAW_NAMES.get(law, law)}")
    ax.set_xlabel("t (s)"); ax.set_ylabel("|v_M| (m/s)", color="#455a64")
    ax.grid(alpha=0.25)
    ax2 = ax.twinx()
    ax2.plot(tr0["t"], tr0["t_go"], color="#6a1b9a", lw=1.4, ls="--",
             label=f"t_go（{LAW_NAMES.get(law0, law0)}）")
    ax2.set_ylabel("t_go (s)", color="#6a1b9a", labelpad=4)
    tg = np.asarray(tr0["t_go"], dtype=float)
    tg_max = float(np.nanmax(tg)) if np.any(np.isfinite(tg)) else 2.0
    ax2.set_ylim(0, max(1.0, tg_max * 1.25))
    h1, l1 = ax.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    ax.legend(h1 + h2, l1 + l2, fontsize=7.5, loc="lower left", framealpha=0.9)
    ax.set_title("(f) 无人机速度 & 剩余时间 t_go", fontsize=11)

    fig.tight_layout(rect=(0, 0, 1, 0.95))
    fig.savefig(out_path, bbox_inches="tight")
    plt.close(fig)
    print(f"  ✓ {out_path}")


# ==========================================================================
#  五、动画（可选）
# ==========================================================================
def fig_anim(scene, law, res, tr, nom_pts, out_path, fps=20):
    try:
        from matplotlib.animation import FuncAnimation, PillowWriter
    except Exception:
        print("  ! 无 matplotlib.animation，跳过动画")
        return
    tower, fld = scene.tower, scene.field_
    n = len(tr["t"])
    step = max(1, int(round((1.0 / fps) / (tr["t"][1] - tr["t"][0]))))

    fig = plt.figure(figsize=(13.5, 6.2))
    # 左：俯视；右：侧视
    ax1 = fig.add_subplot(1, 2, 1)
    ax2 = fig.add_subplot(1, 2, 2)

    def base(ax, top):
        draw_field_2d(ax, fld)
        draw_tower_2d(ax, tower)
        ax.grid(alpha=0.25)
        if top:
            ax.set_aspect("equal")
            ax.set_xlim(-19, 17); ax.set_ylim(-14, 9)
            ax.set_xlabel("x 东 (m)"); ax.set_ylabel("y 北 (m)")
            ax.set_title("俯视 XY")
        else:
            ax.set_xlim(-19, 17); ax.set_ylim(-1, 22)
            ax.set_xlabel("x 东 (m)"); ax.set_ylabel("z 上 (m)")
            ax.set_title("侧视 XZ")

    base(ax1, True)
    base(ax2, False)
    ax2.axhline(0, color="#424242", lw=1.0)

    l1, = ax1.plot([], [], color=C["drone"], lw=1.6, alpha=0.6)
    p1, = ax1.plot([], [], "o", color=C["drone"], ms=7)
    l2, = ax1.plot([], [], color=C["proj"], lw=1.6, ls="--", alpha=0.6)
    p2, = ax1.plot([], [], "^", color=C["proj"], ms=7)
    ax1.plot(tr["pMx"], tr["pMy"], color=C["drone"], lw=0.6, alpha=0.25)
    ax1.plot(tr["pTx"], tr["pTy"], color=C["proj"], lw=0.6, ls="--", alpha=0.25)

    l3, = ax2.plot([], [], color=C["drone"], lw=1.6, alpha=0.6)
    p3, = ax2.plot([], [], "o", color=C["drone"], ms=7)
    l4, = ax2.plot([], [], color=C["proj"], lw=1.6, ls="--", alpha=0.6)
    p4, = ax2.plot([], [], "^", color=C["proj"], ms=7)
    ax2.plot(tr["pMx"], tr["pMz"], color=C["drone"], lw=0.6, alpha=0.25)
    ax2.plot(tr["pTx"], tr["pTz"], color=C["proj"], lw=0.6, ls="--", alpha=0.25)

    txt = ax1.text(0.02, 0.97, "", transform=ax1.transAxes, va="top",
                   fontsize=9, family="monospace",
                   bbox=dict(fc="white", ec="#bdbdbd", alpha=0.85))

    def upd(i):
        l1.set_data(tr["pMx"][:i], tr["pMy"][:i]); p1.set_data([tr["pMx"][i]], [tr["pMy"][i]])
        l2.set_data(tr["pTx"][:i], tr["pTy"][:i]); p2.set_data([tr["pTx"][i]], [tr["pTy"][i]])
        l3.set_data(tr["pMx"][:i], tr["pMz"][:i]); p3.set_data([tr["pMx"][i]], [tr["pMz"][i]])
        l4.set_data(tr["pTx"][:i], tr["pTz"][:i]); p4.set_data([tr["pTx"][i]], [tr["pTz"][i]])
        txt.set_text(f"t = {tr['t'][i]:5.2f} s\n"
                     f"R = {tr['R_true'][i]:5.2f} m\n"
                     f"|v_M| = {math.hypot(tr['vMx'][i], tr['vMy'][i], tr['vMz'][i]):4.2f} m/s\n"
                     f"|a_cmd| = {tr['a_norm'][i]:5.2f} m/s²")
        return l1, p1, l2, p2, l3, p3, l4, p4, txt

    fig.suptitle(f"拦截过程动画 · {LAW_NAMES.get(law, law)}"
                 f"（脱靶 {res['min_R']:.3f} m）", fontsize=13, fontweight="bold")
    frames = list(range(0, n, step))
    ani = FuncAnimation(fig, upd, frames=frames, interval=1000 / fps, blit=False)
    ani.save(out_path, writer=PillowWriter(fps=fps))
    plt.close(fig)
    print(f"  ✓ {out_path}")


# ==========================================================================
def build_parser():
    ap = argparse.ArgumentParser(
        description="场景复现 + 拦截过程可视化（Python 绘图 / C++ 核心）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--scene", help="从场景 JSON 加载")
    ap.add_argument("--preset", default="pilot", choices=sorted(SM.PRESETS),
                    help="内置预设场景")
    ap.add_argument("--laws", default="2",
                    help="制导律：0=PPN 1=TPN 2=APN，可逗号分隔对比，如 0,2")
    ap.add_argument("--out", default=".", help="输出目录")
    ap.add_argument("--anim", action="store_true", help="额外产出 GIF 动画")
    ap.add_argument("--fps", type=float, default=10.0,
                    help="动画帧率（交会仅 ~0.5 s，10 fps 播放约 1.1 s，更易看清）")
    return ap


def main():
    ap = build_parser()
    a = ap.parse_args()

    scene = SM.Scene.load(a.scene) if a.scene else SM.PRESETS[a.preset]()
    laws = [int(x) for x in a.laws.replace(" ", "").split(",") if x != ""]
    os.makedirs(a.out, exist_ok=True)

    print("=" * 78)
    print("场景复现 + 可视化（Python 绘图；制导/弹丸算法 = 真 C++ 核）")
    print("=" * 78)
    print(scene.describe())
    print(f"  制导律: {', '.join(LAW_NAMES.get(l, str(l)) for l in laws)}")
    print("-" * 78)

    L = open_core()

    # 标称弹道 + 塔威胁评估
    sc0 = scene.to_sim_dict()
    nom_t, nom_pts = nominal_projectile(L, sc0)
    h = configure_core(L, sc0)
    tev = SIM.evaluate_tower(L, h, scene.tower)
    L.pgu_destroy(h)
    if tev:
        mark = "⚠ 构成威胁" if tev["threatened"] else "✓ 不构成威胁"
        print(f"  [若不拦截] 弹丸最近距塔 {tev['d_min']:.2f} m"
              f"（t={tev['t_at']:.2f}s @ {v3txt(tev['p_at'])}）→ {mark}")

    # 逐条制导律跑仿真
    cases = []
    for law in laws:
        csv_path = os.path.join(a.out, f"traj_law{law}.csv")
        res, _ = run_case(L, scene, law, csv_path)
        tr = read_traj(csv_path)
        cases.append((law, res, tr))
        print(f"  [{LAW_NAMES.get(law, law)}] 最近距离 {res['min_R']:.3f} m"
              f"  {'命中' if res['hit'] else '脱靶'}"
              f"  峰值指令 {res['peak_a_cmd']:.2f} m/s²"
              f"  航程 {res['path_len']:.2f} m")

    print("-" * 78)
    fig_layout(scene, nom_pts, nom_t, tev,
               os.path.join(a.out, "viz_scene_layout.png"))
    fig_intercept(scene, cases, nom_pts, tev,
                  os.path.join(a.out, "viz_intercept.png"))
    if a.anim:
        law0, res0, tr0 = cases[0]
        fig_anim(scene, law0, res0, tr0, nom_pts,
                 os.path.join(a.out, "viz_intercept_anim.gif"), fps=a.fps)
    print("=" * 78)
    print("完成。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
