#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线单测：固定装甲板几何 + 撞板判据 + 场景序列化（不需要 ROS / Gazebo）。

运行：  python test_armor_plate.py        （Windows / WSL 均可，纯 stdlib）
"""
from __future__ import annotations

import math
import os
import sys

# Windows 控制台默认 GBK：直接把 emoji / 特殊符号打出去会 UnicodeEncodeError。
# 统一转 UTF-8 + 容错替换 ⇒ 结果文件用 UTF-8 读即可（控制台看到乱码不影响判定）。
try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
except Exception:                                            # noqa: BLE001
    pass

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from armor_plate import ArmorPlate  # noqa: E402

PASS = 0
FAIL = 0
SEC = "?"


def sec(name):
    global SEC
    SEC = name
    print(f"\n=== {name} ===")


def ok(cond, msg):
    global PASS, FAIL
    if cond:
        PASS += 1
        print(f"  [OK]   {msg}")
    else:
        FAIL += 1
        print(f"  [FAIL] [{SEC}] {msg}")


def close(a, b, tol=1e-9):
    return abs(float(a) - float(b)) <= tol


# ==========================================================================
def main():
    # ---------------------------------------------------------------- 基向量
    sec("用例 1 · yaw=0 的基向量与半尺寸")
    pl = ArmorPlate(center=(0.0, 0.0, 3.0), width=0.6, height=0.6,
                    thickness=0.05, yaw_deg=0.0)
    n, u, v = pl.axes()
    ok(close(n[0], 1.0) and close(n[1], 0.0) and close(n[2], 0.0),
       f"yaw=0 ⇒ 法向 +x  ({n})")
    ok(close(u[0], 0.0) and close(u[1], 1.0), f"板面内水平 u = +y  ({u})")
    ok(close(v[2], 1.0), f"竖直 v = +z  ({v})")
    hu, hv, hn = pl.half_extents()
    ok(close(hu, 0.3) and close(hv, 0.3) and close(hn, 0.025),
       f"半尺寸 (hu,hv,hn)=({hu},{hv},{hn})")

    sec("用例 2 · 点到板盒的距离")
    ok(close(pl.distance_to_plate(pl.center), 0.0), "板中心点在盒内 ⇒ 0")
    ok(close(pl.distance_to_plate((0.0, 0.0, 3.02)), 0.0),
       "沿法向 0.02 m（< 半厚 0.025）仍在盒内 ⇒ 0")
    ok(close(pl.distance_to_plate((1.0, 0.0, 3.0)), 0.975),
       "法向前方 1 m ⇒ 1 − 0.025 = 0.975")
    ok(close(pl.distance_to_plate((0.0, 0.4, 3.0)), 0.1),
       "侧向偏 0.4 m（超过半宽 0.3）⇒ 0.1")
    ok(close(pl.distance_to_plate((0.0, 0.0, 3.4)), 0.1),
       "竖直偏 0.4 m（超过半高 0.3）⇒ 0.1")

    sec("用例 3 · 穿越判定（THROUGH）")
    a = (1.0, 0.0, 3.0)          # 法向 +x 一侧
    b = (-1.0, 0.0, 3.0)         # 另一侧
    cr = pl.crossing(a, b)
    ok(cr is not None, "从 +x 穿到 −x、过板心 ⇒ 判定穿越")
    ok(cr is not None and close(cr[0], 0.0) and close(cr[1], 0.0),
       f"穿点面内偏移 (u,v)=({cr[0]:.6f},{cr[1]:.6f}) ⇒ 板心")
    ok(cr is not None and close(cr[2], 0.5, 1e-9), f"段参数 s={cr[2]:.6f}（中点）")
    ok(pl.crossing((1.0, 0.0, 3.0), (1.5, 0.0, 3.0)) is None, "同侧移动 ⇒ 不穿越")

    sec("用例 4 · 穿越但落在板外（MISS）")
    a2 = (1.0, 0.6, 3.0)
    b2 = (-1.0, 0.6, 3.0)
    ok(pl.crossing(a2, b2) is None, "横向偏 0.6 m（> 半宽 0.3）⇒ 不穿越")
    vd = pl.verdict(a2, b2)
    ok(vd["kind"] == "MISS", f"verdict ⇒ MISS（实际 {vd['kind']}, dist={vd['dist']:.3f}）")

    sec("用例 5 · 机身半径外扩（body_radius）")
    a3 = (1.0, 0.45, 3.0)
    b3 = (-1.0, 0.45, 3.0)
    ok(pl.crossing(a3, b3) is None, "偏 0.45 m（> 0.3）不穿越")
    ok(pl.crossing(a3, b3, expand=0.2) is not None,
       "偏 0.45 m 但机身半径 0.2 外扩 ⇒ 算穿越（机身边缘擦到）")
    vd = pl.verdict(a3, b3, tol=0.15, body_radius=0.20)
    ok(vd["kind"] == "THROUGH", f"verdict ⇒ THROUGH（实际 {vd['kind']}）")

    sec("用例 6 · 贴到板面（HIT）与容差")
    hit_p = (0.125, 0.0, 3.0)     # 距板面 0.125−0.025 = 0.10 m
    vd = pl.verdict(None, hit_p, tol=0.15, body_radius=0.20)
    ok(vd["kind"] == "HIT", f"距板面 0.10 m，tol+body=0.35 ⇒ HIT（{vd['kind']}）")
    far_p = (0.60, 0.0, 3.0)      # 距板面 0.575 m
    vd = pl.verdict(None, far_p, tol=0.15, body_radius=0.20)
    ok(vd["kind"] == "MISS", f"距板面 0.575 m ⇒ MISS（{vd['kind']}）")

    sec("用例 7 · 旋转板（yaw=90°）")
    pl90 = ArmorPlate(center=(0.0, 0.0, 3.0), width=0.6, height=0.6,
                      thickness=0.05, yaw_deg=90.0)
    n90, u90, _ = pl90.axes()
    ok(close(n90[0], 0.0, 1e-12) and close(n90[1], 1.0),
       f"yaw=90 ⇒ 法向 +y（{n90[0]:.2e},{n90[1]:.2f}）")
    ok(close(pl90.distance_to_plate((1.0, 0.0, 3.0)), 0.7),
       "此时 +x 方向 1 m 落在板面内（宽方向）⇒ 1−0.3=0.7")
    ok(pl90.verdict((1.0, 0.0, 3.0), (-1.0, 0.0, 3.0))["kind"] == "MISS",
       "沿 +x 走不会穿过 yaw=90 的板 ⇒ MISS")

    sec("用例 7.5 · 俯仰板（pitch=30°，2026-10-19 新增）")
    pl30 = ArmorPlate(center=(0.0, 0.0, 3.0), width=0.6, height=0.6,
                      thickness=0.05, yaw_deg=0.0, pitch_deg=30.0)
    n30, u30, v30 = pl30.axes()
    ok(close(n30[0], math.cos(math.radians(30.0)))
       and close(n30[2], math.sin(math.radians(30.0))),
       f"pitch=30 ⇒ 法向抬高（n={tuple(round(x, 3) for x in n30)}）")
    ok(close(u30[0], 0.0) and close(u30[1], 1.0),
       f"板宽方向 u 仍是水平 +y（{tuple(round(x, 3) for x in u30)}）")
    ok(close(v30[1], 0.0) and abs(v30[2] - math.cos(math.radians(30.0))) < 1e-12,
       f"板高方向 v 随俯仰前倾（v={tuple(round(x, 3) for x in v30)}）")
    # 沿法向 1 m 的点：距板面 = 1 − 半厚
    p_n = tuple(pl30.center[k] + n30[k] for k in range(3))
    ok(close(pl30.distance_to_plate(p_n), 1.0 - 0.025),
       "沿法向 1 m ⇒ 距板面 0.975（俯仰后法向仍正确）")
    # 沿板高方向 2 m 的点：距板面 = 2 − 半高（pitch 不改变高度尺寸）
    p_v = tuple(pl30.center[k] + 2.0 * v30[k] for k in range(3))
    ok(close(pl30.distance_to_plate(p_v), 2.0 - 0.3),
       "沿板高方向 2 m ⇒ 1.7（俯仰不改板高尺寸）")
    # 沿新法向穿越（两端各 1 m）⇒ THROUGH，且板面内偏移为 0
    a30 = tuple(pl30.center[k] + n30[k] for k in range(3))
    b30 = tuple(pl30.center[k] - n30[k] for k in range(3))
    vd = pl30.verdict(a30, b30, tol=0.15, body_radius=0.20)
    ok(vd["kind"] == "THROUGH",
       f"沿俯仰后的法向穿越 ⇒ THROUGH（{vd['kind']}）")
    ok(close(float(vd["du"]), 0.0, 1e-6) and close(float(vd["dv"]), 0.0, 1e-6),
       f"穿点板面残差 ≈0（du={vd['du']:.2e}, dv={vd['dv']:.2e}）")
    # pitch=0 必须与旧实现逐位一致（后向兼容）
    pl_flat = ArmorPlate(center=(0.0, 0.0, 3.0), pitch_deg=0.0)
    nf, uf, vf = pl_flat.axes()
    ok(nf == (1.0, 0.0, 0.0) and uf == (-0.0, 1.0, 0.0) and vf == (0.0, 0.0, 1.0),
       f"pitch=0 的基向量与旧实现一致（n={nf}, u={uf}, v={vf}）")
    # 俯仰 90° ⇒ 板面水平（法向朝天），竖向点落在法向上
    pl_up = ArmorPlate(center=(0.0, 0.0, 3.0), pitch_deg=90.0)
    nu, _, vu = pl_up.axes()
    ok(close(nu[2], 1.0) and close(abs(vu[2]), 0.0),
       f"pitch=90 ⇒ 法向朝天、板面水平（n={tuple(round(x, 3) for x in nu)}）")

    sec("用例 8 · 序列化（to_dict / from_dict 往返）")
    d = pl30.to_dict()
    pl2 = ArmorPlate.from_dict(d)
    ok(pl2.center == pl30.center and close(pl2.width, pl30.width)
       and close(pl2.yaw_deg, pl30.yaw_deg)
       and close(pl2.pitch_deg, pl30.pitch_deg), "to_dict → from_dict 往返一致（含 pitch）")
    ok("armor_plate" in ArmorPlate.from_dict({}).name,
       "空 dict ⇒ 用默认名 armor_plate")
    ok(close(ArmorPlate.from_dict({}).pitch_deg, 0.0), "空 dict ⇒ pitch 默认 0（后向兼容）")

    sec("用例 9 · scene_model 集成（Scene 带上装甲板，JSON 往返）")
    try:
        import scene_model
        sc = scene_model.Scene(name="plate-test")
        sc.armor_plate = ArmorPlate(center=(-14.0, 0.0, 3.0))
        d = sc.to_dict()
        ok("armor_plate" in d
           and tuple(d["armor_plate"]["center"]) == (-14.0, 0.0, 3.0),
           "Scene.to_dict() 含 armor_plate 字段")
        sc2 = scene_model.Scene.from_dict(d)
        ok(sc2.armor_plate is not None
           and sc2.armor_plate.center == (-14.0, 0.0, 3.0),
           "Scene.from_dict() 还原装甲板")
        ok("装甲板" in sc.describe(), "Scene.describe() 打印装甲板摘要")
        sc3 = scene_model.Scene()
        ok(sc3.armor_plate is None, "默认（不加板）⇒ armor_plate=None，旧行为不变")
    except Exception as e:                                   # noqa: BLE001
        ok(False, f"scene_model 集成失败：{e}")

    sec("用例 10 · Gazebo 姿态四元数与几何同源（plate_orientation）")
    try:
        import gz_scene_sync as GZ
        q0 = GZ.plate_orientation(0.0, 0.0)
        ok(all(close(q0[i], v) for i, v in enumerate((0.0, 0.0, 0.0, 1.0))),
           f"yaw=0,pitch=0 ⇒ 单位四元数（{tuple(round(x, 6) for x in q0)}）")
        # pitch=90°：板面水平、法向朝天 ⇒ 绕 −y 转 90°：q=(0,−sin45,0,cos45)
        q90 = GZ.plate_orientation(0.0, math.radians(90.0))
        s45 = math.sin(math.radians(45.0))
        ok(close(q90[0], 0.0) and close(q90[1], -s45, 1e-12)
           and close(q90[2], 0.0) and close(q90[3], s45, 1e-12),
           f"pitch=90 ⇒ (0,−0.7071,0,0.7071)（{tuple(round(x, 4) for x in q90)}）")
        # 与 ArmorPlate.axes() 的局部轴一致性：用四元数把局部 x/y/z 转到世界，应等于 (n,u,v)
        for yaw_d, pitch_d in ((0.0, 0.0), (60.0, 0.0), (0.0, 30.0), (37.0, -20.0)):
            qx, qy, qz, qw = GZ.plate_orientation(math.radians(yaw_d),
                                                  math.radians(pitch_d))
            # 四元数 → 旋转矩阵
            R = (
                (1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)),
                (2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)),
                (2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)),
            )
            n_w = tuple(R[i][0] for i in range(3))
            u_w = tuple(R[i][1] for i in range(3))
            v_w = tuple(R[i][2] for i in range(3))
            plx = ArmorPlate(yaw_deg=yaw_d, pitch_deg=pitch_d)
            n_a, u_a, v_a = plx.axes()
            ok(all(close(n_w[i], n_a[i], 1e-9) for i in range(3))
               and all(close(u_w[i], u_a[i], 1e-9) for i in range(3))
               and all(close(v_w[i], v_a[i], 1e-9) for i in range(3)),
               f"yaw={yaw_d:g},pitch={pitch_d:g} ⇒ 四元数转出的 (n,u,v) 与 ArmorPlate.axes() 一致")
    except Exception as e:                                   # noqa: BLE001
        ok(False, f"gz_scene_sync 导入/校验失败：{e}")

    print(f"\n{'=' * 60}")
    print(f"结果：{PASS} PASS / {FAIL} FAIL")
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
