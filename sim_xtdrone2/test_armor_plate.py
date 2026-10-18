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

    sec("用例 8 · 序列化（to_dict / from_dict 往返）")
    d = pl.to_dict()
    pl2 = ArmorPlate.from_dict(d)
    ok(pl2.center == pl.center and close(pl2.width, pl.width)
       and close(pl2.yaw_deg, pl.yaw_deg), "to_dict → from_dict 往返一致")
    ok("armor_plate" in ArmorPlate.from_dict({}).name,
       "空 dict ⇒ 用默认名 armor_plate")

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

    print(f"\n{'=' * 60}")
    print(f"结果：{PASS} PASS / {FAIL} FAIL")
    return 0 if FAIL == 0 else 1


if __name__ == "__main__":
    sys.exit(main())
