#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""守备拦截 · **固定装甲板**（撞板目标）——纯几何，零依赖

【为什么需要这个模块】（2026-10-18 任务变更）
  目标从「拦截飞行中的弹丸」改成「撞击敌方基地/前哨站上部的**固定装甲板**」。
  与"点目标 + 球形命中判据"相比，板目标有两个本质区别：
    ① 目标是**面**不是点 ⇒ 命中判据要看"板面内两维偏差"，不能只看三维距离；
    ② 目标有**厚度和朝向** ⇒ 必须判"是否穿过板面、且穿点落在矩形内"。

【规则依据 / 假设分级】（★ 不要把假设当手册值）
  · 装甲板/M 板的存在与"可移动靶 + 固定撞击板"两板结构 —— **用户口述 + 社区 CAD 导出**；
  · 可动靶行程 ±280 mm、600 ms 内换位 —— 规则手册 V1.0.0 §5.7.3 + CAD 双证（**可信**）；
  · **固定撞击板的尺寸/安装高度、空中机器人撞击的判据** —— 规则表 5-1 写的是「待定」
    ⇒ 本模块里的板尺寸默认值（0.60×0.60×0.05 m）与命中容差（0.15 m）都是**我们的假设**。

【坐标与朝向约定】
  · 一律 ENU（x=东 y=北 z=上），单位 m。
  · `yaw_deg` = **板面法向的方位角**（从 +x 逆时针，deg）：
      yaw=0   ⇒ 法向 +x，板面横跨 y 方向、竖直沿 z（正对从 +x 方向飞来的无人机）；
      yaw=90  ⇒ 法向 +y，板面横跨 x 方向。
  · 板面内坐标 (u, v)：u = 水平方向（右手：u = n × z），v = 竖直向上。
  · 板盒半尺寸：hu = width/2（沿 u）、hv = height/2（沿 v）、hn = thickness/2（沿 n）。

【与制导解耦】
  本模块**不含任何算法**，只回答"这个位置算不算撞上了"。
  制导/决策自己决定往哪飞；这里只做几何判定 ⇒ 可离线单测，也能给决策层（T2）复用。
"""
from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Dict, Optional, Sequence, Tuple

Vec3 = Tuple[float, float, float]


def _dot(a: Sequence[float], b: Sequence[float]) -> float:
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def _sub(a: Sequence[float], b: Sequence[float]) -> Vec3:
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


# ==========================================================================
@dataclass
class ArmorPlate:
    """固定装甲板：一块竖直薄板（静态、不可移动）。

    字段
    ----
    center    : 板**几何中心** ENU (m)
    width     : 板宽（水平，沿 u）(m)
    height    : 板高（竖直，沿 z）(m)
    thickness : 板厚（沿法向 n）(m)
    yaw_deg   : 板面**法向**的方位角 (deg)，0 ⇒ 法向 +x
    name      : 名字（日志/Gazebo 实体名用）
    """

    name: str = "armor_plate"
    center: Vec3 = (-14.0, 0.0, 5.0)
    width: float = 0.60
    height: float = 0.60
    thickness: float = 0.05
    yaw_deg: float = 0.0

    # ------------------------------------------------------------ 基向量
    def axes(self) -> Tuple[Vec3, Vec3, Vec3]:
        """返回 (n, u, v)：法向 / 板面内水平 / 竖直。"""
        a = math.radians(self.yaw_deg)
        n = (math.cos(a), math.sin(a), 0.0)
        u = (-math.sin(a), math.cos(a), 0.0)     # = n × z
        v = (0.0, 0.0, 1.0)
        return n, u, v

    def half_extents(self) -> Tuple[float, float, float]:
        """(hu, hv, hn) 半尺寸。"""
        return (self.width * 0.5, self.height * 0.5, self.thickness * 0.5)

    # ------------------------------------------------------------ 坐标变换
    def to_local(self, p: Sequence[float]) -> Tuple[float, float, float]:
        """世界点 → 板局部坐标 (u, v, n)。n>0 表示在板法向一侧。"""
        n_ax, u_ax, v_ax = self.axes()
        d = _sub(p, self.center)
        return (_dot(d, u_ax), _dot(d, v_ax), _dot(d, n_ax))

    # ------------------------------------------------------------ 几何量
    def distance_to_plate(self, p: Sequence[float]) -> float:
        """点到**板盒（薄长方体）表面**的最近距离；点在盒内返回 0。"""
        hu, hv, hn = self.half_extents()
        du, dv, dn = self.to_local(p)
        eu = max(abs(du) - hu, 0.0)
        ev = max(abs(dv) - hv, 0.0)
        en = max(abs(dn) - hn, 0.0)
        return math.sqrt(eu * eu + ev * ev + en * en)

    def inside_face(self, p: Sequence[float], expand: float = 0.0) -> bool:
        """点在板面上的投影是否落在矩形内（可按 `expand` 外扩，用于机身半径）。"""
        hu, hv, _ = self.half_extents()
        du, dv, _ = self.to_local(p)
        return abs(du) <= hu + expand and abs(dv) <= hv + expand

    def crossing(self, p_prev: Sequence[float], p: Sequence[float],
                 expand: float = 0.0) -> Optional[Tuple[float, float, float]]:
        """线段 p_prev→p 是否**穿过板面平面**，且穿点在矩形内。

        返回 None（没穿）或 `(u, v, s)`：穿点在板面内的两维偏移 + 段参数 s∈[0,1]。
        `expand` 把矩形外扩（无人机有体积，机身边缘擦到也算）。
        """
        _, _, n_prev = self.to_local(p_prev)
        _, _, n_cur = self.to_local(p)
        # ★ 数值稳健性（yu 例 7 实测踩到）：板恰好转 90°、无人机恰好沿**板面方向**飞时，
        #   n 的理论值恒为 0，但浮点残差是 ±6e-17 ⇒ 两端**符号相反** ⇒ 会被误判成"穿越"。
        #   ⇒ 只要有一端落在板面平面附近（|n| < eps），就不算"穿越"；
        #     那种情况本来就会被 `distance_to_plate` 的"贴板容差"覆盖，不会漏判。
        _eps = 1.0e-9
        if abs(n_prev) < _eps or abs(n_cur) < _eps:
            return None
        if n_prev == n_cur:
            return None
        if (n_prev > 0.0) == (n_cur > 0.0):
            return None                        # 同侧，没穿过
        s = n_prev / (n_prev - n_cur)
        if not (0.0 <= s <= 1.0):
            return None
        q = tuple(p_prev[k] + s * (p[k] - p_prev[k]) for k in range(3))
        if not self.inside_face(q, expand=expand):
            return None
        du, dv, _ = self.to_local(q)
        return (du, dv, s)

    def verdict(self, p_prev: Optional[Sequence[float]], p: Sequence[float],
                tol: float = 0.15, body_radius: float = 0.20) -> Dict[str, object]:
        """撞板判定。

        优先级：**穿过板面**（最硬的证据）> **贴到板面**（距离在容差内）> 未命中。

        参数
        ----
        p_prev      : 上一拍位置（None ⇒ 只做距离判定）
        p           : 当前拍位置
        tol         : 板面距离容差 (m) —— 我们自定的判据（规则表 5-1 为「待定」）
        body_radius : 机体等效半径 (m)，用于把矩形与板面外扩（机身 450 mm ⇒ ~0.20~0.22）

        返回 dict：kind ∈ {"THROUGH","HIT","MISS"}、dist、du、dv、cross、tol
        """
        out: Dict[str, object] = {
            "kind": "MISS", "dist": float("inf"), "du": float("nan"),
            "dv": float("nan"), "cross": None, "tol": tol,
        }
        if p_prev is not None:
            cr = self.crossing(p_prev, p, expand=body_radius)
            if cr is not None:
                out.update(kind="THROUGH", dist=0.0, du=cr[0], dv=cr[1], cross=cr)
                return out
        d = self.distance_to_plate(p)
        du, dv, _ = self.to_local(p)
        out.update(dist=d, du=du, dv=dv)
        if d <= tol + body_radius:
            out["kind"] = "HIT"
        return out

    # ------------------------------------------------------------ 摘要
    def describe(self) -> str:
        n_ax, _, _ = self.axes()
        return (f"固定装甲板「{self.name}」中心 ENU=({self.center[0]:.2f},"
                f"{self.center[1]:.2f},{self.center[2]:.2f})  "
                f"宽×高×厚 = {self.width:.2f}×{self.height:.2f}×{self.thickness:.3f} m  "
                f"法向=({n_ax[0]:+.2f},{n_ax[1]:+.2f},0)  yaw={self.yaw_deg:+.1f}°")

    def to_dict(self) -> dict:
        return {
            "name": self.name,
            "center": list(self.center),
            "width": self.width,
            "height": self.height,
            "thickness": self.thickness,
            "yaw_deg": self.yaw_deg,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "ArmorPlate":
        return cls(
            name=str(d.get("name", "armor_plate")),
            center=tuple(float(x) for x in d.get("center", (-14.0, 0.0, 3.0))),
            width=float(d.get("width", 0.60)),
            height=float(d.get("height", 0.60)),
            thickness=float(d.get("thickness", 0.05)),
            yaw_deg=float(d.get("yaw_deg", 0.0)),
        )


# ==========================================================================
if __name__ == "__main__":
    plate = ArmorPlate()
    print(plate.describe())
    print("中心点的距离 =", plate.distance_to_plate(plate.center))
    print("法向前方 1 m 的距离 =", round(plate.distance_to_plate(
        (plate.center[0] + 1.0, plate.center[1], plate.center[2])), 4))
    print("穿越判定 =", plate.verdict(
        (plate.center[0] + 1.0, plate.center[1], plate.center[2]),
        (plate.center[0] - 1.0, plate.center[1], plate.center[2])))
