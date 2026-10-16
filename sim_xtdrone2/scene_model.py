#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""守备拦截 · 场景实体模型（塔 / 发射场 / 弹丸 / 无人机 / 场地）

对应《守备拦截系统规划.md》任务包 **T1（场景与实体建模）** 的模型部分。

【设计原则】
  · 本模块只做 **几何 + 数据**，不含任何算法（决策 / 制导都不在这里）。
    —— 将来决策层（威胁判定 T2）消费它，制导仿真也消费它。
  · 纯 stdlib，零依赖（可以在 Windows / WSL / 真机任何地方 import）。
  · 坐标一律 **ENU**（与 `guidance_libs` 一致）：x = 东，y = 北，z = 上。
  · 全部字段可序列化（JSON），便于把"一次对抗"存成一个场景文件。

【实体清单】
  Tower          防御塔（被保护目标）
  Launcher       敌方发射场
  ProjectileSpec 弹丸/飞镖（对接 TargetModel 的配置参数）
  DroneSpec      我方无人机（巡航态初值 + 机动边界）
  Field          场地（安全飞行盒 + 障碍）
  Scene          以上五者的容器 + 预设场景

用法：
    from scene_model import Scene, preset_pilot
    s = preset_pilot()
    print(s.describe())
    s.save("scene.json")
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, field, asdict
from typing import List, Optional, Sequence, Tuple

Vec3 = Tuple[float, float, float]


# ==========================================================================
#  一、防御塔（被保护目标）
# ==========================================================================
@dataclass
class Tower:
    """防御塔 —— 建模为**实心圆柱**（底面圆 + 高度）。

    对应 RM 里需要保护的基地 / 前哨站。
    `safety_margin` 是"富裕度"：判定威胁时额外加的余量（对应原则
    "宁可多飞一些，也绝对不要让弹丸撞上防御塔"）。
    """

    name: str = "tower"
    center: Vec3 = (0.0, 0.0, 0.0)      # **底面圆心**（ENU；z 一般为 0）
    radius: float = 1.2                  # 塔身水平半径 (m)
    height: float = 3.0                  # 塔高 (m)
    safety_margin: float = 1.0           # 安全垫 (m)：判定威胁的额外余量

    # ---------------- 几何 ----------------
    def distance_to_body(self, p: Vec3) -> float:
        """点到**塔体（实心圆柱）表面**的最近距离；点在塔内返回 0。

        这是最"诚实"的几何量：0 表示直接打中塔，>0 表示还有多少空隙。
        """
        cx, cy, cz = self.center
        dx, dy, dz = p[0] - cx, p[1] - cy, p[2] - cz
        r_xy = math.hypot(dx, dy)

        # 已经进入塔体内部
        if 0.0 <= dz <= self.height and r_xy <= self.radius:
            return 0.0

        # 否则求"点到实心圆柱"的最近距离：
        #   把点的水平半径夹到 [0, radius]、把高度夹到 [0, height]，
        #   再算三维距离即可覆盖"侧面 / 顶面 / 底面 / 棱"所有情况。
        r_near = min(r_xy, self.radius)
        z_near = min(max(dz, 0.0), self.height)
        dr = r_xy - r_near
        hz = dz - z_near
        return math.hypot(dr, hz)

    def closest_approach(self, points: Sequence[Vec3]) -> Tuple[float, int]:
        """一条轨迹（离散点列）到塔的最近距离及对应下标。"""
        best_d, best_i = float("inf"), -1
        for i, p in enumerate(points):
            d = self.distance_to_body(p)
            if d < best_d:
                best_d, best_i = d, i
        return best_d, best_i

    def is_threatened_by(self, points: Sequence[Vec3]) -> Tuple[bool, float, int]:
        """轨迹是否构成**威胁**：最近距塔 < 安全垫。

        返回 (是否威胁, 最近距离, 下标)。
        ⚠ 这是"单条弹道"的判定；真实判定还要考虑弹道不确定性（T2-2 弹道束），
          届时对每条分支调用本函数即可。
        """
        d, i = self.closest_approach(points)
        return (d < self.safety_margin), d, i

    def contains_horizontal(self, p: Vec3) -> bool:
        """点是否落在塔的**水平保护圈**内（俯视判定，用于画图/直观检查）。"""
        return math.hypot(p[0] - self.center[0], p[1] - self.center[1]) \
            <= self.radius + self.safety_margin

    def describe(self) -> str:
        return (f"塔「{self.name}」底面 @({self.center[0]:.1f},{self.center[1]:.1f},"
                f"{self.center[2]:.1f})  半径 {self.radius:.2f} m  高 {self.height:.2f} m  "
                f"安全垫 {self.safety_margin:.2f} m")


# ==========================================================================
#  二、敌方发射场
# ==========================================================================
@dataclass
class Launcher:
    """敌方发射场（建模用，当前阶段只记录几何，不参与制导）。"""

    name: str = "launcher"
    position: Vec3 = (-22.0, 0.0, 12.0)   # 发射口位置 (ENU)
    aim_azimuth_deg: float = 0.0           # 标称朝向（从 +x 逆时针，deg）
    azimuth_span_deg: float = 0.0          # 可调方位范围 ±（deg）
    elev_min_deg: float = 0.0              # 可调仰角下限
    elev_max_deg: float = 45.0             # 可调仰角上限
    reload_sec: float = 0.0                # 装填周期（0 = 未知/不限）

    def describe(self) -> str:
        return (f"发射场「{self.name}」@({self.position[0]:.1f},{self.position[1]:.1f},"
                f"{self.position[2]:.1f})  朝向 {self.aim_azimuth_deg:.0f}°"
                f"±{self.azimuth_span_deg:.0f}°  仰角 {self.elev_min_deg:.0f}~"
                f"{self.elev_max_deg:.0f}°")


# ==========================================================================
#  三、弹丸 / 飞镖
# ==========================================================================
#  shim 的目标类型编号（见 pure_guidance_shim.cpp）
PROJ_KINDS = {"STRAIGHT": 0, "BALLISTIC": 1, "DART": 2, "CIRCLE": 3, "SNAKE": 4}


@dataclass
class ProjectileSpec:
    """弹丸 / 飞镖 —— 直接对接 `TargetModel`（真 C++ 代码，含气动 + 重力）。

    ⚠ 气动参数（mass / drag_cd / drag_area）目前是**估计值**，
      见《待决问题.md》Q2（等飞镖组给真值）。
    """

    kind: str = "DART"                     # STRAIGHT / BALLISTIC / DART / CIRCLE / SNAKE
    launch_point: Vec3 = (-14.0, 0.0, 10.0)
    speed: float = 14.0                    # 发射速率 (m/s)
    elev_deg: float = -10.0                # 仰角 (deg)：正=上抬，负=俯冲
    azim_deg: float = 0.0                  # 方位角 (deg)：从 +x 绕 +z 逆时针
    # 气动（仅 DART 用）
    mass: float = 0.40                     # kg
    drag_cd: float = 0.50
    drag_area: float = 2.83e-3             # m²（≈ φ60 mm 圆截面）
    rho: float = 1.225                     # kg/m³
    # 环境
    gravity: Vec3 = (0.0, 0.0, -9.81)
    wind: Vec3 = (0.0, 0.0, 0.0)

    def launch_velocity(self) -> Vec3:
        """由 速度 + 仰角 + 方位角 解析出的初速度矢量（ENU）。"""
        e = math.radians(self.elev_deg)
        a = math.radians(self.azim_deg)
        return (self.speed * math.cos(e) * math.cos(a),
                self.speed * math.cos(e) * math.sin(a),
                self.speed * math.sin(e))

    def to_cfg(self, t_max: float = 10.0, dt_table: float = 0.0005,
               ground_z: float = 0.0) -> List[float]:
        """生成 shim 要求的 29 元素目标配置数组。"""
        c = [0.0] * 29
        c[0] = float(PROJ_KINDS.get(self.kind, 2))
        c[1:4] = list(self.launch_point)
        c[4:7] = [0.0, 0.0, 0.0]           # v0（用 launch_speed 生成）
        c[7] = self.speed
        c[8] = math.radians(self.elev_deg)
        c[9] = math.radians(self.azim_deg)
        c[10], c[11], c[12], c[13] = self.mass, self.drag_cd, self.drag_area, self.rho
        c[14], c[15], c[16] = self.gravity
        c[17], c[18], c[19] = self.wind
        c[26], c[27], c[28] = dt_table, t_max, ground_z
        return c

    def describe(self) -> str:
        v = self.launch_velocity()
        return (f"弹丸「{self.kind}」从 ({self.launch_point[0]:.1f},{self.launch_point[1]:.1f},"
                f"{self.launch_point[2]:.1f}) 发射：{self.speed:.1f} m/s  "
                f"仰角 {self.elev_deg:+.0f}°  方位 {self.azim_deg:.0f}°  "
                f"→ v0=({v[0]:.1f},{v[1]:.1f},{v[2]:.1f})  "
                f"m={self.mass}kg Cd={self.drag_cd} A={self.drag_area*1e4:.1f}cm²")


# ==========================================================================
#  四、我方无人机
# ==========================================================================
@dataclass
class DroneSpec:
    """我方无人机 —— 巡航态初值 + 机动边界。

    ⚠ 机动边界（a_max_*）目前是**假设值**，见《待决问题.md》Q6
      （需要用仿真/真机标定）。
    """

    position: Vec3 = (-4.0, 0.0, 9.0)      # 制导开始时的位置
    velocity: Vec3 = (-5.0, 0.0, 0.0)      # 制导开始时的速度（**巡航速度，非零**）
    a_max: float = 10.0                    # 合加速度上限 (m/s²)（水平为主）
    a_max_up: float = 6.0                  # 向上加速上限（受推力余量限制）
    a_max_down: float = 8.0                # 向下加速上限（近自由落体）
    v_max: float = 15.0                    # 最大水平速度 (m/s)
    rate_hz: float = 20.0                  # 制导/指令频率
    tau: float = 0.20                      # 加速度一阶响应时间常数 (s)

    def speed(self) -> float:
        return math.sqrt(sum(x * x for x in self.velocity))

    def describe(self) -> str:
        return (f"无人机 @({self.position[0]:.1f},{self.position[1]:.1f},{self.position[2]:.1f})  "
                f"巡航速度 ({self.velocity[0]:+.1f},{self.velocity[1]:+.1f},{self.velocity[2]:+.1f})"
                f" = {self.speed():.2f} m/s  a_max={self.a_max} m/s²  "
                f"指令 {self.rate_hz:.0f} Hz  tau={self.tau} s")


# ==========================================================================
#  五、场地
# ==========================================================================
@dataclass
class Field:
    """场地：安全飞行盒 + 障碍物（用于安全围栏与可视化）。

    默认值来自 `walls.sdf` 的实测几何（墙高 30 m，四面墙把可用空间切成一个盒子）。
    """

    name: str = "walls"
    bounds_min: Vec3 = (-12.0, -9.0, 0.0)
    bounds_max: Vec3 = (4.0, 4.0, 30.0)
    obstacles: List[Tuple[Vec3, Vec3]] = field(default_factory=list)   # [(中心, 尺寸)]

    def contains(self, p: Vec3) -> bool:
        return all(self.bounds_min[k] <= p[k] <= self.bounds_max[k] for k in range(3))

    def describe(self) -> str:
        return (f"场地「{self.name}」安全盒 x[{self.bounds_min[0]:.0f},{self.bounds_max[0]:.0f}] "
                f"y[{self.bounds_min[1]:.0f},{self.bounds_max[1]:.0f}] "
                f"z[{self.bounds_min[2]:.0f},{self.bounds_max[2]:.0f}]  "
                f"障碍 {len(self.obstacles)} 个")


# ==========================================================================
#  六、场景（容器）
# ==========================================================================
@dataclass
class Scene:
    """一次对抗场景 = 塔 + 发射场 + 弹丸 + 无人机 + 场地。"""

    name: str = "scene"
    tower: Tower = field(default_factory=Tower)
    launcher: Launcher = field(default_factory=Launcher)
    projectile: ProjectileSpec = field(default_factory=ProjectileSpec)
    drone: DroneSpec = field(default_factory=DroneSpec)
    field_: Field = field(default_factory=Field)

    # ---------------- 序列化 ----------------
    def to_dict(self) -> dict:
        d = asdict(self)
        d["field"] = d.pop("field_")          # 避免与 dataclass 的 field 重名
        return d

    @classmethod
    def from_dict(cls, d: dict) -> "Scene":
        d = dict(d)
        f = d.pop("field", d.pop("field_", {}))
        return cls(
            name=d.get("name", "scene"),
            tower=Tower(**d.get("tower", {})),
            launcher=Launcher(**d.get("launcher", {})),
            projectile=ProjectileSpec(**d.get("projectile", {})),
            drone=DroneSpec(**d.get("drone", {})),
            field_=Field(**f),
        )

    def save(self, path: str) -> None:
        with open(path, "w", encoding="utf-8") as fp:
            json.dump(self.to_dict(), fp, ensure_ascii=False, indent=2)

    @classmethod
    def load(cls, path: str) -> "Scene":
        with open(path, encoding="utf-8") as fp:
            return cls.from_dict(json.load(fp))

    # ---------------- 与仿真脚本对接 ----------------
    def to_sim_dict(self) -> dict:
        """转成 `sim_phase_b_offline.py` 需要的扁平参数表。"""
        p = self.projectile
        d = self.drone
        return dict(
            mode="pilot",
            drone0=d.position, drone_v0=d.velocity,
            p0=p.launch_point, speed=p.speed, elev=p.elev_deg, azim=p.azim_deg,
            mass=p.mass, cd=p.drag_cd, area=p.drag_area, rho=p.rho,
            a_max=d.a_max, t_go_freeze=0.10, law=0, N=6.0,
            tau=d.tau, dt_g=1.0 / d.rate_hz, dt_sim=0.0025, hit_radius=0.5,
            t_max=10.0,
            tower_center=self.tower.center, tower_radius=self.tower.radius,
            tower_height=self.tower.height, tower_margin=self.tower.safety_margin,
        )

    # ---------------- 摘要 ----------------
    def describe(self) -> str:
        lines = [f"场景「{self.name}」",
                 "  " + self.tower.describe(),
                 "  " + self.launcher.describe(),
                 "  " + self.projectile.describe(),
                 "  " + self.drone.describe(),
                 "  " + self.field_.describe()]
        return "\n".join(lines)


# ==========================================================================
#  预设场景
# ==========================================================================
def preset_pilot() -> Scene:
    """★ 初步仿真默认场景：「无人机巡航态 + 较近弹丸 + 立即拦截」。

    对应需求：
      "先让无人机飞起来并以一定速度巡航的基础上，给一个较近的弹丸
       （考虑空气阻力、重力等）（速度、方向先给定）
       （立马开始拦截，假定已经判断在这个时刻开始拦截）"

    几何（ENU，**全部在 y=0 走廊**，walls 世界里该走廊净空）：
      塔    : 原点，半径 1.2 m，高 3 m，安全垫 1.0 m
      无人机: 从 (−4, 0, 8) 朝 **−x（西，迎击方向）** 巡航，5 m/s
      弹丸  : 从 (−14, 0, 10) 以 14 m/s、**俯角 6°** 朝 +x 飞向塔
              （即弹丸处于**末段下坠**，指向塔 —— 若不拦会打到塔）

    ⚠ 这组参数是**扫参得出**的（见 `sim_phase_b_offline.py --mode pilot`）：
      · 无人机巡航高度 8 m（比弹丸低 2 m）—— 弹丸俯冲比无人机下落快，
        高度差越小越好补；8 m 时 PPN 脱靶 0.39 m、**APN 脱靶 0.018 m**
      · 弹丸俯角 −6°（平缓），俯冲越陡越难补
    """
    return Scene(
        name="pilot-intercept",
        tower=Tower(name="our-tower", center=(0.0, 0.0, 0.0),
                    radius=1.2, height=3.0, safety_margin=1.0),
        launcher=Launcher(name="enemy-launcher", position=(-22.0, 0.0, 12.0),
                          aim_azimuth_deg=0.0, azimuth_span_deg=30.0,
                          elev_min_deg=0.0, elev_max_deg=45.0),
        projectile=ProjectileSpec(kind="DART", launch_point=(-14.0, 0.0, 10.0),
                                  speed=14.0, elev_deg=-6.0, azim_deg=0.0),
        drone=DroneSpec(position=(-4.0, 0.0, 8.0), velocity=(-5.0, 0.0, 0.0),
                        a_max=10.0, rate_hz=20.0, tau=0.20),
        field_=Field(),
    )


def preset_phase_b() -> Scene:
    """阶段 B 的原演示场景（无人机从原点起步预加速），保留用于回归对比。"""
    return Scene(
        name="phase-b-demo",
        tower=Tower(name="our-tower", center=(0.0, 0.0, 0.0),
                    radius=1.2, height=3.0, safety_margin=1.0),
        projectile=ProjectileSpec(kind="DART", launch_point=(-12.0, 0.0, 11.0),
                                  speed=15.0, elev_deg=5.0, azim_deg=0.0),
        drone=DroneSpec(position=(0.0, 0.0, 10.0), velocity=(0.0, 0.0, 0.0),
                        a_max=10.0, rate_hz=20.0, tau=0.20),
    )


PRESETS = {
    "pilot": preset_pilot,
    "phase_b": preset_phase_b,
}


# ==========================================================================
if __name__ == "__main__":
    import sys
    name = sys.argv[1] if len(sys.argv) > 1 else "pilot"
    if name not in PRESETS:
        print(f"可用预设：{', '.join(PRESETS)}")
        sys.exit(1)
    s = PRESETS[name]()
    print(s.describe())
    print()
    print("初速度矢量（弹丸）:", tuple(round(x, 2) for x in s.projectile.launch_velocity()))
    print("无人机巡航速度  :", round(s.drone.speed(), 2), "m/s")
    print("塔保护圈半径    :", s.tower.radius + s.tower.safety_margin, "m")
