#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Gazebo 场景可视化同步 —— 把仿真里的「防御塔 / 弹丸 / 轨迹」送进 Gazebo GUI。

【定位】与 `viz_scene.py` 一样：**不含任何算法**，只负责"取数 + 画出来"。
    算法（制导 / 弹丸弹道）始终在 C++ 制导核里，本模块只把已有的坐标摆到 Gazebo 里。

【为什么能"零侵入"XTDrone2】
    `walls.sdf` 里已经启用了 `gz-sim-user-commands-system` 插件，
    它提供 `/world/<world>/create`、`/world/<world>/set_pose`、`/world/<world>/remove`
    三个运行时服务 ⇒ 我们可以**在运行时动态创建/移动实体**，
    完全不用改 XTDrone2 的任何文件（不改 walls.sdf、不用 colcon build、不影响队友）。

【坐标系】★ Gazebo 世界是 **ENU**（x 东 / y 北 / z 上），
    与制导核内部的 `p_enu = (东, 北, 上)` **直接对应** ⇒ **不需要任何换算**。
    （这是本模块唯一需要记住的坐标约定；不要在这里再"转一次"。）

【线程模型】`gz.transport` 的服务请求是**阻塞**的（等响应）。
    为保证不拖慢 ROS 回调，本模块用一个**后台线程**按固定频率发 `set_pose`，
    调用方只更新缓存值（`set_projectile()` 立即返回）。

【依赖】`gz.transport13` + `gz.msgs10`（protobuf 生成），
    二者随 Gazebo Harmonic 一起装在 `/usr/lib/python3/dist-packages/gz/`，
    在 WSL 里 **不需要 ROS** 就能 import。

【用法】
    python3 gz_scene_sync.py --demo          # ★ 不放 ROS，先看效果（塔 + 弹丸飞一条线）
    python3 gz_scene_sync.py --tower         # 只放一座塔
    python3 gz_scene_sync.py --clear         # 清掉本模块创建过的实体
    （节点用法见 phase_b_guidance.py 的 --gz-viz）
"""

from __future__ import annotations

import argparse
import math
import sys
import threading
import time

# --------------------------------------------------------------------------
#  gz python 绑定（延迟 import，便于在没有 Gazebo 的机器上给出友好提示）
# --------------------------------------------------------------------------
_GZ_IMPORT_ERROR = None
try:
    from gz.transport13 import Node as _GzNode
    from gz.msgs10 import (boolean_pb2, entity_factory_pb2, entity_pb2,
                           marker_pb2, pose_pb2)
except Exception as _e:            # pragma: no cover - 取决于本机环境
    _GZ_IMPORT_ERROR = _e


# --------------------------------------------------------------------------
#  消息字段 / 枚举名 —— **已逐条核对** gz-msgs10 的 proto 描述符（2026-10-16）
#
#  说明：这些名字在不同 gz-msgs 版本里可能变，所以集中在常量里，
#       并在 `_verify_enums()` 里做启动自检（名字没了会**立刻报错**，
#       而不是运行到一半 AttributeError）。
#
#  EntityFactory（gz/msgs/entity_factory.proto）：
#      header(1) sdf(2) sdf_filename(3) model(4) light(5) clone_name(6)
#      pose(7) name(8) allow_renaming(9) relative_to(10) spherical_coordinates(11)
#  Pose：      header(1) name(2) id(3) position(4:Vector3d) orientation(5:Quaternion)
#  Quaternion：header(1) x(2) y(3) z(4) w(5)
#  Vector3d：  header(1) x(2) y(3) z(4)
#  Entity：    header(1) id(2) name(3) type(4)      Type.MODEL = 2
#  Marker：    header(1) action(2) ns(3) id(4) layer(5) type(6) lifetime(7:**Time**)
#              pose(8) scale(9) material(10) point(11) text(12) parent(13) ...
#      ⚠ Action 枚举是 ADD_MODIFY=0 / DELETE_MARKER=1 / DELETE_ALL=2
#        —— **没有 `ADD`**（写成 Marker.ADD 会 AttributeError）
#      ⚠ lifetime 是 **gz.msgs.Time**（sec/nsec），**不是 double**
#      Type：BOX=1 CYLINDER=2 LINE_STRIP=3 LIST=4 POINTS=5 SPHERE=6 ...
#  Color：     header(1) r(2) g(3) b(4) a(5)
#  Material：  header(1) script(2) shader_type(3) normal_map(4)
#              ambient(5) diffuse(6) specular(7) emissive(8) ...
# --------------------------------------------------------------------------
MARKER_ACTION_ADD = "ADD_MODIFY"      # ⚠ 不是 "ADD"
MARKER_ACTION_DELETE_ALL = "DELETE_ALL"
MARKER_TYPE_LINE_STRIP = "LINE_STRIP"
MARKER_TYPE_SPHERE = "SPHERE"
ENTITY_TYPE_MODEL = "MODEL"


def _resolve_enum(holder, name: str, attr: str):
    """取 `holder.<attr>`，取不到时列出该枚举的全部可用名（便于排错）。"""
    val = getattr(holder, attr, None)
    if val is None:
        avail = [k for k in dir(holder) if k.isupper()]
        raise RuntimeError(
            f"gz-msgs 版本不兼容：{name} 里没有 '{attr}'；可用名 = {avail}")
    return val


def _verify_enums():
    """启动自检：本模块用到的枚举名必须存在。名字变了要立刻报错。"""
    if _GZ_IMPORT_ERROR is not None:
        return
    _resolve_enum(marker_pb2.Marker, "Marker.Action", MARKER_ACTION_ADD)
    _resolve_enum(marker_pb2.Marker, "Marker.Action", MARKER_ACTION_DELETE_ALL)
    _resolve_enum(marker_pb2.Marker, "Marker.Type", MARKER_TYPE_LINE_STRIP)
    _resolve_enum(marker_pb2.Marker, "Marker.Type", MARKER_TYPE_SPHERE)
    _resolve_enum(entity_pb2.Entity, "Entity.Type", ENTITY_TYPE_MODEL)
    # 字段存在性（protobuf 的 DESCRIPTOR.fields_by_name）
    for pb, fields in (
        (entity_factory_pb2.EntityFactory, ("sdf", "name", "pose", "allow_renaming")),
        (pose_pb2.Pose, ("name", "position", "orientation")),
        (entity_pb2.Entity, ("name", "type")),
    ):
        for f in fields:
            if f not in pb.DESCRIPTOR.fields_by_name:
                raise RuntimeError(
                    f"gz-msgs 版本不兼容：{pb.DESCRIPTOR.name} 里没有字段 '{f}'")


# --------------------------------------------------------------------------
#  常量
# --------------------------------------------------------------------------
DEFAULT_WORLD = "walls"            # XTDrone2 单机 demo 默认世界（见 README §1）

TOWER_NAME = "defense_tower"       # 防御塔（我方，原点）
PROJECTILE_NAME = "enemy_dart"     # 弹丸（敌方）
LAUNCHER_NAME = "enemy_launcher"   # 发射架（纯视觉，可选）

TRAIL_NS = "pilot_trail"           # 轨迹 marker 的命名空间

# 弹丸"藏起来"时放到这里（远在地下，不在任何默认视野里）
HIDDEN_Z = -500.0

# 服务名模板（gz-sim 8 / Harmonic）
SVC_CREATE = "/world/{world}/create"
SVC_SET_POSE = "/world/{world}/set_pose"
SVC_REMOVE = "/world/{world}/remove"
SVC_MARKER = "/marker"             # 由 GUI 的 MarkerManager 插件提供


# --------------------------------------------------------------------------
#  SDF 模板（内联字符串 ⇒ 不依赖 models 目录，做成一件事只动一个文件）
# --------------------------------------------------------------------------
def tower_sdf(name: str, radius: float, height: float) -> str:
    """防御塔：实心圆柱，底部贴地 ⇒ 几何中心抬到 height/2。

    颜色用**蓝青**——我方目标；弹丸用橙红，形成对比（也照顾色觉差异）。
    """
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry>
          <cylinder><radius>{radius}</radius><length>{height}</length></cylinder>
        </geometry>
        <material>
          <ambient>0.10 0.45 0.75 1</ambient>
          <diffuse>0.15 0.55 0.90 1</diffuse>
          <specular>0.25 0.25 0.25 1</specular>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def launcher_sdf(name: str, size: float = 0.5) -> str:
    """敌方发射架：一个放在地上的方块（纯视觉，指示"弹丸从哪来"）。"""
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><box><size>{size} {size} {size}</size></box></geometry>
        <material>
          <ambient>0.85 0.35 0.10 1</ambient>
          <diffuse>0.95 0.45 0.12 1</diffuse>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def projectile_sdf(name: str, radius: float) -> str:
    """弹丸：小球。

    ⚠ 关键设计：`static=false` + **`<gravity>false</gravity>`** + **无 collision**：
        · 不是 static ⇒ pose 可以被 set_pose 服务直接改（动态实体的标准做法）；
        · 关重力、无碰撞、无外力 ⇒ 物理引擎不会把它推走，位置完全由我们说了算
          ⇒ **所见即 TargetModel 所算**（与制导用的弹道逐帧一致），
             而不是让 Gazebo 自己另算一条弹道（那样会和制导的依据对不上）。
    """
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>false</static>
    <link name="link">
      <gravity>false</gravity>
      <inertial>
        <mass>0.4</mass>
        <inertia>
          <ixx>0.0006</ixx><ixy>0</ixy><ixz>0</ixz>
          <iyy>0.0006</iyy><iyz>0</iyz>
          <izz>0.0006</izz>
        </inertia>
      </inertial>
      <visual name="visual">
        <geometry><sphere><radius>{radius}</radius></sphere></geometry>
        <material>
          <ambient>0.85 0.25 0.05 1</ambient>
          <diffuse>1.00 0.40 0.10 1</diffuse>
          <emissive>0.55 0.15 0.00 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


# ==========================================================================
#  GzScene
# ==========================================================================
class GzScene:
    """把场景实体同步到 Gazebo。

    典型用法（节点里）：

        scene = GzScene(world="walls")
        scene.start()                       # 连不上就自动降级（打一条警告）
        scene.ensure_tower()                # 放塔
        scene.ensure_projectile()           # 放弹丸（先藏在视野外）
        ...
        scene.set_projectile((x, y, z))     # 每帧调用，**立即返回**（只更新缓存）
        ...
        scene.close()                       # 停止后台线程
    """

    def __init__(self, world: str = DEFAULT_WORLD, hz: float = 30.0,
                 timeout_ms: int = 500, verbose: bool = True):
        self.world = world
        self.hz = max(1.0, float(hz))
        self.timeout_ms = int(timeout_ms)
        self.verbose = verbose

        self.available = False          # 是否成功接上 Gazebo
        self._node = None
        self._svc_create = SVC_CREATE.format(world=world)
        self._svc_set_pose = SVC_SET_POSE.format(world=world)
        self._svc_remove = SVC_REMOVE.format(world=world)

        self._lock = threading.Lock()
        self._pending = None            # 待写入的弹丸位置（ENU）
        self._thread = None
        self._running = False
        self._warned = False            # 只对"发不出去"告警一次，不刷屏
        self._created = set()           # 已创建的实体名
        self.stats = {"set_pose": 0, "failed": 0}

        if _GZ_IMPORT_ERROR is not None:
            self._log(f"⚠ 无法 import gz 的 Python 绑定（{_GZ_IMPORT_ERROR}）"
                      f" ⇒ Gazebo 可视化已禁用")
            return
        try:
            _verify_enums()          # 枚举 / 字段名自检（版本不兼容时立刻报错）
            self._node = _GzNode()
            self.available = True
        except Exception as e:
            self._log(f"⚠ 初始化 gz.transport 失败（{e}）⇒ Gazebo 可视化已禁用")

    # ---------------------------------------------------------------- 基础
    def _log(self, s: str):
        if self.verbose:
            print(s, flush=True)

    def _req(self, service: str, msg, rep_type):
        """发一个服务请求。返回 (ok, response)；异常一律吞掉并计数。"""
        if not self.available:
            return False, None
        try:
            ok, rep = self._node.request(service, msg, type(msg), rep_type,
                                         self.timeout_ms)
            return bool(ok), rep
        except Exception:
            self.stats["failed"] += 1
            return False, None

    def _create(self, name: str, sdf: str, pose=(0.0, 0.0, 0.0)) -> bool:
        """创建一个实体（同名先删，避免 allow_renaming 引发的重复名）。"""
        self._remove(name)                       # 忽略失败
        req = entity_factory_pb2.EntityFactory()
        req.sdf = sdf
        req.name = name
        req.pose.position.x = float(pose[0])
        req.pose.position.y = float(pose[1])
        req.pose.position.z = float(pose[2])
        req.pose.orientation.w = 1.0
        ok, rep = self._req(self._svc_create, req, boolean_pb2.Boolean)
        if ok and rep is not None and rep.data:
            self._created.add(name)
            return True
        if not self._warned:
            self._warned = True
            self._log(f"⚠ Gazebo 未响应 {self._svc_create}"
                      f" ⇒ 请确认仿真在跑（gz topic -l 能看到 /world/{self.world}）；"
                      f"可视化将静默跳过")
        return False

    def _remove(self, name: str) -> bool:
        req = entity_pb2.Entity()
        req.name = name
        req.type = _resolve_enum(entity_pb2.Entity, "Entity.Type", ENTITY_TYPE_MODEL)
        ok, rep = self._req(self._svc_remove, req, boolean_pb2.Boolean)
        got = bool(ok and rep is not None and rep.data)
        if got:
            self._created.discard(name)
        return got

    def _do_set_pose(self, name: str, p, yaw: float = 0.0) -> bool:
        req = pose_pb2.Pose()
        req.name = name
        req.position.x = float(p[0])
        req.position.y = float(p[1])
        req.position.z = float(p[2])
        # 绕 z（上）转 yaw：ENU 下 z 轴向上 ⇒ 四元数 (0,0,sin(yaw/2),cos(yaw/2))
        req.orientation.x = 0.0
        req.orientation.y = 0.0
        req.orientation.z = math.sin(yaw * 0.5)
        req.orientation.w = math.cos(yaw * 0.5)
        ok, _ = self._req(self._svc_set_pose, req, boolean_pb2.Boolean)
        if ok:
            self.stats["set_pose"] += 1
        else:
            self.stats["failed"] += 1
            if not self._warned:
                self._warned = True
                self._log(f"⚠ set_pose 失败 ⇒ 请确认仿真在跑、世界名是 "
                          f"'{self.world}'（当前启动命令见 README §1）")
        return ok

    # ------------------------------------------------------------ 场景元素
    def ensure_tower(self, radius: float = 1.2, height: float = 3.0,
                     center=(0.0, 0.0)) -> bool:
        """（重新）创建防御塔。塔底贴地 ⇒ 圆柱几何中心在 (cx, cy, height/2)。"""
        pose = (center[0], center[1], height * 0.5)
        return self._create(TOWER_NAME, tower_sdf(TOWER_NAME, radius, height), pose)

    def ensure_launcher(self, p=(0.0, 0.0, 0.0), size: float = 0.6) -> bool:
        """（重新）创建敌方发射架（纯视觉地标）。"""
        return self._create(LAUNCHER_NAME, launcher_sdf(LAUNCHER_NAME, size),
                            (p[0], p[1], size * 0.5))

    def ensure_projectile(self, radius: float = 0.06) -> bool:
        """（重新）创建弹丸，**先藏在视野外**（HIDDEN_Z）。"""
        return self._create(PROJECTILE_NAME, projectile_sdf(PROJECTILE_NAME, radius),
                            (0.0, 0.0, HIDDEN_Z))

    def hide_projectile(self) -> None:
        """把弹丸移出视野（等价于"消失"），命中/脱靶后调用。"""
        self._do_set_pose(PROJECTILE_NAME, (0.0, 0.0, HIDDEN_Z))

    def set_projectile(self, p, yaw: float = 0.0) -> None:
        """更新弹丸位置（ENU, m）。**非阻塞** —— 只写缓存，后台线程负责发送。"""
        with self._lock:
            self._pending = (float(p[0]), float(p[1]), float(p[2]), float(yaw))

    # ---------------------------------------------------------------- 轨迹
    def draw_trail(self, points, ns: str = TRAIL_NS, width: float = 0.04,
                   color=(1.0, 0.35, 0.05)) -> bool:
        """把一串点画成折线（LINE_STRIP marker）。失败静默（只是装饰）。"""
        pts = [(float(p[0]), float(p[1]), float(p[2])) for p in points if p is not None]
        if len(pts) < 2:
            return False
        m = marker_pb2.Marker()
        m.ns = ns
        m.id = 0
        m.action = _resolve_enum(marker_pb2.Marker, "Marker.Action", MARKER_ACTION_ADD)
        m.type = _resolve_enum(marker_pb2.Marker, "Marker.Type", MARKER_TYPE_LINE_STRIP)
        m.scale.x = float(width)
        m.material.ambient.r, m.material.ambient.g, m.material.ambient.b = color
        m.material.ambient.a = 1.0
        m.material.diffuse.r, m.material.diffuse.g, m.material.diffuse.b = color
        m.material.diffuse.a = 1.0
        for (x, y, z) in pts:
            pt = m.point.add()
            pt.x, pt.y, pt.z = x, y, z
        ok, _ = self._req(SVC_MARKER, m, boolean_pb2.Boolean)
        return ok

    def mark_point(self, p, ns: str = "pilot_mark", mid: int = 0,
                   size: float = 0.5, color=(1.0, 1.0, 0.0),
                   ttl_ms: int = 8000) -> bool:
        """在一点画一个球（默认黄色）。用来标"命中点 / 交会点"。"""
        m = marker_pb2.Marker()
        m.ns = ns
        m.id = int(mid)
        m.action = _resolve_enum(marker_pb2.Marker, "Marker.Action", MARKER_ACTION_ADD)
        m.type = _resolve_enum(marker_pb2.Marker, "Marker.Type", MARKER_TYPE_SPHERE)
        m.scale.x = m.scale.y = m.scale.z = float(size)
        m.pose.position.x = float(p[0])
        m.pose.position.y = float(p[1])
        m.pose.position.z = float(p[2])
        m.pose.orientation.w = 1.0
        # ⚠ lifetime 是 **gz.msgs.Time**（sec: int64 / nsec: int32），不是 double。
        #   （已核对 gz-msgs10 描述符：lifetime(7) → .gz.msgs.Time）
        m.lifetime.sec = int(ttl_ms // 1000)
        m.lifetime.nsec = int((ttl_ms % 1000) * 1_000_000)
        m.material.ambient.r, m.material.ambient.g, m.material.ambient.b = color
        m.material.ambient.a = 1.0
        m.material.emissive.r, m.material.emissive.g, m.material.emissive.b = color
        m.material.emissive.a = 1.0
        ok, _ = self._req(SVC_MARKER, m, boolean_pb2.Boolean)
        return ok

    def clear_trail(self) -> None:
        m = marker_pb2.Marker()
        m.ns = TRAIL_NS
        m.id = 0
        m.action = marker_pb2.Marker.DELETE_ALL
        self._req(SVC_MARKER, m, boolean_pb2.Boolean)

    # -------------------------------------------------------- 后台同步线程
    def start(self) -> "GzScene":
        """启动后台同步线程（幂等）。"""
        if not self.available or self._running:
            return self
        self._running = True
        self._thread = threading.Thread(target=self._worker, daemon=True,
                                        name="gz_scene_sync")
        self._thread.start()
        return self

    def _worker(self):
        dt = 1.0 / self.hz
        while self._running:
            t0 = time.monotonic()
            with self._lock:
                pend = self._pending
                self._pending = None
            if pend is not None:
                self._do_set_pose(PROJECTILE_NAME, pend[:3], pend[3])
            rest = dt - (time.monotonic() - t0)
            if rest > 0:
                time.sleep(rest)

    def close(self, wait: float = 0.5):
        """停止后台线程（不删除实体 —— 留着便于事后观察）。"""
        self._running = False
        t = self._thread
        if t is not None and t.is_alive():
            t.join(timeout=wait)
        self._thread = None

    def clear_all(self):
        """清掉本模块创建过的实体 + 轨迹 marker。"""
        self.clear_trail()
        for name in (PROJECTILE_NAME, LAUNCHER_NAME, TOWER_NAME):
            self._remove(name)
        self._created.clear()


# ==========================================================================
#  CLI：让用户在"不用跑 ROS"的情况下先确认可视化链路通不通
# ==========================================================================
def _demo(scene: GzScene, tower_r: float, tower_h: float,
          p0, p1, seconds: float, trail: bool) -> int:
    """放塔 + 发射架 + 弹丸，让弹丸沿 p0→p1 飞一趟。"""
    scene.start()
    ok_t = scene.ensure_tower(tower_r, tower_h)
    scene.ensure_launcher((p0[0], p0[1], 0.0))
    ok_p = scene.ensure_projectile()
    if not (ok_t and ok_p):
        print("❌ 创建实体失败 —— 请确认仿真正在跑（ros2 launch ...），"
              "且世界名与实际一致（默认 walls）。")
        return 1
    print(f"✅ 已放置：防御塔 r={tower_r} h={tower_h} @ 原点；"
          f"弹丸 r=0.06（将从 {p0} 飞到 {p1}）")
    print("   ↑ 现在切到 Gazebo 窗口应该能看到塔和弹丸")

    pts = []
    n = max(1, int(seconds * scene.hz))
    for i in range(n + 1):
        s = i / n
        p = (p0[0] + (p1[0] - p0[0]) * s,
             p0[1] + (p1[1] - p0[1]) * s,
             p0[2] + (p1[2] - p0[2]) * s)
        scene.set_projectile(p)
        pts.append(p)
        time.sleep(1.0 / scene.hz)
    if trail:
        scene.draw_trail(pts)
        print("   已画弹道轨迹线")
    scene.mark_point(p1, mid=1, size=0.4, color=(1.0, 0.9, 0.1), ttl_ms=12000)
    print(f"   set_pose 成功 {scene.stats['set_pose']} 次，失败 {scene.stats['failed']} 次")
    print("   弹丸停在终点（8 s 后标记自动消失）。按 Ctrl-C 结束，"
          "或跑 `python3 gz_scene_sync.py --clear` 清理。")
    try:
        while True:
            time.sleep(0.5)
    except KeyboardInterrupt:
        print("\n（保留实体，未清理）")
    return 0


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(
        description="Gazebo 场景可视化同步（塔 / 弹丸 / 轨迹）—— 不含算法",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--world", default=DEFAULT_WORLD,
                    help="Gazebo 世界名（XTDrone2 单机 demo 默认 walls）")
    ap.add_argument("--hz", type=float, default=30.0, help="弹丸位置同步频率 (Hz)")
    ap.add_argument("--quiet", action="store_true")
    g = ap.add_mutually_exclusive_group()
    g.add_argument("--demo", action="store_true",
                   help="★ 演示：放塔 + 让弹丸飞一条线（不用跑 ROS 即可验证）")
    g.add_argument("--tower", action="store_true", help="只放一座防御塔")
    g.add_argument("--clear", action="store_true", help="清掉本模块创建过的实体")
    ap.add_argument("--tower-radius", type=float, default=1.2)
    ap.add_argument("--tower-height", type=float, default=3.0)
    ap.add_argument("--from", dest="p0", default="-14,0,10",
                    help="demo 弹丸起点（ENU，逗号分隔）")
    ap.add_argument("--to", dest="p1", default="2,0,2.0",
                    help="demo 弹丸终点（ENU，逗号分隔）")
    ap.add_argument("--seconds", type=float, default=8.0, help="demo 飞行时长 (s)")
    ap.add_argument("--no-trail", action="store_true", help="demo 不画轨迹线")
    args = ap.parse_args(argv)

    def _vec(s):
        v = [float(x) for x in str(s).replace(" ", "").split(",")]
        if len(v) != 3:
            raise SystemExit(f"需要 3 个分量（x,y,z）：{s!r}")
        return tuple(v)

    scene = GzScene(world=args.world, hz=args.hz, verbose=not args.quiet)

    if args.clear:
        scene.clear_all()
        print("已清理：塔 / 发射架 / 弹丸 / 轨迹 marker")
        return 0
    if args.tower:
        ok = scene.ensure_tower(args.tower_radius, args.tower_height)
        print("✅ 已放置防御塔" if ok else "❌ 放置失败（仿真在跑吗？世界名对吗？）")
        return 0 if ok else 1
    if args.demo:
        return _demo(scene, args.tower_radius, args.tower_height,
                     _vec(args.p0), _vec(args.p1), args.seconds, not args.no_trail)

    ap.print_help()
    print("\n提示：先用 `--demo` 验证可视化链路（不需要 ROS 节点）。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
