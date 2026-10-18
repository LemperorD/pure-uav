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
                           marker_pb2, physics_pb2, pose_pb2)
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
# 发射架方块边长 (m)：`ensure_launcher` 默认 size，方块中心在 z=size/2，
# **顶面在 z=size**。弹丸"贴在发射架顶面"时要用到它（见节点里的 preset 逻辑）。
LAUNCHER_SIZE = 0.6

TRAIL_NS = "pilot_trail"           # 轨迹 marker 的命名空间

# 弹丸"藏起来"时放到这里（远在地下，不在任何默认视野里）
HIDDEN_Z = -500.0

# ★ 建实体的服务超时（ms）—— 与 set_pose 的短超时**分开**
#   实测：`GzScene` 默认 500 ms，"解析 SDF + 插入世界"很容易超时
#   ⇒ 表现为"塔建成功、发射架和弹丸双双失败"，而且因为"先删后建"还会把旧的删掉。
CREATE_TIMEOUT_MS = 3000

# ★ 摆位置（set_pose）在**开场**也要长超时（2026-10-17 实跑根因）
#   实测日志（v30 第一轮）：t=2.106s 处出现 **2028 ms 主回路空洞**，
#   正好等于 `2 × 1000 ms` —— 就是开场那两次 set_pose 各超时一次：
#       ✓ defense_tower 已存在（复用，没发请求）
#       ⚠ set_pose 失败
#       ✓ enemy_launcher 已存在但偏了 2.00 m ⇒ 只移动 ⚠ 移动失败
#       ✓ enemy_dart     已存在但偏了 14.00 m ⇒ 只移动 ⚠ 移动失败
#   而同一次 run 的 GUIDED 段 30 Hz 的 set_pose **全部成功**（统计 15 次）。
#   ⇒ 结论：**进程内第一条服务请求要花时间做发现/握手**，1 s 不够。
#   ⇒ 新增 `warmup()`（开场先做一次长超时的无害往返）+ `SET_POSE_TIMEOUT_MS`
#      + `_do_set_pose(..., retries=1)`（只给"摆场景"这类一次性调用用，
#      后台 30 Hz 的逐帧同步不加，避免失败时累积背压）。
SET_POSE_TIMEOUT_MS = 3000

# ★ **逐帧**同步（后台线程每帧推送弹丸位置）必须用**短**超时（ms）。
#   实测（2026-10-18 实跑）：GUIDED 段出现 **1004 ms 主回路空洞**，
#   日志是 `set_pose 失败：服务回了 data=False` —— 逐帧调用沿用了 1000 ms 超时，
#   而 `gz.transport` 阻塞期间**占着 GIL** ⇒ 后台线程一次失败就把控制回路卡 1 秒。
#   飞行段卡 1 s ≥ COM_OF_LOSS_T(0.5~1.0 s) ⇒ PX4 停用 offboard ⇒ 自由落体风险。
#   ⇒ 逐帧同步限到 150 ms（< LOOP_GAP_WARN 200 ms），失败就**丢这一帧**，
#     下一帧（33 ms 后）自然重试 —— 位置同步是"尽力而为"，绝不能拖累飞行。
FRAME_SETPOSE_TIMEOUT_MS = 150

# 服务名模板（gz-sim 8 / Harmonic）
SVC_CREATE = "/world/{world}/create"
SVC_SET_POSE = "/world/{world}/set_pose"
SVC_REMOVE = "/world/{world}/remove"
SVC_SET_PHYSICS = "/world/{world}/set_physics"   # 调实时率（慢放）用
SVC_MARKER = "/marker"             # 由 GUI 的 MarkerManager 插件提供

# 回放用的 marker 命名空间（蓝色球 = 无人机重放位置）
UAV_REPLAY_NS = "uav_replay"
UAV_REPLAY_ID = 1
PROJ_TRAIL_NS = "replay_proj_trail"
UAV_TRAIL_NS = "replay_uav_trail"

# 轨迹颜色（两条线/两串珠子共用，保证 marker 模式与珠子模式视觉一致）
PROJ_COLOR = (1.00, 0.35, 0.05)    # 弹丸：橙红
UAV_COLOR = (0.20, 0.95, 0.35)     # 无人机：绿

# 无 marker 时的"珠子"实体名前缀（注意：不要与其他实体重名）
PROJ_BEAD_PREFIX = "replay_proj_bead"
UAV_BEAD_PREFIX = "replay_uav_bead"
# 无 marker 时代表"无人机当前位置"的可移动小球（marker 的替代品）
UAV_BALL_NAME = "replay_uav_ball"

# ★ 探针用的**虚构实体名**（永远不存在于世界里）。
#   用途：① `warmup()` 预热连接；② `probe_services()` 探测 set_pose 是否可达。
#   ⚠ 2026-10-17 实跑教训：`--check` 原来拿**真弹丸**当探针，而 `Pose` 的
#     position 默认是 (0,0,0) ⇒ 一个"只读诊断"把弹丸搬到了世界原点（塔基里）。
#     ⇒ 探针**绝不能**碰塔/发射架/弹丸，必须用虚构名。
PROBE_POSE_NAME = "__pgu_warmup__"


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


def bead_sdf(name: str, radius: float, color=(1.0, 0.35, 0.05)) -> str:
    """★ 轨迹"珠子"：**静态**小球，用来在没有 `/marker` 时画出轨迹。

    为什么需要它（2026-10-17 实飞踩到）：
      `/marker` 服务由 Gazebo **GUI** 的 MarkerManager 插件提供。
      实测用户环境里 `marker` 服务**不可用**（但 `create` / `set_pose` 都通）
      ⇒ 轨迹线和命中点一条都画不出来，回放只剩两个小球在飞，看着像"没回放"。
      ⇒ 兜底方案：用一串**静态小球实体**沿轨迹摆出来（虚线感），
        只依赖 `create` 服务，**完全不需要 GUI 插件**。

    ⚠ 与 projectile_sdf 的区别：
      · 这里 `static=true` ⇒ 不会被物理推走，也不需要 `<inertial>`
      · 无 collision ⇒ 不参与碰撞
    """
    r, g, b = color
    return f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="{name}">
    <static>true</static>
    <link name="link">
      <visual name="visual">
        <geometry><sphere><radius>{radius}</radius></sphere></geometry>
        <material>
          <ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient>
          <diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>
          <emissive>{min(r * 1.0, 1.0):.3f} {g * 0.4:.3f} {b * 0.2:.3f} 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>"""


def movable_ball_sdf(name: str, radius: float,
                     color=(0.20, 0.95, 0.35)) -> str:
    """★ **可被 set_pose 移动**的小球（回放里代表无人机）。

    为什么需要它（2026-10-17 审查发现）：
      回放里"无人机位置"原来只用 `marker_move()` 画，而 `/marker` 在用户环境
      **不可用** ⇒ 降级成"珠子"模式后，弹丸小球会动、**无人机小球压根不存在**，
      但控制台还在打印"绿 小球 = 无人机重放位置" ⇒ 用户以为回放又坏了。
    ⇒ 这里用一个真实实体当"无人机替身"，只依赖 create + set_pose。

    结构与 `projectile_sdf` 同构（dynamic + 关重力 + 无 collision）：
      · 不是 static ⇒ set_pose 一定能推动它（这条已在实机上验证过）；
      · 关重力、无碰撞 ⇒ 物理引擎不会把它推走。
    """
    r, g, b = color
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
          <ambient>{r:.3f} {g:.3f} {b:.3f} 1</ambient>
          <diffuse>{r:.3f} {g:.3f} {b:.3f} 1</diffuse>
          <emissive>{r * 0.4:.3f} {g * 1.0:.3f} {b * 0.4:.3f} 1</emissive>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


def projectile_sdf(name: str, radius: float) -> str:
    """弹丸：小球。

    ★★ 保持 `static=false`（2026-10-17 实测确认这个结构**能加载**）。
      用 `--list-entities` 实测：`enemy_dart (+0.00, +0.00, -500.00)` 确实存在于世界里
      ⇒ **SDF 没问题**。之前"看不见"的真正原因是它的 z 被放在 -500（藏在视野外），
         以及 GUIDED 段只有 0.5 s。
      ⇒ 所以**不要**改成 static（那会引入"set_pose 是否仍有效"的新不确定性）。

    ⚠ 关键设计（原来就对，不要动）：
        `static=false` + **`<gravity>false</gravity>`** + **无 collision**：
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
        # ★ 服务调用**串行化**（2026-10-17 审查新增）
        #   同一个 `gz.transport.Node` 会被两个线程用：
        #     · 主线程：create / set_pose（gz_setup / gz_finish 的兜底路径）
        #     · 后台线程：每帧 set_pose（_worker）
        #   pybind 在阻塞请求期间会放开 GIL ⇒ 两个 `request()` 可能真的**同时在飞**。
        #   gz-transport 内部按 id 配对响应，功能上大多不炸，但顺序得不到保证
        #   （例如"先 create 再 set_pose"可能被重排 ⇒ 报"实体不存在"）。
        #   ⇒ 所有服务请求走同一把锁，代价只是后台线程多等几十 ms（它不参与控制）。
        self._req_lock = threading.Lock()
        self._pending = None            # 待写入的弹丸位置（ENU）
        self._actions = []              # ★ 待执行的"一次性绘制"动作（非阻塞队列）
        self._thread = None
        self._running = False
        self._warned = False            # 只对"发不出去"告警一次，不刷屏
        self._created = set()           # 已创建的实体名
        self.stats = {"set_pose": 0, "failed": 0, "actions": 0, "reused": 0}
        self._snap = None               # 世界真实状态缓存（_ensure 判断"已存在否"）
        self._last_err = None           # 最近一次服务请求失败的原因（错误提示用）
        # ★ pose/info 只订阅**一次**（原来是"每次快照都 subscribe"⇒ 订阅泄漏，
        #   每次调用都多一个回调，pose/info 又是高频话题 ⇒ 越跑越慢）
        self._pose_sub = None
        self._pose_latest = None        # 最近一帧 {名字: (x,y,z)}
        self._pose_hits = 0             # 收到的消息条数（判"有没有新帧"）

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

    # ------------------------------------------------------ 读回世界真实状态
    def _ensure_pose_sub(self):
        """★ 建立**唯一**一个 `/pose/info` 订阅（幂等）。

        原来是"每调一次 `snapshot_positions` 就 subscribe 一次、从不退订"：
          · 同一个进程里调 3 次 ⇒ 3 个回调，每条 pose/info 消息算 3 遍；
          · `pose/info` 是高频话题（几十 Hz × 世界上所有实体）⇒ 越跑越卡，
            而且旧回调里的 `out["d"]` 是**已经返回给调用方**的临时字典（纯浪费）。
        现在改成：订阅一次、回调只刷新 `self._pose_latest`，快照直接读缓存。
        """
        if self._pose_sub is not None or not self.available:
            return self._pose_sub
        try:
            from gz.msgs10 import pose_v_pb2

            def _cb(msg):
                try:
                    self._pose_latest = {pz.name: (pz.position.x, pz.position.y,
                                                   pz.position.z)
                                         for pz in msg.pose}
                    self._pose_hits += 1
                except Exception:                      # noqa: BLE001
                    pass

            self._pose_sub = self._node.subscribe(
                pose_v_pb2.Pose_V, f"/world/{self.world}/pose/info", _cb)
        except Exception as e:                                  # noqa: BLE001
            self._log(f"   ⚠ 订阅 /pose/info 失败（{e}）⇒ 无法读回世界状态")
            self._pose_sub = None
        return self._pose_sub

    def snapshot_positions(self, timeout: float = 3.0, fresh: bool = True) -> dict:
        """★ 读回 `/world/<w>/pose/info`，返回 `{名字: (x,y,z)}`（一次性快照）。

        ★ 为什么这是**唯一可信**的验证手段（2026-10-17 三次踩坑的总结）：
          · `create` 返回 True ≠ 模型真的在世界里
          · `set_pose` 返回 True ≠ 模型真的动了
          ⇒ 只有读回世界的真实状态才算数。
        失败时返回 `{}`（调用方应把"空字典"当作"读不到"，不要当作"世界是空的"）。

        `fresh=True` 时**等一帧新的**再说（默认）：否则可能拿到上一拍的缓存，
        刚 create 完就查会误判"世界里没有它"。
        """
        if not self.available or self._ensure_pose_sub() is None:
            return {}
        t0 = self._pose_hits
        deadline = time.monotonic() + max(0.05, float(timeout))
        while fresh and self._pose_hits == t0 and time.monotonic() < deadline:
            time.sleep(0.02)
        return dict(self._pose_latest) if self._pose_latest else {}

    def pose_of(self, name: str, timeout: float = 1.0):
        """单个实体的真实位置；读不到返回 None。"""
        snap = self.snapshot_positions(timeout=timeout, fresh=False)
        return snap.get(name)

    # ------------------------------------------------------------------ 基础
    def _log(self, s: str):
        if self.verbose:
            print(s, flush=True)

    def _req(self, service: str, msg, rep_type, timeout_ms=None):
        """发一个服务请求。返回 (ok, response)；异常一律吞掉并计数。

        `timeout_ms` 可**按调用点覆盖**：建实体（解析 SDF + 插入世界）比
        set_pose 慢得多，用同一个短超时会误判成失败（见 `_create` 注释）。

        ⚠ **`ok` 只代表"服务响应了"**，不代表操作成功 —— 响应体里的 `data`
          才是结果。判断成败要用 `ok and rep.data`（`_create` / `_do_set_pose` 已这么做）。
        """
        if not self.available:
            self._last_err = "gz.transport 不可用"
            return False, None
        try:
            # ★ 加锁：主线程与后台线程共用同一个 gz Node（已发生过的坑见 __init__）
            with self._req_lock:
                ok, rep = self._node.request(service, msg, type(msg), rep_type,
                                            int(timeout_ms or self.timeout_ms))
            self._last_err = None if ok else "超时/无响应"
            return bool(ok), rep
        except Exception as e:                          # noqa: BLE001
            self.stats["failed"] += 1
            self._last_err = f"{type(e).__name__}: {e}"
            return False, None

    def warmup(self, timeout_ms: int = CREATE_TIMEOUT_MS) -> bool:
        """★ **开场预热**：做一次无害的服务往返，把发现/握手做掉。

        为什么需要（2026-10-17 实跑实测）：
          进程内**第一条**服务请求要花时间做服务发现，1 s 的默认超时不够
          ⇒ 表现为"开场两次 set_pose 各超时 1 s，主回路空洞 2028 ms"，
            而同一进程后面 30 Hz 的 set_pose 全部成功。
          ⇒ 先把连接养起来，个别请求的超时就不会算到"摆场景"头上。

        实现：给 `set_pose` 发一个**不存在的实体名**（`PROBE_POSE_NAME`）
        ⇒ 服务会立刻回一个 `data=False`，**不动任何东西**，只看"有没有响应"。
        """
        if not self.available:
            return False
        ok, _ = self._warmup_send(timeout_ms)
        self._log(f"   gz 服务预热：{'✅ 已连通' if ok else '⚠ 无响应（继续尝试）'}")
        return ok

    def _warmup_send(self, timeout_ms: int):
        """预热请求本体（唯一碰 protobuf 的地方 ⇒ 单测里替换它）。"""
        req = pose_pb2.Pose()
        req.name = PROBE_POSE_NAME
        req.orientation.w = 1.0
        return self._req(self._svc_set_pose, req, boolean_pb2.Boolean,
                         timeout_ms=timeout_ms)

    def _probe_pose_req(self):
        """`--check` 的 set_pose 探针消息：**虚构实体名**（不动任何真实体）。

        ⚠ 与 `_warmup_send` 里的消息同一个名字 —— 两处都必须用 `PROBE_POSE_NAME`。
        """
        req = pose_pb2.Pose()
        req.name = PROBE_POSE_NAME
        req.position.x = req.position.y = req.position.z = 0.0
        req.orientation.w = 1.0
        return req

    # ------------------------------------------- 轨迹"珠子"（无 marker 时的兜底）
    def ensure_beads(self, points, prefix: str, radius: float = 0.09,
                     color=(1.0, 0.35, 0.05), stride: int = 2,
                     max_beads: int = 40) -> int:
        """沿轨迹摆一串**静态小球实体**，返回实际创建的数量（0 = 失败）。

        · `stride`：每隔几个点放一个（2 ⇒ 虚线感，实体数减半，创建更快）
        · `max_beads`：上限，避免长轨迹创建几百个实体拖慢回放
        · 只依赖 `create` 服务 ⇒ 不需要 GUI 的 MarkerManager 插件
        """
        if not points or not self.available:
            return 0
        pts = [p for p in points if p is not None]
        if len(pts) < 2:
            return 0
        idx = list(range(0, len(pts), max(1, int(stride))))
        if idx and idx[-1] != len(pts) - 1:
            idx.append(len(pts) - 1)
        if len(idx) > max_beads:                      # 均匀抽稀
            step = len(idx) / float(max_beads)
            idx = [idx[int(i * step)] for i in range(max_beads - 1)] + [idx[-1]]
        ok_n = 0
        for k, i in enumerate(idx):
            nm = f"{prefix}_{k}"
            p = pts[i]
            # ⚠ `pre_remove=False`：调用方（回放）**已经**先 `clear_beads()` 清过场了。
            #   原来每个珠子都走"先删后建" ⇒ 40 颗珠子要发 80 次服务请求
            #   （40 删 + 40 建，每次都是阻塞调用）⇒ 回放开场要等十几秒甚至更久，
            #   用户看到的就是"提示说回放已启动，画面半天没动静"。
            if self._create(nm, bead_sdf(nm, radius, color),
                            (float(p[0]), float(p[1]), float(p[2])),
                            pre_remove=False):
                ok_n += 1
        return ok_n

    def clear_beads(self, prefix: str, max_beads: int = 40) -> int:
        """删掉 `ensure_beads` 创建的那串珠子（按名字逐个删）。

        ⚠ **无条件尝试删**（不看 self._created）：上一次回放是**另一个进程**创建的，
          它的名字不在本进程的 `_created` 里；只按 `_created` 过滤会留下旧珠子。
          删不存在的实体只是返回 False，无害。
        """
        n = 0
        for k in range(max_beads):
            if self._remove(f"{prefix}_{k}"):
                n += 1
        return n

    def ensure_hit_bead(self, p, radius: float = 0.22,
                        color=(1.0, 0.95, 0.10)) -> bool:
        """用**实体**标命中点（`marker` 不可用时的替代）。"""
        if p is None or not self.available:
            return False
        return self._create("replay_hit_bead", bead_sdf("replay_hit_bead",
                                                        radius, color),
                            (float(p[0]), float(p[1]), float(p[2])))

    def marker_available(self) -> bool:
        """`/marker` 是否可用（用一次无害的 DELETE_ALL 探测）。"""
        if not self.available:
            return False
        try:
            m = marker_pb2.Marker()
            m.ns = "__pgu_probe_ns__"
            m.action = _resolve_enum(marker_pb2.Marker, "Marker.Action",
                                     MARKER_ACTION_DELETE_ALL)
            ok, _ = self._req(SVC_MARKER, m, boolean_pb2.Boolean)
            return bool(ok)
        except Exception:                               # noqa: BLE001
            return False

    # ------------------------------------------------------------ 服务自检
    def probe_services(self) -> dict:
        """★ 逐个探测本模块用到的 gz 服务是否**真的可用**，返回 {名字: (ok, 说明)}。

        为什么需要它（2026-10-17 实飞踩到）：
          回放"启动了但画面没东西"。而轨迹线 / 命中点全靠 `/marker` 服务，
          它由 **Gazebo GUI 的 MarkerManager 插件**提供 —— 如果 GUI 没起、
          或插件没加载，`_req` 只会**静默返回 False**（异常被吞、只计数），
          于是"看起来在放、其实什么都没画"。
          ⇒ 必须有一个能直接回答"这个服务到底通不通"的工具。

        探测方式：用一个**无害**的请求打一下，只看有没有响应
        （marker 用 DELETE_ALL 删一个不存在的 ns，不影响任何东西）。
        """
        res = {}
        if not self.available:
            return {"_transport": (False, "gz.transport 初始化失败：" +
                                   str(_GZ_IMPORT_ERROR or "未知"))}

        # ① 实体创建服务
        try:
            req = entity_factory_pb2.EntityFactory()
            req.sdf = "<sdf version='1.9'><model name='__probe__'/></sdf>"
            req.name = "__pgu_probe__"
            req.allow_renaming = True
            ok, _ = self._req(self._svc_create, req, boolean_pb2.Boolean)
            res["create"] = (ok, "建实体（塔/发射架/弹丸）")
        except Exception as e:                              # noqa: BLE001
            res["create"] = (False, f"异常：{e}")
        # 顺手删掉探针实体
        try:
            ent = entity_pb2.Entity()
            ent.name = "__pgu_probe__"
            self._req(self._svc_remove, ent, boolean_pb2.Boolean)
        except Exception:                                   # noqa: BLE001
            pass

        # ② 移动实体服务（弹丸飞行、回放靠它）
        try:
            # ★★ 2026-10-17 实跑发现：这里原来用 `PROJECTILE_NAME` 探测，
            #   而 `Pose` 的 position 默认是 (0,0,0) ⇒ **`--check` 会把弹丸搬到世界原点**，
            #   而它自称"只读诊断"。实测后果：跑飞行时弹丸 `drift=14.00 m`
            #   （在原点，正好在塔基里），然后 set_pose 又超时 ⇒ 球就留在塔里了。
            #   ⇒ 改成给一个**不存在的名字**：gz 会立刻回 `data=False`，
            #     我们只看"有没有响应"来判断服务是否可用，**不动任何实体**。
            p = self._probe_pose_req()
            ok, _ = self._req(self._svc_set_pose, p, boolean_pb2.Boolean,
                              timeout_ms=SET_POSE_TIMEOUT_MS)
            res["set_pose"] = (ok, f"移动实体（弹丸 / 回放小球）"
                                   f"{'' if ok else ' ← 弹丸不会动！'}")
        except Exception as e:                              # noqa: BLE001
            res["set_pose"] = (False, f"异常：{e}")

        # ③ marker 服务（轨迹线 / 命中点 —— 需要 GUI 插件）
        try:
            m = marker_pb2.Marker()
            m.ns = "__pgu_probe_ns__"
            m.action = _resolve_enum(marker_pb2.Marker, "Marker.Action",
                                     MARKER_ACTION_DELETE_ALL)
            ok, _ = self._req(SVC_MARKER, m, boolean_pb2.Boolean)
            res["marker"] = (ok, "画轨迹线 / 命中点（需 Gazebo **GUI** 的 "
                                 "MarkerManager 插件）"
                                 + ("" if ok else " ← 轨迹线和黄球都不会出现！"))
        except Exception as e:                              # noqa: BLE001
            res["marker"] = (False, f"异常：{e}")

        return res

    def _create(self, name: str, sdf: str, pose=(0.0, 0.0, 0.0),
                pre_remove: bool = True) -> bool:
        """创建一个实体。**同名先删**（避免 allow_renaming 引发重复名）。

        ★★ 两次实测教训（2026-10-17）：

        ① **超时太短会假失败**：本类的默认 `timeout_ms=500`，但"建实体"
           要先让 gz 解析 SDF 再插入世界，**很容易超过 500 ms**
           （尤其同一次 run 里建第 2、3 个实体时）。
           ⇒ 症状正是用户看到的：**塔建成功、发射架和弹丸双双失败**。
           ⇒ 现在建实体单独用 `CREATE_TIMEOUT_MS`（3000 ms），并**失败重试 1 次**。

        ② **"先删后建"是破坏性的**：`create` 失败时旧实体已被删掉 ⇒ 模型凭空消失。
           ⇒ 失败时必须把 SDF 打出来 + 指向 Gazebo 终端的报错，不能静默。
        """
        if pre_remove:
            self._remove(name)                       # 忽略失败

        # ★ 重试/超时/报错的核心逻辑抽成 `_create_with_retry`，
        #   它**不碰任何 protobuf** ⇒ 无 gz 环境也能单测（用例 19 就是这么测的）。
        def _attempt():
            req = entity_factory_pb2.EntityFactory()
            req.sdf = sdf
            req.name = name
            req.pose.position.x = float(pose[0])
            req.pose.position.y = float(pose[1])
            req.pose.position.z = float(pose[2])
            req.pose.orientation.w = 1.0
            ok, rep = self._req(self._svc_create, req, boolean_pb2.Boolean,
                                timeout_ms=CREATE_TIMEOUT_MS)
            return bool(ok and rep is not None and rep.data), rep

        return self._create_with_retry(name, sdf, _attempt)

    def _create_with_retry(self, name: str, sdf: str, attempt_fn,
                           attempts: int = 2) -> bool:
        """`_create` 的重试/报错内核。**纯逻辑，不依赖 protobuf / gz。**

        `attempt_fn()` 返回 `(成功?, 响应)`；失败会重试，最终失败时把 SDF 打出来。
        抽出来的目的：这一层是"超时假失败 + 先删后建把模型弄没了"的关键，
        必须能被离线单测覆盖（本机没有 Gazebo）。
        """
        ok = False
        rep = None
        for i in range(max(1, attempts)):
            ok, rep = attempt_fn()
            if ok:
                self._created.add(name)
                return True
            if i + 1 < max(1, attempts):
                self._log(f"⚠ 创建 `{name}` 第 {i + 1} 次失败（ok={ok}, "
                          f"data={None if rep is None else getattr(rep, 'data', None)}）"
                          f"⇒ 可能是超时，重试…")
                time.sleep(0.2)
        # ★ 失败必须**说清楚是哪一步、SDF 长什么样**（原来是静默 + 只告警一次）
        self.stats["failed"] = self.stats.get("failed", 0) + 1
        self._log(f"⚠ 创建实体 `{name}` **失败**（重试后仍失败："
                  f"ok={ok}, data={None if rep is None else getattr(rep, 'data', None)}）")
        self._log("   ⇒ 若该实体原本存在，它现在已被删掉了（先删后建）。")
        self._log("   ⇒ 请到 **Gazebo 自己的终端**看紧接着的报错"
                  "（通常含 `Error` / `SDF` / `Unable to find`）。")
        self._log("   ⇒ 本次用到的 SDF：")
        for _ln in str(sdf).splitlines():
            if _ln.strip():
                self._log("       " + _ln.rstrip())
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

    def _do_set_pose(self, name: str, p, yaw: float = 0.0,
                     timeout_ms=None, retries: int = 0) -> bool:
        """把实体移动到 `p`（ENU）。

        ★★ 2026-10-17 实跑后补强（原来只管发出去，不管真的成功）：
          ① **必须看响应体 `data`**：`ok=True` 只说明"服务回了话"，
             `data=False`（实体名不对 / 找不到）原来被当成**成功**；
          ② **超时可覆盖 + 可重试**：开场第一次请求要多花时间做发现，
             1 s 不够（实测两次各超时 1 s ⇒ 2028 ms 空洞、发射架和弹丸留在旧位置）；
          ③ 失败时把**具体原因**（超时 / 响应 False / 异常）打出来
             —— 原来只有一句"请确认仿真在跑"，对排查没有帮助。
        重试只给"摆场景"这类一次性调用用；后台 30 Hz 逐帧同步不重试
        （否则失败时会累积背压，把位置同步饿死）。
        """
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

        def _send():
            ok, rep = self._req(self._svc_set_pose, req, boolean_pb2.Boolean,
                                timeout_ms=timeout_ms)
            return ok, rep

        # 重试/判定内核抽成纯逻辑（不碰 protobuf）⇒ 无 gz 也能单测（用例 21）
        return self._set_pose_with_retry(name, _send, retries=retries)

    def _set_pose_with_retry(self, name: str, send_fn, retries: int = 0) -> bool:
        """`_do_set_pose` 的**重试 + 成功判定**内核（纯逻辑，不依赖 protobuf/gz）。

        `send_fn()` 返回 `(收到了响应?, 响应对象)`。成功条件 = 收到响应 **且**
        `response.data` 为 True。

        ★ 为什么要单独抽出来（2026-10-17 实跑）：
          ① `ok=True` 只代表"服务回了话"，`data=False`（实体名找不到）
             原来被当成**成功**；
          ② 开场第一次请求要多花时间做发现，1 s 不够（两次各超时 1 s ⇒
             2028 ms 空洞 + 发射架/弹丸留在旧位置）；
          ③ 这一层是"成败判定"的关键，必须能离线单测（本机没有 Gazebo）。
        """
        ok = False
        reason = ""
        n = max(1, 1 + int(retries))
        for i in range(n):
            ok, rep = send_fn()
            if ok and rep is not None and bool(getattr(rep, "data", False)):
                self.stats["set_pose"] += 1
                return True
            ok = False
            if rep is not None and getattr(rep, "data", None) is False:
                reason = "服务回了 data=False（实体名找不到？）"
                break                      # 名字问题重试也没用
            reason = f"无响应/超时（{self._last_err or '未知'}）"
            if i + 1 < n:
                self._log(f"⚠ set_pose(`{name}`) 第 {i + 1} 次失败（{reason}）⇒ 重试…")
                time.sleep(0.15)
        self.stats["failed"] += 1
        if not self._warned:
            self._warned = True
            self._log(f"⚠ set_pose 失败：{reason}"
                      f" ⇒ 请确认仿真在跑、世界名是 '{self.world}'"
                      f"（当前启动命令见 README §1）")
        return False

    # ------------------------------------------------------------ 场景元素
    def set_rtf(self, factor: float) -> bool:
        """设置 Gazebo 的**实时率**（`real_time_factor`）—— 即"慢放"。

        `factor = 1.0` 实时；`0.2` = 5 倍慢放；`0.0` = 不限速（按物理步长全速跑）。

        ⚠⚠ **重要警告**：这会同时放慢**墙钟**上的一切，包括我们的控制节点定时器。
             我们的超时判据（`--cruise-timeout` / `--brake-timeout` / `--land-timeout` /
             OFFBOARD 确认超时…）都是**墙钟秒**，所以 RTF 调低后它们会**相对仿真时间变早**
             （RTF=0.2 时，8 s 的巡航超时只对应仿真 1.6 s ⇒ 会提前 abort）。
             ⇒ 只建议在"无人机已进入巡航/制导段、只想看画面"时手动临时调低，
               而且**不要低于 0.3**。默认请保持 1.0。
             报告里的 `[实时率]` 一行会记录实际 RTF，方便事后判断结果是否受影响。
        """
        if not self.available:
            return False
        if factor < 0.0:
            raise ValueError("RTF 不能为负")
        req = physics_pb2.Physics()
        req.real_time_factor = float(factor)
        ok, _ = self._req(self._svc_set_physics, req, boolean_pb2.Boolean)
        if ok:
            self._log(f"✅ 已设置 Gazebo 实时率 RTF={factor:g}"
                      + ("（实时）" if abs(factor - 1.0) < 1e-9
                         else f"（{1.0 / factor:.1f} 倍慢放）" if factor > 0
                         else "（不限速）"))
            self._log("   ⚠ 提醒：RTF≠1 时**墙钟超时判据会偏早**（见 --set-rtf 文档）")
        else:
            self._log("⚠ 设置实时率失败（Gazebo 在跑吗？世界名对吗？）")
        return ok

    def marker_move(self, p, ns: str = UAV_REPLAY_NS, mid: int = UAV_REPLAY_ID,
                    size: float = 0.35, color=(0.20, 0.95, 0.35),
                    ttl_ms: int = 0) -> bool:
        """把一个球 marker **移动到**新位置（同 ns+id 的 ADD_MODIFY 会更新它）。

        比 `mark_point` 多两个用途：颜色可配、`ttl_ms=0` 表示不过期
        （回放要长时间显示，不能 8 秒后自己消失）。
        """
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
        if ttl_ms > 0:
            m.lifetime.sec = int(ttl_ms // 1000)
            m.lifetime.nsec = int((ttl_ms % 1000) * 1_000_000)
        m.material.ambient.r, m.material.ambient.g, m.material.ambient.b = color
        m.material.ambient.a = 1.0
        m.material.emissive.r, m.material.emissive.g, m.material.emissive.b = color
        m.material.emissive.a = 1.0
        ok, _ = self._req(SVC_MARKER, m, boolean_pb2.Boolean)
        return ok

    def marker_delete(self, ns: str, mid: int = 0) -> bool:
        """删掉**某个 ns 下的全部** marker（按 `DELETE_ALL`）。

        ⚠ 原名/文档写的是"按 ns+id 删单个"，但实现一直是 `DELETE_ALL`
          （2026-10-17 审查发现文档与行为不符）。这里保留 `DELETE_ALL` 行为
          —— 回放开场要清的就是整条轨迹/整串无人机标记，
          但**把文档改对**，`mid` 仅作兼容保留，不参与过滤。
        """
        m = marker_pb2.Marker()
        m.ns = ns
        m.id = int(mid)
        # DELETE_ALL：删该 ns 下全部。gz 的 DELETE_MARKER(1) 只删单个 id，
        # 而回放里一条轨迹/一串标记会用多个 id ⇒ 必须用 DELETE_ALL。
        m.action = _resolve_enum(marker_pb2.Marker, "Marker.Action",
                                 MARKER_ACTION_DELETE_ALL)
        ok, _ = self._req(SVC_MARKER, m, boolean_pb2.Boolean)
        return ok

    # ------------------------------------------- 幂等"确保存在"（绝不先删）
    def _ensure(self, name: str, sdf: str, pose, tol: float = 0.10,
                recreate_if_stuck: bool = False) -> bool:
        """★ **幂等**：已存在就复用，不存在才创建；**绝不"先删后建"**。

        为什么必须这样（2026-10-17 用户实测"起飞后球和发射塔又没有了"）：
          `gz_setup()` 是在**起飞完成**（`[5/6] 稳定到位`）时才调用的。
          原来三个 `ensure_*` 都走 `_create`，而 `_create` 第一步是 `_remove(name)`
          ⇒ 等于**起飞后把已有模型删掉再重建**。只要重建失败（超时/任何原因），
            模型就当场消失 —— 而且恰好是"起飞后"才消失，非常难查。
          ⇒ 现在：先读回世界状态，
             · 已存在且位置对 ⇒ **什么都不做**（返回 True）
             · 已存在但位置不对 ⇒ **只 set_pose**（移动，不重建）
             · 不存在 ⇒ 才创建
        """
        snap = self._snap if self._snap is not None else {}
        got = snap.get(name)
        if got is not None:
            drift = max(abs(got[k] - float(pose[k])) for k in range(3))
            if drift <= tol:
                self._created.add(name)
                self.stats["reused"] = self.stats.get("reused", 0) + 1
                self._log(f"   ✓ `{name}` 已存在（drift={drift:.2f} m）⇒ 复用，未重建")
                return True
            # 存在但位置不对 ⇒ 只移动，**不删**
            #   ★ 用长超时 + 重试（开场第一次请求要多花时间做发现，见 SET_POSE_TIMEOUT_MS）
            ok = self._do_set_pose(name, pose, timeout_ms=SET_POSE_TIMEOUT_MS,
                                   retries=1)
            self.stats["reused"] = self.stats.get("reused", 0) + 1
            self._log(f"   ✓ `{name}` 已存在但偏了 {drift:.2f} m ⇒ **只移动**"
                      f"（不重建）{'成功' if ok else '⚠ 移动失败'}")
            if ok:
                return True
            # ★★ 移动失败 + 偏得离谱 ⇒ **重建**（2026-10-17 实跑新增）
            #   实测场景：`--check` 把弹丸搬到原点、上一轮的发射架停在旧发射点，
            #   而 set_pose 又超时 ⇒ 实体留在**错误位置**，画面上"模型在但位置不对"，
            #   比"模型消失"更难判断。
            #   ⚠ 只有调用方明确允许时才重建（`recreate_if_stuck=True`），且**只在
            #     地面段调用**（节点在 PRECHECK 里摆场景）⇒ 删了再建失败也不会影响飞行。
            #     回放里的"无人机替身"**不**走这条路（它本来就每帧在动）。
            if recreate_if_stuck:
                self._log(f"   ⇒ 移动失败且偏了 {drift:.2f} m ⇒ **重建**"
                          f"（此刻在地面，安全；重建失败的话模型会消失，日志会说明）")
                return self._create(name, sdf, pose, pre_remove=True)
            return False
        # 不存在 ⇒ 创建（pre_remove=False：既然不存在，没必要再删一次）
        ok = self._create(name, sdf, pose, pre_remove=False)
        if not ok:
            # ★ 2026-10-17 审查：`snapshot_positions` 读不到（仿真刚起 / 话题还没发）
            #   时 `_snap` 是空字典 ⇒ 这里会走"创建"，而实体其实**已经存在**
            #   ⇒ create 因重名失败，日志报"创建失败"，但模型好好的在场上。
            #   用一次 set_pose 做**探针**：能推动 ⇒ 实体确实存在，按"已就位"处理。
            #   （set_pose 打到不存在的名字会返回 False，所以这个探针是可靠的）
            if self._do_set_pose(name, pose):
                self._created.add(name)
                self.stats["reused"] = self.stats.get("reused", 0) + 1
                self._log(f"   ✓ `{name}` 创建报错但 set_pose 成功 ⇒ "
                          f"判定为**已存在**（按已就位处理，未重建）")
                return True
        return ok

    def refresh_snapshot(self, timeout: float = 1.5) -> dict:
        """刷新"世界真实状态"缓存（供 `_ensure` 判断"已存在否"）。

        ⚠ 读不到时返回 `{}` 并**记录一次告警**：此时 `_ensure` 会退化为"直接创建"，
          但**仍然不先删**（`pre_remove=False`）⇒ 最坏情况是"重名创建失败"，
          也不会把已有模型弄没。
        """
        snap = self.snapshot_positions(timeout=timeout)
        self._snap = snap
        if not snap:
            self._log("   ⚠ 读不到 /pose/info ⇒ 无法判断实体是否已存在，"
                      "将直接创建（不删旧的）")
        return snap

    def ensure_tower(self, radius: float = 1.2, height: float = 3.0,
                     center=(0.0, 0.0)) -> bool:
        """**确保**防御塔存在（幂等）。塔底贴地 ⇒ 圆柱几何中心在 (cx, cy, height/2)。"""
        pose = (center[0], center[1], height * 0.5)
        return self._ensure(TOWER_NAME, tower_sdf(TOWER_NAME, radius, height), pose,
                            recreate_if_stuck=True)

    def ensure_launcher(self, p=(0.0, 0.0, 0.0), size: float = 0.6) -> bool:
        """**确保**敌方发射架存在（幂等）。"""
        return self._ensure(LAUNCHER_NAME, launcher_sdf(LAUNCHER_NAME, size),
                            (p[0], p[1], size * 0.5), recreate_if_stuck=True)

    def ensure_projectile(self, radius: float = 0.06, at=None) -> bool:
        """**确保**弹丸存在（幂等；已存在就只移动，不重建）。

        `at` 给定时放到该位置（推荐），否则藏在视野外（HIDDEN_Z）。

        ★ 为什么推荐传 `at`（2026-10-17）：
          旧流程是"创建在 HIDDEN_Z → 再 set_pose 移到发射点"，**两步都可能静默失败**。
          直接建在目标位置 ⇒ 少一层依赖：即使 `set_pose` 有问题，
          球一开始就在能看见的地方。
        ★ 为什么改幂等（同一天）：
          `gz_setup` 在**起飞后**才调用；原来"先删后建"等于起飞后把球删了再重建，
          一失败球就消失。现在已存在就**只移动不重建**。
        """
        pose = (0.0, 0.0, HIDDEN_Z) if at is None else (float(at[0]),
                                                        float(at[1]),
                                                        float(at[2]))
        return self._ensure(PROJECTILE_NAME,
                            projectile_sdf(PROJECTILE_NAME, radius), pose,
                            tol=0.30,      # 球常被 park/飞行移动过，容差放宽
                            recreate_if_stuck=True)

    def hide_projectile(self) -> None:
        """把弹丸移出视野（等价于"消失"），命中/脱靶后调用。"""
        self._do_set_pose(PROJECTILE_NAME, (0.0, 0.0, HIDDEN_Z))

    def set_projectile(self, p, yaw: float = 0.0) -> None:
        """更新弹丸位置（ENU, m）。**非阻塞** —— 只写缓存，后台线程负责发送。"""
        with self._lock:
            self._pending = (float(p[0]), float(p[1]), float(p[2]), float(yaw))

    # ---------------------------------------------------------------- 轨迹
    # ★★ 非阻塞队列（2026-10-16 实飞踩到）
    #   背景：`draw_trail` / `mark_point` / `hide_projectile` 都是**阻塞**服务调用，
    #         超时就 500 ms（`timeout_ms`）。实飞日志实测 `gz_finish` 里两次调用
    #         **各花 0.50 s ⇒ 主回路被卡了整整 1.0 s**：
    #             624.414 已画弹丸轨迹 / 624.914 已标记命中点（差 0.50 s）
    #         而这 1 s 内 50 Hz 的加速度 setpoint **一条都没发出去**，
    #         正好落在命中瞬间 ⇒ 违反本项目"可视化绝不允许影响制导回路"的硬规矩。
    #   ⇒ 收尾绘制一律走 `queue_*`：主回路只入队（微秒级），后台线程负责真正发送。
    def _enqueue(self, kind: str, payload) -> bool:
        """把一个一次性绘制动作排进后台队列（**立即返回**，不阻塞调用者）。"""
        if not self.available:
            return False
        with self._lock:
            self._actions.append((kind, payload))
        return True

    def queue_trail(self, points, ns: str = TRAIL_NS, width: float = 0.04,
                    color=(1.0, 0.35, 0.05)) -> bool:
        """非阻塞版 `draw_trail`。"""
        pts = [(float(p[0]), float(p[1]), float(p[2]))
               for p in points if p is not None]
        if len(pts) < 2:
            return False
        return self._enqueue("trail", (pts, ns, float(width), tuple(color)))

    def queue_mark(self, p, ns: str = "pilot_mark", mid: int = 0,
                   size: float = 0.5, color=(1.0, 1.0, 0.0),
                   ttl_ms: int = 8000) -> bool:
        """非阻塞版 `mark_point`。"""
        return self._enqueue("mark", ((float(p[0]), float(p[1]), float(p[2])),
                                      ns, int(mid), float(size), tuple(color),
                                      int(ttl_ms)))

    def queue_hide_projectile(self) -> bool:
        """非阻塞版 `hide_projectile`。"""
        return self._enqueue("hide", None)

    def queue_clear_trail(self) -> bool:
        """非阻塞版 `clear_trail`。"""
        return self._enqueue("clear_trail", None)

    def _run_action(self, kind: str, payload) -> None:
        if kind == "trail":
            pts, ns, width, color = payload
            self.draw_trail(pts, ns=ns, width=width, color=color)
        elif kind == "mark":
            p, ns, mid, size, color, ttl = payload
            self.mark_point(p, ns=ns, mid=mid, size=size, color=color, ttl_ms=ttl)
        elif kind == "hide":
            self.hide_projectile()
        elif kind == "clear_trail":
            self.clear_trail()
        self.stats["actions"] += 1

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
        # ⚠ 原来这里直接写 `marker_pb2.Marker.DELETE_ALL`（绕过 _resolve_enum）
        #   ⇒ 换 gz-msgs 版本时不会给出"可用名列表"的友好报错，只会 AttributeError。
        m.action = _resolve_enum(marker_pb2.Marker, "Marker.Action",
                                 MARKER_ACTION_DELETE_ALL)
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
                # 每周期最多取几个动作：服务调用可能各花 100~500 ms，
                # 取太多会让弹丸位置同步饿死（位置同步是"看起来连不连贯"的关键）
                acts = self._actions[:4]
                del self._actions[:len(acts)]
            if pend is not None:
                # ★ 逐帧同步用短超时（见 FRAME_SETPOSE_TIMEOUT_MS）：失败只丢一帧，
                #   绝不为了这一帧把主回路卡 1 s。`retries=0` —— 下一帧会自然重试。
                self._do_set_pose(PROJECTILE_NAME, pend[:3], pend[3],
                                  timeout_ms=FRAME_SETPOSE_TIMEOUT_MS)
            for kind, payload in acts:
                try:
                    self._run_action(kind, payload)
                except Exception:                      # noqa: BLE001
                    self.stats["failed"] += 1
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


def draw_from_json_file(path: str, world: str = DEFAULT_WORLD,
                        verbose: bool = False) -> int:
    """★ 一次性绘制（子进程模式）：读 JSON → 画轨迹 / 标记命中点 / 隐藏弹丸。

    ⚠ 为什么需要"子进程"这条路（2026-10-16 实飞事故）：
      `gz.transport` 的服务请求是**阻塞**的，且它和我们的控制节点共用同一个
      Python 解释器 ⇒ 后台线程里发请求同样会**抢 GIL**，主回路照样被卡住。
      实测：命中瞬间主回路空洞 **1015 ms**，而那 1 s 里 PX4 因为收不到 offboard
      setpoint 而失控，无人机**自由落体 6.9 m 并横向漂 5.8 m** ⇒ 最终落点漂 12.9 m。
      ⇒ 唯一可靠的办法是让绘制发生在**另一个进程**里：`subprocess.Popen` 立刻返回，
        子进程自己去连 gz、画图、退出，永远不碰控制节点的 GIL。

    JSON 格式：
      {"world": "walls",
       "trail": [[x,y,z], ...],          # 可选，≥2 点
       "trail_color": [r,g,b],           # 可选
       "mark": [x,y,z] | null,           # 可选
       "mark_color": [r,g,b],            # 可选
       "mark_ttl_ms": 15000,             # 可选
       "clear_trail": false,             # 可选
       "hide": true}                     # 可选
    """
    import json
    import os
    try:
        with open(path, encoding="utf-8") as f:
            job = json.load(f)
    except Exception as e:                                  # noqa: BLE001
        print(f"⚠ 读绘制任务失败（{e}）", file=sys.stderr)
        return 1
    try:
        return _draw_job(job, world, verbose)
    finally:
        # ★ 任务文件是本进程的**一次性输入**（回放任务除外，用户要能手动重放），
        #   原来从不删除 ⇒ %TEMP% 里每跑一轮就多两个 json（还带完整轨迹）。
        if job.get("delete_job"):
            try:
                os.remove(path)
            except OSError:
                pass


def _draw_job(job: dict, world: str, verbose: bool) -> int:
    """`draw_from_json_file` 的实际绘制部分（拆出来才能用 finally 兜清理）。"""
    world = job.get("world") or world
    scene = GzScene(world=world, timeout_ms=800, verbose=verbose)
    if not scene.available:
        if verbose:
            print("⚠ 子进程接不上 Gazebo —— 跳过绘制")
        return 1
    try:
        if job.get("clear_trail"):
            scene.clear_trail()
        trail = job.get("trail") or []
        if len(trail) >= 2:
            scene.draw_trail(trail, color=tuple(job.get("trail_color",
                                                         (1.0, 0.35, 0.05))))
        mk = job.get("mark")
        if mk:
            scene.mark_point(mk, ns="hit_mark", mid=1, size=0.6,
                             color=tuple(job.get("mark_color", (1.0, 0.95, 0.1))),
                             ttl_ms=int(job.get("mark_ttl_ms", 15000)))
        # ★ 收尾位置：**优先 park（放回发射架顶面）**，其次 hide（藏到视野外）
        #   （2026-10-17：原来默认 hide=True ⇒ 每次跑完球消失，用户以为坏了）
        park = job.get("park")
        if park:
            scene._do_set_pose(PROJECTILE_NAME,
                               (float(park[0]), float(park[1]), float(park[2])))
            if verbose:
                print(f"   弹丸已放回发射架顶面 ({park[0]:+.2f},{park[1]:+.2f},"
                      f"{park[2]:.2f})")
        elif job.get("hide", False):
            scene.hide_projectile()
        if verbose:
            print(f"✅ 子进程已完成绘制：轨迹 {len(trail)} 点"
                  f"{' + 命中标记' if mk else ''}")
    except Exception as e:                                  # noqa: BLE001
        print(f"⚠ 子进程绘制异常（{e}）", file=sys.stderr)
        return 1
    finally:
        scene.close()          # 本函数没起后台线程，关一下（幂等、无副作用）
    return 0


def _interp(trail, f: float):
    """按归一化进度 f∈[0,1] 在折线上线性插值取点（f 会先夹到 [0,1]）。"""
    if not trail:
        return None
    if len(trail) == 1:
        return trail[0]
    f = max(0.0, min(1.0, float(f)))
    x = f * (len(trail) - 1)
    i = int(x)
    if i >= len(trail) - 1:
        return trail[-1]
    t = x - i
    a, b = trail[i], trail[i + 1]
    return tuple(a[k] + (b[k] - a[k]) * t for k in range(3))


def replay_from_json_file(path: str, world: str = DEFAULT_WORLD,
                          verbose: bool = True) -> int:
    """★ **慢速回放**：把一次交会用 `seconds` 秒重放一遍。

    【为什么需要它】
      弹丸交会只有 **~0.5 s**（`t_rel` 从 0 到命中），而弹丸同步是 30 Hz
      ⇒ 整段只有 **15 帧**。人眼在实时里根本抓不住"球怎么飞、什么时候撞上"。
      ⇒ 用回放把 0.5 s 拉成 3~5 s（可调），并且**在相遇那一帧停顿**，
        再补上两条轨迹线和命中标记，就一目了然了。

    【为什么放子进程】
      与 `--draw-json` 同一个理由（见 `gz_finish` 注释）：gz 服务调用是**阻塞**的，
      在主进程里做会抢 GIL 卡住制导回路。回放要连续调用几十次，更必须隔离进程。

    【JSON 格式】
      {"world": "walls",
       "proj": [[x,y,z], ...],          # 弹丸轨迹（ENU）
       "uav":  [[x,y,z], ...],          # 无人机轨迹（ENU，可选）
       "hit":  [x,y,z] | null,          # 命中点（可选）
       "seconds": 3.0,                  # 回放总时长（墙钟秒）
       "hz": 25.0,                      # 回放帧率
       "hold": 1.0,                     # 相遇帧停顿
       "lead_in": 0.6,                  # 先展示完整轨迹线的停顿
       "clear": true,                   # 回放前清掉旧轨迹/无人机标记
       "hide_after": false}             # 回放结束后是否把弹丸藏起来
    """
    import json
    try:
        with open(path, encoding="utf-8") as f:
            job = json.load(f)
    except Exception as e:                                  # noqa: BLE001
        print(f"⚠ 读回放任务失败（{e}）", file=sys.stderr)
        return 1

    proj = [[float(c) for c in p] for p in (job.get("proj") or [])]
    uav = [[float(c) for c in p] for p in (job.get("uav") or [])]
    hit = job.get("hit")
    seconds = max(0.2, float(job.get("seconds", 3.0)))
    hz = max(1.0, float(job.get("hz", 25.0)))
    hold = max(0.0, float(job.get("hold", 1.0)))
    lead_in = max(0.0, float(job.get("lead_in", 0.6)))
    world = job.get("world") or world

    scene = GzScene(world=world, timeout_ms=800, verbose=verbose)
    if not scene.available:
        if verbose:
            print("⚠ 子进程接不上 Gazebo —— 跳过回放")
        return 1

    try:
        # ---- ① 探测 marker，再决定"用线画轨迹"还是"用珠子画轨迹" ----
        #   ★ 2026-10-17 实飞：用户环境 `/marker` 不可用（GUI 插件缺失），
        #     但 create/set_pose 都通。此时必须自动降级，否则回放只剩两个小球。
        use_marker = scene.marker_available()

        # ---- ② 清场（两种模式都要清，否则旧轨迹会叠在一起）----
        if job.get("clear", True):
            scene.clear_beads(PROJ_BEAD_PREFIX)
            scene.clear_beads(UAV_BEAD_PREFIX)
            scene._remove("replay_hit_bead")
            if use_marker:
                scene.clear_trail()                # 清 pilot_trail（实时那次的轨迹）
                scene.marker_delete(PROJ_TRAIL_NS)  # 清上次回放留下的线/球
                scene.marker_delete(UAV_TRAIL_NS)
                scene.marker_delete(UAV_REPLAY_NS)
                scene.marker_delete("hit_mark")

        # ---- ③ 摆静态参照：轨迹 + 命中点 ----
        n_beads_p = n_beads_u = 0
        uav_ball_ok = False        # 无 marker 时"无人机替身"是否建好
        if use_marker:
            if len(proj) >= 2:
                scene.draw_trail(proj, ns=PROJ_TRAIL_NS, width=0.05,
                                 color=PROJ_COLOR)
            if len(uav) >= 2:
                scene.draw_trail(uav, ns=UAV_TRAIL_NS, width=0.05,
                                 color=UAV_COLOR)
            if hit:
                scene.mark_point(hit, ns="hit_mark", mid=1, size=0.7,
                                 color=(1.0, 0.95, 0.1), ttl_ms=60000)
        else:
            # ★ 降级：用一串静态小球实体摆出轨迹（只依赖 create 服务）
            if len(proj) >= 2:
                n_beads_p = scene.ensure_beads(proj, PROJ_BEAD_PREFIX,
                                               radius=0.09, color=PROJ_COLOR)
            if len(uav) >= 2:
                n_beads_u = scene.ensure_beads(uav, UAV_BEAD_PREFIX,
                                               radius=0.09, color=UAV_COLOR)
            if hit:
                scene.ensure_hit_bead(hit, radius=0.22)
            # ★★ 2026-10-17 审查：没有 marker 时，"无人机当前位置"原来**完全没有**
            #   —— `marker_move()` 会静默失败，画面上只有弹丸在飞，
            #   而控制台还写着"绿 小球 = 无人机重放位置" ⇒ 用户以为回放又坏了。
            #   ⇒ 用一个可移动实体当替身，逐帧 set_pose（同样只依赖 create+set_pose）。
            if uav:
                scene.refresh_snapshot(timeout=1.0)
                if scene._ensure(UAV_BALL_NAME,
                                 movable_ball_sdf(UAV_BALL_NAME, 0.35, UAV_COLOR),
                                 (uav[0][0], uav[0][1], uav[0][2]), tol=0.50):
                    uav_ball_ok = True

        if verbose:
            if use_marker:
                print(f"✅ 回放准备（轨迹线模式）：弹丸 {len(proj)} 点 / "
                      f"无人机 {len(uav)} 点，停 {lead_in:.1f}s 先看清全局…")
            else:
                print("⚠ `/marker` 服务不可用（Gazebo GUI 的 MarkerManager 插件缺失）")
                print("  ⇒ **自动降级为「珠子」模式**：轨迹用一串静态小球摆出来")
                print(f"     弹丸轨迹 {n_beads_p} 颗珠子 / 无人机轨迹 {n_beads_u} 颗珠子"
                      f"{' / 命中点 1 颗黄球' if hit else ''}")
                print("     视觉差别：轨迹是**虚线**（不是连续线），其余一样")
                print(f"     无人机替身：{'✅ 已创建实体（绿色小球会跟着动）' if uav_ball_ok else '⚠ 没建出来，只有弹丸会动'}")
                if n_beads_p == 0 and n_beads_u == 0:
                    print("  ⚠⚠ 连珠子都没建出来（create 也失败了）⇒ 跑 "
                          "`python3 gz_scene_sync.py --check` 自查")
        # 先停顿，给用户时间"看到两条轨迹长什么样"
        time.sleep(lead_in)

        # ---- ② 逐帧回放：两条轨迹按**同一归一化进度**推进，保证"相遇"同时发生 ----
        #   ★ 为什么用归一化进度而不是真实时间戳：两条轨迹都取自 GUIDED 段，
        #     起止时刻一致（弹丸发射 = 开始制导），所以按进度对齐是正确的，
        #     而且不需要额外的同步信息。
        n_frames = max(2, int(seconds * hz))
        dt = seconds / n_frames
        near_idx = None
        if len(proj) >= 2 and len(uav) >= 2:
            # 找两条轨迹最接近的进度 ⇒ 那就是"相撞时刻"
            best = None
            for i in range(n_frames + 1):
                f = i / n_frames
                pp, pu = _interp(proj, f), _interp(uav, f)
                dd = sum((pp[k] - pu[k]) ** 2 for k in range(3))
                if best is None or dd < best[0]:
                    best = (dd, i, f)
            near_idx = best[1]
        for i in range(n_frames + 1):
            f = i / n_frames
            pp = _interp(proj, f)
            pu = _interp(uav, f)
            t0 = time.monotonic()
            if pp is not None:
                scene._do_set_pose(PROJECTILE_NAME, pp)
            if pu is not None:
                if use_marker:
                    scene.marker_move(pu, ns=UAV_REPLAY_NS, mid=UAV_REPLAY_ID,
                                      size=0.35, color=(0.20, 0.95, 0.35))
                elif uav_ball_ok:
                    # ★ 降级路径：无人机替身实体逐帧移动（原来这里什么都不做）
                    scene._do_set_pose(UAV_BALL_NAME, pu)
            if verbose and near_idx is not None and i == near_idx:
                print(f"   ★ 交会帧（进度 {f * 100:.0f}%）—— 停顿 {hold:.1f}s")
            if hold > 0 and near_idx is not None and i == near_idx:
                time.sleep(hold)
            rest = dt - (time.monotonic() - t0)
            if rest > 0:
                time.sleep(rest)

        if job.get("hide_after"):
            scene.hide_projectile()
        if verbose:
            print("✅ 回放结束（轨迹线与命中点保留，便于你截图/复看）")
    except Exception as e:                                  # noqa: BLE001
        print(f"⚠ 回放异常（{e}）", file=sys.stderr)
        return 1
    return 0


def watch_projectile(seconds: float = 8.0, world: str = DEFAULT_WORLD,
                     verbose: bool = True) -> int:
    """★ **观察弹丸是否真的在动**：让球沿一条直线飞，同时读回它的真实位置。

    为什么需要它（2026-10-17）：
      `set_pose` 返回 True **只代表服务调用成功**，不代表模型真的移动了
      （同理 `create` 返回 True 也不代表加载成功 —— 我们已经被这一点坑过两次）。
      ⇒ 这个命令做**闭环验证**：自己发 set_pose，同时用 `/pose/info` 读回
        世界的真实状态，逐帧对比"我要它去哪" vs "它实际在哪"。

    输出示例：
        t=0.2s  要 (+0.0,+0.0,+10.0)  实际 (+0.0,+0.0,+10.0)   ✅
        t=0.4s  要 (+0.0,+0.0,+10.5)  实际 (+0.0,+0.0,+10.5)   ✅
        ...
        ⇒ set_pose **确实生效**（实际位置跟随指令）
    """
    scene = GzScene(world=world, timeout_ms=1000, verbose=verbose)
    if not scene.available:
        print("❌ 接不上 Gazebo（见上面的原因）")
        return 1

    # 先建球（摆在起始点）
    p0 = (-14.0, 0.0, 10.0)
    if not scene.ensure_projectile(0.35, at=p0):
        print("❌ 弹丸创建失败 ⇒ 先跑 --rebuild-scene 看是哪一步挂了")
        return 1

    # 用**共享**的 pose/info 订阅持续更新"弹丸真实位置"
    #   （原来这里再 subscribe 一次 ⇒ 与 snapshot_positions 各留一个永不退订的回调）
    if scene._ensure_pose_sub() is None:
        print("⚠ 订阅 /pose/info 失败 —— 只能看「指令是否发出」，"
              "无法读回真实位置")
    time.sleep(0.5)          # 先收几帧初始状态

    n = max(2, int(seconds / 0.4))
    ok_n = 0
    got_last = None
    print()
    print("=" * 72)
    print("  弹丸移动闭环验证（发 set_pose → 读回真实位置）")
    print("=" * 72)
    for i in range(n + 1):
        tgt = (p0[0] + 0.5 * i, p0[1], p0[2] + 0.15 * i)   # 斜向上慢飞
        scene._do_set_pose(PROJECTILE_NAME, tgt)
        time.sleep(0.4)
        got = scene.pose_of(PROJECTILE_NAME)     # 读共享订阅的最新缓存
        got_last = got if got is not None else got_last
        if got is None:
            print(f"  t={i * 0.4:4.1f}s  要 ({tgt[0]:+6.2f},{tgt[1]:+6.2f},"
                  f"{tgt[2]:+6.2f})  实际 读不到位置  ⚠")
            continue
        err = max(abs(got[k] - tgt[k]) for k in range(3))
        good = err < 0.15
        ok_n += 1 if good else 0
        print(f"  t={i * 0.4:4.1f}s  要 ({tgt[0]:+6.2f},{tgt[1]:+6.2f},"
              f"{tgt[2]:+6.2f})  实际 ({got[0]:+6.2f},{got[1]:+6.2f},"
              f"{got[2]:+6.2f})  误差 {err:4.2f} m  {'✅' if good else '❌'}")
    print("-" * 72)
    if got_last is None:
        print("  ⚠ 全程读不到弹丸位置 ⇒ /pose/info 里没有 enemy_dart")
        print("    ⇒ 球可能压根没加载（跑 --rebuild-scene / --list-entities 确认）")
        rc = 1
    elif ok_n >= max(1, int((n + 1) * 0.8)):
        print(f"  ✅ set_pose **确实生效**（{ok_n}/{n + 1} 帧跟随，误差 < 0.15 m）")
        print("     ⇒ 弹丸会飞。之前看不到只是因为**藏在 z=-500 / 只有 0.5 s**。")
        rc = 0
    else:
        print(f"  ❌ set_pose 发了但模型没跟上（{ok_n}/{n + 1} 帧）")
        print("     ⇒ 球建出来了但动不了。把这段输出发我。")
        rc = 1
    print("=" * 72)
    print()
    scene.hide_projectile()
    scene.close()
    return rc


def rebuild_scene(p=(-14.0, 0.0, 10.0), radius: float = 0.25,
                  world: str = DEFAULT_WORLD, verbose: bool = True) -> int:
    """★ **重建场景并逐个报告**：清掉旧的 → 建塔 → 建发射架 → 建弹丸（摆在发射点）。

    为什么需要它（2026-10-17 实测 `--list-entities` 后）：
      · `enemy_dart` **存在于世界里**，但 z=**-500**（藏在视野外）⇒ 用户看不到
      · `enemy_launcher` **压根不存在** ⇒ 创建失败，而且没人发现
        （`gz_setup` 里 `ensure_launcher` 的返回值**是忽略的**）
      ⇒ 需要一个"一次把三样建好、并逐个报成功/失败"的命令，
        而不是散在飞行流程里靠日志推断。
    """
    scene = GzScene(world=world, timeout_ms=1000, verbose=verbose)
    if not scene.available:
        print("❌ 接不上 Gazebo（见上面的原因）")
        return 1

    print()
    print("=" * 68)
    print(f"  重建场景    world={world}")
    print("=" * 68)
    try:
        # ① 先清掉旧的三样（避免残留同名实体干扰）
        for nm in (TOWER_NAME, LAUNCHER_NAME, PROJECTILE_NAME):
            scene._remove(nm)
        print("  已清掉旧的塔 / 发射架 / 弹丸")

        # ② 逐个创建并**报告结果**
        results = []
        ok_t = scene.ensure_tower(1.2, 3.0)
        results.append(("defense_tower", "防御塔", ok_t,
                        (0.0, 0.0, 1.5)))
        ok_l = scene.ensure_launcher((p[0], p[1], 0.0))
        results.append(("enemy_launcher", "发射架", ok_l,
                        (p[0], p[1], 0.3)))
        # 弹丸：**创建时就放在发射点**（不是 HIDDEN_Z）
        ok_p = scene.ensure_projectile(radius, at=(p[0], p[1], p[2]))
        results.append(("enemy_dart", "弹丸", ok_p, (p[0], p[1], p[2])))

        print()
        for nm, label, ok, pos in results:
            mark = "✅" if ok else "❌"
            print(f"  {mark} {nm:<20} {label:<6} "
                  f"({pos[0]:+7.2f},{pos[1]:+7.2f},{pos[2]:+7.2f})"
                  + ("" if ok else "   ← **create 服务说失败**"))

        # ★★ 关键一步：**读回世界的真实状态**，不信用 create 的返回值
        #   （`create` 返回 True ≠ 模型真的加载了 —— 已被坑过多次）
        print()
        print("  --- 读回世界真实状态（/pose/info）---")
        snap = scene.snapshot_positions(timeout=3.0)
        if not snap:
            print("  ⚠ 读不到 /pose/info（订阅超时）⇒ 无法验证，只能看 Gazebo 画面")
        else:
            for nm, label, ok, want in results:
                got = snap.get(nm)
                if got is None:
                    print(f"  ❌ {nm:<20} {label:<6} **读回：世界里没有它**")
                else:
                    d = max(abs(got[k] - want[k]) for k in range(3))
                    flag = "✅" if d < 0.3 else "⚠"
                    print(f"  {flag} {nm:<20} {label:<6} "
                          f"读回 ({got[0]:+7.2f},{got[1]:+7.2f},{got[2]:+7.2f})"
                          f"  期望 z={want[2]:.2f}")
        print("-" * 68)
        if all(r[2] for r in results):
            print("  ✅ 三样都建好了。现在去 Gazebo 看：")
            print(f"     · 蓝青色圆柱 = 防御塔（原点，直径 2.4 m、高 3 m）")
            print(f"     · 橙色方块   = 敌方发射架（地面上，{p[0]:+.1f},{p[1]:+.1f}）")
            print(f"     · 橙红色小球 = 弹丸（悬空在 {p[2]:.1f} m 高，"
                  f"直径 {radius * 2:.2f} m）")
            print()
            print("  如果**只能看到塔和方块、看不到小球**：")
            print("     → 再跑一次并把半径放大： --proj-radius 0.6")
            print("     → 若放大后能看到，说明原尺寸在远景下太小（不是渲染问题）")
            print("  ⚠ 这些实体是**常驻**的，跑飞行时会复用/覆盖；")
            print("     飞行结束后弹丸会被移回视野外（正常）。")
        else:
            bad = [r[0] for r in results if not r[2]]
            print(f"  ⚠ 有 {len(bad)} 个创建失败：{bad}")
            print("     ⇒ 这些实体的 SDF 被 gz-sim 拒绝了。请把 **Gazebo 终端**里")
            print("       从创建那一刻起的报错（通常含 `Error` / `SDF`）发我。")
        print("=" * 68)
        print()
        sys.stdout.flush()
        return 0 if all(r[2] for r in results) else 1
    finally:
        scene.close()


def list_entities(world: str = DEFAULT_WORLD, verbose: bool = True) -> int:
    """★ 列出世界里**所有模型**的名字 + 位置，并标出我们关心的那几个。

    为什么需要它（2026-10-17）：
      用户反馈"Gazebo 模型列表里没有弹丸，但塔和发射架有"。
      这时候最需要的是**客观事实**：`enemy_dart` 到底存不存在？在哪？
      `create` 服务返回 True ≠ 模型真的被加载了（可能是 SDF 解析失败但被 accept）。

    实现：读 `/world/<world>/pose/info`（`gz.msgs.Pose_V`）的一次性快照
    （共享订阅 + 等一帧新的），比逐个 `gz model` 查询更省事。
    """
    scene = GzScene(world=world, timeout_ms=1500, verbose=verbose)
    if not scene.available:
        print("❌ 接不上 Gazebo（见上面的原因）")
        return 1

    got = {"names": None, "err": None}
    try:
        # ★ 用 `/world/<world>/pose/info`（`gz.msgs.Pose_V`）—— 一条消息就带**所有实体**
        #   的 name + position。
        #   ⚠ 别用 `/world/<world>/stats`（WorldStatistics）：它只有 `model_count`
        #     （数量），**没有模型名列表** —— 我第一版就写错了。
        # ★ 2026-10-17 审查：复用 `snapshot_positions` 的**共享**订阅（原来在这里
        #   再 subscribe 一个永不退订的回调；dict 保序，所以打印顺序不变）。
        snap = scene.snapshot_positions(timeout=4.0)
        if snap:
            got["names"] = list(snap.items())
    except Exception as e:                                  # noqa: BLE001
        got["err"] = e

    print()
    print("=" * 70)
    print(f"  世界里的模型列表    world={world}")
    print("=" * 70)
    if got["err"] is not None:
        print(f"  ⚠ 读 stats 失败：{got['err']}")
        print("  ⇒ 可以退一步用 CLI 查：")
        print(f"       gz model --list")
        print(f"       gz model -m enemy_dart -p")
        print()
        return 1
    if got["names"] is None:
        print("  ⚠ 3 s 内没收到 /stats 消息（仿真没在跑？世界名不对？）")
        print(f"  可以退一步用 CLI：gz model --list")
        print()
        return 1

    want = {TOWER_NAME: "防御塔", LAUNCHER_NAME: "发射架",
            PROJECTILE_NAME: "★弹丸★"}
    found = {}
    for nm, pos in got["names"]:
        tag = ""
        if nm in want:
            tag = f"   ← {want[nm]}"
            found[nm] = pos
        pos_s = ("(?,?,?)" if pos is None
                 else f"({pos[0]:+8.2f},{pos[1]:+8.2f},{pos[2]:+8.2f})")
        print(f"  {nm:<24} {pos_s}{tag}")
    print("-" * 70)
    print("  我们创建的三样东西：")
    for nm, label in ((TOWER_NAME, "防御塔"), (LAUNCHER_NAME, "发射架"),
                      (PROJECTILE_NAME, "弹丸")):
        if nm in found:
            pos = found[nm]
            extra = ""
            if nm == PROJECTILE_NAME and pos is not None and pos[2] < -100:
                extra = ("   ⚠ z 极低 ⇒ 仍被藏在视野外（HIDDEN_Z），"
                         "说明 set_pose 没把它移回来")
            print(f"    ✅ {nm:<22} {label}{extra}")
        else:
            print(f"    ❌ {nm:<22} {label} —— **世界里没有这个模型！**")
    if PROJECTILE_NAME not in found:
        print()
        print("  ⇒ 弹丸压根没创建成功（create 返回 True 也不代表真加载了）。")
        print("    可能原因：SDF 结构被 gz-sim 拒绝。已把 projectile_sdf 改成")
        print("    与塔/发射架**同构**的 static 结构（2026-10-17 修正）。")
        print("    换新版后重跑本命令应能看到 ✅。")
    print("=" * 70)
    print()
    scene.close()
    return 0


def show_projectile(p=(-14.0, 0.0, 10.0), radius: float = 0.25,
                    seconds: float = 10.0, world: str = DEFAULT_WORLD,
                    with_tower: bool = True, verbose: bool = True) -> int:
    """★ **弹丸可见性诊断**：把弹丸静止摆在指定位置 N 秒，让你确认"能不能看见这个球"。

    为什么需要它（2026-10-17）：
      · `ensure_projectile()` 创建时把弹丸放在 `HIDDEN_Z = -500`（地下 500 m）
        ⇒ **发射前它根本不在场上**，只有 GUIDED 段那 ~0.5 s 才出现、且从 14 m 外飞来。
      · 所以"看不见弹丸"可能有两个完全不同的原因：
          ① 球压根没渲染出来（尺寸/颜色/世界名/Create 失败）
          ② 渲染正常，但**只存在 0.5 秒 + 距离远**，人眼抓不住
      ⇒ 这个命令把变量分离掉：**球静止、单独存在、旁边有塔做参照物**。
        能看见 ⇒ 渲染没问题，问题在"时间太短"；
        看不见 ⇒ 是尺寸/位置/颜色问题，与制导无关。

    同时放**防御塔**做尺寸参照（不然"球到底多小"没有直觉）。

    用法：
        python3 gz_scene_sync.py --show-projectile 15
        python3 gz_scene_sync.py --show-projectile 15 --proj-radius 0.5
    """
    scene = GzScene(world=world, timeout_ms=800, verbose=verbose)
    if not scene.available:
        print("❌ 接不上 Gazebo（见上面的原因）")
        return 1
    try:
        if with_tower:
            scene.ensure_tower(1.2, 3.0)          # 原点：给个尺寸参照
            scene.ensure_launcher((p[0], p[1], 0.0))
        ok = scene.ensure_projectile(radius)      # 先在 HIDDEN_Z 建出来
        if not ok:
            print("❌ 弹丸实体创建失败（create 服务不通？跑 --check 看看）")
            return 1
        # ★ 关键：从"藏起来"移到**可见位置**
        moved = scene._do_set_pose(PROJECTILE_NAME, (float(p[0]), float(p[1]),
                                                     float(p[2])))
        if not moved:
            print("❌ set_pose 失败 ⇒ 球建出来了但移不到该位置"
                  "（跑 --check 看 set_pose 是否 ✅）")
            return 1
        print()
        print("=" * 66)
        print("  ★ 弹丸可见性诊断")
        print("=" * 66)
        print(f"  弹丸位置  ENU = ({p[0]:+.2f}, {p[1]:+.2f}, {p[2]:+.2f})")
        print(f"  球半径    = {radius:.3f} m （直径 {radius * 2:.2f} m）")
        print(f"  在旁边放了防御塔（原点，r=1.2 m h=3.0 m）做尺寸参照")
        print(f"  静止展示  {seconds:.0f} 秒 —— 现在去看 Gazebo")
        print()
        print("  请回答：**你能不能看见那个橙红色小球？**")
        print("    · 能看见 ⇒ 渲染没问题，问题是它只在场上存在 0.5 s（飞太快），")
        print("               可以降速或加长展示时间")
        print("    · 看不见 ⇒ 是尺寸/颜色/视角问题 ⇒ 加大 --proj-radius 再试")
        print("=" * 66)
        print()
        sys.stdout.flush()
        time.sleep(max(0.5, float(seconds)))
        # 展示完藏回去，免得留在场上干扰后续
        scene.hide_projectile()
        print("（展示结束，弹丸已移回视野外）")
        return 0
    finally:
        scene.close()


def run_check(world: str = DEFAULT_WORLD, verbose: bool = True) -> int:
    """★ 服务自检：一眼看出"画不出来"到底卡在哪个服务。

    输出示例：
        gz.transport          ✅
        create     (建实体)    ✅
        set_pose   (移动弹丸)  ✅
        marker     (轨迹/命中点) ❌  ← 需要 Gazebo GUI 的 MarkerManager 插件

    退出码：全通 0；有失败 1。
    """
    scene = GzScene(world=world, timeout_ms=1200, verbose=verbose)
    res = scene.probe_services()
    print()
    print("=" * 68)
    print(f"  Gazebo 服务自检    world={world}")
    print("=" * 68)
    all_ok = True
    for key, (ok, desc) in res.items():
        mark = "✅" if ok else "❌"
        if not ok:
            all_ok = False
        print(f"  {key:<12} {mark}   {desc}")
    print("=" * 68)
    if not all_ok:
        print("  诊断建议：")
        if not res.get("marker", (True,))[0]:
            print("   · marker 失败 ⇒ **轨迹线和命中点不会出现**（回放只剩小球在飞）。")
            print("     原因：`/marker` 服务由 Gazebo **GUI** 的 MarkerManager 插件提供。")
            print("     处理：确认启动的是**带 GUI 的** Gazebo（不要 headless / -s），")
            print("           或先在 GUI 里看一眼 Plugins 里有没有 MarkerManager。")
        if not res.get("set_pose", (True,))[0]:
            print("   · set_pose 失败 ⇒ 弹丸（和回放小球）根本不会动。")
            print("     处理：确认世界名对（--world）、仿真正在跑、实体已创建。")
        if not res.get("create", (True,))[0]:
            print("   · create 失败 ⇒ 塔 / 发射架 / 弹丸都建不出来。")
            print("     处理：确认世界名对、仿真没卡死。")
        print()
        print("   另外：这些服务名依赖 gz-sim 8（Harmonic），可用 `gz service -l` 核对。")
    else:
        print("  ✅ 全部可用 —— 可视化链路没问题。")
    print("  ℹ 本自检只发**虚构实体名**（`%s`）的探针请求，"
          "不会移动 / 创建任何真实模型。" % PROBE_POSE_NAME)
    print()
    return 0 if all_ok else 1


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
    g.add_argument("--draw-json", default=None, metavar="FILE",
                   help="★ 一次性绘制任务（JSON）—— 由控制节点以**子进程**方式调用，"
                        "避免 gz 阻塞调用抢 GIL 卡住制导回路")
    g.add_argument("--replay-json", default=None, metavar="FILE",
                   help="★ **慢速回放**一次交会（JSON）—— 弹丸交会只有 ~0.5 s，"
                        "实时看不清；用回放拉长到几秒并在相遇帧停顿")
    g.add_argument("--check", action="store_true",
                   help="★ **服务自检**：逐个探测 create / set_pose / marker 服务是否可用。"
                        "画面上没东西（比如回放没动）时先跑这个")
    g.add_argument("--list-entities", action="store_true",
                   help="★ **列出世界里的所有模型**（名字+位置），并标出塔/发射架/弹丸"
                        "是否真的存在。'模型列表里没有弹丸'时跑这个")
    g.add_argument("--rebuild-scene", action="store_true",
                   help="★ **重建场景并逐个报告**：清旧的 → 建塔 → 建发射架 → "
                        "建弹丸（摆在发射点），逐个打印 ✅/❌")
    g.add_argument("--watch-projectile", type=float, default=None, metavar="SEC",
                   help="★ **闭环验证弹丸是否真的会动**：让球沿直线慢飞 SEC 秒，"
                        "同时读回 `/pose/info` 的真实位置，逐帧对比"
                        "「要它去哪」vs「它实际在哪」")
    g.add_argument("--show-projectile", type=float, default=None, metavar="SEC",
                   help="★ **弹丸可见性诊断**：把弹丸静止摆在发射点展示 SEC 秒"
                        "（旁边放塔做参照）。用来区分「球没渲染」和「飞太快看不见」。"
                        "用法：--show-projectile 15")
    g.add_argument("--set-rtf", type=float, default=None, metavar="FACTOR",
                   help="设置 Gazebo 实时率（慢放）：1.0=实时，0.2=5 倍慢放。"
                        "⚠ 会让**墙钟超时判据偏早**，建议只在看画面时临时用，别低于 0.3")
    ap.add_argument("--tower-radius", type=float, default=1.2)
    ap.add_argument("--tower-height", type=float, default=3.0)
    ap.add_argument("--from", dest="p0", default="-14,0,10",
                    help="demo / --show-projectile 的弹丸位置（ENU，逗号分隔）")
    ap.add_argument("--proj-radius", type=float, default=0.25,
                    help="--show-projectile 用的球半径 (m)。默认 0.25（与制导节点一致）；"
                         "看不见就加大到 0.5 再试")
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

    if args.draw_json:
        return draw_from_json_file(args.draw_json, world=args.world,
                                   verbose=not args.quiet)
    if args.replay_json:
        return replay_from_json_file(args.replay_json, world=args.world,
                                     verbose=not args.quiet)
    if args.check:
        return run_check(args.world, verbose=not args.quiet)
    if args.list_entities:
        return list_entities(args.world, verbose=not args.quiet)
    if args.rebuild_scene:
        return rebuild_scene(p=_vec(args.p0), radius=args.proj_radius,
                             world=args.world, verbose=not args.quiet)
    if args.watch_projectile is not None:
        return watch_projectile(args.watch_projectile, world=args.world,
                                verbose=not args.quiet)
    if args.show_projectile is not None:
        return show_projectile(p=_vec(args.p0), radius=args.proj_radius,
                               seconds=args.show_projectile, world=args.world,
                               verbose=not args.quiet)
    if args.set_rtf is not None:
        scene0 = GzScene(world=args.world, verbose=not args.quiet)
        ok = scene0.set_rtf(args.set_rtf)
        return 0 if ok else 1

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
