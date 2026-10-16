#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase B —— XTDrone2 「制导闭环」验证节点
================================================================================

对应《制导讲义/XTDrone2接入方案.md》§7.2 阶段 B：
    「把 guidance_libs 接进仿真，跑真实制导闭环」

与阶段 A 的分工：
  · 阶段 A（phase_a_accel_path.py）：验证「指令回写通路」—— 发常值加速度，看无人机是否照做。
  · 阶段 B（本节点）           ：验证「制导闭环」—— 制导律**真的在飞**：
        无人机状态（Gazebo/PX4 真值）→ 制导核（仓库的真 C++ 代码）→ a_cmd → PX4 → 无人机

本节点做的事：
    起飞 → 朝迎击方向预加速（CRUISE）→ **目标发射** → PPN 制导（GUIDED）
         → 命中 / 脱靶 / 超时 → 自动 LAND + 停桨 → 退出

--------------------------------------------------------------------------------
两个关键设计（都有依据）
--------------------------------------------------------------------------------
① **目标用数学模型，不用 Gazebo 实体。**
   目标轨迹由仓库的 `TargetModel`（真 C++ 代码，含 DART 气动模型）按时间推进 ——
   它只依赖 t，是确定性的（见 target.hpp 的设计说明）。
   无人机则是**真的在 PX4 里飞**（真动力学、真姿态环、真限幅）。
   ⇒ 制导闭环是真的，只是"目标"是虚拟的。命中判定用数学距离 R_true。
   好处：不需要写 Gazebo 插件、飞镖动力学直接复用仓库代码、可复现。
   （阶段 C 才引入相机/视觉，届时目标才会变成 Gazebo 里的真实体。）

② **必须"预加速"（CRUISE 段），不能从悬停直接开制导。**
   PPN 的指令是 `a = N·(ω × v_M)` —— **v_M = 0 时恒等于零指令**（死区）。
   而且从悬停起步时，无人机根本没有横向速度可用来修正弹道，
   交会时间又只有零点几秒 → 必然脱靶（离线仿真已证实）。
   所以像真实比赛一样：无人机先飞起来迎击，有速度之后再交给制导。
   （`--no-cruise` 可关掉，用于复现"悬停直接制导"的失败案例。）

--------------------------------------------------------------------------------
场景（默认，可在命令行改）
--------------------------------------------------------------------------------
  ENU 系（Gazebo 世界系），**全部在 y=0 走廊里**（walls 世界的墙都在 y=±5/±10，此走廊净空）：

    无人机: 从 (0, 0, 10) 起飞，朝 −x（西）预加速到 5 m/s
    飞镖  : 从 (−12, 0, 11) 以 15 m/s、仰角 5° 朝 +x 发射（正对无人机）
    制导  : PPN，N=6，a_max=10 m/s²（x500 真实水平过载上限量级），t_go_freeze=0.10

  这套参数是**离线仿真（sim_phase_b_offline.py）扫出来的**：
  命中精度 ~0.46 m（对 12 m 距离 = 3.9%）。

  ⚠ 场景设计的硬约束（离线扫参得出的结论）：
    · **飞镖飞行距离要短（10 m 量级）**：飞镖同时受重力+气动阻力，弹道持续下坠。
      若把发射点放到 22 m 外，飞镖要飞 1.5 s+，期间下坠十几米 —— **PN 无论如何够不着**。
      真实 RM 里飞镖只需飞 5~10 m，所以 10 m 级才是对的尺度。
    · 无人机横向机动能力 = ½·a_max·t_go²。t_go=0.4 s、a_max=10 → 只能修 0.8 m。
      ⇒ 交班时的初始偏差必须控制在 ~1 m 量级，这靠**场景几何**保证（共线迎头）。

--------------------------------------------------------------------------------
状态机
--------------------------------------------------------------------------------
  ARM → CONFIRM_ARM → TAKEOFF → WAIT_HEIGHT
      → OFFBOARD → CONFIRM_OFFBOARD → SETTLE
      → CRUISE（预加速）→ GUIDED（目标已发射，PPN 闭环）
      → [命中 / 脱靶 / 超时 / 安全中止] → LAND → DISARM → DONE（退出）

  与阶段 A 相同的四条铁律（都在阶段 A 踩过坑，这里直接照搬修法）：
    · 服务返回 success ≠ 生效 —— 关键状态一律订阅 `vehicle_status` 确认
      （`arming_state==2` 解锁、`nav_state==14` OFFBOARD、`arming_state==1` 停桨）
    · 不要在回调里 `spin_until_future_complete`（Jazzy 会崩）—— 全用 call_async + 轮询
    · 降落阶段改发 `cmd_vel_ned`（否则 `cmd_accel_ned` 的 NaN 速度 setpoint 会打死
      PX4 的 land detector —— 阶段 A 踩过的坑 8）
    · `SignalHandlerOptions` 成员名要动态探测（Jazzy 上是 `NO` 不是 `NONE`）

--------------------------------------------------------------------------------
用法
--------------------------------------------------------------------------------
  source /opt/ros/jazzy/setup.bash
  source ~/XTDrone2/install/setup.bash
  cd ~/rm_guidance_sim
  bash run_phase_b.sh --namespace x500_0

  常用开关：
    --no-cruise            不做预加速（复现"悬停直接制导"的失败案例）
    --target-p0 "-12,0,11" 换飞镖发射点
    --target-speed 15 --target-elev 5
    --N 6 --a-max 10 --law 0    0=PPN 1=TPN 2=APN
    --hit-radius 0.5       命中判定半径（m）
    --csv phase_b_log.csv

--------------------------------------------------------------------------------
安全
--------------------------------------------------------------------------------
  · 只发"制导指令量级"的加速度（≤a_max，默认 10 m/s²，比阶段 A 的脉冲更温和）
  · 安全围栏：离地 < 3 m、或水平位移 > 25 m、或制导段超时 → 立即中止并转入降落
  · 跑完/中止/Ctrl-C（一次）→ 自动 LAND → 停桨 → 退出；Ctrl-C 两次 = 强制退出
  · walls 世界：+x/+y 方向 4.5 m 处有墙（高 30 m），−x/−y 净空 ⇒ 默认场景朝西迎击
"""
import argparse
import ctypes as C
import csv
import math
import os
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, qos_profile_sensor_data
from geometry_msgs.msg import Twist
from px4_msgs.msg import VehicleLocalPosition, VehicleStatus, VehicleCommand
from xtd2_msgs.srv import XTD2Cmd

try:
    from nav_msgs.msg import Odometry
    HAVE_ODOM = True
except ImportError:          # 没装 nav_msgs 也不影响主流程
    HAVE_ODOM = False


# ============================================================================
#  常量
# ============================================================================
NAV_STATE_OFFBOARD = 14            # vehicle_status_s::NAVIGATION_STATE_OFFBOARD
NAV_STATE_AUTO_TAKEOFF = 17
NAV_STATE_AUTO_LAND = 18
ARMING_STATE_DISARMED = 1
ARMING_STATE_ARMED = 2

VEHICLE_CMD_DO_SET_MODE = 176
VEHICLE_CMD_NAV_LAND = 21
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6

ARM_RESEND_SEC = 1.0
ARM_CONFIRM_TIMEOUT = 15.0
OFFBOARD_RESEND_SEC = 0.5
OFFBOARD_CONFIRM_TIMEOUT = 10.0
POST_ARM_DELAY = 0.5
STABLE_SEC = 1.5
SRV_TIMEOUT = 3.0

LAND_RESEND_SEC = 1.0
DISARM_RETRY_SEC = 1.0
LAND_TIMEOUT = 90.0
DISARM_TIMEOUT = 20.0
LANDED_ALT = 0.30
LANDED_VZ = 0.08
LANDED_HOLD = 1.0
FORCE_DISARM_AFTER = 6.0
FORCE_DISARM_HOLD = 3.0

ALT_KP = 1.0                       # 起飞/预加速段的垂直保持 PD（不是制导）
ALT_KD = 2.0

MIN_ALTITUDE = 3.0                 # 安全围栏
MAX_H_DISP = 25.0
GUIDED_TIMEOUT = 8.0               # GUIDED 段超时（s，按目标相对时间）
CRUISE_TIMEOUT = 8.0               # 巡航/预加速段超时（s，墙钟）
ALT_ADJ_A_MAX = 3.0                # SETTLE 段垂直调整的最大加速度 (m/s²)
ALT_SETTLE_TOL = 0.35              # 高度到位容差 (m)
ALT_SETTLE_VTOL = 0.30             # 高度到位时的垂直速度容差 (m/s)
ALT_SETTLE_TIMEOUT = 8.0           # 高度调整等待上限 (s)
SETTLE_KP = 1.0                    # SETTLE 段水平位置增益 (1/s²)
SETTLE_KD = 2.0                    # SETTLE 段水平速度阻尼 (1/s)
SETTLE_XY_TOL = 0.25               # SETTLE 水平位置到位容差 (m)
SETTLE_XY_VTOL = 0.20              # SETTLE 水平速度到位容差 (m/s)
BRAKE_KD = 2.0                     # 着陆前刹车增益 (1/s)
BRAKE_VTOL = 0.30                  # 刹车完成判据：|v_h| ≤ 此值 (m/s)
BRAKE_TIMEOUT = 5.0                # 刹车段超时 (s)

# 制导核的 mode 枚举（同 guidance_types.hpp）
GMODE_PN, GMODE_APN, GMODE_HOLD, GMODE_SEARCH = 0, 1, 2, 3
GMODE_NAME = {0: "PN", 1: "APN", 2: "HOLD", 3: "SEARCH"}

# ---------------------------------------------------------------------------
#  CSV / 回传报告的列定义（**唯一**定义处）
#    ⚠ 报告块按**列名**取索引（`hdr.index(name)`），所以以后加/删列不会让报告错位。
# ---------------------------------------------------------------------------
CSV_COLUMNS = [
    "t_wall", "t_rel", "state",
    "ax_enu", "ay_enu", "az_enu", "a_norm", "sat", "mode", "t_go",
    "lam_dot", "Vc", "R_est",
    "px4_x", "px4_y", "px4_z", "px4_vx", "px4_vy", "px4_vz", "heading",
    "enu_x", "enu_y", "enu_z", "enu_vx", "enu_vy", "enu_vz",
    "roll_deg", "pitch_deg", "yaw_deg",
    "q_w", "q_x", "q_y", "q_z",
    "tgt_x", "tgt_y", "tgt_z", "tgt_vx", "tgt_vy", "tgt_vz",
    "R_true", "sensor_valid",
    "nav_state", "arming_state",
]

# 回传报告里逐行表格用到的列（按这个顺序打印，列太宽的 NED 位置等不进表）
#   ⚠ 必须有 `t_wall`：`t_rel` 是**制导相对时**，GUIDED 一结束就**冻结**在最后值
#     （实测收尾段所有行的 t_rel 都相同）⇒ 只有 `t_wall`（墙钟，全程单调）能区分收尾段各步。
REPORT_TRACE_COLS = [
    "t_wall", "t_rel", "state", "nav_state", "mode", "t_go", "Vc", "R_true", "R_est",
    "a_norm", "sat",
    "enu_x", "enu_y", "enu_z", "enu_vx", "enu_vy", "enu_vz",
    "roll_deg", "pitch_deg", "yaw_deg",
]

# 报告里"制导相关段"按 `--report-hz` 采样；**其余段（起飞前爬升 / 降落收尾）
# 一律降到 REPORT_SLOW_HZ** —— 否则光是爬升 11 s + 下降 11 s 就会灌进几百行
# 几乎相同的采样，把真正要看的东西淹掉。
#   ★ 用**白名单**（默认慢速）而不是黑名单：以后新增状态不会意外把报告撑爆。
REPORT_FAST_STATES = {"GUIDED"}
REPORT_MID_STATES = {"SETTLE", "CRUISE", "BRAKE"}
REPORT_SLOW_HZ = 1.0
REPORT_MID_HZ = 5.0

# 控制回路"空洞"告警阈值（s）：相邻 tick 的墙钟间隔超过它 ⇒ 说明**本周期没发出
# setpoint**（被 gz 服务调用 / 日志 / 磁盘阻塞）。PX4 的 offboard 失联阈值默认 ~1 s，
# 所以这里取 0.2 s 作为"该管了"的界限（50 Hz 名义周期 0.02 s，是它的 10 倍）。
LOOP_GAP_WARN = 0.20

# 报告块的起止标记（用户直接整段复制回传；我按标记截取）
REPORT_BEGIN = "<<<<<<< BEGIN RM-PHASE-B-REPORT >>>>>>>"
REPORT_END = "<<<<<<< END RM-PHASE-B-REPORT >>>>>>>"


# ============================================================================
#  制导核动态库（ctypes）
# ============================================================================
def load_guidance_lib(path):
    """加载 libpure_guidance.so / pure_guidance.dll（由 build_guidance_shim.sh 编译）。"""
    if path is None:
        here = os.path.dirname(os.path.abspath(__file__))
        for d in (here, os.getcwd()):
            for n in ("libpure_guidance.so", "pure_guidance.dll",
                      "libpure_guidance.dll", "libpure_guidance.dylib"):
                p = os.path.join(d, n)
                if os.path.exists(p):
                    path = p
                    break
            if path:
                break
    if not path or not os.path.exists(path):
        raise SystemExit(
            "找不到制导核动态库（libpure_guidance.so / pure_guidance.dll）。\n"
            "先运行 bash build_guidance_shim.sh 编译，或用 --lib 指定路径。")

    lib = C.CDLL(os.path.abspath(path))
    lib.pgu_version.restype = C.c_int
    lib.pgu_create.restype = C.c_void_p
    lib.pgu_destroy.argtypes = [C.c_void_p]
    lib.pgu_config_target.argtypes = [C.c_void_p, C.POINTER(C.c_double)]
    lib.pgu_config_guidance.argtypes = [C.c_void_p, C.POINTER(C.c_double)]
    lib.pgu_config_sensor.argtypes = [C.c_void_p, C.POINTER(C.c_double)]
    lib.pgu_target_flight_time.restype = C.c_double
    lib.pgu_target_flight_time.argtypes = [C.c_void_p]
    lib.pgu_target_hits_ground.restype = C.c_int
    lib.pgu_target_hits_ground.argtypes = [C.c_void_p]
    lib.pgu_target_state.argtypes = [C.c_void_p, C.c_double, C.POINTER(C.c_double)]
    lib.pgu_sensor_stats.argtypes = [C.c_void_p, C.POINTER(C.c_double)]
    lib.pgu_step.argtypes = [C.c_void_p, C.c_double, C.c_double,
                             C.POINTER(C.c_double), C.POINTER(C.c_double),
                             C.POINTER(C.c_double), C.POINTER(C.c_double),
                             C.POINTER(C.c_double)]
    return lib, os.path.abspath(path)


def darr(vals):
    return (C.c_double * len(vals))(*vals)


def target_cfg(p0, typ, speed, elev_deg, azim_deg,
               mass, cd, area, rho, t_max=10.0):
    """29 元素目标配置数组（布局见 pure_guidance_shim.cpp 文档）。"""
    c = [0.0] * 29
    c[0] = float(typ)
    c[1], c[2], c[3] = p0
    c[7] = speed
    c[8] = math.radians(elev_deg)
    c[9] = math.radians(azim_deg)
    c[10], c[11], c[12], c[13] = mass, cd, area, rho
    c[14], c[15], c[16] = 0.0, 0.0, -9.81
    c[26], c[27], c[28] = 0.0005, t_max, 0.0
    return c


def guidance_cfg(law, N, a_max, t_go_freeze):
    """7 元素制导配置数组。"""
    return [float(law), N, a_max, 0.0, t_go_freeze, 5.0, 0.0]


def sensor_cfg():
    """15 元素传感器配置：IDEAL 档（真值透传），其余字段本阶段不消费。"""
    return [0.0, 0.1, 0.0, 0.0, 0.05, 0.05, 12345.0,
            1200.0, 1200.0, 320.0, 240.0, 640.0, 480.0, 0.5, 1.0]


_SIG_OPTION_NAMES = ("NO", "NONE", "None", "none", "No", "NONE_OPTION")


def resolve_no_signal_handler_option():
    """SignalHandlerOptions 里"不装 handler"的成员值；解析不到返回 None。

    ⚠ 跨版本坑：ROS2 Jazzy 上这个成员叫 **`NO`**，**没有 `NONE`**
      （写死 `NONE` 会 AttributeError，节点一启动就崩 —— 阶段 A 踩过）。
    ⚠ Jazzy 上 `NO` 的**值就是 0**，所以判断必须用 `is not None`，不能用真值判断。
    """
    try:
        from rclpy.signals import SignalHandlerOptions
    except Exception:
        return None
    for name in _SIG_OPTION_NAMES:
        opt = getattr(SignalHandlerOptions, name, None)
        if opt is not None:
            return opt
    return None


def parse_vec3(s):
    parts = [float(x) for x in str(s).replace(" ", "").split(",")]
    if len(parts) != 3:
        raise argparse.ArgumentTypeError(f"需要 3 个分量（x,y,z）：{s!r}")
    return tuple(parts)


# ============================================================================
#  节点
# ============================================================================
class PhaseBGuidance(Node):

    def __init__(self, args):
        super().__init__("phase_b_guidance")
        self.args = args
        self.ns = args.namespace

        # ---- 场景参数 ----
        self.target_p0 = args.target_p0
        self.hit_radius = args.hit_radius
        self.a_boost = args.a_boost
        self.v_boost_min = args.v_boost_min
        self.boost_dir = args.boost_dir
        self.cruise_on = not args.no_cruise
        self.cruise_x = args.cruise_x          # None = 速度达标即发射（旧行为）
        self.v_kp = args.v_kp                  # 巡航段速度保持增益
        self.cruise_timeout = args.cruise_timeout
        # SETTLE 段水平稳定目标点（ENU）。"none"/"off" ⇒ 只做速度阻尼、不收敛位置
        _sxy = str(args.settle_xy).strip().lower()
        if _sxy in ("none", "off", "no", ""):
            self.settle_xy = None
        else:
            _p = [float(x) for x in _sxy.replace(" ", "").split(",")]
            if len(_p) != 2:
                raise SystemExit(f"--settle-xy 需要 2 个分量（x,y）或 none：{args.settle_xy!r}")
            self.settle_xy = (_p[0], _p[1])
        self.settle_kp = args.settle_kp
        self.settle_kd = args.settle_kd
        self.target_height = args.target_height
        self.rate = args.rate

        # ---- 制导核 ----
        self.lib, lib_path = load_guidance_lib(args.lib)
        self.guide = self.lib.pgu_create()
        self.lib.pgu_config_target(self.guide, darr(target_cfg(
            self.target_p0, args.target_type, args.target_speed,
            args.target_elev, args.target_azim,
            args.target_mass, args.target_cd, args.target_area, args.target_rho)))
        self.lib.pgu_config_guidance(self.guide, darr(guidance_cfg(
            args.law, args.N, args.a_max, args.t_go_freeze)))
        self.lib.pgu_config_sensor(self.guide, darr(sensor_cfg()))
        self.lib_path = lib_path
        self.t_flight = self.lib.pgu_target_flight_time(self.guide)
        self.t_hits_ground = bool(self.lib.pgu_target_hits_ground(self.guide))

        # ---- Gazebo 可视化（可选） ----
        # ⚠ 只做"取数 + 摆位置"，不含算法；接不上 Gazebo 时自动降级，不报错。
        self.gz = None
        self.gz_trail = []
        self.gz_tower_r = 1.2      # 与 scene_model.preset_pilot / 制导讲义一致
        self.gz_tower_h = 3.0
        if args.gz_viz:
            try:
                from gz_scene_sync import GzScene
                self.gz = GzScene(world=args.gz_world, hz=args.gz_hz)
            except Exception as e:                      # noqa: BLE001
                self.warn(f"⚠ 加载 gz_scene_sync 失败（{e}）⇒ 跳过 Gazebo 可视化")

        # 制导核 I/O 缓冲
        self.gcmd = darr([0.0] * 19)
        self.gdbg = darr([0.0] * 17)
        self.gtgt = darr([0.0] * 9)
        # 姿态四元数 **分量顺序 [w,x,y,z]**（与 PX4 / MAVLink 一致；单位四元数 = [1,0,0,0]）
        self.q_identity = darr([1.0, 0.0, 0.0, 0.0])   # IDEAL 传感器不消费

        # ---- 收尾开关 ----
        self.auto_land = not args.no_auto_land
        self.do_disarm = not args.no_disarm
        self.land_timeout = args.land_timeout
        self.linger = args.linger
        self.force_disarm_ok = not args.no_force_disarm
        self.alt_hold = not args.no_alt_hold
        self.alt_kp = args.alt_kp
        self.alt_kd = args.alt_kd
        self.csv_path = args.csv
        self.csv_hz = max(float(getattr(args, "csv_hz", 10.0) or 10.0), 0.1)
        self.csv_period = 1.0 / self.csv_hz      # CSV 采样间隔（s）；默认 0.1 s
        self.last_csv_t = -1e9                    # 上一次写 CSV 的墙钟（monotonic）
        # ---- 回传报告（可直接复制粘贴的评估块）----
        #   `--report-hz 0` ⇒ 用 `--rate`（50 Hz）：交会段只有 ~0.5 s，
        #   用 10 Hz 只剩 4~5 行，根本看不出交会过程（实飞已踩）。
        _rhz = float(getattr(args, "report_hz", 0.0) or 0.0)
        self.report_on = not getattr(args, "no_report", False)
        self.report_hz = _rhz if _rhz > 0 else self.rate
        self.report_period = 1.0 / self.report_hz
        self.report_mid_period = max(self.report_period, 1.0 / REPORT_MID_HZ)
        self.report_slow_period = max(self.report_period, 1.0 / REPORT_SLOW_HZ)
        self.last_report_t = -1e9
        self.report_rows = []                     # 报告用采样行（与 CSV 同格式）

        # ---- ★ 控制回路健康监测 ----
        self.t_last_tick = None
        self.loop_gap_max = 0.0                   # 相邻 tick 最大间隔（s）
        self.loop_gap_max_at = None               # 出现时刻（t_wall）
        self.loop_gap_max_state = None            # 出现时的状态
        self.loop_gaps = []                       # [(t_wall, gap, state), ...]
        # ---- ★ 落点 / 漂移（**实测**，不是估算）----
        self.ending_pos = None                    # 收尾开始时的 ENU 位置
        self.stop_pos = None                      # 停桨时的 ENU 位置
        self.max_h_disp = 0.0                     # 全程最大水平位移（相对起点）

        # ---- 遥测 ----
        self.pos = None                # VehicleLocalPosition（NED）
        self.status = None             # VehicleStatus
        self.nav_state = None
        self.arming_state = None
        self.odom = None
        self.odom_ok = False

        # ---- 状态机 ----
        # ARM → CONFIRM_ARM → TAKEOFF → WAIT_HEIGHT
        #     → OFFBOARD → CONFIRM_OFFBOARD → SETTLE
        #     → CRUISE → GUIDED → LAND → DISARM → DONE
        self.state = "ARM"
        self.t_state = time.monotonic()
        self.t_start = time.monotonic()
        self.done_t = None
        self.should_exit = False
        self.force_exit_requested = False
        self.aborted = False
        self.abort_reason = ""
        self.finish_reason = None             # 结束原因（命中/脱靶/超时/中止）
        self._exit_logged = False
        self._linger_logged = False

        self.stable_t = None
        self.last_alt_log = 0.0
        self.last_arm_cmd = 0.0
        self.last_resend = 0.0
        self.offboard_confirmed = False
        self.alt_target = None
        self.cur_a_enu = (0.0, 0.0, 0.0)     # 本周期要发的加速度（ENU）

        # 制导段
        self.t_launch_us = None               # 目标发射时刻（PX4 仿真时间 µs）
        self.t_rel = 0.0
        self.hit = False
        self.missed = False
        self.min_R = None
        self.t_min_R = 0.0
        self.R_prev = None
        self.sat_steps = 0
        self.guided_steps = 0
        self.peak_a_cmd = 0.0
        self.modes = {}
        self.last_cmd = None                  # 最近一次制导输出（19 元组）
        self.last_dbg = None                  # 最近一次真值（17 元组）
        self.guided_t0 = None
        self.cruise_t0 = None
        self.launch_geo = None                # 发射时刻几何（回传报告用）

        # 降落 / 停桨
        self.land_t0 = 0.0
        self.last_land_cmd = 0.0
        self.landed = False
        self.disarmed = False
        self.land_stable_t = None
        self.disarm_t0 = 0.0
        self.force_disarm_sent = False
        self.ending = False                   # 收尾流程已启动（BRAKE/LAND/DISARM 全算）
        self.landing_phase = False            # True 时改发 cmd_vel_ned（刹车段仍为 False）
        self.brake_t0 = 0.0
        self.soft_landing = None              # 停桨时是否"软着陆"（None=未判）
        self.off_ground_warned = False

        # ---- 接口 ----
        self.pub_accel = self.create_publisher(
            Twist, f"/xtdrone2/{self.ns}/cmd_accel_ned", 10)
        self.pub_vel = self.create_publisher(
            Twist, f"/xtdrone2/{self.ns}/cmd_vel_ned", 10)
        self.pub_vcmd = self.create_publisher(
            VehicleCommand, f"/{self.ns}/fmu/in/vehicle_command", 10)
        self.cli_cmd = self.create_client(XTD2Cmd, f"/xtdrone2/{self.ns}/cmd")

        qos = QoSProfile(depth=10, reliability=qos_profile_sensor_data.reliability)
        self.create_subscription(
            VehicleLocalPosition, f"/{self.ns}/fmu/out/vehicle_local_position",
            self.pos_cb, qos)
        self.create_subscription(
            VehicleStatus, f"/{self.ns}/fmu/out/vehicle_status",
            self.status_cb, qos)
        if not args.no_odom and HAVE_ODOM:
            self.create_subscription(
                Odometry, f"/{self.ns}/odometry", self.odom_cb, 10)

        # 非阻塞服务调用
        self.fut = None
        self.fut_cmd = None
        self.fut_t = 0.0
        self.srv_warned = False
        self.no_pos_warned = False

        # ---- CSV ----
        self.csv_f = None
        self.csv_w = None
        if self.csv_path:
            self.csv_f = open(self.csv_path, "w", newline="", encoding="utf-8")
            self.csv_w = csv.writer(self.csv_f)
            self.csv_w.writerow(CSV_COLUMNS)

        self.timer = self.create_timer(1.0 / self.rate, self.tick)
        self.report_banner()

    # ---------------------------------------------------------------- 工具
    def log(self, s):
        self.get_logger().info(s)

    def warn(self, s):
        self.get_logger().warn(s)

    def report_banner(self):
        bar = "=" * 76
        law_name = ["PPN", "TPN", "APN"][int(self.args.law)] if 0 <= int(self.args.law) <= 2 else "?"
        self.log(bar)
        self.log("Phase B · 制导闭环验证")
        self.log(f"  ns={self.ns}   制导核={os.path.basename(self.lib_path)} (ABI {self.lib.pgu_version()})")
        self.log(f"  目标: {'DART' if self.args.target_type == 2 else self.args.target_type} "
                 f"从 ({self.target_p0[0]:.1f},{self.target_p0[1]:.1f},{self.target_p0[2]:.1f}) 发射，"
                 f"{self.args.target_speed:.1f} m/s，仰角 {self.args.target_elev:.1f}°，"
                 f"方位 {self.args.target_azim:.1f}°")
        self.log(f"        目标飞行时间={self.t_flight:.2f}s  hits_ground={self.t_hits_ground}")
        self.log(f"  制导: law={law_name} N={self.args.N} a_max={self.args.a_max} "
                 f"t_go_freeze={self.args.t_go_freeze}   指令率 {self.rate:.0f} Hz")
        self.log(f"  预加速: {'ON' if self.cruise_on else 'OFF'}"
                 f"（朝 {self.boost_dir}，a={self.a_boost} m/s² 至 |v|≥{self.v_boost_min} m/s）")
        xy_txt = "不收敛位置（仅速度阻尼）" if self.settle_xy is None \
            else f"收敛到 ENU({self.settle_xy[0]:g},{self.settle_xy[1]:g})"
        self.log(f"  稳定段: 高度→{self.target_height:.1f} m；水平 {xy_txt}"
                 f"；横向保持={'ON' if self.args.cruise_lat_hold else 'OFF'}")
        self.log(f"  命中判定: R_true < {self.hit_radius} m")
        self.log(f"  Gazebo 可视化: {'ON（塔 + 弹丸 + ' + ('轨迹线' if self.args.gz_trail else '无轨迹线') + '）' if self.args.gz_viz else 'OFF'}"
                 + (f"  world={self.args.gz_world} @ {self.args.gz_hz:.0f} Hz" if self.args.gz_viz else ""))
        self.log("  符号约定: 日志里的 alt / 上升率 一律**上为正**；"
                 "CSV 同时给 px4_*(NED, z 向下为正) 与 enu_*(ENU, z 向上为正) 两套")
        self.log(f"  垂直保持辅助(非制导段): {'ON' if self.alt_hold else 'OFF'}（kp={self.alt_kp}, kd={self.alt_kd}）")
        self.log(f"  收尾: {'自动 LAND + 停桨' if self.auto_land else '不自动降落'} "
                 f"(land_timeout={self.land_timeout:.0f}s, 强制停桨兜底="
                 f"{'ON' if self.force_disarm_ok else 'OFF'})  Ctrl-C 一次=降落收尾，两次=强制退出")
        self.log(f"  CSV={os.path.abspath(self.csv_path) if self.csv_path else '(关闭)'}"
                 + (f"（采样 {self.csv_period * 1000:.0f} ms / {self.csv_hz:g} Hz，"
                    f"列含 px4_*(NED) / enu_*(ENU) / roll,pitch,yaw(deg)+quaternion）"
                    if self.csv_path else ""))
        self.log(f"  回传评估块  : {'ON' if self.report_on else 'OFF'}"
                 + (f"（运行结束时打印在末尾，前后有 {REPORT_BEGIN} 标记，"
                    f"整段复制即可；GUIDED {self.report_hz:g} Hz / "
                    f"稳定巡航刹车 {REPORT_MID_HZ:g} Hz / 其余 {REPORT_SLOW_HZ:g} Hz）"
                    if self.report_on else ""))
        self.log(bar)

    def now_us(self):
        """PX4 微秒时间戳（µs 仿真时间）。"""
        s, ns = self.get_clock().now().seconds_nanoseconds()
        return int(s * 1e6 + ns / 1000)

    def px4_us(self):
        """当前 PX4 仿真时间（µs）—— 用 vehicle_local_position 的时间戳。"""
        if self.pos is not None:
            return int(self.pos.timestamp)
        return self.now_us()

    # ---------------------------------------------------------------- 回调
    def pos_cb(self, msg):
        self.pos = msg

    def status_cb(self, msg):
        self.nav_state = int(msg.nav_state)
        self.arming_state = int(msg.arming_state)

    def odom_cb(self, msg):
        self.odom = msg
        self.odom_ok = True

    def odom_pose(self):
        if self.odom is None:
            return None
        p = self.odom.pose.pose.position
        return (p.x, p.y, p.z)

    def odom_att(self):
        """从 /odometry 解 ENU 姿态：返回 (roll, pitch, yaw, q) 或 None。

        - 四元数分量顺序 [w,x,y,z]（gz.msgs / nav_msgs/Odometry 都是**具名字段**，
          按名访问，与顺序无关；与 PX4 uORB / MAVLink 的顺序也一致）。
        - ENU：z 朝上 ⇒ yaw=0 朝东(+x)、yaw=+90° 朝北(+y)，与 `--target-azim`
          同一口径（"方位角从 +x 逆时针"）。
        - ⚠ NED 的 vehicle_local_position 只有 heading、没有 roll/pitch ⇒ 姿态
          **只能**来自 Odometry（ENU 真值）。不自己发假值。
        """
        if self.odom is None:
            return None
        q = self.odom.pose.pose.orientation
        w, x, y, z = q.w, q.x, q.y, q.z
        if w * w + x * x + y * y + z * z < 1e-9:
            return None
        # ZYX 欧拉角（yaw-pitch-roll）标准式
        sinp = 2.0 * (w * y - z * x)
        sinp = max(-1.0, min(1.0, sinp))
        roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        pitch = math.asin(sinp)
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return (math.degrees(roll), math.degrees(pitch), math.degrees(yaw),
                (w, x, y, z))

    # ------------------------------------------------ PX4 状态量 → ENU
    def state_enu(self):
        """从 PX4 的 NED 状态换算出 ENU 的 (p, v)。

        PX4 NED: x=北, y=东, z=下  →  ENU: p=(y, x, −z)，v=(vy, vx, −vz)
        （阶段 A 已实测验证：max|err| = 0.025 m）
        """
        if self.pos is None:
            return None, None
        p = self.pos
        p_enu = (p.y, p.x, -p.z)
        v_enu = (p.vy, p.vx, -p.vz)
        return p_enu, v_enu

    def h_disp(self):
        """相对起飞点的水平位移（安全围栏用）。"""
        return math.hypot(self.pos.x - getattr(self, "start_x", 0.0),
                          self.pos.y - getattr(self, "start_y", 0.0))

    def record_stop(self):
        """记下**停桨时刻的真实位置**（报告里算实测着陆漂移用）。"""
        p_enu, _ = self.state_enu()
        if p_enu is not None:
            self.stop_pos = p_enu

    def track_h_disp(self):
        """更新全程最大水平位移（相对起飞点，NED 直算，不受 ENU 换算影响）。"""
        if self.pos is None:
            return
        h = self.h_disp()
        if h > self.max_h_disp:
            self.max_h_disp = h

    def landing_drift(self):
        """**实测**着陆漂移：收尾起点 → 停桨点（水平）。

        返回 (漂移 m, Δx, Δy) 或 None。
        ⚠ 为什么必须实测：原来日志只给估算 `v²/(2·a_max)`，实飞里写着
          「残余漂移 ≈ 0.02 m」而**实际漂了 10 m**（差 500 倍）——
          估算完全没有把"刹车阶段实际发生了什么"算进去。
        """
        if self.ending_pos is None or self.stop_pos is None:
            return None
        dx = self.stop_pos[0] - self.ending_pos[0]
        dy = self.stop_pos[1] - self.ending_pos[1]
        return (math.hypot(dx, dy), dx, dy)

    # ---------------------------------------------------------------- 发布
    def publish_a(self):
        """发布加速度指令（ENU → NED）。"""
        if self.landing_phase:
            return
        a = self.cur_a_enu
        m = Twist()
        m.linear.x = float(a[1])      # NED 北 = ENU 东
        m.linear.y = float(a[0])      # NED 东 = ENU 北
        m.linear.z = float(-a[2])     # NED 下 = −ENU 上
        self.pub_accel.publish(m)

    def publish_descend_vel(self):
        """降落阶段发 cmd_vel_ned：给一个真实的下降速度（ENU 下 → NED +z）。

        ⚠ 为什么必须这样：`cmd_accel_ned` 在 PX4 侧是纯前馈，XTDrone2 会以 20 Hz
          灌 `velocity = [NaN,NaN,NaN]` 的 trajectory_setpoint，把 PX4 的落地检测
          判据 `_in_descend` 永久打成 false ⇒ 落地判不出 ⇒ DISARM 被拒（阶段 A 的坑 8）。
          改发 cmd_vel_ned 后，两个发布者的信号一致，落地检测恢复正常。
        """
        m = Twist()
        m.linear.x = 0.0
        m.linear.y = 0.0
        m.linear.z = float(self.args.land_down_speed)   # NED +z = 向下
        self.pub_vel.publish(m)

    def _vcmd(self, command, param1=0.0, param2=0.0, param4=0.0,
              param5=0.0, param6=0.0, param7=0.0):
        m = VehicleCommand()
        m.timestamp = self.now_us()
        m.command = int(command)
        m.param1 = float(param1)
        m.param2 = float(param2)
        m.param4 = float(param4)
        m.param5 = float(param5)
        m.param6 = float(param6)
        m.param7 = float(param7)
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        self.pub_vcmd.publish(m)

    def publish_offboard_mode(self):
        """直接对 PX4 重发 DO_SET_MODE(OFFBOARD)（绕开 XTDrone2 的一次性切换时序）。"""
        self._vcmd(VEHICLE_CMD_DO_SET_MODE, param1=1.0,
                   param2=float(PX4_CUSTOM_MAIN_MODE_OFFBOARD))

    def publish_land(self):
        self._vcmd(VEHICLE_CMD_NAV_LAND, param1=0.0, param4=0.0,
                   param5=0.0, param6=0.0, param7=0.0)

    def publish_disarm(self, force=False):
        """停桨。force=False 时 param2=0（PX4 未落地会拒绝，这是安全网）；
        force=True 时 param2=21196（强制停桨，仅在我们自己的落地判据持续满足时兜底）。"""
        self._vcmd(VEHICLE_CMD_COMPONENT_ARM_DISARM,
                   param1=0.0, param2=(21196.0 if force else 0.0))

    # ---------------------------------------------------------------- 服务
    def issue(self, command):
        if self.fut is not None and not self.fut.done():
            return False
        if not self.cli_cmd.service_is_ready():
            if not self.srv_warned:
                self.srv_warned = True
                self.warn(f"服务 /xtdrone2/{self.ns}/cmd 还没就绪，等待中…")
            return False
        req = XTD2Cmd.Request()
        req.command = command
        self.fut = self.cli_cmd.call_async(req)
        self.fut_cmd = command
        self.fut_t = time.monotonic()
        return True

    def poll_service(self, now):
        if self.fut is None:
            return
        if not self.fut.done():
            if now - self.fut_t > SRV_TIMEOUT:
                self.warn(f"服务 {self.fut_cmd} 超时（{SRV_TIMEOUT}s）无响应，重试…")
                self.fut = None
            return
        cmd = self.fut_cmd
        try:
            res = self.fut.result()
            ok = bool(getattr(res, "success", False))
        except Exception as e:                        # noqa: BLE001
            ok = False
            self.warn(f"服务 {cmd} 异常：{e!r}")
        self.fut = None
        self.log(f"  service {cmd:<9} → success={ok}")
        self.on_ack(cmd, ok)

    def on_ack(self, cmd, ok):
        if self.state == "ARM" and cmd == "ARM":
            self.state = "CONFIRM_ARM"
            self.t_state = time.monotonic()
            self.log("      （ARM 服务已回；是否真解锁以 vehicle_status.arming_state 为准）")
        elif self.state == "TAKEOFF" and cmd == "TAKEOFF":
            self.state = "WAIT_HEIGHT"
            self.t_state = time.monotonic()
        elif self.state == "OFFBOARD" and cmd == "OFFBOARD":
            self.cur_a_enu = (0.0, 0.0, 0.0)
            self.state = "CONFIRM_OFFBOARD"
            self.t_state = time.monotonic()
            self.last_resend = 0.0
            self.log("      （服务 success 只说明通信节点收到了；是否真进 OFFBOARD 以 vehicle_status 为准）")

    # ================================================================ 主循环
    def tick(self):
        now = time.monotonic()

        # ---------------- ★ 控制回路健康监测（2026-10-16 新增）----------------
        #   动机：实飞里 `gz_finish` 的阻塞调用把主回路卡了 1.0 s（命中瞬间！），
        #   但那 1 s 在日志里**看不出来** —— 只有对比两行的时间戳才发现。
        #   ⇒ 现在主动测"相邻 tick 的墙钟间隔"，超过阈值就当场 warn 并记进报告。
        #   ⚠ 这里量的是**主回路是否被阻塞**（含 gz 服务调用 / 日志 / 磁盘），
        #     不是 PX4 的控制周期 —— 但正是它决定了 setpoint 有没有按时发出去。
        if self.t_last_tick is not None:
            gap = now - self.t_last_tick
            if gap > self.loop_gap_max:
                self.loop_gap_max = gap
                self.loop_gap_max_at = now - self.t_start
                self.loop_gap_max_state = self.state
            if gap >= LOOP_GAP_WARN:
                tw = now - self.t_start
                self.loop_gaps.append((round(tw, 3), round(gap, 3), self.state))
                self.warn(f"⚠ 控制回路空洞 {gap * 1000:.0f} ms"
                          f"（t_wall={tw:.3f}s, state={self.state}）"
                          f"—— 本周期**没有**发出加速度 setpoint；"
                          f"检查是不是可视化 / 日志在阻塞主线程")
        self.t_last_tick = now

        self.poll_service(now)

        self.track_h_disp()          # 全程最大水平位移（报告用）

        # ---------------- Ctrl-C 收尾请求 ----------------
        # ⚠ 用 `ending` 而不是 `landing_phase` —— BRAKE 段还没进 landing_phase，
        #   若用它判断，第二次 Ctrl-C 会重复触发收尾。
        if self.force_exit_requested and self.state != "DONE" and not self.ending:
            self.request_landing("Ctrl-C")

        # ---------------- 状态机 ----------------
        if self.state == "ARM":
            if self.issue("ARM"):
                self.log("  → 发出 ARM")

        elif self.state == "CONFIRM_ARM":
            if self.arming_state == ARMING_STATE_ARMED:
                self.log(f"[1/6] ✓ 已解锁（arming_state=ARMED）")
                self.state = "POST_ARM"
                self.t_state = now
            else:
                if now - self.last_arm_cmd >= ARM_RESEND_SEC:
                    self.last_arm_cmd = now
                    self.issue("ARM")
                if now - self.t_state > ARM_CONFIRM_TIMEOUT:
                    self.abort(f"解锁确认超时（{ARM_CONFIRM_TIMEOUT:.0f}s）："
                               f"arming_state={self.arming_state}（期望 2=ARMED）"
                               f" —— PX4 拒绝了 ARM。检查 QGC 的预解锁检查项"
                               f"（EKF / 磁力计 / 遥控 / 电池）。")
                    return

        elif self.state == "POST_ARM":
            if now - self.t_state >= POST_ARM_DELAY:
                self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            if self.issue("TAKEOFF"):
                self.log(f"[2/6] TAKEOFF …（目标 {self.target_height:.1f} m）")

        elif self.state == "WAIT_HEIGHT":
            alt = -self.pos.z if self.pos is not None else None
            if alt is None:
                if not self.no_pos_warned:
                    self.no_pos_warned = True
                    self.warn("收不到 vehicle_local_position —— 仿真在跑吗？")
                if now - self.t_state > 10.0:
                    self.abort("收不到位置遥测")
                return
            # ⚠ PX4 的 vehicle_local_position.vz 是 **NED**：z 轴朝下 ⇒ vz>0 = 下降。
            #   而上面的 alt = −z 是"上为正"。两者直接并排打印会让人以为"爬升时 vz 为负=有 bug"。
            #   这里统一转成 **上升率为正**（vz_up = −vz_ned），与 alt 同一约定。
            vz_ned = self.pos.vz
            vz_up = -vz_ned
            near = abs(alt - self.target_height) <= self.args.takeoff_tol
            airborne = (alt > 0.75 * self.target_height) and abs(vz_ned) < 0.4
            if near or airborne:
                if self.stable_t is None:
                    self.stable_t = now
                if now - self.stable_t >= STABLE_SEC:
                    self.log(f"[3/6] 起飞到位：alt={alt:.2f} m, 上升率={vz_up:+.2f} m/s"
                             f"（目标 {self.target_height:.1f} m）→ 请求 OFFBOARD …")
                    self.alt_target = alt
                    self.state = "OFFBOARD"
            else:
                self.stable_t = None
            if now - self.last_alt_log > 3.0:
                self.last_alt_log = now
                self.log(f"     爬升中… alt={alt:.2f} m  上升率={vz_up:+.2f} m/s  "
                         f"nav_state={self.nav_state}  arming_state={self.arming_state}")
                if self.arming_state == ARMING_STATE_DISARMED:
                    self.warn("     ⚠ 车辆处于 DISARMED —— 不可能起飞。")
                elif self.nav_state is not None and self.nav_state not in (
                        NAV_STATE_AUTO_TAKEOFF, 4):    # 4 = AUTO_LOITER（起飞完成后的正常态）
                    self.warn(f"     ⚠ nav_state={self.nav_state} 异常")
            if now - self.t_state > self.args.takeoff_timeout:
                self.abort(f"起飞超时（当前 {alt:.2f} m）")

        elif self.state == "OFFBOARD":
            if self.issue("OFFBOARD"):
                self.log("  → 发出 OFFBOARD")

        elif self.state == "CONFIRM_OFFBOARD":
            self.publish_a()          # 持续流 setpoint 把 offboard 信号养起来
            if self.nav_state == NAV_STATE_OFFBOARD:
                dt = now - self.t_state
                self.offboard_confirmed = True
                self.log(f"[4/6] ✓ PX4 已确认进入 OFFBOARD（nav_state=14，耗时 {dt:.2f}s）")
                self.state = "SETTLE"
                self.t_state = now
            else:
                if now - self.last_resend >= OFFBOARD_RESEND_SEC:
                    self.last_resend = now
                    self.publish_offboard_mode()
                if now - self.t_state > OFFBOARD_CONFIRM_TIMEOUT:
                    self.abort(f"OFFBOARD 确认超时（{OFFBOARD_CONFIRM_TIMEOUT:.0f}s）："
                               f"nav_state={self.nav_state}（期望 14）")
                    return

        elif self.state == "SETTLE":
            # ★ 关键 1：把高度**主动调到** --target-height
            #   原因：XTDrone2 的 TAKEOFF 把高度写死成 10 m
            #     （multirotor_communication.py:320 `param7=10.0`），
            #   我们的 --target-height 控制不了 PX4 爬多高 ⇒ 必须在这里补一次垂直调整。
            #
            # ★ 关键 2：**横向也要稳**（2026-10-16 实飞踩到）
            #   accel-only 是纯前馈无阻尼（坑 1）：起飞残留的水平速度进 OFFBOARD 后
            #   **永不衰减**。原先 SETTLE 只控垂直 ⇒ 横向一路漂（实测 PPN 发射时
            #   ENU y=-0.99 m、APN 起点 x=+2.39 m，都偏离走廊）⇒ 横向偏差直接吃掉
            #   "½·a_max·t_go² ≈ 1.25 m" 的全部修正预算 ⇒ PPN 脱靶。
            #   ⇒ 这里把水平位置**收敛到 --settle-xy**（默认 0,0，即塔的正上方走廊）。
            alt = -self.pos.z if self.pos is not None else None
            v_z_up = 0.0
            a = [0.0, 0.0, 0.0]
            xy_err = None
            v_h = 0.0
            if alt is not None:
                p_enu, v_enu = self.state_enu()
                v_z_up = v_enu[2]
                if self.alt_hold:
                    a[2] = self.alt_kp * (self.target_height - alt) - self.alt_kd * v_z_up
                    a[2] = max(-ALT_ADJ_A_MAX, min(ALT_ADJ_A_MAX, a[2]))
                if self.settle_xy is not None:
                    sx, sy = self.settle_xy
                    a[0] = (self.settle_kp * (sx - p_enu[0]) - self.settle_kd * v_enu[0])
                    a[1] = (self.settle_kp * (sy - p_enu[1]) - self.settle_kd * v_enu[1])
                    a[0] = max(-ALT_ADJ_A_MAX, min(ALT_ADJ_A_MAX, a[0]))
                    a[1] = max(-ALT_ADJ_A_MAX, min(ALT_ADJ_A_MAX, a[1]))
                    xy_err = math.hypot(p_enu[0] - sx, p_enu[1] - sy)
                    v_h = math.hypot(v_enu[0], v_enu[1])
            self.cur_a_enu = tuple(a)
            self.publish_a()

            # --no-alt-hold 时退回旧行为：只等固定 0.5 s，不做任何调整
            settled = (not self.alt_hold) or (
                alt is not None
                and abs(alt - self.target_height) <= self.args.alt_settle_tol
                and abs(v_z_up) <= self.args.alt_settle_vtol
                and (xy_err is None
                     or (xy_err <= self.args.settle_xy_tol
                         and v_h <= self.args.settle_xy_vtol)))
            if (settled and now - self.t_state >= 0.5) \
                    or (now - self.t_state >= self.args.alt_settle_timeout):
                if alt is not None:
                    xy_txt = "" if xy_err is None else f"，水平偏差 {xy_err:.2f} m"
                    if settled:
                        self.log(f"[5/6] 稳定到位：alt={alt:.2f} m（目标 "
                                 f"{self.target_height:.1f} m）{xy_txt}")
                    else:
                        self.warn(f"[5/6] ⚠ 未完全稳定（alt={alt:.2f} m / 目标 "
                                  f"{self.target_height:.1f} m{xy_txt}）——"
                                  f"已等 {self.args.alt_settle_timeout:.0f}s，继续")
                self.alt_target = self.target_height   # 之后 alt_hold 维持这个高度
                self.start_x = self.pos.x
                self.start_y = self.pos.y
                self.gz_setup()          # 摆塔 / 发射架 / 弹丸（--gz-viz 才动作）
                if self.cruise_on:
                    self.check_cruise_geometry()
                    self.state = "CRUISE"
                    self.cruise_t0 = now
                    goal = ""
                    if self.cruise_x is not None:
                        sign = "≤" if self.boost_dir[0] < 0.0 else "≥"
                        goal = f"，巡航到 x {sign} {self.cruise_x:+.2f} m 后发射"
                    self.log(f"  ── CRUISE 巡航：朝 ({self.boost_dir[0]:+.2f},"
                             f"{self.boost_dir[1]:+.2f},{self.boost_dir[2]:+.2f}) "
                             f"a={self.a_boost:.1f} m/s²，目标 |v|≥{self.v_boost_min:.1f} m/s{goal}")
                else:
                    self.begin_guided(now)

        elif self.state == "CRUISE":
            self.do_cruise(now)
            if self.state == "CRUISE":
                self.check_safety()

        elif self.state == "GUIDED":
            self.do_guided(now)
            self.check_safety()

        elif self.state == "BRAKE":
            # 着陆前刹车：水平按 −kd·v_h 减速；垂直保持目标高度（仍在 OFFBOARD，可以发 accel）
            if self.pos is None:
                return
            p_enu, v_enu = self.state_enu()
            vh = math.hypot(v_enu[0], v_enu[1])
            alt = p_enu[2]
            a = [-self.args.brake_kd * v_enu[0],
                 -self.args.brake_kd * v_enu[1],
                 self.alt_kp * (self.target_height - alt) - self.alt_kd * v_enu[2]]
            # 矢量等比限幅（铁律 2：禁止逐分量 clip）
            na = math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)
            if na > self.args.a_max and na > 1e-9:
                k = self.args.a_max / na
                a = [x * k for x in a]
            self.cur_a_enu = tuple(a)
            self.publish_a()

            done = vh <= self.args.brake_vtol
            timed_out = now - self.brake_t0 > self.args.brake_timeout
            if done or timed_out:
                if done:
                    self.log(f"   ✓ 刹车完成：|v_h|={vh:.2f} m/s"
                             f"（耗时 {now - self.brake_t0:.2f}s）")
                    # ⚠ 这里只能给"剩余速度对应的估算刹车距离"，
                    #   真实漂移必须用实测 —— 实飞里估算 0.02 m 而实测 10 m。
                    self.log(f"      剩余速度对应的估算刹车距离 ≈ {vh**2 / (2 * 2.0):.2f} m"
                             f"（**实测**着陆漂移见末尾报告的 [落点] 行）")
                else:
                    self.warn(f"   ⚠ 刹车超时（{self.args.brake_timeout:.0f}s）："
                              f"|v_h|={vh:.2f} m/s —— 继续 LAND（会漂更多）")
                self.log("→ 转入自动降落：LAND（AUTO_LAND 下降 + cmd_vel_ned）"
                         "→ 落地后 DISARM 停桨。")
                self.landing_phase = True
                self.alt_hold = False
                self.state = "LAND"
                self.t_state = now
                self.land_t0 = now
                self.last_land_cmd = 0.0

        elif self.state == "LAND":
            self.publish_descend_vel()
            if now - self.last_land_cmd >= LAND_RESEND_SEC:
                self.last_land_cmd = now
                self.publish_land()
                if self.nav_state != NAV_STATE_AUTO_LAND:
                    self.log(f"      LAND 已发（nav_state={self.nav_state}，"
                             f"期望 {NAV_STATE_AUTO_LAND}=AUTO_LAND）")
            alt = -self.pos.z if self.pos is not None else None
            vz = self.pos.vz if self.pos is not None else None
            on_ground = (alt is not None and vz is not None
                         and alt <= LANDED_ALT and abs(vz) <= LANDED_VZ)
            if on_ground:
                if self.land_stable_t is None:
                    self.land_stable_t = now
                elif now - self.land_stable_t >= LANDED_HOLD:
                    self.log(f"✓ 已落地：alt={alt:.2f} m, 下降率={vz:+.3f} m/s"
                             f"（判据：alt≤{LANDED_ALT} m 且 |vz|≤{LANDED_VZ} m/s "
                             f"持续 {LANDED_HOLD:.0f}s）→ 停桨 DISARM")
                    self.landed = True
                    self.state = "DISARM"
                    self.t_state = now
                    self.disarm_t0 = now
                    self.last_land_cmd = 0.0
                    self.land_stable_t = None
                    return
            else:
                self.land_stable_t = None
            if self.arming_state == ARMING_STATE_DISARMED:
                # ★ PX4 自己停的桨 —— 必须检查**高度**（2026-10-16 实飞踩到）
                #   实飞 APN 那次在 **alt=0.73 m** 就报"✓ 已停桨（PX4 自动）"，
                #   而 0.73 m 掉下来是**硬摔**，不是正常落地。原代码只看 + 号就打了 ✓。
                alt_s = "?" if alt is None else f"{alt:.2f}"
                if alt is not None and alt > LANDED_ALT + 0.15:
                    self.warn(f"⚠ PX4 在我们的落地判据之外就停桨了：alt={alt_s} m "
                              f"（判据要求 ≤{LANDED_ALT} m）")
                    self.warn(f"   ⇒ 无人机是从 {alt_s} m **掉下来**的，不是落下来的；"
                              f"真实机上可能摔坏桨/起落架。")
                    self.warn("   可能原因：PX4 落地检测提前触发（LNDMC_* 参数、"
                              "高度估计漂移、或下降速度太慢被误判为静止）。")
                    self.soft_landing = False
                else:
                    self.log(f"✓ 已停桨（PX4 自动，alt={alt_s} m）")
                    self.soft_landing = True
                self.disarmed = True
                self.record_stop()
                self.state = "DONE"
                self.done_t = now
                return
            if self.pos is None and self.arming_state is None and now - self.land_t0 > 5.0:
                self.warn("⚠ 收不到任何遥测，无法确认降落 —— 直接退出。")
                self.state = "DONE"
                self.done_t = now
            elif now - self.land_t0 > self.land_timeout:
                self.warn(f"⚠ 降落超时（{self.land_timeout:.0f}s）：alt={alt}，"
                          f"arming_state={self.arming_state}，nav_state={self.nav_state}")
                self.warn("   放弃自动收尾 —— 请到 QGC 里手动 LAND；或再按一次 Ctrl-C 强制退出。")
                self.state = "DONE"
                self.done_t = now

        elif self.state == "DISARM":
            self.publish_descend_vel()
            if self.arming_state == ARMING_STATE_DISARMED:
                self.disarmed = True
                self.record_stop()
                self.log("✓ 已停桨（arming_state=DISARMED）。")
                self.state = "DONE"
                self.done_t = now
                return
            alt = -self.pos.z if self.pos is not None else None
            vz = self.pos.vz if self.pos is not None else None
            still_on_ground = (alt is not None and vz is not None
                               and alt <= LANDED_ALT and abs(vz) <= LANDED_VZ)
            if not still_on_ground:
                self.land_stable_t = None
                if not self.off_ground_warned:
                    self.off_ground_warned = True
                    self.warn(f"⚠ 已离开地面判据（alt={alt:.2f} m, "
                              f"下降率={vz:+.3f} m/s）—— 暂停发停桨指令。")
                if now - self.disarm_t0 > DISARM_TIMEOUT:
                    self.warn(f"⚠ 停桨超时（{DISARM_TIMEOUT:.0f}s）：arming_state="
                              f"{self.arming_state}。请在 QGC 里确认。")
                    self.state = "DONE"
                    self.done_t = now
                return
            if now - self.disarm_t0 > DISARM_TIMEOUT:
                self.warn(f"⚠ 停桨超时（{DISARM_TIMEOUT:.0f}s）：arming_state="
                          f"{self.arming_state}。请在 QGC 里确认。")
                self.state = "DONE"
                self.done_t = now
            elif now - self.last_land_cmd >= DISARM_RETRY_SEC:
                self.last_land_cmd = now
                if not self.do_disarm:
                    self.log(f"      等待 PX4 自动停桨（arming_state={self.arming_state}）")
                elif (self.force_disarm_ok and not self.force_disarm_sent
                      and now - self.disarm_t0 >= FORCE_DISARM_AFTER):
                    self.force_disarm_sent = True
                    self.warn("⚠ 普通 DISARM 无效 → 改用**强制停桨**（param2=21196）。")
                    self.publish_disarm(force=True)
                else:
                    self.publish_disarm(force=False)
                    self.log(f"      DISARM 已发（arming_state={self.arming_state}）")

        elif self.state == "DONE":
            if not self._exit_logged and now - self.done_t > self.linger:
                self._exit_logged = True
                self.log_row(force=True)      # 收尾补最后一行（绕过 0.1s 采样节流）
                if self.disarmed:
                    self.log("收尾完成：已落地并停桨。节点退出。")
                elif self.landed:
                    self.log("收尾完成：已落地（未确认停桨）。节点退出。")
                else:
                    self.log("收尾完成（未走完整降落流程）。节点退出。")
                # ★ 打印"可直接复制回传"的评估块（裸 print，无 ROS 前缀）
                self.print_report()
                self.should_exit = True
            return

        self.log_row()

    # ---------------------------------------------------------------- 段
    # ------------------------------------------------- Gazebo 可视化（可选）
    def gz_setup(self):
        """在 Gazebo 里摆好防御塔 + 敌方发射架 + 弹丸（弹丸先藏在视野外）。

        ⚠ 只在 `--gz-viz` 且成功接上 Gazebo 时才有动作；任何失败都只警告不中断
          （可视化是"锦上添花"，绝不能因为它挂掉而影响制导回路）。
        """
        if self.gz is None or not self.gz.available:
            if self.gz is not None:
                self.warn("⚠ Gazebo 可视化不可用（见上面的原因）—— 继续纯控制台运行")
            return False
        # ⚠ 全程 try/except：可视化是"锦上添花"，**任何**异常都不得中断制导回路
        try:
            self.gz.start()
            ok_t = self.gz.ensure_tower(self.gz_tower_r, self.gz_tower_h)
            self.gz.ensure_launcher((self.target_p0[0], self.target_p0[1], 0.0))
            ok_p = self.gz.ensure_projectile()
        except Exception as e:                          # noqa: BLE001
            self.warn(f"⚠ Gazebo 初始化抛异常（{e}）—— 跳过可视化，制导不受影响")
            self.gz = None
            return False
        if ok_t and ok_p:
            self.log(f"  ── Gazebo 可视化已就绪：防御塔 r={self.gz_tower_r} m / "
                     f"h={self.gz_tower_h} m @ 原点；敌方发射架 @ "
                     f"({self.target_p0[0]:.1f},{self.target_p0[1]:.1f},0)；弹丸已就位")
        else:
            self.warn("⚠ Gazebo 实体创建失败（仿真在跑吗？世界名对吗？）"
                      "—— 制导不受影响，继续运行")
        return bool(ok_t and ok_p)

    def gz_update(self, p_T_enu):
        """每帧把**弹丸（目标）**位置推给 Gazebo（**非阻塞**，失败静默）。同时记轨迹点。

        ⚠ 传的是 `p_T`（弹丸），不是 `p_M`（无人机）—— 无人机本身由 PX4/Gazebo 自己渲染。
        ⚠ 包 try/except：绝不允许可视化把制导循环搞挂。
        """
        if self.gz is None or not self.gz.available:
            return
        try:
            self.gz.set_projectile(p_T_enu)
            if self.args.gz_trail:
                self.gz_trail.append(tuple(p_T_enu))
        except Exception:                               # noqa: BLE001
            self.gz = None          # 一次失败就彻底停掉可视化，避免每帧刷屏
            self.warn("⚠ Gazebo 更新失败 —— 已停用可视化（制导不受影响）")

    def gz_finish(self, hit):
        """收尾：命中画爆点 + 可选轨迹线。

        ★★ **必须非阻塞**（2026-10-16 实飞实测教训）：
          以前这里直接调 `draw_trail` / `mark_point` / `hide_projectile`，
          三次阻塞服务调用**各花 0.50 s**（= `GzScene.timeout_ms` 上限），
          实飞日志证据：624.414「已画弹丸轨迹」→ 624.914「已标记命中点」
          ⇒ **主回路被卡了整整 1.0 s**，而这 1 s 内 50 Hz 的加速度 setpoint
          **一条都没发出去**，正好落在命中瞬间。
          这违反本项目硬规矩：「可视化任何情况下都不得影响制导回路」。
          ⇒ 现在只把动作**入队**，由 GzScene 的后台线程去发（主回路微秒级返回）。
        """
        if self.gz is None or not self.gz.available:
            return
        try:
            n_pts = 0
            if self.gz_trail:
                n_pts = len(self.gz_trail)
                self.gz.queue_trail(self.gz_trail)
            if hit and self.gz_trail:
                self.gz.queue_mark(self.gz_trail[-1], ns="hit_mark", mid=1,
                                   size=0.6, color=(1.0, 0.95, 0.1), ttl_ms=15000)
            self.gz.queue_hide_projectile()
            self.log(f"  ── Gazebo 收尾绘制已**派发给后台线程**"
                     f"（轨迹 {n_pts} 点{' + 命中标记' if hit and n_pts else ''}，"
                     f"不阻塞控制回路）")
        except Exception as e:             # noqa: BLE001
            self.warn(f"⚠ Gazebo 收尾绘制排队失败（{e}）—— 不影响结果")

    def check_cruise_geometry(self):
        """巡航几何自检（仅在指定 --cruise-x 时有意义）。

        ⚠ 为什么需要它：跑完一个 law 后是 LAND + DISARM，无人机停在**拦截点附近**
          而非原点。若紧接着跑第二个 law，起点就漂了 ⇒
            ① 到 --cruise-x 的位移变短（甚至起点已在阈值之外，一达标就发射）；
            ② 弹丸发射点是**绝对坐标**，与无人机的相对几何随之改变 ⇒ 两个 law 不可比。
        这里把它变成**显式告警**，而不是静默给出不可比的结果。
        """
        if self.cruise_x is None:
            return
        d = self.boost_dir
        if abs(d[0]) < 0.5:
            self.warn(f"  ⚠ --cruise-x 只对沿 ±x 巡航有意义，但 boost_dir[0]="
                      f"{d[0]:+.2f} ⇒ 位置判据可能永不成立")
            return
        # ⚠ 必须用 **ENU** 的 x —— `--cruise-x` 是按 ENU 定义的；
        #   而 `self.start_x` 取自 PX4 的 NED（x=北），两者不可直接比较。
        p_enu, _ = self.state_enu()
        if p_enu is None:
            return
        x0 = p_enu[0]
        beyond = (x0 <= self.cruise_x) if d[0] < 0.0 else (x0 >= self.cruise_x)
        if beyond:
            cmp_sign = "<" if d[0] < 0.0 else ">"
            self.warn(f"  ⚠ 起点 x={x0:+.2f} m 已在巡航阈值 {self.cruise_x:+.2f} m "
                      f"的 {cmp_sign} 侧 ⇒ 速度一达标就立刻发射，几何与守备场景不符。")
            self.warn("     多半是上一次跑完停在拦截点、没回原点 —— "
                      "请**重启仿真**后再跑（或接受本次几何已漂移）。")
        elif abs(x0) > 1.5:
            self.warn(f"  ⚠ 起点 x={x0:+.2f} m 偏离世界原点较多（守备场景预期 ≈0）"
                      f" ⇒ 弹丸发射点（绝对坐标）与无人机的相对几何可能不符预期。")

    def do_cruise(self, now):
        """巡航/预加速段：**两阶段** —— 先加速到 v_boost_min，再把速度保持住。

        · 阶段 1（|v| < v_boost_min）：朝 boost_dir 恒加速 a_boost（= 旧行为）
        · 阶段 2（|v| ≥ v_boost_min）：沿 boost_dir 的 P 控制把速度**按住**，
          不再继续加速 —— 守备场景要求"以一定速度匀速巡航"，而不是一路顶到上限。

        发射判据：
          · 未指定 --cruise-x → 速度一达标就发射（与旧版行为一致）
          · 指定 --cruise-x   → 还要"x 已越过该值"才发射
            （boost_dir 朝 −x 用 ≤，朝 +x 用 ≥；仅适用于沿 ±x 巡航）
        """
        if self.pos is None:
            return
        p_enu, v_enu = self.state_enu()
        alt = -self.pos.z          # 以"地面 z=0"计的离地高度（本仿真无人机从原点起飞）
        v_z_up = v_enu[2]          # ENU 的 z 速度（上为正）
        d = self.boost_dir
        v_norm = math.sqrt(v_enu[0] ** 2 + v_enu[1] ** 2 + v_enu[2] ** 2)
        speed_ok = v_norm >= self.v_boost_min

        if not speed_ok:
            a_along = self.a_boost                                  # 阶段 1：恒加速
        else:
            v_along = v_enu[0] * d[0] + v_enu[1] * d[1] + v_enu[2] * d[2]
            a_along = self.v_kp * (self.v_boost_min - v_along)      # 阶段 2：速度保持
            a_along = max(-self.a_boost, min(self.a_boost, a_along))

        a = [a_along * d[0], a_along * d[1], a_along * d[2]]
        if self.alt_hold and self.alt_target is not None:
            # ⚠ 符号必须让"太高 → 向下加速"。ENU 的 z 向上为正，所以
            #   a[2] = kp·(目标 − 当前) − kd·v_z
            #   （原写成 (当前 − 目标) 是**正反馈**：偏差会被放大 —— 实飞日志里
            #     起飞到位 10.13 m 却在发射时涨到 10.76 m，就是这个 bug 的表现。）
            a[2] += self.alt_kp * (self.alt_target - alt) - self.alt_kd * v_z_up
        # ★ 横向保持：巡航要走直线，不能被 accel-only 的"残留速度永不衰减"带偏。
        #   （2026-10-16 实飞：PPN 发射时横向已偏 0.99 m，吃掉全部修正预算。）
        #   ⚠ 只阻尼**垂直于 boost_dir 的水平分量** —— 不能直接阻尼 v_enu[0]/[1]：
        #     巡航主方向常就是 ENU 的 ±x（boost_dir=(-1,0,0)），那样会把加速也阻尼掉
        #     （实测：巡航段速度涨不起来、一路飞到 x=-22.6 然后超时）。这个错我犯过。
        if self.args.cruise_lat_hold:
            dhx, dhy = d[0], d[1]
            nd = math.hypot(dhx, dhy)
            if nd > 1e-6:
                ux, uy = dhx / nd, dhy / nd
                v_along_h = v_enu[0] * ux + v_enu[1] * uy
                p_x = v_enu[0] - v_along_h * ux     # 水平速度的垂直分量
                p_y = v_enu[1] - v_along_h * uy
                a[0] += -self.settle_kd * p_x
                a[1] += -self.settle_kd * p_y
        self.cur_a_enu = tuple(a)
        self.publish_a()

        if speed_ok:
            if self.cruise_x is None:
                reached = True                                      # 旧行为
            else:
                reached = (p_enu[0] <= self.cruise_x) if d[0] < 0.0 \
                    else (p_enu[0] >= self.cruise_x)
            if reached:
                self.log(f"  ── CRUISE 完成：|v|={v_norm:.2f} m/s"
                         f"（耗时 {now - self.cruise_t0:.2f}s）"
                         f" @ {tuple(round(x, 2) for x in p_enu)}")
                if self.cruise_x is not None:
                    self.log(f"     到位判据 x={p_enu[0]:.2f} m（阈值 "
                             f"{self.cruise_x:+.2f} m，偏差 {p_enu[0] - self.cruise_x:+.2f} m）")
                self.begin_guided(now)
                return
        if now - self.cruise_t0 > self.cruise_timeout:
            extra = ""
            if self.cruise_x is not None:
                extra = f"；当前 x={p_enu[0]:.2f} m（目标 {self.cruise_x:+.2f} m）"
            self.abort(f"巡航超时（{self.cruise_timeout:.0f}s）：|v|={v_norm:.2f} m/s "
                       f"< {self.v_boost_min}{extra}")

    def begin_guided(self, now):
        """进入制导段：记录目标发射时刻，t_rel 从这里开始。"""
        self.state = "GUIDED"
        self.guided_t0 = now
        self.t_launch_us = self.px4_us()
        self.t_rel = 0.0
        self.stable_t = None
        law_name = ["PPN", "TPN", "APN"][int(self.args.law)] \
            if 0 <= int(self.args.law) <= 2 else "?"
        self.log(f"[6/6] ── 目标发射！进入 GUIDED（{law_name} 闭环）")
        self.log(f"       发射时刻 PX4 t={self.t_launch_us} µs；"
                 f"目标 {self.args.target_speed:.1f} m/s 仰角 {self.args.target_elev:.1f}° "
                 f"从 ({self.target_p0[0]:.1f},{self.target_p0[1]:.1f},{self.target_p0[2]:.1f}) 出发")
        # ★ 发射时刻的**几何报告**：判断"这次跑得可比吗"。
        #   设计动机：2026-10-16 两次实跑的横向起点不同
        #     （PPN 发射时 y=-0.99 m、APN 起点 x=+2.39 m），严格说不可比 ——
        #   把偏差直接打出来，就不用靠翻日志推断了。
        p_enu, v_enu = self.state_enu()
        if p_enu is not None:
            dx = p_enu[0] - (self.cruise_x if self.cruise_x is not None else 0.0)
            dy = p_enu[1]
            dz = p_enu[2] - self.target_height
            v_h = math.hypot(v_enu[0], v_enu[1])
            # ★ 存下来给"回传报告"用（用户不用再翻日志找发射几何）
            self.launch_geo = {
                "t": self.t_rel, "p": p_enu, "v": v_enu, "vh": v_h,
                "dx": dx, "dy": dy, "dz": dz,
                "budget": 0.5 * self.args.a_max * 0.5 ** 2,
            }
            self.log(f"       发射几何：位置 ENU=({p_enu[0]:+.2f},{p_enu[1]:+.2f},"
                     f"{p_enu[2]:+.2f})  速度=({v_enu[0]:+.2f},{v_enu[1]:+.2f},"
                     f"{v_enu[2]:+.2f}) |v_h|={v_h:.2f} m/s")
            budget = 0.5 * self.args.a_max * 0.5 ** 2
            self.log(f"                 相对理想：Δx={dx:+.2f} Δy={dy:+.2f} "
                     f"Δz={dz:+.2f} m   ← 横向偏差会直接吃掉"
                     f"『½·a_max·t_go² ≈ {budget:.2f} m』的修正预算")
            # ⚠ 阈值必须**与修正预算挂钩**，不能写死 0.5 m：
            #   SETTLE 的容差本来就有 0.35 m + CRUISE 会再抬 0.15 m ⇒ Δz≈+0.50 是
            #   **系统性**的、两个 law 都一样 ⇒ 根本不影响可比性，写死阈值会每次都误报，
            #   导致告警被无视（实飞已踩：Δz=+0.50 触发"结果可能不可比"）。
            geo_tol = 0.6 * budget          # ≈0.75 m（a_max=10 时）
            if abs(dy) > geo_tol or abs(dz) > geo_tol:
                self.warn(f"       ⚠ 发射几何偏差较大（Δy={dy:+.2f} Δz={dz:+.2f} m，"
                          f"阈值 ±{geo_tol:.2f} = 60% 修正预算）"
                          f"—— 结果可能不可与其它 law 直接比较")

    def do_guided(self, now):
        """制导段：真值 → 制导核 → a_cmd → PX4。"""
        if self.pos is None:
            return
        p_enu, v_enu = self.state_enu()

        # t_rel：用 PX4 仿真时间推进目标模型
        self.t_rel = max(0.0, (self.px4_us() - self.t_launch_us) * 1e-6)

        # --- 制导核 ---
        self.lib.pgu_step(
            self.guide, self.t_rel, 1.0,
            darr(list(p_enu)), darr(list(v_enu)), self.q_identity,
            self.gcmd, self.gdbg)

        cmd = list(self.gcmd)
        dbg = list(self.gdbg)
        self.last_cmd = cmd
        self.last_dbg = dbg

        # Gazebo 可视化：把弹丸真值位置推过去（dbg[0..2] = p_T，ENU）
        self.gz_update((dbg[0], dbg[1], dbg[2]))

        a_enu = (cmd[1], cmd[2], cmd[3])
        self.cur_a_enu = a_enu
        self.publish_a()

        # --- 统计 ---
        self.guided_steps += 1
        mode = int(cmd[8])
        self.modes[mode] = self.modes.get(mode, 0) + 1
        if cmd[5] > 0.5:
            self.sat_steps += 1
        if cmd[4] > self.peak_a_cmd:
            self.peak_a_cmd = cmd[4]

        R_true = dbg[15]
        if self.min_R is None or R_true < self.min_R:
            self.min_R = R_true
            self.t_min_R = self.t_rel

        # --- 命中 / 脱靶 ---
        if R_true < self.hit_radius and not self.hit:
            self.hit = True
            self.log("")
            self.log("★" * 36)
            self.log(f"★ 命中！R_true={R_true:.3f} m（阈值 {self.hit_radius} m）"
                     f" @ t_rel={self.t_rel:.3f}s")
            self.log(f"★ 命中时 |v_M|={math.sqrt(sum(x * x for x in v_enu)):.2f} m/s  "
                     f"a_cmd={math.sqrt(a_enu[0]**2 + a_enu[1]**2 + a_enu[2]**2):.2f} m/s²")
            self.log("★" * 36)
            self.finish("命中")
            return

        if self.R_prev is not None and R_true > self.R_prev + 1e-9:
            if self.min_R is not None and R_true > self.min_R + 1.0:
                self.missed = True
                self.finish(f"脱靶（最近 {self.min_R:.3f} m @ t_rel={self.t_min_R:.3f}s）")
                return
        self.R_prev = R_true

        # --- 超时 / 目标落地 ---
        if self.t_rel > self.t_flight + 0.4:
            self.finish(f"未命中（目标已落地：t_rel={self.t_rel:.2f}s > "
                        f"t_flight={self.t_flight:.2f}s；最近 {self.min_R:.3f} m）")
            return
        if self.t_rel > GUIDED_TIMEOUT:
            self.finish(f"未命中（制导超时 t_rel={self.t_rel:.2f}s，最近 {self.min_R:.3f} m）")
            return

    # ---------------------------------------------------------------- 收尾
    def check_safety(self):
        """安全围栏（只在 CRUISE/GUIDED 段生效）。"""
        if self.pos is None or self.state not in ("CRUISE", "GUIDED"):
            return
        alt = -self.pos.z
        if alt < MIN_ALTITUDE:
            self.abort(f"离地高度 {alt:.2f} m < {MIN_ALTITUDE} m")
            return
        h = math.hypot(self.pos.x - getattr(self, "start_x", 0.0),
                       self.pos.y - getattr(self, "start_y", 0.0))
        if h > MAX_H_DISP:
            self.abort(f"水平位移 {h:.1f} m > {MAX_H_DISP} m")

    def finish(self, reason):
        """测试段正常结束（命中/脱靶/超时）→ 打印总结 → 转入降落。"""
        self.finish_reason = reason        # 记下来给"回传评估块"用
        self.gz_finish(self.hit)          # 命中画爆点 / 画轨迹（--gz-viz 才动作）
        self.print_summary(reason)
        self.cur_a_enu = (0.0, 0.0, 0.0)
        self.request_landing(f"测试结束（{reason}）")

    def abort(self, reason):
        """安全中止 → 打印总结 → 转入降落。"""
        if self.aborted:
            return
        self.aborted = True
        self.abort_reason = reason
        self.finish_reason = f"中止：{reason}"
        self.cur_a_enu = (0.0, 0.0, 0.0)
        self.publish_a()
        bar = "!" * 76
        self.warn(bar)
        self.warn(f"!! 安全中止：{reason}")
        self.warn(bar)
        self.print_summary(f"中止（{reason}）")
        self.request_landing(f"安全中止：{reason}")

    def request_landing(self, note=""):
        """统一的收尾入口：跑完 / 中止 / Ctrl-C 都走这里。"""
        if self.ending:
            return
        self.ending = True
        self.cur_a_enu = (0.0, 0.0, 0.0)
        now = time.monotonic()
        # ★ 记下收尾起点的**真实位置**：报告里要用它算"实测着陆漂移"
        #   （原来只给估算 v²/2a，实测差 500 倍：日志说 0.02 m、实际漂了 10 m）
        p_enu, _ = self.state_enu()
        self.ending_pos = p_enu
        self.land_t0 = now
        self.last_land_cmd = 0.0
        self.landed = False
        self.disarmed = False
        self.land_stable_t = None
        self.disarm_t0 = now
        self.force_disarm_sent = False
        if note:
            self.log(f"→ 转入收尾：{note}")
        if not self.auto_land:
            self.log("→ --no-auto-land：不做自动降落，保持 0 加速度悬停。")
            self.log("   ⚠ accel-only 无阻尼，无人机会保持原速度继续漂；请尽快在 QGC 里 LAND。")
            self.state = "DONE"
            self.done_t = now
            return

        # ★ 着陆前刹车（2026-10-16 数值审计发现）
        #   PX4 的 AUTO_LAND 水平减速度有限（约 2 m/s²）⇒ 脱靶时那 5.87 m/s 会变成
        #   刹车距离 v²/(2a) ≈ 8.6 m 的漂移（数值审计实测 8.55 m，落在 ENU x=−15.6，
        #   超出声明安全盒 6.6 m）。我们在 OFFBOARD 里自己能控 ⇒ 先刹再 LAND，
        #   a_max=10 时刹车距离只要 v²/(2·10) ≈ 1.7 m（约 1/5）。
        vh = None
        if self.pos is not None:
            _, v_enu = self.state_enu()
            vh = math.hypot(v_enu[0], v_enu[1])
        if self.args.brake and vh is not None and vh > self.args.brake_vtol:
            est = vh ** 2 / (2.0 * min(self.args.a_max, 2.0))
            self.log(f"   ★ 先刹车：水平速度 {vh:.2f} m/s。若直接 LAND，AUTO_LAND 的"
                     f"减速度(~2 m/s²)会带来 ~{est:.1f} m 漂移；先刹到 "
                     f"≤{self.args.brake_vtol:.2f} m/s 再 LAND。")
            self.state = "BRAKE"
            self.brake_t0 = now
            self.t_state = now
            return

        self.log("→ 转入自动降落：LAND（AUTO_LAND 下降 + cmd_vel_ned）→ 落地后 DISARM 停桨。")
        self.landing_phase = True
        self.alt_hold = False
        self.state = "LAND"
        self.t_state = now

    def print_summary(self, reason):
        bar = "=" * 76
        self.log("")
        self.log(bar)
        self.log(f"Phase B 结果汇总 —— {reason}")
        self.log("-" * 76)
        if self.min_R is None:
            self.log("  未采到制导段数据（还没进入 GUIDED？）")
        else:
            self.log(f"  最近距离     : {self.min_R:.3f} m  @ t_rel={self.t_min_R:.3f} s"
                     f"（命中阈值 {self.hit_radius} m）")
            self.log(f"  命中         : {'是 ✓' if self.hit else '否 ✗'}")
            self.log(f"  峰值指令     : {self.peak_a_cmd:.2f} m/s²"
                     f"（a_max={self.args.a_max}）")
            self.log(f"  指令饱和步数 : {self.sat_steps} / {self.guided_steps}"
                     f"（{100.0 * self.sat_steps / max(self.guided_steps, 1):.1f}%）")
            modes_txt = ", ".join(f"{GMODE_NAME.get(k, k)}={v}" for k, v in sorted(self.modes.items()))
            self.log(f"  制导模式分布 : {modes_txt or '—'}   (PN/APN/HOLD/SEARCH)")
        if self.offboard_confirmed:
            self.log(f"  OFFBOARD     : ✓ 已确认进入（nav_state=14）")
        else:
            self.log(f"  OFFBOARD     : ✗ 未确认")
        if self.soft_landing is False:
            self.log(f"  着陆质量     : ⚠ 非软着陆（停桨时离地过高，见上面 warn）")
        elif self.soft_landing is True:
            self.log(f"  着陆质量     : ✓ 软着陆（停桨时已贴地且近零速）")
        self.log(bar)

    # ---------------------------------------------------------------- 记录
    def build_row(self):
        """拼一行采样数据（列序 = `CSV_COLUMNS`）。

        CSV 落盘与"回传报告"共用这一份，保证两者**逐字段一致**（不会一边新一边旧）。

        ★ 指令列的口径（2026-10-16 修正）：
          · `state == GUIDED`  → 显示**制导核输出**（含 mode / t_go / lam_dot / Vc / R_est）
          · 其余段            → 显示**本节点实际下发的加速度**（SETTLE/CRUISE/BRAKE 的
                                 稳定与刹车指令），制导专属列**留空**
        ⚠ 为什么要这样：`last_cmd` 在 GUIDED 结束后就**冻结**了，若继续打印，
          收尾段会一直显示命中瞬间那一步的退化值（实测 `t_go=410969.7493` ——
          因为终端时刻接近速度趋零、`t_go=R/Vc` 数值爆炸）。
          把这些噪声当成"数据"展示，比留空危险得多。
        """
        row = [round(time.monotonic() - self.t_start, 3), round(self.t_rel, 4), self.state]
        if self.state == "GUIDED" and self.last_cmd is not None:
            c = self.last_cmd
            row += [round(c[1], 5), round(c[2], 5), round(c[3], 5),
                    round(c[4], 5), int(c[5]), int(c[8]), round(c[7], 4),
                    round(c[9], 6), round(c[10], 4), round(c[11], 4)]
        else:
            a = self.cur_a_enu
            an = math.sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2])
            row += [round(a[0], 5), round(a[1], 5), round(a[2], 5), round(an, 5),
                    "", "", "", "", "", ""]
        if self.pos is not None:
            row += [round(self.pos.x, 4), round(self.pos.y, 4), round(self.pos.z, 4),
                    round(self.pos.vx, 4), round(self.pos.vy, 4), round(self.pos.vz, 4),
                    round(self.pos.heading, 4)]
            row += [round(self.pos.y, 4), round(self.pos.x, 4), round(-self.pos.z, 4),
                    round(self.pos.vy, 4), round(self.pos.vx, 4), round(-self.pos.vz, 4)]
        else:
            row += [""] * 13
        # 姿态：roll/pitch/yaw (deg) + 原始四元数 [w,x,y,z]（均 ENU，来自 /odometry）
        att = self.odom_att()
        if att is not None:
            roll, pitch, yaw, q = att
            row += [round(roll, 3), round(pitch, 3), round(yaw, 3),
                    round(q[0], 6), round(q[1], 6), round(q[2], 6), round(q[3], 6)]
        else:
            row += [""] * 7
        if self.last_dbg is not None:
            d = self.last_dbg
            row += [round(d[0], 4), round(d[1], 4), round(d[2], 4),
                    round(d[3], 4), round(d[4], 4), round(d[5], 4),
                    round(d[15], 4), int(d[16])]
        else:
            row += [""] * 8
        row.append("" if self.nav_state is None else self.nav_state)
        row.append("" if self.arming_state is None else self.arming_state)
        return row

    def log_row(self, force=False):
        """按各自频率落盘 CSV / 记录报告采样。

        · CSV  ：`--csv-hz`（默认 10 Hz = 0.1 s）
        · 报告：`--report-hz`（默认跟随 `--csv-hz`）

        指令仍是 `--rate`（50 Hz）—— 这里只**节流落盘**，控制回路一点不变。
        `force=True` 绕过两者的节流（收尾补最后一行用）。
        想看细节把 `--csv-hz` 给 50 即可。
        """
        if self.csv_w is None and not self.report_on:
            return
        now = time.monotonic()
        row = None
        if self.csv_w is not None:
            if force or now - self.last_csv_t >= self.csv_period - 1e-6:
                row = self.build_row()
                self.csv_w.writerow(row)
                self.last_csv_t = now
        if self.report_on:
            # 三档节奏：GUIDED 密（交会段短且是最关键的）；SETTLE/CRUISE/BRAKE 中；
            # 起飞爬升 / 降落停桨等低信息量段最粗（1 Hz），避免刷屏。
            st = self.state
            if st in REPORT_FAST_STATES:
                period = self.report_period
            elif st in REPORT_MID_STATES:
                period = self.report_mid_period
            else:
                period = self.report_slow_period
            if force or now - self.last_report_t >= period - 1e-6:
                if row is None:
                    row = self.build_row()
                self.report_rows.append(row)
                self.last_report_t = now

    # ------------------------------------------------- 回传报告（可复制粘贴）
    def print_report(self):
        """打印**可直接整段复制回传**的评估块。

        ⚠ 故意用 `print()` 而**不是** ROS logger：rclpy 会给每行加
          `[INFO] [时间] [节点名]: ` 前缀，复制出来全是噪声、也没法直接喂给脚本。
          这里裸打印，前后有固定标记（`REPORT_BEGIN/REPORT_END`）便于截取。
        """
        if not self.report_on:
            return
        law_name = ["PPN", "TPN", "APN"][int(self.args.law)] \
            if 0 <= int(self.args.law) <= 2 else "?"
        p = print
        p("")
        p(REPORT_BEGIN)
        p("# phase_b_guidance.py 自动生成 · 整段复制回传即可（含场景 + 时间序列 + 汇总）")
        p("")
        p("[运行] 节点=phase_b_guidance  制导核=%s  ABI=%s"
          % (os.path.basename(self.lib_path), self.lib.pgu_version()))
        p("[制导] law=%s(%s)  N=%g  a_max=%g  t_go_freeze=%g  指令率=%g Hz  "
          "命中半径=%g m"
          % (law_name, int(self.args.law), self.args.N, self.args.a_max,
             self.args.t_go_freeze, self.rate, self.hit_radius))
        p("[目标] type=%s  发射点 ENU=(%.2f,%.2f,%.2f)  速度=%.2f m/s  "
          "仰角=%.1f°  方位=%.1f°  t_flight=%.2fs  hits_ground=%s"
          % (self.args.target_type, self.target_p0[0], self.target_p0[1],
             self.target_p0[2], self.args.target_speed, self.args.target_elev,
             self.args.target_azim, self.t_flight, self.t_hits_ground))
        p("[巡航] %s  boost_dir=%s  a_boost=%g  v_min=%g  cruise_x=%s  "
          "lat_hold=%s  timeout=%gs"
          % ("ON" if self.cruise_on else "OFF",
             tuple(round(v, 3) for v in self.boost_dir), self.a_boost,
             self.v_boost_min, ("%.2f" % self.cruise_x) if self.cruise_x is not None else "None",
             "ON" if self.args.cruise_lat_hold else "OFF", self.args.cruise_timeout))
        p("[稳定] 高度目标=%.2f m  水平收敛=%s  刹车=%s  alt_hold=%s"
          % (self.target_height,
             "None" if self.settle_xy is None else "(%g,%g)" % (self.settle_xy[0], self.settle_xy[1]),
             "ON" if getattr(self.args, "brake", False) else "OFF",
             "ON" if self.alt_hold else "OFF"))
        g = getattr(self, "launch_geo", None)
        if g is None:
            p("[发射] (未采到发射时刻状态)")
        else:
            p("[发射] t_rel=%.3f  ENU=(%+.2f,%+.2f,%+.2f)  v=(%+.2f,%+.2f,%+.2f)  |v_h|=%.2f"
              % (g["t"], g["p"][0], g["p"][1], g["p"][2],
                 g["v"][0], g["v"][1], g["v"][2], g["vh"]))
            p("[偏差] Δx=%+.2f Δy=%+.2f Δz=%+.2f m   修正预算 ½·a_max·t_go² = %.2f m"
              % (g["dx"], g["dy"], g["dz"], g["budget"]))
        # ★ 控制回路健康：一眼看出"有没有东西阻塞主回路"（2026-10-16 新增）
        nominal_ms = 1000.0 / self.rate
        gap_ms = self.loop_gap_max * 1000.0
        if self.loop_gap_max_at is None:
            p("[健康] 控制周期: 名义 %.1f ms；未采到相邻 tick 间隔（运行太短）"
              % nominal_ms)
        else:
            tag = "✅ 正常" if self.loop_gap_max < LOOP_GAP_WARN else "⚠ 有空洞"
            p("[健康] 控制周期: 名义 %.1f ms；**实测最大间隔 %.1f ms**"
              " @ t_wall=%.3fs (state=%s)  %s"
              % (nominal_ms, gap_ms, self.loop_gap_max_at,
                 self.loop_gap_max_state, tag))
            if self.loop_gaps:
                tot = sum(g for _, g, _ in self.loop_gaps)
                p("[健康] 空洞(≥%d ms) 共 %d 次、累计 %.0f ms —— 这些周期**没发出 setpoint**："
                  % (int(LOOP_GAP_WARN * 1000), len(self.loop_gaps), tot * 1000))
                for tw, gp, st in self.loop_gaps[:8]:
                    p("         t_wall=%.3f  %5.0f ms  state=%s" % (tw, gp * 1000, st))
                if len(self.loop_gaps) > 8:
                    p("         …（另有 %d 次，见完整 CSV）" % (len(self.loop_gaps) - 8))
        # ★ 实测落点 / 漂移（不是估算）
        dr = self.landing_drift()
        if dr is None:
            p("[落点] (未采到收尾起点或停桨点)")
        else:
            d, ddx, ddy = dr
            p("[落点] 收尾起点 ENU=(%+.2f,%+.2f) → 停桨点 ENU=(%+.2f,%+.2f)"
              % (self.ending_pos[0], self.ending_pos[1],
                 self.stop_pos[0], self.stop_pos[1]))
            p("[落点] **实测**着陆漂移 = %.2f m（Δx=%+.2f Δy=%+.2f）"
              "   全程最大水平位移 = %.1f m" % (d, ddx, ddy, self.max_h_disp))
        p("")
        p("--- 时间序列（CSV 格式 · 共 %d 行）---" % len(self.report_rows))
        p("# 时间列口径：t_wall=墙钟秒（全程单调，收尾段只有它能区分各步）；"
          "t_rel=制导相对秒（GUIDED 结束后冻结）")
        p("# 指令列口径：state=GUIDED 时是**制导核输出**（含 mode/t_go/Vc/R_est）；"
          "其余段 mode/t_go/Vc/R_est 留空，ax/ay/az/a_norm 是**本节点实际下发的加速度**")
        p("# 三档采样：GUIDED=%g Hz（交会段只有约 0.5 s，必须密）；"
          "SETTLE/CRUISE/BRAKE=%g Hz；起飞爬升/降落停桨=%g Hz"
          % (self.report_hz, REPORT_MID_HZ, REPORT_SLOW_HZ))
        # ⚠ 用**逗号**分隔而不是空格：空字段（未进制导段时 mode/t_go/R_true… 都为空）
        #   在空格分隔下会**塌成 0 个 token** ⇒ 列错位、没法解析。
        #   逗号分隔后空字段是"两个逗号之间什么都没有"，列数恒定、可直接当 CSV 喂脚本。
        p("# " + ",".join(REPORT_TRACE_COLS))
        idx = [CSV_COLUMNS.index(c) for c in REPORT_TRACE_COLS]
        for r in self.report_rows:
            p(",".join(str(r[i]) for i in idx))
        p("")
        p("--- 汇总 ---")
        if self.finish_reason:
            p("结束原因 = %s" % self.finish_reason)
        if self.min_R is None:
            p("最近距离 = (未采到制导段数据)")
        else:
            p("最近距离 = %.3f m @ t_rel=%.3f s   (阈值 %.2f m)  命中 = %s"
              % (self.min_R, self.t_min_R, self.hit_radius,
                 "是" if self.hit else "否"))
            p("峰值指令 = %.2f m/s²   饱和 = %d/%d 步 (%.1f%%)   mode: %s"
              % (self.peak_a_cmd, self.sat_steps, self.guided_steps,
                 100.0 * self.sat_steps / max(self.guided_steps, 1),
                 ", ".join("%s=%d" % (GMODE_NAME.get(k, k), v)
                           for k, v in sorted(self.modes.items())) or "-"))
        p("OFFBOARD = %s" % ("已确认" if self.offboard_confirmed else "未确认"))
        p("着陆 = landed:%s  disarmed:%s  软着陆:%s"
          % (self.landed, self.disarmed,
             {None: "未判", True: "是", False: "否(离地过高)"}[self.soft_landing]))
        if dr is not None:
            p("着陆漂移(实测) = %.2f m   全程最大水平位移 = %.1f m" % (dr[0], self.max_h_disp))
        p("控制回路最大间隔 = %.0f ms  (空洞 %d 次)"
          % (self.loop_gap_max * 1000.0, len(self.loop_gaps)))
        if self.aborted:
            p("异常 = ABORT: %s" % self.abort_reason)
        p("")
        p("# 提示：完整 CSV 见 --csv 指定的文件（列名与上面 [运行] 行一致）")
        p(REPORT_END)
        p("")
        sys.stdout.flush()

    def close(self):
        if self.gz is not None:
            try:
                self.gz.close()        # 停掉后台同步线程（实体保留，便于事后观察）
            except Exception:                          # noqa: BLE001
                pass
            self.gz = None
        if self.csv_f is not None:
            try:
                self.csv_f.close()
            except Exception:                          # noqa: BLE001
                pass
            self.csv_f = None
        if getattr(self, "guide", None) is not None:
            try:
                self.lib.pgu_destroy(self.guide)
            except Exception:                          # noqa: BLE001
                pass
            self.guide = None


# ============================================================================
def build_parser():
    ap = argparse.ArgumentParser(
        description="Phase B · 制导闭环验证（PPN + 真 DART 目标 + PX4 SITL）",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--namespace", default="x500_0")
    ap.add_argument("--lib", default=None, help="制导核动态库路径（默认自动查找）")
    # 场景
    ap.add_argument("--target-p0", type=parse_vec3, default=(-12.0, 0.0, 11.0),
                    help="目标发射点（ENU）")
    ap.add_argument("--target-type", type=int, default=2, help="0=STRAIGHT 1=BALLISTIC 2=DART 3=CIRCLE 4=SNAKE")
    ap.add_argument("--target-speed", type=float, default=15.0)
    ap.add_argument("--target-elev", type=float, default=5.0, help="仰角 (deg)")
    ap.add_argument("--target-azim", type=float, default=0.0, help="方位角 (deg，从 +x 逆时针)")
    ap.add_argument("--target-mass", type=float, default=0.40)
    ap.add_argument("--target-cd", type=float, default=0.50)
    ap.add_argument("--target-area", type=float, default=2.83e-3)
    ap.add_argument("--target-rho", type=float, default=1.225)
    # 制导
    ap.add_argument("--law", type=int, default=0, help="0=PPN 1=TPN 2=APN")
    ap.add_argument("--N", type=float, default=6.0)
    ap.add_argument("--a-max", type=float, default=10.0, help="过载上限 (m/s²)")
    ap.add_argument("--t-go-freeze", type=float, default=0.10)
    ap.add_argument("--hit-radius", type=float, default=0.5, help="命中判定半径 (m)")
    # 预加速
    ap.add_argument("--a-boost", type=float, default=6.0)
    ap.add_argument("--v-boost-min", type=float, default=5.0)
    ap.add_argument("--boost-dir", type=parse_vec3, default=(-1.0, 0.0, 0.0))
    ap.add_argument("--no-cruise", action="store_true", help="跳过预加速（悬停直接制导）")
    # 巡航到位（守备场景用；不指定时行为与旧版完全一致）
    ap.add_argument("--cruise-x", type=float, default=None,
                    help="巡航发射位置判据（ENU x 坐标）：指定后需"
                         "「速度达标 **且** x 已越过该值」才发射目标；"
                         "不指定则速度一达标就发射（旧行为）")
    ap.add_argument("--v-kp", type=float, default=1.5,
                    help="巡航段速度保持增益 (1/s)：a = v_kp·(v_boost_min − v_沿boost_dir)")
    ap.add_argument("--cruise-timeout", type=float, default=CRUISE_TIMEOUT,
                    help="巡航/预加速段超时 (s)")
    # Gazebo 可视化（塔 / 弹丸 / 轨迹）—— 见 gz_scene_sync.py
    ap.add_argument("--gz-viz", action="store_true",
                    help="在 Gazebo GUI 里显示防御塔 + 弹丸（含轨迹线）；"
                         "接不上 Gazebo 时自动降级为纯控制台输出，不影响制导")
    ap.add_argument("--gz-world", default="walls",
                    help="Gazebo 世界名（XTDrone2 单机 demo 默认 walls）")
    ap.add_argument("--gz-hz", type=float, default=30.0,
                    help="弹丸位置同步频率 (Hz)")
    ap.add_argument("--gz-trail", action="store_true",
                    help="命中/脱靶后在 Gazebo 里画出弹丸轨迹线")
    # 飞行
    ap.add_argument("--target-height", type=float, default=10.0, help="起飞高度 (m)")
    ap.add_argument("--takeoff-tol", type=float, default=1.0)
    ap.add_argument("--takeoff-timeout", type=float, default=60.0)
    ap.add_argument("--rate", type=float, default=50.0, help="制导/指令频率 (Hz)")
    # 收尾
    ap.add_argument("--no-auto-land", action="store_true",
                    help="跑完不自动降落（保持 0 加速度悬停）")
    ap.add_argument("--no-disarm", action="store_true")
    ap.add_argument("--no-force-disarm", action="store_true",
                    help="禁止兜底的强制停桨（param2=21196）")
    ap.add_argument("--land-timeout", type=float, default=LAND_TIMEOUT)
    ap.add_argument("--land-down-speed", type=float, default=0.5,
                    help="降落阶段的下降速度 (m/s，cmd_vel_ned 用)")
    ap.add_argument("--linger", type=float, default=3.0)
    ap.add_argument("--no-alt-hold", action="store_true", help="关闭非制导段的垂直保持辅助")
    ap.add_argument("--alt-kp", type=float, default=ALT_KP)
    ap.add_argument("--alt-kd", type=float, default=ALT_KD)
    # 高度调整（SETTLE 段）：XTDrone2 的 TAKEOFF 高度写死 10 m，不受 --target-height 控制
    ap.add_argument("--alt-settle-tol", type=float, default=ALT_SETTLE_TOL,
                    help="SETTLE 段高度到位容差 (m)")
    ap.add_argument("--alt-settle-vtol", type=float, default=ALT_SETTLE_VTOL,
                    help="SETTLE 段高度到位时的垂直速度容差 (m/s)")
    ap.add_argument("--alt-settle-timeout", type=float, default=ALT_SETTLE_TIMEOUT,
                    help="SETTLE 段高度调整等待上限 (s)，超时则带警告继续")
    # SETTLE 段水平稳定（★ 修 accel-only 无阻尼导致的横向漂移）
    ap.add_argument("--settle-xy", default="0,0",
                    help="SETTLE 段把水平位置收敛到的 ENU 点（如 0,0；"
                         "填 none 只做速度阻尼、不收敛位置）")
    ap.add_argument("--settle-kp", type=float, default=SETTLE_KP,
                    help="SETTLE 段水平位置增益 (1/s²)")
    ap.add_argument("--settle-kd", type=float, default=SETTLE_KD,
                    help="SETTLE 段水平速度阻尼 (1/s)；巡航段横向阻尼也用它")
    ap.add_argument("--settle-xy-tol", type=float, default=SETTLE_XY_TOL,
                    help="SETTLE 水平位置到位容差 (m)")
    ap.add_argument("--settle-xy-vtol", type=float, default=SETTLE_XY_VTOL,
                    help="SETTLE 水平速度到位容差 (m/s)")
    ap.add_argument("--cruise-lat-hold", action="store_true", default=True,
                    help="巡航段保持横向（阻尼横向速度，走直线）")
    ap.add_argument("--no-cruise-lat-hold", dest="cruise_lat_hold",
                    action="store_false", help="关闭巡航段横向保持")
    # ★ 着陆前刹车：避免 AUTO_LAND 的有限水平减速度导致大幅漂移
    ap.add_argument("--brake", action="store_true", default=True,
                    help="转 LAND 前先在 OFFBOARD 里把水平速度刹掉（大幅减小着陆漂移）")
    ap.add_argument("--no-brake", dest="brake", action="store_false",
                    help="关闭着陆前刹车（直接 LAND）")
    ap.add_argument("--brake-kd", type=float, default=BRAKE_KD,
                    help="刹车增益 (1/s)：a_h = −brake_kd·v_h")
    ap.add_argument("--brake-vtol", type=float, default=BRAKE_VTOL,
                    help="刹车完成判据 |v_h| ≤ 此值 (m/s)")
    ap.add_argument("--brake-timeout", type=float, default=BRAKE_TIMEOUT,
                    help="刹车段超时 (s)")
    ap.add_argument("--srv-timeout", type=float, default=SRV_TIMEOUT)
    # 其它
    ap.add_argument("--no-odom", action="store_true")
    ap.add_argument("--csv", default="phase_b_log.csv")
    ap.add_argument("--csv-hz", type=float, default=10.0,
                    help="CSV 采样频率 (Hz)。指令仍是 --rate（50 Hz），CSV 只按此"
                         "间隔落盘；默认 0.1 s 一行，便于回传分析；想看细节可给 50")
    ap.add_argument("--no-report", action="store_true",
                    help="不打印末尾的『可复制回传评估块』（默认打印）")
    ap.add_argument("--report-hz", type=float, default=0.0,
                    help="回传评估块的时间序列采样频率 (Hz)；0 = 跟随 --csv-hz")
    return ap


def main(argv=None):
    argv = sys.argv[1:] if argv is None else argv
    ap = build_parser()
    args, _ = ap.parse_known_args(argv)

    _no_opt = resolve_no_signal_handler_option()
    if _no_opt is not None:
        rclpy.init(args=argv, signal_handler_options=_no_opt)
        own_sigint = True
    else:
        rclpy.init(args=argv)
        own_sigint = False
        try:
            from rclpy.signals import uninstall_signal_handlers
            uninstall_signal_handlers()
            own_sigint = True
        except Exception as _e:                        # noqa: BLE001
            print(f"[warn] 无法接管 SIGINT（{_e!r}）：Ctrl-C 将直接退出，不会自动降落。",
                  file=sys.stderr)

    node = PhaseBGuidance(args)

    if own_sigint:
        def _on_sigint(signum, frame):                # noqa: ARG001
            if node.force_exit_requested:
                node.log("再次 Ctrl-C：强制退出（跳过降落收尾）。")
                raise KeyboardInterrupt
            node.force_exit_requested = True
            node.request_landing("Ctrl-C")
        signal.signal(signal.SIGINT, _on_sigint)

    try:
        while rclpy.ok() and not node.should_exit:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        node.log("强制退出，未完成降落收尾。")
        node.cur_a_enu = (0.0, 0.0, 0.0)
        try:
            node.publish_a()
        except Exception:                              # noqa: BLE001
            pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
