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

# 制导核的 mode 枚举（同 guidance_types.hpp）
GMODE_PN, GMODE_APN, GMODE_HOLD, GMODE_SEARCH = 0, 1, 2, 3
GMODE_NAME = {0: "PN", 1: "APN", 2: "HOLD", 3: "SEARCH"}


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

        # 降落 / 停桨
        self.land_t0 = 0.0
        self.last_land_cmd = 0.0
        self.landed = False
        self.disarmed = False
        self.land_stable_t = None
        self.disarm_t0 = 0.0
        self.force_disarm_sent = False
        self.landing_phase = False            # True 时改发 cmd_vel_ned
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
            self.csv_w.writerow([
                "t_wall", "t_rel", "state",
                "ax_enu", "ay_enu", "az_enu", "a_norm", "sat", "mode", "t_go",
                "lam_dot", "Vc", "R_est",
                "px4_x", "px4_y", "px4_z", "px4_vx", "px4_vy", "px4_vz", "heading",
                "enu_x", "enu_y", "enu_z", "enu_vx", "enu_vy", "enu_vz",
                "tgt_x", "tgt_y", "tgt_z", "tgt_vx", "tgt_vy", "tgt_vz",
                "R_true", "sensor_valid",
                "nav_state", "arming_state",
            ])

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
        self.log(f"  命中判定: R_true < {self.hit_radius} m")
        self.log("  符号约定: 日志里的 alt / 上升率 一律**上为正**；"
                 "CSV 同时给 px4_*(NED, z 向下为正) 与 enu_*(ENU, z 向上为正) 两套")
        self.log(f"  垂直保持辅助(非制导段): {'ON' if self.alt_hold else 'OFF'}（kp={self.alt_kp}, kd={self.alt_kd}）")
        self.log(f"  收尾: {'自动 LAND + 停桨' if self.auto_land else '不自动降落'} "
                 f"(land_timeout={self.land_timeout:.0f}s, 强制停桨兜底="
                 f"{'ON' if self.force_disarm_ok else 'OFF'})  Ctrl-C 一次=降落收尾，两次=强制退出")
        self.log(f"  CSV={os.path.abspath(self.csv_path) if self.csv_path else '(关闭)'}")
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
        self.poll_service(now)

        # ---------------- Ctrl-C 收尾请求 ----------------
        if self.force_exit_requested and self.state != "DONE" \
                and not self.landing_phase:
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
            self.publish_a()
            if now - self.t_state >= 0.5:
                self.log("[5/6] 稳定完毕 → 进入预加速" if self.cruise_on
                         else "[5/6] 稳定完毕 → 直接开始制导（--no-cruise）")
                self.start_x = self.pos.x
                self.start_y = self.pos.y
                if self.cruise_on:
                    self.state = "CRUISE"
                    self.cruise_t0 = now
                    self.log(f"  ── CRUISE 预加速：朝 ({self.boost_dir[0]:+.2f},"
                             f"{self.boost_dir[1]:+.2f},{self.boost_dir[2]:+.2f}) "
                             f"a={self.a_boost:.1f} m/s²，直到 |v|≥{self.v_boost_min:.1f} m/s")
                else:
                    self.begin_guided(now)

        elif self.state == "CRUISE":
            self.do_cruise(now)
            if self.state == "CRUISE":
                self.check_safety()

        elif self.state == "GUIDED":
            self.do_guided(now)
            self.check_safety()

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
                self.log(f"✓ 已停桨（PX4 自动，alt={alt if alt is None else round(alt, 2)} m）")
                self.disarmed = True
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
                if self.disarmed:
                    self.log("收尾完成：已落地并停桨。节点退出。")
                elif self.landed:
                    self.log("收尾完成：已落地（未确认停桨）。节点退出。")
                else:
                    self.log("收尾完成（未走完整降落流程）。节点退出。")
                self.should_exit = True
            return

        self.log_row()

    # ---------------------------------------------------------------- 段
    def do_cruise(self, now):
        """预加速段：朝固定方向加速 + 垂直保持；到速度后交班制导。"""
        if self.pos is None:
            return
        p_enu, v_enu = self.state_enu()
        d = self.boost_dir
        a = [self.a_boost * d[0], self.a_boost * d[1], self.a_boost * d[2]]
        if self.alt_hold and self.alt_target is not None:
            a[2] += self.alt_kp * (p_enu[2] - self.alt_target) - self.alt_kd * v_enu[2]
        self.cur_a_enu = tuple(a)
        self.publish_a()

        v_norm = math.sqrt(v_enu[0] ** 2 + v_enu[1] ** 2 + v_enu[2] ** 2)
        if v_norm >= self.v_boost_min:
            self.log(f"  ── CRUISE 完成：|v|={v_norm:.2f} m/s（耗时 {now - self.cruise_t0:.2f}s）"
                     f" @ {tuple(round(x, 2) for x in p_enu)}")
            self.begin_guided(now)
            return
        if now - self.cruise_t0 > 8.0:
            self.abort(f"预加速超时（8s）：|v|={v_norm:.2f} m/s < {self.v_boost_min}")

    def begin_guided(self, now):
        """进入制导段：记录目标发射时刻，t_rel 从这里开始。"""
        self.state = "GUIDED"
        self.guided_t0 = now
        self.t_launch_us = self.px4_us()
        self.t_rel = 0.0
        self.stable_t = None
        self.log(f"[6/6] ── 目标发射！进入 GUIDED（PPN 闭环）")
        self.log(f"       发射时刻 PX4 t={self.t_launch_us} µs；"
                 f"目标 {self.args.target_speed:.1f} m/s 仰角 {self.args.target_elev:.1f}° "
                 f"从 ({self.target_p0[0]:.1f},{self.target_p0[1]:.1f},{self.target_p0[2]:.1f}) 出发")

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
        self.print_summary(reason)
        self.cur_a_enu = (0.0, 0.0, 0.0)
        self.request_landing(f"测试结束（{reason}）")

    def abort(self, reason):
        """安全中止 → 打印总结 → 转入降落。"""
        if self.aborted:
            return
        self.aborted = True
        self.abort_reason = reason
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
        if self.landing_phase:
            return
        self.cur_a_enu = (0.0, 0.0, 0.0)
        self.alt_hold = False
        self.landing_phase = True
        now = time.monotonic()
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
        self.log("→ 转入自动降落：LAND（AUTO_LAND 下降 + cmd_vel_ned）→ 落地后 DISARM 停桨。")
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
        self.log(bar)

    # ---------------------------------------------------------------- 记录
    def log_row(self):
        if self.csv_w is None:
            return
        row = [round(time.monotonic() - self.t_start, 3), round(self.t_rel, 4), self.state]
        if self.last_cmd is not None:
            c = self.last_cmd
            row += [round(c[1], 5), round(c[2], 5), round(c[3], 5),
                    round(c[4], 5), int(c[5]), int(c[8]), round(c[7], 4),
                    round(c[9], 6), round(c[10], 4), round(c[11], 4)]
        else:
            row += [""] * 10
        if self.pos is not None:
            row += [round(self.pos.x, 4), round(self.pos.y, 4), round(self.pos.z, 4),
                    round(self.pos.vx, 4), round(self.pos.vy, 4), round(self.pos.vz, 4),
                    round(self.pos.heading, 4)]
            row += [round(self.pos.y, 4), round(self.pos.x, 4), round(-self.pos.z, 4),
                    round(self.pos.vy, 4), round(self.pos.vx, 4), round(-self.pos.vz, 4)]
        else:
            row += [""] * 13
        if self.last_dbg is not None:
            d = self.last_dbg
            row += [round(d[0], 4), round(d[1], 4), round(d[2], 4),
                    round(d[3], 4), round(d[4], 4), round(d[5], 4),
                    round(d[15], 4), int(d[16])]
        else:
            row += [""] * 8
        row.append("" if self.nav_state is None else self.nav_state)
        row.append("" if self.arming_state is None else self.arming_state)
        self.csv_w.writerow(row)

    def close(self):
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
    ap.add_argument("--srv-timeout", type=float, default=SRV_TIMEOUT)
    # 其它
    ap.add_argument("--no-odom", action="store_true")
    ap.add_argument("--csv", default="phase_b_log.csv")
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
