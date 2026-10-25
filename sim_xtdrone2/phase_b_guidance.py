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
import json
import math
import os
import signal
import subprocess
import sys
import tempfile
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

# ★★ 降落两阶段（2026-10-17 实飞事故后改：原来一发 NAV_LAND 就把控制权全交给
#   PX4 的 AUTO_LAND，它的内置减速曲线让高度**卡在 0.54 m 不动**，
#   随后提前判定落地并停桨 ⇒ 无人机从 0.54 m 掉下来。
#   ⇒ 先自己（OFFBOARD + cmd_vel_ned）主动降到 LAND_HANDOVER_ALT，
#     再交给 AUTO_LAND 完成最后一段（低空交给 PX4 更稳，落地检测也正常）。
LAND_HANDOVER_ALT = 0.60          # 交给 AUTO_LAND 的高度 (m)
LAND_STUCK_SEC = 4.0              # 主动下降"卡住"多久就提前交给 AUTO_LAND (s)

# ★★ 收尾"归位"（2026-10-17 用户提出：飞回起点再降落，省掉每次重启仿真）
#   【为什么值得做】跑完一个 law 后无人机停在拦截点（实测 x≈−8~−10 m），
#     下一次跑就会在**远离原点处起飞** ⇒ 低空 58° 姿态 ⇒ 桨叶打地（已两次实飞）。
#     归位后落点回到起飞点附近 ⇒ 下一轮 `PRECHECK` 能直接通过，不必重启仿真。
#   【为什么用"位置级联"而不是"先刹停再飞回去"】
#     刹车与归位都是水平机动，可以合并：一次减速 + 一次回飞最省时间，
#     而且不会出现"急刹 → 掉头 → 再加速"的来回摆动。
#     控制律：v_target = kp·(p_start − p_now) 限幅 ±RETURN_V_MAX
#             a_h      = kd·(v_target − v_now)        限幅 ±a_max
#   ★ 高度上限保护：归位期间高度目标 = 进入时高度（不低于 MIN_ALTITUDE），
#     并且**复用 TAKEOFF_MIN_AGL 那道闸** —— 低空一律不许水平机动。
RETURN_KP_POS = 0.45              # 位置环增益 (1/s)
RETURN_KD_VEL = 2.5               # 速度环增益 (1/s)
RETURN_V_MAX = 2.0                # 归位最大水平速度 (m/s) —— 保守，别飞太快
RETURN_XY_TOL = 0.25              # 归位到位容差 (m)
RETURN_TIMEOUT = 30.0             # 归位超时 (s) —— 超时就在当前位置降落
RETURN_LOW_ALT = 3.0              # 归位期间保底高度（低于它先拉高再归位）
FORCE_DISARM_AFTER = 6.0
FORCE_DISARM_HOLD = 3.0

ALT_KP = 1.0                       # 起飞/预加速段的垂直保持 PD（不是制导）
ALT_KD = 2.0

MIN_ALTITUDE = 3.0                 # 安全围栏
MAX_H_DISP = 25.0

# ★★ 起飞安全门限（2026-10-17 用户指出，实测复核后确认是真隐患）
#   **物理原因**：多旋翼要靠"倾斜"产生水平加速度。离地太近时一倾斜，
#   低侧桨叶就直接扫到地面（x500 桨叶半径约 0.25 m，机体再倾斜 17° 就有
#   约 7 cm 的下沉量；叠加起落架高度与 PX4 高度估计误差，低空非常危险）。
#   ⇒ 规矩：**先纯垂直爬升到 safe height，再允许任何水平加速度**。
#
#   ⚠ 为什么必须是**绝对高度**，不能用 `0.75 × target_height` 这种相对判据：
#     原来 WAIT_HEIGHT 用 `alt > 0.75*target_height` 判"够高了"，于是
#       · `--target-height 1.5` ⇒ 门限掉到 **1.125 m**
#       · `--target-height 0.5` ⇒ 门限掉到 **0.375 m** ← 此高度给 3 m/s² 必打地
#     另一个漏洞是 `near = |alt−target| ≤ --takeoff-tol`：
#       · `--takeoff-tol 8`（target 8）⇒ **alt ≥ 0 就算到位**，1.5 s 后就进
#         OFFBOARD/SETTLE 立刻给水平加速度。
#   ⇒ 现在两条判据都**同时**要求 `alt ≥ TAKEOFF_MIN_AGL`，与 target/tol 无关。
TAKEOFF_MIN_AGL = 2.5              # 允许水平机动的最小离地高度 (m)
#   取值依据：3 m/s² 水平加速度 ⇒ 倾角 atan(3/9.8) ≈ 17°；
#   桨叶半径 0.25 m ⇒ 桨尖下沉 ≈ 0.25·sin17° ≈ 0.073 m；
#   再算起落架 ~0.2 m + EKF 低空高度误差（可达 0.2 m）⇒ 2.5 m 有 >8 倍余量。
#   同时它低于 MIN_ALTITUDE(3.0)，不会和安全围栏互相打架。

# ★★ 起飞前自检 + 起飞段姿态监控（2026-10-17 第二次实飞事故后新增）
#   实测事故：两轮实飞都是**在远离原点的位置起飞**（9.61 m / 10.49 m），
#   而且 APN 那次起点正好是 PPN 的停桨点 ⇒ **跑第二个 law 前没重启仿真**。
#   在这种位置上 PX4 的 AUTO_TAKEOFF 会在低空出现巨大姿态：
#     PPN: roll=58° @ alt=0.08 m ；APN: roll=59° @ 0.24 m、pitch=−74° @ 0.68 m
#   ⇒ 桨叶必然扫地。而这些姿态**不是我们发的**（此刻 nav_state=17，PX4 自控），
#     所以只能在"起飞前"和"起飞中"两个环节拦截。
PAD_MAX_H = 1.5                    # 地面启动时允许的最大水平偏离原点距离 (m)
PAD_MAX_ALT = 1.0                  # 高度低于它视为"在地面启动"（否则按空中接管处理）
PAD_MAX_TILT = 15.0                # 地面启动时允许的最大姿态角 (deg)
TAKEOFF_MAX_TILT = 25.0            # 起飞段（低空）允许的最大姿态角 (deg)，超过就告警
TAKEOFF_ABORT_TILT = 45.0          # 起飞段（低空）姿态超过它 ⇒ 直接中止（必打地）

GUIDED_TIMEOUT = 8.0               # GUIDED 段超时（s，按目标相对时间）
PLATE_TAIL_SEC = 0.30              # ★ 撞板 HIT 后的判定尾窗 (s)：继续检测真穿板
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

# RTF 只在相邻 tick 间隔 ≥ 此值时统计（s）。理由：`pos.timestamp` 是按**话题周期**
# 量化的（SITL 20~30 ms），若两次 tick 只隔 1~2 ms，sim_dt 却是整个话题周期
# ⇒ RTF 会算出 100+ 的**假值**（实测：报告里出现 `RTF max 103.10`，纯噪声）。
RTF_MIN_GAP = 0.005

# 超过这个空洞长度就必须怀疑 PX4 的 offboard 失联保护已触发
# （`COM_OF_LOSS_T` 默认 0.5~1.0 s）—— 一旦触发，PX4 会停止执行我们的 setpoint，
#  实飞表现就是无人机**自由落体**（实测 6.87 m / 1.15 s、a≈7.8 m/s²）。
OFFBOARD_LOSS_HINT = 0.60

# ★★ 这些状态里我们**本来就不承担 setpoint 义务**，主回路被阻塞是**无害的**
#   （2026-10-17 实测：把 `gz_setup` 挪到 ARM 之前后，开场出现过一次 2028 ms 空洞，
#    但当时 state=ARM、飞机还在地上、PX4 也没进 offboard ⇒ 起飞完全正常
#    —— 而旧文案照旧喊"很可能已触发 offboard 失联 ⇒ 自由落体"，属于**误报**）。
#   ⇒ 空洞按"是否在 offboard 义务期"分级：地面段只记录、不喊自由落体，
#     报告里也分开统计（判"有没有问题"只看飞行段）。
NO_SETPOINT_STATES = ("PRECHECK", "ARM", "CONFIRM_ARM", "POST_ARM",
                      "TAKEOFF", "WAIT_HEIGHT")

# ★ 只在这几个状态统计"最低离地高度"：它们本该在 8~10 m 高度飞行，
#   贴地就说明失去了高度控制。降阶段（LAND/DISARM）本来就该贴地，不算。
AIRBORNE_AUDIT_STATES = ("SETTLE", "CRUISE", "GUIDED", "BRAKE")

# ★ PRECHECK 等 `/odometry` 的宽限（s）：没有 odom 就没有 roll/pitch，
#   姿态检查会被静默跳过（实跑 `[0/6] 姿态=?`）。等待是**非阻塞**的（下一拍再判）。
PRECHECK_ODOM_GRACE = 3.0

# BRAKE 段的"地面安全地板"（m）：刹车过程中若离地低于它，说明已不是"正常刹车"
# （大概率是失控/遥测跳变）⇒ 立刻放弃横向刹车，转入 LAND 让它软着陆。
BRAKE_ALT_FLOOR = 1.50

# ★ 弹丸"停在发射架顶面"时，球底与顶面之间留的间隙 (m)
#   0.02 就够（纯视觉，避免球嵌进方块里产生 z-fighting 闪烁）
PROJ_PAD_GAP = 0.02

# 发射架方块边长 (m)。⚠ **必须与 `gz_scene_sync.LAUNCHER_SIZE` 一致**
#   （那边是延迟导入的，模块级拿不到；启动时会做一次一致性自检并 warn）
GZ_LAUNCHER_SIZE = 0.6
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


# ===========================================================================
#  ★ 自动瞄准：让弹丸的（解析）弹道**对准被守护的防御塔**
# ===========================================================================
#  【为什么需要它】（2026-10-18 实跑）
#    用户把发射台挪到 (-18,-3,10)，却忘了同时改 `--target-azim`（还是 0）：
#      · 弹丸沿 y=-3 往 +x 直飞，无人机沿 y≈0 迎击 ⇒ 两条线**平行错开 3.13 m**；
#      · 实测脱靶 3.009 m ≈ 3.13 m（完全吻合）；
#      · 离线复算"若不拦截"：弹丸离塔最近 **1.82 m**（安全垫 1.0 m 内）
#        ⇒ 这一发**本来就打不到塔**，拦截没有意义。
#    ⇒ 方位角、仰角与发射点是**耦合**的，靠人手维护必然漏。交给程序算。
#
#  【两件事】
#    ① 方位角：解析解 —— 从发射点**水平指向塔轴心**；
#       注意不能用"塔在原点 ⇒ atan2(-y,-x)"，塔可以不在原点。
#    ② 仰角  ：**打靶法** —— 扫仰角，使弹道穿入塔体（到塔体最近距离 → 0）。
#       弹丸是"重力 + 气动阻力"的开环解析弹道（C++ `TargetModel`），
#       距离/速度一变，原来配平的 -6° 就不成立了（14 m/s 下射程只有 16.8 m）。
#
#  【边界】
#    · 只**解算角度**，不改弹道模型、不改制导律 ⇒ 与"所见即 TargetModel 所算"一致。
#    · 速度不动（那是用户的实验变量）；解不出照射角时给出**具体建议**（含建议速度）。
# ===========================================================================

AIM_AUTO = "auto"

# 仰角搜索范围（deg）与步长。粗扫 2° + 细扫 0.05°：
#   18 m 处 0.05° ≈ 1.6 cm，远小于塔半径 1.2 m ⇒ 精度足够。
AIM_ELEV_MIN = -70.0
AIM_ELEV_MAX = 35.0
AIM_COARSE_STEP = 2.0
AIM_FINE_STEP = 0.05
AIM_SAMPLES = 240          # 每条弹道采样点数（含端点）
AIM_HIT_TOL = 1e-3         # 到塔体距离小于它就认为"穿入塔体"(m)
# ★ 首选仰角（deg）：本项目配平过的值（14 m/s 时射程 16.8 m、塔在原点）。
#   `auto` 的语义是"**保证对得上塔**"，不是"求穿塔最正的角度"：
#   实测 -14,0,10 时 -13.4° 穿塔更"正"（塔内停留 0.186 s vs 擦顶 0.049 s），
#   但交会点更靠内、t_go 更短 ⇒ 制导**脱靶 0.653 m**（-6° 时命中 0.209 m）。
#   ⇒ 先用这个值试探，能穿塔就用它（保持对照实验可比），打不到才真正打靶求解。
AIM_ELEV_PREFER = -6.0
# 解不出时用来给"建议速度"的候选档（m/s）
AIM_SPEED_LADDER = (14.0, 16.0, 18.0, 20.0, 24.0, 28.0)


def parse_angle_or_auto(text):
    """argparse 用：接受一个角度（deg）或 `auto`。

    ⚠ 保持对旧命令行的完全兼容：`--target-azim=0` 仍然是一个数字。
    """
    s = str(text).strip().lower()
    if s in ("auto", "tower", "aim"):
        return AIM_AUTO
    try:
        return float(s)
    except ValueError:
        raise argparse.ArgumentTypeError(
            f"需要角度（deg）或 `auto`：{text!r}")


def tower_from_args(args):
    """从命令行参数构造"被守护目标"（防御塔）的几何对象。

    复用 `scene_model.Tower`（**唯一**几何定义处）—— 它同时被
    `viz_scene.py` / `sim_phase_b_offline.py` 用来做威胁评估，
    两边必须在同一份实现上，否则"报告说构成威胁、仿真说没打中"。
    """
    import scene_model as _SM
    xy = parse_vec2(getattr(args, "tower_xy", "0,0"), (0.0, 0.0))
    return _SM.Tower(
        center=(float(xy[0]), float(xy[1]), 0.0),
        radius=float(getattr(args, "tower_radius", 1.2)),
        height=float(getattr(args, "tower_height", 3.0)),
        safety_margin=float(getattr(args, "tower_margin", 1.0)))


def aim_azimuth_deg(p0, tower):
    """弹丸应取的**方位角**（deg）：从发射点在水平面内指向塔轴心。

    ENU 下 azim 的口径 = "速度方向与 +x 的夹角，向 +y 为正"（与 `--target-azim` 一致）。
    """
    dx = float(tower.center[0]) - float(p0[0])
    dy = float(tower.center[1]) - float(p0[1])
    if math.hypot(dx, dy) < 1e-9:
        return 0.0                      # 正好在塔正上方，方位角无意义
    return math.degrees(math.atan2(dy, dx))


def aim_pitch_deg(p0, tower):
    """"直线对准"的仰角（deg）：发射点 → 塔**中部**的连线的俯仰角。

    只用来做**初始猜测/合理性对照**；真正的仰角由 `solve_elev_deg` 打靶求解
    （弹丸受重力下坠，必须比直线更抬高一点）。
    """
    dx = float(tower.center[0]) - float(p0[0])
    dy = float(tower.center[1]) - float(p0[1])
    z_mid = float(tower.center[2]) + float(tower.height) * 0.5
    dz = z_mid - float(p0[2])
    h = math.hypot(dx, dy)
    if h < 1e-9:
        return -90.0
    return math.degrees(math.atan2(dz, h))


def min_tower_distance(lib, guide, tower, n=AIM_SAMPLES):
    """采样**当前已配置**的弹丸弹道，返回到塔体的最近距离与时刻。

    返回 `(最近距离 m, 该时刻 t_rel, 穿入塔体的采样点数, 穿塔时长 s)`。
    最近距离 0 表示弹道**真的穿进了塔体**（不是"擦过附近"）。
    """
    t_flight = float(lib.pgu_target_flight_time(guide))
    if not (t_flight > 0.0):
        return float("inf"), 0.0, 0, 0.0
    buf = darr([0.0] * 9)
    best_d, best_t, n_inside = float("inf"), 0.0, 0
    n = max(2, int(n))
    for i in range(n + 1):
        t = t_flight * i / float(n)
        lib.pgu_target_state(guide, t, buf)
        d = tower.distance_to_body((buf[0], buf[1], buf[2]))
        if d <= 0.0:
            n_inside += 1
        if d < best_d:
            best_d, best_t = d, t
    return best_d, best_t, n_inside, t_flight


def aim_boost_dir(p0, tower=None):
    """巡航迎击方向：从起飞点（世界原点）指向**弹丸发射点**的水平单位矢量。

    ★ 为什么要自动化（2026-10-18）：
      弹丸发射点一离轴（y≠0），"沿 −x 迎击"就不再对准弹丸来向 ⇒
      交班时横向偏差 ≈ |p0_y|，直接吃掉 `½·a_max·t_go²`（≈1.25 m）的修正预算。
      实测 (-18,-3,10) + boost_dir=(-1,0,0)：离线脱靶 **0.844 m**；
      改成指向发射点 (-0.986,-0.164,0)：命中 **0.064 m**。
    ⚠ **零行为变化**：发射点在 x 轴上时（`p0_y == 0`）结果就是 `(-1,0,0)`，
      与原默认值逐位相同 ⇒ 旧的对照实验完全可比。
    """
    dx = float(p0[0]) - 0.0            # 起飞点默认在世界原点
    dy = float(p0[1]) - 0.0
    h = math.hypot(dx, dy)
    if h < 1e-9:
        return (-1.0, 0.0, 0.0)
    return (dx / h, dy / h, 0.0)


def probe_elev(lib, guide, cfg_fn, p0, speed, azim_deg, tower, elev_deg,
               n_samples=AIM_SAMPLES):
    """体检：**给定仰角**下，这一发会不会打到塔？返回与 `solve_elev_deg` 同构的 dict。

    用途有两个：
      · `--target-elev=auto` 的第一步（先试配平值，见 `AIM_ELEV_PREFER`）；
      · 用户**手填**仰角时也给一次结论（不要让人以为"对着"就是"打得到"）。
    """
    lib.pgu_config_target(guide, darr(cfg_fn(p0, speed, azim_deg, elev_deg)))
    d, t, ins, tf = min_tower_distance(lib, guide, tower, n=n_samples)
    step = (tf / float(max(2, int(n_samples)))) if tf > 0 else 0.0
    return {"elev": float(elev_deg), "dist": d, "t_rel": t, "inside": ins,
            "inside_time": ins * step, "t_flight": tf,
            "ok": d <= AIM_HIT_TOL,
            "straight": aim_pitch_deg(p0, tower),
            "coarse_best": float(elev_deg), "coarse_dist": d}


def _elev_key(res, straight_deg):
    """候选仰角的排序键（越小越好）。

    ★ 为什么要 tie-break（2026-10-18 实测）：
      塔是**圆柱**，很多仰角都能"打进塔体"（dist 都是 0）：
        -14,0,10 时 -6° 擦着塔顶(3.0 m)穿过、-13.4° 从塔身中部穿过。
      如果只按 dist 排序，粗扫从最陡端开始 ⇒ 会选到最陡的那个，
      与"原来配平的 -6°"完全不同 ⇒ 对照实验失去可比性。
      判据优先级：
        ① 到塔体最近距离（越小越好，0 = 穿入）；
        ② **穿塔时长**（越长说明穿得越正、越深入，而不是擦边）；
        ③ 与"直线瞄准角"的差（越接近直线越自然，射程损失最小）。
      ⚠ 这个排序只在"首选值打不到"时才会用到（见 `AIM_ELEV_PREFER`）：
        因为"穿塔最正"未必"最好拦"（实测 -13.4° 反而脱靶 0.653 m）。
    """
    return (round(res["dist"], 6), -round(res["inside_time"], 6),
            abs(res["elev"] - straight_deg))


def solve_elev_deg(lib, guide, cfg_fn, p0, speed, azim_deg, tower,
                   elev_lo=AIM_ELEV_MIN, elev_hi=AIM_ELEV_MAX,
                   coarse_step=AIM_COARSE_STEP, fine_step=AIM_FINE_STEP,
                   n_samples=AIM_SAMPLES):
    """**打靶法**求仰角：使弹道穿入塔体（到塔体最近距离最小）。

    `cfg_fn(p0, speed, azim_deg, elev_deg)` 负责把参数写进 lib（测试可注入）。
    返回 dict：
      elev   最终仰角 (deg)
      dist   该仰角下弹道到塔体的最近距离 (m)
      t_rel  最近时刻 (s)
      inside 穿入塔体的采样点数（0 ⇒ 没打中塔体）
      inside_time 穿塔时长 (s) —— 越大越"正"（而不是擦边）
      ok     是否真的穿入塔体
    """
    straight = aim_pitch_deg(p0, tower)

    def _eval(elev):
        lib.pgu_config_target(guide, darr(cfg_fn(p0, speed, azim_deg, elev)))
        d, t, ins, tf = min_tower_distance(lib, guide, tower, n=n_samples)
        step = (tf / float(max(2, int(n_samples)))) if tf > 0 else 0.0
        return {"elev": float(elev), "dist": d, "t_rel": t, "inside": ins,
                "inside_time": ins * step, "t_flight": tf}

    # ---- ① 粗扫：找到大致最优区间 ----
    n_coarse = max(1, int(round((elev_hi - elev_lo) / coarse_step)))
    coarse = []
    for i in range(n_coarse + 1):
        coarse.append(_eval(elev_lo + coarse_step * i))
    coarse.sort(key=lambda r: _elev_key(r, straight))
    best = coarse[0]

    # ---- ② 细扫：在最优附近 ±coarse_step 内加密 ----
    lo = max(elev_lo, best["elev"] - coarse_step)
    hi = min(elev_hi, best["elev"] + coarse_step)
    n_fine = max(1, int(round((hi - lo) / fine_step)))
    for i in range(n_fine + 1):
        e = lo + fine_step * i
        if abs(e - best["elev"]) < 1e-9:
            continue                    # 粗扫已经算过
        r = _eval(e)
        if _elev_key(r, straight) < _elev_key(best, straight):
            best = r

    out = dict(best)
    out["ok"] = best["dist"] <= AIM_HIT_TOL
    out["straight"] = straight
    out["coarse_best"] = coarse[0]["elev"]
    out["coarse_dist"] = coarse[0]["dist"]
    return out


def suggest_speed_for_tower(lib, guide, cfg_fn, p0, azim_deg, tower,
                            speed_now, ladder=AIM_SPEED_LADDER):
    """当前速度打不到塔时，找一个**能打到**的速度档，用于给建议文案。

    返回 `(建议速度, 该速度下最优仰角, 最近距离)`；都不行则返回 None。
    ⚠ 只**建议**，不自动改 —— 速度是用户做对照实验的变量。
    """
    for v in ladder:
        if v <= speed_now:
            continue
        res = solve_elev_deg(lib, guide, cfg_fn, p0, v, azim_deg, tower)
        if res["ok"]:
            return v, res["elev"], res["dist"]
    return None


def describe_aim(p0, tower, azim_deg, elev_deg, res=None):
    """把瞄准解算结果整理成几行可读文本（启动日志 / 回传报告共用）。"""
    dx = float(tower.center[0]) - float(p0[0])
    dy = float(tower.center[1]) - float(p0[1])
    h = math.hypot(dx, dy)
    lines = [
        f"防御塔 @({tower.center[0]:+.2f},{tower.center[1]:+.2f}) "
        f"r={tower.radius:.2f} m h={tower.height:.2f} m，安全垫 {tower.safety_margin:.2f} m",
        f"发射点 → 塔心：水平距离 {h:.2f} m，方位角 {azim_deg:+.2f}°，"
        f"直线俯仰 {aim_pitch_deg(p0, tower):+.2f}°",
    ]
    if res is not None:
        tag = "✅ 穿入塔体" if res["ok"] else "❌ **打不到塔**"
        lines.append(
            f"打靶求解仰角 = {res['elev']:+.2f}°（弹道离塔最近 {res['dist']:.3f} m"
            f" @ t_rel={res['t_rel']:.3f}s，穿塔采样 {res['inside']} 点）⇒ {tag}")
    return lines


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


def parse_vec3_or_auto(s):
    """`--boost-dir` 用：接受 `auto`（= 指向弹丸发射点）或 "x,y,z"。"""
    t = str(s).strip().lower()
    if t in ("auto", "aim", ""):
        return None                    # None ⇒ 节点里按发射点自动算
    return parse_vec3(s)


def parse_vec2(s, default=(0.0, 0.0)):
    """解析 "x,y"（容忍空格/多余分隔符）；解析不了就回退默认值并返回默认。

    ⚠ 故意**不抛异常**：这是可视化参数，写错了不该让整个飞行节点起不来。
    """
    try:
        parts = [float(x) for x in str(s).replace(" ", "").split(",") if x != ""]
        if len(parts) >= 2:
            return (parts[0], parts[1])
    except Exception:                                  # noqa: BLE001
        pass
    return tuple(default)


# ============================================================================
#  节点
# ============================================================================
class PhaseBGuidance(Node):

    def __init__(self, args):
        super().__init__("phase_b_guidance")
        self.args = args
        self.ns = args.namespace

        # ---- ★ 固定装甲板（撞板任务，2026-10-18）----
        #   打开 `--armor-plate` 后：
        #     · **目标 = 板心（静止点）** ⇒ TargetModel 用 STRAIGHT + speed 0（原生支持：
        #       v_T=0、a_T=0、hits_ground=false、t_flight=t_max）
        #     · **命中判据 = 板形**（穿越板面 / 贴板容差 + 机身半径），不再用球形 R_true
        #   ⚠ 默认关 ⇒ 下面这些字段都在、但 plate 为 None ⇒ 老流程零行为变化。
        self.plate = None
        self.plate_hit = False
        self.plate_verdict_rec = None
        self.plate_min_dist = None
        self.plate_min_t = None
        self._plate_prev = None
        # ★ 2026-10-25：HIT 后的判定尾窗（见 plate_tail_step）
        self.plate_tail_until = None      # 尾窗截止（time.monotonic）
        self.plate_tail_prev = None       # 尾窗内的上一拍位置（ENU）
        self.plate_tail_confirmed = False
        self.plate_hit_tol = float(getattr(args, "plate_hit_tol", 0.15))
        self.plate_body_radius = float(getattr(args, "plate_body_radius", 0.20))
        if getattr(args, "armor_plate", False):
            from armor_plate import ArmorPlate as _ArmorPlate
            _sz = [float(x) for x in
                   str(args.plate_size).replace(" ", "").split(",") if x != ""]
            if len(_sz) != 3:
                raise SystemExit(f"--plate-size 需要 3 个分量（宽,高,厚）："
                                 f"{args.plate_size!r}")
            self.plate = _ArmorPlate(
                name="enemy_armor_plate",
                center=tuple(float(x) for x in args.plate_center),
                width=_sz[0], height=_sz[1], thickness=_sz[2],
                yaw_deg=float(getattr(args, "plate_yaw", 0.0)),
                pitch_deg=float(getattr(args, "plate_pitch", 0.0)))
            # 目标改为静止点：STRAIGHT(0) + speed 0 ⇒ v_T=0、a_T=0、hits_ground=false
            args.target_p0 = self.plate.center
            args.target_type = 0
            args.target_speed = 0.0
            args.target_elev = 0.0
            args.target_azim = 0.0
            # ★ 安全围栏提醒：MIN_ALTITUDE 是硬 abort 线，板放太低必然被它打断
            if float(self.plate.center[2]) < MIN_ALTITUDE + 0.5:
                self.warn(f"⚠ 装甲板中心 z={self.plate.center[2]:.2f} m 低于安全围栏 "
                          f"MIN_ALTITUDE({MIN_ALTITUDE:.1f})+0.5 m ⇒ 飞机会在接近板之前"
                          f"被 check_safety **abort**（撞不到板）。"
                          f"建议 --plate-center 的 z ≥ {MIN_ALTITUDE + 1.0:.1f}")

        # ---- 场景参数 ----
        self.target_p0 = args.target_p0
        self.hit_radius = args.hit_radius
        self.a_boost = args.a_boost
        self.v_boost_min = args.v_boost_min
        self.boost_dir = (aim_boost_dir(self.target_p0)
                          if args.boost_dir is None else args.boost_dir)
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

        # ---- ★ 自写起飞（2026-10-19）：不调 XTDrone2 的 TAKEOFF ----
        #   为什么要自写：XTDrone2 的 TAKEOFF 把高度写死 10 m
        #   （multirotor_communication.py:320 `param7=10.0`），而 PX4 在 AUTO_TAKEOFF
        #   期间**不执行我们的 setpoint** ⇒ 想"边爬边平动"或"只飞到 5 m"都做不到。
        #   自写路径：ARM → OFFBOARD（速度 setpoint 养模式）→ cmd_vel_ned 定速爬升。
        self.self_takeoff = bool(getattr(args, "self_takeoff", False))
        self.climb_rate = float(getattr(args, "climb_rate", 1.5))
        self.climb_tol = float(getattr(args, "climb_tol", 0.30))
        self.climb_t0 = None                 # 进入 CLIMB 的时刻（算爬升耗时）
        self.self_takeoff_secs = None        # 爬升实际耗时（报告用）
        if self.self_takeoff:
            if self.climb_rate <= 0.0:
                raise SystemExit("--climb-rate 必须 > 0")
            if self.target_height < TAKEOFF_MIN_AGL + 1.0:
                self.warn(f"⚠ 自写起飞目标高度 {self.target_height:.2f} m 太低"
                          f"（安全门限 TAKEOFF_MIN_AGL={TAKEOFF_MIN_AGL:.1f} m + 1 m）"
                          f"⇒ 建议 --target-height ≥ {TAKEOFF_MIN_AGL + 2.0:.1f}")

        # ---- 制导核 ----
        self.lib, lib_path = load_guidance_lib(args.lib)
        self.guide = self.lib.pgu_create()
        self.lib_path = lib_path

        # ---- ★ 自动瞄准（2026-10-18）：把"弹丸对着防御塔"变成计算而不是手填 ----
        #   ① 方位角解析解 ② 仰角打靶（弹道受重力+阻力，必须解）
        #   ⚠ 顺序：必须先 `pgu_create`（打靶要反复 config + 采样），
        #     但**可以**在 `pgu_config_guidance` 之前 —— 制导参数与弹道无关。
        self.tower = tower_from_args(args)

        def _cfg_fn(p0, speed, azim_deg, elev_deg):
            """把目标参数整理成 29 元素配置（打靶时反复调用）。"""
            return target_cfg(p0, args.target_type, speed, elev_deg, azim_deg,
                              args.target_mass, args.target_cd,
                              args.target_area, args.target_rho)

        # —— 方位角 ——
        if self.plate is not None:
            # ★ 撞板模式：目标静止 ⇒ **不需要**自动瞄准（那套是给"弹丸打塔"解弹道用的）
            self.azim_deg = 0.0
            self.elev_deg = 0.0
            self.aim_azim_auto = False
            self.aim_elev_auto = False
            self.aim_speed_hint = None
            self.aim_res = {"ok": True, "chosen_by": "plate",
                            "dist": 0.0, "t_rel": 0.0, "inside": 0}
        elif args.target_azim == AIM_AUTO:
            self.azim_deg = aim_azimuth_deg(self.target_p0, self.tower)
            self.aim_azim_auto = True
        else:
            self.azim_deg = float(args.target_azim)
            self.aim_azim_auto = False

        # —— 仰角 ——
        #   `auto` 分两步（见 AIM_ELEV_PREFER 注释）：
        #     ① 先用**配平过的首选值**试：能穿塔就用 ⇒ 与旧对照实验完全可比；
        #     ② 打不到才真正**打靶求解**（并且只改角度，不改速度）。
        self.aim_speed_hint = None      # 打不到塔时给的建议速度
        if self.plate is not None:
            pass                        # 撞板模式：上面已设好 elev_deg=0，跳过打靶
        elif args.target_elev == AIM_AUTO:
            self.aim_elev_auto = True
            self.aim_res = probe_elev(
                self.lib, self.guide, _cfg_fn, self.target_p0,
                args.target_speed, self.azim_deg, self.tower, AIM_ELEV_PREFER)
            if self.aim_res["ok"]:
                self.elev_deg = AIM_ELEV_PREFER
                self.aim_res["chosen_by"] = "prefer"
            else:
                self.aim_res = solve_elev_deg(
                    self.lib, self.guide, _cfg_fn, self.target_p0,
                    args.target_speed, self.azim_deg, self.tower)
                self.aim_res["chosen_by"] = "solver"
                self.elev_deg = self.aim_res["elev"]
                if not self.aim_res["ok"]:
                    # 打不到 ⇒ 帮用户算一个"能打到"的速度（只建议，不自动改）
                    self.aim_speed_hint = suggest_speed_for_tower(
                        self.lib, self.guide, _cfg_fn, self.target_p0,
                        self.azim_deg, self.tower, args.target_speed)
        else:
            self.elev_deg = float(args.target_elev)
            self.aim_elev_auto = False
            # 手填角度时也做一次体检：这发到底打不打得到塔？
            self.aim_res = probe_elev(
                self.lib, self.guide, _cfg_fn, self.target_p0,
                args.target_speed, self.azim_deg, self.tower, self.elev_deg)
            self.aim_res["chosen_by"] = "manual"

        self.lib.pgu_config_target(self.guide, darr(_cfg_fn(
            self.target_p0, args.target_speed, self.azim_deg, self.elev_deg)))
        self.lib.pgu_config_guidance(self.guide, darr(guidance_cfg(
            args.law, args.N, args.a_max, args.t_go_freeze)))
        self.lib.pgu_config_sensor(self.guide, darr(sensor_cfg()))
        self.t_flight = self.lib.pgu_target_flight_time(self.guide)
        self.t_hits_ground = bool(self.lib.pgu_target_hits_ground(self.guide))

        # ---- Gazebo 可视化（可选） ----
        # ⚠ 只做"取数 + 摆位置"，不含算法；接不上 Gazebo 时自动降级，不报错。
        self.gz = None
        self.gz_trail = []                    # 弹丸轨迹（ENU 真值）
        self.gz_uav_trail = []                # 无人机轨迹（ENU，回放用）
        self.gz_proj_pos_sent = 0             # 实际推给 Gazebo 的弹丸位置次数
        self.gz_proj_pos_req = 0              # 请求更新的次数（每帧一次）
        self.gz_last_p_T = None               # ★ 最后一帧弹丸位置（命中标记用，与 --gz-trail 解耦）
        self.gz_replay_job = None             # 回放任务 JSON 路径（收尾时启动）
        self.gz_setpose_t0 = None             # 制导开始时的 set_pose 计数（算有效帧率）
        self.gz_setpose_t1 = None             # 命中/结束时
        self.gz_flight_secs = None            # 交会段时长（s）
        self.gz_tower_r = float(self.tower.radius)   # ★ 视觉塔 = 被守护目标
        self.gz_tower_h = float(self.tower.height)
        # ★ 2026-10-18：视觉塔的位置**默认跟随 `--tower-xy`**（= 被守护目标，
        #   也是自动瞄准的靶心）。原来两套坐标各自独立，很容易"弹丸照着一个方向飞、
        #   画面里的塔在另一个地方"。显式给 `--gz-tower-xy` 才覆盖，并会告警。
        _gxy_raw = getattr(args, "gz_tower_xy", None)
        _txy = (float(self.tower.center[0]), float(self.tower.center[1]))
        if _gxy_raw is None or str(_gxy_raw).strip() == "":
            self.gz_tower_xy = _txy
        else:
            self.gz_tower_xy = parse_vec2(_gxy_raw, _txy)
            if math.hypot(self.gz_tower_xy[0] - _txy[0],
                          self.gz_tower_xy[1] - _txy[1]) > 1e-6:
                self.warn(f"⚠ `--gz-tower-xy`={self.gz_tower_xy} 与瞄准目标 "
                          f"`--tower-xy`={_txy} **不一致** ⇒ 画面里的塔不是被瞄准的那个，"
                          f"看到弹丸从塔旁边飞过是正常的（弹丸打的是 {_txy}）")
        self.gz_ready = False      # ★ 场景是否已摆好（gz_setup 幂等开关）
        # 发射架是边长此值的方块（与 gz_scene_sync.LAUNCHER_SIZE 一致）；
        # **顶面在 z=size**（中心在 size/2）⇒ 弹丸"贴顶面"时用得到
        self.gz_launcher_size = GZ_LAUNCHER_SIZE
        if args.gz_viz:
            try:
                from gz_scene_sync import GzScene
                # ⚠ timeout_ms 原来是默认 500 ms —— 太短！
                #   建实体要"解析 SDF + 插入世界"，实测常超 500 ms ⇒ 假失败
                #   （而且"先删后建"会把旧实体删掉 ⇒ 模型凭空消失）。
                #   `_create` 内部已改用 CREATE_TIMEOUT_MS(3000 ms)，
                #   这里把基础超时提到 1000 ms，set_pose 也更稳。
                self.gz = GzScene(world=args.gz_world, hz=args.gz_hz,
                                  timeout_ms=1000)
            except Exception as e:                      # noqa: BLE001
                self.warn(f"⚠ 加载 gz_scene_sync 失败（{e}）⇒ 跳过 Gazebo 可视化")
        # ★ 2026-10-17 审查：`--gz-replay` 只放"轨迹 + 命中点"，而轨迹只有
        #   `--gz-trail` 才记 ⇒ 两者没同时给时，"回放已启动"但画面空。
        #   这里提前说清楚（不要等到落地后才发现）。
        if args.gz_viz and args.gz_replay and not args.gz_trail:
            self.warn("⚠ --gz-replay 还需要 --gz-trail（否则没有轨迹可放）"
                      "⇒ 本次回放会被跳过")

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
        self.loop_gaps = []                       # [(t_wall, gap, state, sim_dt, on_duty), ...]
        # ★ 只有"飞行段（offboard 义务期）"的空洞才算事故；地面段的无害
        self.loop_gap_flight_max = 0.0
        self.loop_gap_flight_total = 0.0
        self.rtf_stale = 0                        # 被丢弃的"话题陈旧"RTF 样本数
        # ---- ★ 仿真时间基准（PX4 时间戳）----
        #   ★ 为什么要它：实飞里 BRAKE 段出现"0.104 s 墙钟内位置跳 6.87 m"，
        #     光看墙钟会以为是遥测坏了。加上仿真时间才发现那 0.104 s 墙钟里
        #     **仿真推进了 1.149 s**（RTF≈11），位置跳变其实完全自洽。
        self.sim_us_last = None                   # 上一拍的 PX4 时间戳（µs）
        self.rtf_samples = []                     # 相邻拍的 仿真Δt/墙钟Δt
        self.min_alt = None                       # 飞行段最低离地高度（ENU z）
        self.min_alt_at = None                    # 出现时刻
        self.min_alt_state = None                 # 出现时的状态
        self.hard_ground_warned = False           # "制导/刹车段贴地"只警告一次
        self.brake_alt_tgt = None                 # 刹车期间锁定的高度目标
        # 收尾归位（飞回起点再降落 ⇒ 下一轮不必重启仿真）
        self.return_xy = (0.0, 0.0)               # 归位目标（ENU，SETTLE 时记录）
        self.return_ok = False                    # 归位是否成功到位（报告用）
        # 降落两阶段（LAND 段：先主动下降 → 再交给 AUTO_LAND）
        self.land_auto_sent = False               # 是否已交给 PX4 的 AUTO_LAND
        self.land_stuck_t = None                  # 主动下降"卡住"计时起点
        self.land_last_alt = None                 # 上一拍的下降高度（判卡住用）
        # 起飞安全门限相关的一次性告警标记（避免每拍刷屏）
        self.low_takeoff_warned = False           # WAIT_HEIGHT：容差内但低于门限
        self.low_xy_warned = False                # SETTLE/CRUISE：低空被撤掉水平指令
        # ---- ★ 落点 / 漂移（**实测**，不是估算）----
        self.ending_pos = None                    # 收尾开始时的 ENU 位置
        # ★ 进入 LAND 时的位置（真正的"着陆漂移"起点；BRAKE 段还含"归位"机动）
        self.land_pos = None
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
        # PRECHECK → ARM → CONFIRM_ARM → TAKEOFF → WAIT_HEIGHT
        #     → OFFBOARD → CONFIRM_OFFBOARD → SETTLE
        #     → CRUISE → GUIDED → LAND → DISARM → DONE
        self.state = "PRECHECK" if not getattr(args, "no_precheck", False) else "ARM"
        self.t_state = time.monotonic()
        self.t_start = time.monotonic()
        self.done_t = None
        self.should_exit = False
        self.force_exit_requested = False
        self.aborted = False
        self.abort_reason = ""
        self.finish_reason = None             # 结束原因（命中/脱靶/超时/中止）
        # ---- 起飞前自检（★ 2026-10-17）----
        self.refused = False                  # 自检拒绝启动（不进入降落：本来没起飞）
        self.refuse_reason = ""
        self.pad_checked = False              # 自检是否已通过
        # ★ PRECHECK 等 `/odometry` 的日志只打一次（没有 odom 就没有 roll/pitch，
        #   姿态检查会被静默跳过 —— 实跑 `[0/6] 姿态=?` 就是这个问题）
        self.odom_wait_logged = False
        self.on_ground_start = None           # True=地面启动 / False=空中接管
        self.takeoff_tilt_warned = False      # 起飞段大姿态只告警一次
        self.takeoff_tilt_max = 0.0           # 起飞段（低空）最大姿态角（报告用）
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
                 f"{self.args.target_speed:.1f} m/s，仰角 {self.elev_deg:+.2f}°"
                 f"{'（auto 打靶）' if self.aim_elev_auto else ''}，"
                 f"方位 {self.azim_deg:+.2f}°"
                 f"{'（auto 对准塔）' if self.aim_azim_auto else ''}")
        self.log(f"        目标飞行时间={self.t_flight:.2f}s  hits_ground={self.t_hits_ground}")
        if self.plate is not None:
            self.log(f"  ★ 撞板模式：固定装甲板「{self.plate.name}」"
                     f"中心 ENU=({self.plate.center[0]:.2f},{self.plate.center[1]:.2f},"
                     f"{self.plate.center[2]:.2f})  "
                     f"宽×高×厚={self.plate.width:.2f}×{self.plate.height:.2f}×"
                     f"{self.plate.thickness:.3f} m  "
                     f"法向 yaw={self.plate.yaw_deg:+.1f}°  pitch={self.plate.pitch_deg:+.1f}°")
            self.log(f"        ⚠ 板尺寸/容差是**我们的假设**（规则表 5-1 对『空中机器人"
                     f"冲击』写的是『待定』）：tol={self.plate_hit_tol} m + 机身 "
                     f"{self.plate_body_radius} m")
        # ★ 自动瞄准报告（"这发到底会不会打到塔"一目了然）——撞板模式无意义，跳过
        if self.plate is None:
            for _ln in describe_aim(self.target_p0, self.tower,
                                    self.azim_deg, self.elev_deg, self.aim_res):
                self.log("        " + _ln)
        if self.plate is None and not self.aim_res.get("ok", False):
            self.warn(f"⚠⚠ 当前几何下弹丸**打不到防御塔**（最近 {self.aim_res['dist']:.2f} m）"
                      f"—— 这一发拦截**没有意义**（塔不会被击中）。")
            if self.aim_speed_hint:
                _v, _e, _d = self.aim_speed_hint
                self.warn(f"   ⇒ 建议把发射速度提到 ≥ {_v:.1f} m/s"
                          f"（那时仰角 {_e:+.2f}° 可穿塔，最近 {_d:.3f} m）："
                          f"`--target-speed={_v:.1f}`")
            else:
                self.warn(f"   ⇒ 建议把发射点挪近一些（水平距离 < 20 m），"
                          f"或提高 `--target-speed`")
        self.log(f"  制导: law={law_name} N={self.args.N} a_max={self.args.a_max} "
                 f"t_go_freeze={self.args.t_go_freeze}   指令率 {self.rate:.0f} Hz")
        self.log(f"  预加速: {'ON' if self.cruise_on else 'OFF'}"
                 f"（朝 {self.boost_dir}，a={self.a_boost} m/s² 至 |v|≥{self.v_boost_min} m/s）")
        xy_txt = "不收敛位置（仅速度阻尼）" if self.settle_xy is None \
            else f"收敛到 ENU({self.settle_xy[0]:g},{self.settle_xy[1]:g})"
        self.log(f"  稳定段: 高度→{self.target_height:.1f} m；水平 {xy_txt}"
                 f"；横向保持={'ON' if self.args.cruise_lat_hold else 'OFF'}")
        self.log(f"  起飞门限: 离地 < {TAKEOFF_MIN_AGL:.1f} m 时**禁止任何水平加速度**"
                 f"（先纯垂直爬升；防止机体倾斜导致桨叶打地）")
        if self.self_takeoff:
            self.log(f"  ★ 起飞方式: **自写**（ARM→OFFBOARD→cmd_vel_ned 爬升 "
                     f"{self.climb_rate:.2f} m/s 到 {self.target_height:.1f} m；"
                     f"跳过 XTDrone2 TAKEOFF 写死的 10 m）")
        else:
            self.log(f"  起飞方式: XTDrone2 TAKEOFF（高度写死 10 m，随后 SETTLE 段"
                     f"再调到 {self.target_height:.1f} m）")
        if self.plate is not None:
            self.log(f"  命中判定: **撞板**（板形）—— 距板 ≤ {self.plate_hit_tol} m + 机身 "
                     f"{self.plate_body_radius} m，或穿越板面矩形")
        else:
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
        """更新全程最大水平位移 + **飞行段最低离地高度**（报告 / 安全审计用）。"""
        if self.pos is None:
            return
        h = self.h_disp()
        if h > self.max_h_disp:
            self.max_h_disp = h
        alt = -self.pos.z
        # ★ 2026-10-17：**只在飞行段统计**最低高度。
        #   原来从 PRECHECK 就开始记 ⇒ 报告里出现
        #   「全程最低离地 0.06 m @ state=PRECHECK」这种噪声（那是飞机趴在地上，
        #   本来就应该贴地），反而掩盖"飞行中真的掉到 0.44 m"这种真事故。
        #   降落段（LAND/DISARM）也排除 —— 它本来就该贴地。
        if self.state in AIRBORNE_AUDIT_STATES:
            if self.min_alt is None or alt < self.min_alt:
                self.min_alt = alt
                self.min_alt_at = time.monotonic() - self.t_start
                self.min_alt_state = self.state
        # ★ 安全审计：**制导 / 巡航 / 刹车**段本不该贴地（发射高度 8 m），
        #   若贴地说明失去了控制（实飞就是在 BRAKE 段掉到 0.44 m）。
        if (not self.hard_ground_warned
                and self.state in ("CRUISE", "GUIDED", "BRAKE")
                and alt < LANDED_ALT + 0.30):
            self.hard_ground_warned = True
            self.warn(f"⚠⚠ 严重：{self.state} 段离地只有 {alt:.2f} m"
                      f"（此时并未进入降落流程）⇒ 已失去高度控制。"
                      f"请检查该时刻附近有没有『控制回路空洞』告警。")

    def landing_drift(self):
        """**实测**着陆漂移：进入 LAND 的位置 → 停桨点（水平）。

        返回 (漂移 m, Δx, Δy) 或 None。
        ⚠ 为什么必须实测：原来日志只给估算 `v²/(2·a_max)`，实飞里写着
          「残余漂移 ≈ 0.02 m」而**实际漂了 10 m**（差 500 倍）——
          估算完全没有把"刹车阶段实际发生了什么"算进去。
        ★ 2026-10-17：起点改成"进入 LAND 的位置"，而不是"收尾起点"。
          BRAKE 段现在还兼做**归位**（有意飞回起点 7 m），若从收尾起点算，
          会得到"着陆漂移 6.92 m"和"归位 ✓"同时成立的自相矛盾输出。
          `return_pos`/`brake_span` 另外给"归位段飞了多远"。
        """
        start = self.land_pos if self.land_pos is not None else self.ending_pos
        if start is None or self.stop_pos is None:
            return None
        dx = self.stop_pos[0] - start[0]
        dy = self.stop_pos[1] - start[1]
        return (math.hypot(dx, dy), dx, dy)

    def return_span(self):
        """收尾起点 → 进入 LAND 的水平位移（= BRAKE 段"刹车 + 归位"飞了多远）。"""
        if self.ending_pos is None or self.land_pos is None:
            return None
        return math.hypot(self.land_pos[0] - self.ending_pos[0],
                          self.land_pos[1] - self.ending_pos[1])

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

    def publish_climb_vel(self, rate=None):
        """★ 自写起飞：发 `cmd_vel_ned` 的**垂直速度**（NED z 向下为正 ⇒ 爬升填负值）。

        为什么用速度而不是加速度（2026-10-19）：
          · 加速度指令是**纯前馈、无阻尼**（坑 1）⇒ 从地面起步没法"稳住高度"；
          · 速度 setpoint 由 PX4 的位置/速度控制器闭环 ⇒ 能定速爬升、也能随时刹住；
          · 速度 setpoint 有限且非 NaN ⇒ 不会触发"accel 模式污染 trajectory_setpoint"
            那个让落地检测永久失效的坑（阶段 A 坑 8）。
        `rate=0` 用于"保持当前高度"（也给 OFFBOARD 养模式用，保证 setpoint 流不断）。
        """
        r = float(self.climb_rate if rate is None else rate)
        m = Twist()
        m.linear.x = 0.0
        m.linear.y = 0.0
        m.linear.z = float(-r)          # NED +z 向下 ⇒ 爬升为负
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
    # ================================================= 起飞前自检 / 拒绝启动
    def do_precheck(self, now):
        """起飞前自检（★ 2026-10-17 实飞事故后新增，防止"在错误位置起飞"）。

        【为什么必须有这一步】
          两次实飞都是在**远离原点**的位置起飞的：
            · PPN：起点 ENU=(-9.57, +0.89) ⇒ 离原点 **9.61 m**
            · APN：起点 ENU=(-10.50, +0.05) ⇒ 离原点 **10.49 m**
              —— 正好是 PPN 的停桨点，说明跑第二个 law 前**没重启仿真**
          在这种位置上 PX4 的 AUTO_TAKEOFF 会在低空甩出巨大姿态：
            · PPN: roll=58° @ alt=0.08 m
            · APN: roll=59° @ 0.24 m、pitch=−74° @ 0.68 m
          ⇒ 桨叶扫地。**这些姿态不是我们发的**（当时 nav_state=17，PX4 自控），
            所以只能在起飞前拦截 —— 光加上"低空不给水平加速度"是拦不住的。

        【分级】
          · 致命（拒绝启动，不做降落：本来就没起飞）：离原点太远 / 已解锁 / 已在 OFFBOARD
          · 告警（继续跑）：姿态略大、非地面启动
        """
        if self.pos is None:
            if now - self.t_state > 10.0:
                self.refuse("收不到 vehicle_local_position —— 仿真没起来？"
                            "或 ns 不对（--namespace）")
            return

        p_enu, _ = self.state_enu()
        alt = -self.pos.z
        h = math.hypot(p_enu[0], p_enu[1]) if p_enu is not None else 0.0
        att = self.odom_att()
        tilt = 0.0
        if att is not None:
            tilt = max(abs(att[0]), abs(att[1]))
        on_ground = alt < PAD_MAX_ALT

        self.on_ground_start = on_ground
        self.log(f"[0/6] 起飞前自检：ENU=({p_enu[0]:+.2f},{p_enu[1]:+.2f},"
                 f"{p_enu[2]:+.2f})  离原点 {h:.2f} m  "
                 f"姿态={('?' if att is None else '%.1f/%.1f°' % (att[0], att[1]))}  "
                 f"nav_state={self.nav_state}  arming={self.arming_state}")

        fatal = []
        notes = []
        if on_ground:
            # —— 地面启动：必须在起飞点、水平、已停桨 ——
            if h > PAD_MAX_H:
                fatal.append(f"离原点 {h:.2f} m > {PAD_MAX_H:.1f} m")
            if att is not None and tilt > PAD_MAX_TILT:
                fatal.append(f"机体倾斜 {tilt:.1f}° > {PAD_MAX_TILT:.0f}°")
            if self.arming_state == ARMING_STATE_ARMED:
                fatal.append("已经处于 ARMED（上一轮没退干净）")
            if self.nav_state == NAV_STATE_OFFBOARD:
                fatal.append("已经处于 OFFBOARD（上一轮没退干净）")
        else:
            # —— 空中接管：允许，但明确提示（不是从地面开始的测试） ——
            notes.append(f"高度 {alt:.2f} m > {PAD_MAX_ALT:.1f} m ⇒ 按空中接管处理"
                         f"（不是从地面起飞）")
            if not self.arming_state == ARMING_STATE_ARMED:
                fatal.append(f"空中但未解锁（arming_state={self.arming_state}）")

        # ★★ 2026-10-17 实跑：`[0/6] 姿态=?` —— 第一轮 PRECHECK 时 `/odometry`
        #   还没到，于是**姿态那一项检查被静默跳过**（sensor 一旦晚到就形同虚设）。
        #   ⇒ 若"唯一缺的证据"就是姿态，且还在地面：先放一放，下一拍再判
        #     （**不 sleep**，不占用控制回路），最多等 `PRECHECK_ODOM_GRACE` 秒；
        #     等不到就带着明确告警继续（不能因为 odom 没来就永远不飞）。
        if (on_ground and att is None and not fatal
                and now - self.t_state < PRECHECK_ODOM_GRACE):
            if not self.odom_wait_logged:
                self.odom_wait_logged = True
                self.log(f"[0/6] 等 `/odometry` 到了再判姿态"
                         f"（最多 {PRECHECK_ODOM_GRACE:.0f}s；没有它就没有 roll/pitch）")
            return
        if on_ground and att is None and not fatal:
            notes.append(f"始终没收到 `/odometry` ⇒ **姿态检查跳过**"
                         f"（无法判断是否倾斜；请确认 odom 桥接在跑）")

        if fatal:
            self.refuse("；".join(fatal) + "。"
                        " ★ 最可能的原因：**上一次运行结束后没有重启仿真**，"
                        "无人机还停在拦截点。请 Ctrl-C 停掉 launch，"
                        "重新执行 ros2 launch xtd2_launch ros2_single_vehicle_demo_launch.py，"
                        "等无人机回到原点后再跑。"
                        "（确认环境没问题时可用 --no-precheck 跳过本检查）")
            return

        for n in notes:
            self.warn("   " + n)
        self.pad_checked = True
        self.log("[0/6] ✓ 自检通过，开始 ARM")
        # ★★ 2026-10-17 审查：**在这里**把 Gazebo 场景摆好，而不是等到"起飞到位"之后。
        #   为什么（这是本次审查里唯一的**飞行安全**级问题）：
        #     `gz_setup()` 里全是**阻塞**服务调用 —— `refresh_snapshot` 最多 1.5 s，
        #     三个 `ensure_*` 每个最多 3 s×2 次重试 ⇒ 最坏能卡十几秒。
        #     它原来挂在 SETTLE→CRUISE 的转换点上（`[5/6] 稳定到位` 之后），
        #     那一段**每 20 ms 就有一条 offboard setpoint**必须发出去；
        #     一旦被卡住 ≥ COM_OF_LOSS_T(0.5~1.0 s)，PX4 就停用自己的 offboard 控制
        #     ⇒ 10 m 高**自由落体**（这正是 2026-10-16 那次"落点漂 12.9 m"的同一机理）。
        #   ⇒ 挪到**解锁之前**：此刻飞机还在地上、PX4 不在 offboard，卡多久都无害。
        #     （只有在"地面启动"时才做；空中接管时跳过，见 gz_setup 里的判断）
        if on_ground:
            self.gz_setup()
        else:
            self.warn("   ⚠ 空中接管：跳过 Gazebo 场景摆放"
                      "（它会阻塞数秒，空中做会让 PX4 触发 offboard 失联）")
        self.state = "ARM"
        self.t_state = now

    def refuse(self, reason):
        """**拒绝启动**：不进入降落流程（本来就没起飞），直接退出。

        ⚠ 与 `abort()` 的区别：`abort()` 会走 LAND→DISARM 收尾；这里没起飞，
          走降落反而危险（可能在空中无控漂移）。所以只报错 + 置 should_exit。
        """
        if self.refused:
            return
        self.refused = True
        self.refuse_reason = reason
        bar = "!" * 76
        self.log("")
        self.log(bar)
        self.log("!! 起飞前自检**拒绝启动** —— 不执行本次测试")
        self.log(bar)
        for line in str(reason).split("；"):
            if line.strip():
                self.log("   · " + line.strip())
        self.log(bar)
        self.log("   为什么拦：在错误位置/姿态起飞，低空会甩出 50~70° 姿态，"
                 "桨叶必然扫地（2026-10-17 两次实飞都发生了）。")
        self.log("")
        self.should_exit = True

    def update_takeoff_tilt(self):
        """起飞段姿态监控（★ 低空大姿态 = 桨叶打地的直接征兆）。

        量的是**低空**（alt < TAKEOFF_MIN_AGL）时的姿态角。
        这些姿态是 PX4 的 AUTO_TAKEOFF 产生的（不是我们发的），
        但我们能做的只有：**发现异常就告警/中止**，别让它在坏状态上继续跑。
        """
        if self.pos is None:
            return
        alt = -self.pos.z
        if alt >= TAKEOFF_MIN_AGL:
            return
        att = self.odom_att()
        if att is None:
            return
        tilt = max(abs(att[0]), abs(att[1]))
        if tilt > self.takeoff_tilt_max:
            self.takeoff_tilt_max = tilt
        if tilt >= TAKEOFF_ABORT_TILT:
            self.abort(f"起飞段低空姿态 {tilt:.0f}° ≥ {TAKEOFF_ABORT_TILT:.0f}°"
                       f"（alt={alt:.2f} m）—— 桨叶必然打地，立即中止")
        elif tilt >= TAKEOFF_MAX_TILT and not self.takeoff_tilt_warned:
            self.takeoff_tilt_warned = True
            self.warn(f"⚠ 起飞段低空姿态 {tilt:.0f}°（alt={alt:.2f} m，"
                      f"roll={att[0]:+.1f}° pitch={att[1]:+.1f}°）"
                      f"—— 超过 {TAKEOFF_MAX_TILT:.0f}° 有桨叶打地风险。"
                      f"常见原因：起飞点不在原点 / 上一轮没重启仿真")

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
            # 仿真时间在这一拍推进了多少（µs → s）—— 解释"位置跳变"的关键
            sim_dt = None
            if self.pos is not None:
                t_us = int(self.pos.timestamp)
                if self.sim_us_last is not None:
                    d_us = t_us - self.sim_us_last
                    if 0 <= d_us < 60 * 1_000_000:      # 忽略回绕/异常跳变
                        sim_dt = d_us * 1e-6
                self.sim_us_last = t_us
            # ⚠ RTF 只在"足够长的间隔"上算：`pos.timestamp` 是**话题周期**量化的
            #   （SITL 20~30 ms），若两次 tick 挨得很近（gap < 5 ms），
            #   sim_dt 却是整整一个话题周期 ⇒ RTF 会算出 100+ 的假值（实测踩到）。
            # ★ 2026-10-17 又踩到第二种假值：主回路被阻塞后，下一拍拿到的 `pos`
            #   可能**还是被阻塞前的旧消息**（回调还没来得及跑）⇒ 下一拍 sim_dt
            #   一下子等于整个阻塞时长，而 gap 只有 20 ms ⇒ RTF 算出 165.84。
            #   ⇒ 间隔很短却"仿真推进很多"，只可能是话题陈旧，直接丢弃这个样本。
            stale = (gap < 0.10 and sim_dt is not None and sim_dt > 0.30)
            if sim_dt is not None and gap >= RTF_MIN_GAP and not stale:
                self.rtf_samples.append(sim_dt / gap)
                if len(self.rtf_samples) > 40000:
                    del self.rtf_samples[:20000]
            elif stale:
                self.rtf_stale += 1
            if gap > self.loop_gap_max:
                self.loop_gap_max = gap
                self.loop_gap_max_at = now - self.t_start
                self.loop_gap_max_state = self.state
            if gap >= LOOP_GAP_WARN:
                tw = now - self.t_start
                # ★ 分级：地面段（我们本来就不用发 setpoint）与飞行段分开记
                on_duty = self.state not in NO_SETPOINT_STATES
                self.loop_gaps.append((round(tw, 3), round(gap, 3), self.state,
                                       None if sim_dt is None else round(sim_dt, 3),
                                       on_duty))
                if on_duty:
                    self.loop_gap_flight_max = max(self.loop_gap_flight_max, gap)
                    self.loop_gap_flight_total += gap
                extra = ""
                if sim_dt is not None and gap > 1e-6:
                    extra = (f"；该间隔内**仿真推进 {sim_dt:.3f}s**"
                             f"（RTF={sim_dt / gap:.2f}）")
                self.warn(f"⚠ 控制回路空洞 {gap * 1000:.0f} ms"
                          f"（t_wall={tw:.3f}s, state={self.state}）"
                          f"—— 本周期**没有**发出加速度 setpoint{extra}")
                if not on_duty:
                    self.warn(f"   ℹ 但此刻 state={self.state} **还不需要发 setpoint**"
                              f"（飞机在地面 / PX4 自控起飞段）⇒ 不会触发 offboard 失联，"
                              f"对飞行无影响；只是把后续步骤推迟了 {gap:.1f}s。")
                elif gap >= OFFBOARD_LOSS_HINT:
                    self.warn(f"   ⚠⚠ 空洞 ≥ {OFFBOARD_LOSS_HINT:.1f}s："
                              f"PX4 的 offboard 失联超时（COM_OF_LOSS_T，默认 0.5~1.0 s）"
                              f"**很可能已触发** ⇒ PX4 会停用自己的 offboard 控制"
                              f"（实飞表现为无人机**自由落体**）。"
                              f"请查是谁在阻塞主线程（可视化 / 日志 / 磁盘）。")
        self.t_last_tick = now

        self.poll_service(now)

        self.track_h_disp()          # 全程最大水平位移（报告用）
        self.plate_tail_step(now)    # ★ 撞板判定尾窗（HIT 后继续找真穿板）

        # ---------------- Ctrl-C 收尾请求 ----------------
        # ⚠ 用 `ending` 而不是 `landing_phase` —— BRAKE 段还没进 landing_phase，
        #   若用它判断，第二次 Ctrl-C 会重复触发收尾。
        if self.force_exit_requested and self.state != "DONE" and not self.ending:
            self.request_landing("Ctrl-C")

        # ---------------- 状态机 ----------------
        if self.state == "PRECHECK":
            self.do_precheck(now)
            return

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
                # ★ 自写起飞：跳 TAKEOFF/WAIT_HEIGHT，直接请求 OFFBOARD
                self.state = "OFFBOARD" if self.self_takeoff else "TAKEOFF"

        elif self.state == "TAKEOFF":
            if self.issue("TAKEOFF"):
                self.log(f"[2/6] TAKEOFF …（目标 {self.target_height:.1f} m）")

        elif self.state == "WAIT_HEIGHT":
            self.update_takeoff_tilt()      # ★ 低空大姿态监控（防桨叶打地）
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
            # ★ 两条判据都必须**同时**满足"绝对最小离地高度"（见 TAKEOFF_MIN_AGL 注释）。
            #   否则 --target-height 调小、或 --takeoff-tol 放宽，都会在贴地高度
            #   就放行 ⇒ 进 SETTLE 立刻给水平加速度 ⇒ 倾斜打地。
            h_gate = max(TAKEOFF_MIN_AGL, 0.75 * self.target_height)
            near = (abs(alt - self.target_height) <= self.args.takeoff_tol
                    and alt >= TAKEOFF_MIN_AGL)
            airborne = (alt > h_gate) and abs(vz_ned) < 0.4
            if (abs(alt - self.target_height) <= self.args.takeoff_tol
                    and alt < TAKEOFF_MIN_AGL and not self.low_takeoff_warned):
                self.low_takeoff_warned = True
                self.warn(f"⚠ 高度 {alt:.2f} m 已在 --takeoff-tol 容差内，但**低于起飞"
                          f"安全门限 {TAKEOFF_MIN_AGL:.1f} m** ⇒ 继续等爬升"
                          f"（此时给水平加速度会让桨叶打地）")
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
            if self.self_takeoff:
                # ★ 自写起飞：**先发速度 setpoint**（=0），不用加速度流。
                #   这样 trajectory_setpoint 的 velocity 从一开始就是有限的，
                #   不会先被 accel 回调的 NaN 污染，再切回来时产生"落地检测失效"的隐患。
                self.publish_climb_vel(0.0)
            else:
                self.publish_a()          # 持续流 setpoint 把 offboard 信号养起来
            if self.nav_state == NAV_STATE_OFFBOARD:
                dt = now - self.t_state
                self.offboard_confirmed = True
                self.log(f"[4/6] ✓ PX4 已确认进入 OFFBOARD（nav_state=14，耗时 {dt:.2f}s）")
                if self.self_takeoff:
                    self.state = "CLIMB"
                    self.climb_t0 = now
                    self.log(f"[3/6] 开始**自写爬升**：目标 {self.target_height:.1f} m，"
                             f"爬升率 {self.climb_rate:.2f} m/s（cmd_vel_ned，"
                             f"跳过 XTDrone2 写死的 10 m）")
                else:
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

        elif self.state == "CLIMB":
            # ★ 自写起飞（2026-10-19）：`cmd_vel_ned` 定速爬升。
            #   用**速度**而不是加速度的原因见 `publish_climb_vel()`：
            #   加速度是纯前馈无阻尼，从地面起步稳不住；速度由 PX4 速度环闭环。
            #   低空姿态监控（`update_takeoff_tilt`）与 AUTO_TAKEOFF 路径**共用同一套**，
            #   25° 告警 / 45° 中止的保护不打折。
            self.update_takeoff_tilt()
            alt = -self.pos.z if self.pos is not None else None
            if alt is None:
                if not self.no_pos_warned:
                    self.no_pos_warned = True
                    self.warn("收不到 vehicle_local_position —— 仿真在跑吗？")
                if now - self.t_state > 10.0:
                    self.abort("收不到位置遥测")
                return
            if alt >= self.target_height - self.climb_tol:
                # 到位：先发零垂直速度刹住，再交给 SETTLE 做最后收敛
                self.publish_climb_vel(0.0)
                self.self_takeoff_secs = (now - self.climb_t0
                                          if self.climb_t0 is not None else None)
                self.log(f"[3/6] 起飞到位（自写）：alt={alt:.2f} m，爬升耗时 "
                         f"{(self.self_takeoff_secs or 0.0):.2f}s"
                         f"（目标 {self.target_height:.1f} m）→ 进入 SETTLE 收敛")
                self.alt_target = alt
                self.state = "SETTLE"
                self.t_state = now
                return
            self.publish_climb_vel()
            if now - self.last_alt_log > 2.0:
                self.last_alt_log = now
                self.log(f"     自写爬升… alt={alt:.2f} m（目标 {self.target_height:.1f}）"
                         f"  rate={self.climb_rate:.2f} m/s  nav_state={self.nav_state}")
                if self.arming_state == ARMING_STATE_DISARMED:
                    self.warn("     ⚠ 车辆处于 DISARMED —— 不可能起飞。")
            if now - self.t_state > self.args.takeoff_timeout:
                self.abort(f"自写起飞超时（当前 {alt:.2f} m，"
                           f"目标 {self.target_height:.1f} m）")

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
                # ★★ 起飞安全门限：**离地不足只准拉高度、绝不给水平加速度**。
                #   即使前面 WAIT_HEIGHT 判据被放宽（--takeoff-tol / --target-height），
                #   这里也有一道硬闸 —— 倾斜打地是不可逆的，必须有双保险。
                xy_allowed = alt >= TAKEOFF_MIN_AGL
                if not xy_allowed and not self.low_xy_warned:
                    self.low_xy_warned = True
                    self.warn(f"⚠ SETTLE 段离地仅 {alt:.2f} m（< 门限 "
                              f"{TAKEOFF_MIN_AGL:.1f} m）⇒ **本段只做垂直爬升**，"
                              f"暂不给水平收敛指令（防桨叶打地）")
                if self.settle_xy is not None and xy_allowed:
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
            low_alt = (alt is not None and alt < TAKEOFF_MIN_AGL)
            settled = (not self.alt_hold) or (
                alt is not None
                and abs(alt - self.target_height) <= self.args.alt_settle_tol
                and abs(v_z_up) <= self.args.alt_settle_vtol
                # ★ 低空一律不算"稳定到位"（xy_err 为 None 时原逻辑会误判成"已到位"）
                and not low_alt
                and (xy_err is None
                     or (xy_err <= self.args.settle_xy_tol
                         and v_h <= self.args.settle_xy_vtol)))
            timed = (now - self.t_state >= self.args.alt_settle_timeout)
            # ★★ 超时也不能绕过高空门限：原来超时就直接进 CRUISE，
            #   若此时还在贴地高度（起飞异常），就会在低空给水平加速度 ⇒ 打地。
            #   ⇒ 低空 + 超时 = 直接中止（安全优先，宁可这一次不跑）。
            if timed and low_alt:
                self.abort(f"稳定段超时（{self.args.alt_settle_timeout:.0f}s）且离地仅 "
                           f"{alt:.2f} m（< 起飞门限 {TAKEOFF_MIN_AGL:.1f} m）"
                           f"—— 拒绝在贴地高度开始水平机动（会桨叶打地）")
                return
            if (settled and now - self.t_state >= 0.5) or timed:
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
                # ★ 记下**归位目标**（ENU）：收尾时飞回这里再降落 ⇒ 下一轮不必重启仿真
                #   ⚠ start_x/start_y 是 NED（x=北）；归位控制全用 ENU ⇒ 这里单独存一份 ENU。
                p_set, _ = self.state_enu()
                self.return_xy = ((p_set[0], p_set[1]) if p_set is not None
                                  else (0.0, 0.0))
                self.log(f"  ── 收尾归位目标（ENU）=({self.return_xy[0]:+.2f},"
                         f"{self.return_xy[1]:+.2f})   "
                         f"[--no-return 可关闭；关闭后下一轮需重启仿真]")
                # ⚠ 自检查的是"离**世界原点**"，而归位目标是"本次的起飞点"。
                #   若本次起点本身就偏了（如 --settle-xy 改了位置、或上一轮没退干净），
                #   归位回去**仍然过不了下一轮自检** ⇒ 必须提前说清，别让人白跑一轮。
                h0 = math.hypot(self.return_xy[0], self.return_xy[1])
                if h0 > PAD_MAX_H:
                    self.warn(f"⚠ 本次起飞点离世界原点 {h0:.2f} m（> {PAD_MAX_H:.1f} m）"
                              f"⇒ 归位只会回到这个偏掉的起点，**下一轮自检仍会拒绝启动**。"
                              f"请检查 --settle-xy 或先重启仿真。")
                # ★ 2026-10-17 审查：场景已经在 PRECHECK 通过时就摆好了（见 do_precheck），
                #   这里只是**幂等兜底**（`gz_ready` 为真时立刻返回，不碰任何服务）。
                #   ⚠ 千万不要把真正的 create/set_pose 放回这里 —— 本段每 20 ms
                #     都要发一条 offboard setpoint，阻塞 ≥0.5 s 就会触发 PX4 失联超时。
                self.gz_setup()          # --gz-viz 才动作；已就绪时零开销
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
            # 着陆前收尾机动：**刹车 + 归位到起点**（一次做完，避免"急刹→掉头→再飞"）
            #   ① 刹车：AUTO_LAND 水平减速度只有 ~2 m/s²，5 m/s 会漂 7 m ⇒ 先自己刹
            #   ② 归位：飞回 SETTLE 时记下的起点 (start_x, start_y)。
            #      动机（2026-10-17 用户提出）：跑完停在拦截点，下一轮就在远离原点处
            #      起飞 ⇒ 低空 58° 姿态 ⇒ 桨叶打地（已两次实飞）。
            #      归位后落点回到起点附近 ⇒ 下一轮 PRECHECK 直接通过，**不必重启仿真**。
            #   控制律：位置→速度级联（比"位置 PD 直接出加速度"更不容易超调）
            #     v_tgt = kp·(p_start − p)        限幅 ±RETURN_V_MAX
            #     a_h   = kd·(v_tgt − v)          再整体矢量限幅
            if self.pos is None:
                return
            p_enu, v_enu = self.state_enu()
            vh = math.hypot(v_enu[0], v_enu[1])
            alt = p_enu[2]
            # ★ 高度目标 = **进入刹车时的高度**（不是固定 target_height）。
            #   动机：原来固定追 8 m ⇒ 若刹车起点在 5 m，会先"爬回 8 m"再降落，
            #   多出一段无意义的高度修正（用户要求"平稳回到地面"）。
            #   ⚠ 但必须有**下限**：若进入时已低于 MIN_ALTITUDE（说明失控），
            #     要主动拉升，绝不能"保持"在贴地高度。
            alt_tgt = self.brake_alt_tgt
            a = [0.0, 0.0, 0.0]
            a[2] = self.alt_kp * (alt_tgt - alt) - self.alt_kd * v_enu[2]

            xy_err = None
            if self.args.return_to_start:
                # 起点用 ENU（start_x/Y 存的是 NED，见 SETTLE 段；这里换成 ENU 起点）
                tx, ty = self.return_xy
                ex, ey = tx - p_enu[0], ty - p_enu[1]
                xy_err = math.hypot(ex, ey)
                # 位置环 → 目标速度（限幅，避免远距离时飙高速）
                vtx = RETURN_KP_POS * ex
                vty = RETURN_KP_POS * ey
                nv = math.hypot(vtx, vty)
                vmax = max(float(self.args.return_vmax), 0.1)
                if nv > vmax and nv > 1e-9:
                    k = vmax / nv
                    vtx, vty = vtx * k, vty * k
                # 速度环 → 加速度
                a[0] = RETURN_KD_VEL * (vtx - v_enu[0])
                a[1] = RETURN_KD_VEL * (vty - v_enu[1])
            else:
                # 老行为：只刹车（−kd·v），不归位
                a[0] = -self.args.brake_kd * v_enu[0]
                a[1] = -self.args.brake_kd * v_enu[1]

            # ★★ 起飞安全门限复用：低空一律不许水平机动（要归位先拉高）
            if alt < TAKEOFF_MIN_AGL:
                if not self.low_xy_warned:
                    self.low_xy_warned = True
                    self.warn(f"⚠ 收尾机动机动时离地仅 {alt:.2f} m（< 门限 "
                              f"{TAKEOFF_MIN_AGL:.1f} m）⇒ 撤掉水平指令、只拉高度")
                a[0] = 0.0
                a[1] = 0.0
                a[2] = max(a[2], ALT_ADJ_A_MAX)
            # 矢量等比限幅（铁律 2：禁止逐分量 clip）
            na = math.sqrt(a[0] ** 2 + a[1] ** 2 + a[2] ** 2)
            if na > self.args.a_max and na > 1e-9:
                k = self.args.a_max / na
                a = [x * k for x in a]
            self.cur_a_enu = tuple(a)
            self.publish_a()

            # ★★ 地面安全地板：刹车中若离地低于 floor，说明已不是"正常刹车"
            #   （失控 / 遥测跳变）⇒ 立刻放弃横向机动转入 LAND（软着陆最安全）。
            #   实飞事故里刹车段的横向指令 10 m/s² 全被用来"救高度"，
            #   继续刹车没有意义且会继续漂。
            if alt < BRAKE_ALT_FLOOR and not self.hard_ground_warned:
                self.hard_ground_warned = True
                self.warn(f"⚠⚠ 收尾机动中离地仅 {alt:.2f} m（< 地板 {BRAKE_ALT_FLOOR:.1f} m）"
                          f"⇒ 放弃横向机动，立即转 LAND 软着陆（安全优先）")
                self.landing_phase = True
                self.alt_hold = False
                # ★ 记下"进入 LAND 的位置"（真正的着陆漂移起点，见 landing_drift）
                p_land0, _ = self.state_enu()
                self.land_pos = p_land0
                self.state = "LAND"
                self.t_state = now
                self.land_t0 = now
                self.last_land_cmd = 0.0
                return

            # ---- 完成判据 ----
            #   · 不归位：速度刹到 brake_vtol 就算完（老行为）
            #   · 归位  ：速度要小**且**位置到起点容差内
            speed_ok = vh <= self.args.brake_vtol
            if self.args.return_to_start:
                pos_ok = (xy_err is not None and xy_err <= self.args.return_tol
                          and vh <= max(self.args.brake_vtol, 0.30))
                done = pos_ok
                timed_out = (now - self.brake_t0
                             > max(self.args.brake_timeout, RETURN_TIMEOUT))
            else:
                done = speed_ok
                timed_out = now - self.brake_t0 > self.args.brake_timeout

            if done or timed_out:
                if done:
                    if self.args.return_to_start:
                        self.return_ok = True
                        self.log(f"   ✓ 已归位到起点：水平误差 {xy_err:.2f} m、"
                                 f"|v_h|={vh:.2f} m/s（耗时 {now - self.brake_t0:.1f}s）")
                        self.log(f"      ★ 落点回到起飞点附近 ⇒ **下一轮不必重启仿真**")
                    else:
                        self.log(f"   ✓ 刹车完成：|v_h|={vh:.2f} m/s"
                                 f"（耗时 {now - self.brake_t0:.2f}s）")
                    # ⚠ 这里只能给"剩余速度对应的估算刹车距离"，
                    #   真实漂移必须用实测 —— 实飞里估算 0.02 m 而实测 10 m。
                    self.log(f"      剩余速度对应的估算刹车距离 ≈ {vh**2 / (2 * 2.0):.2f} m"
                             f"（**实测**着陆漂移见末尾报告的 [落点] 行）")
                else:
                    if self.args.return_to_start:
                        self.warn(f"   ⚠ 归位超时（{max(self.args.brake_timeout, RETURN_TIMEOUT):.0f}s）："
                                  f"水平误差 {xy_err:.2f} m、|v_h|={vh:.2f} m/s"
                                  f" —— 就在当前位置降落（下一轮**请重启仿真**）")
                    else:
                        self.warn(f"   ⚠ 刹车超时（{self.args.brake_timeout:.0f}s）："
                                  f"|v_h|={vh:.2f} m/s —— 继续 LAND（会漂更多）")
                self.log("→ 转入自动降落：LAND（先自己降到 "
                         f"{LAND_HANDOVER_ALT:.1f} m，再交给 AUTO_LAND）→ 落地后 DISARM 停桨。")
                self.landing_phase = True
                self.alt_hold = False
                # ★ 记下"进入 LAND 的位置"（真正的着陆漂移起点，见 landing_drift）
                p_land0, _ = self.state_enu()
                self.land_pos = p_land0
                self.state = "LAND"
                self.t_state = now
                self.land_t0 = now
                self.last_land_cmd = 0.0

        elif self.state == "LAND":
            # ★★ 主动下降优先，AUTO_LAND 兜底（2026-10-17 实飞事故后改）
            #   事故现象：LAND 段高度**卡在 0.54 m 不动**（vz≈+0.02 m/s），
            #     然后 PX4 提前判定落地、直接停桨 ⇒ 无人机从 0.54 m **掉下来**，
            #     报告里 `landed=False 软着陆=否`。
            #   根因：一发 `NAV_LAND` 就把控制权交给 PX4 的 AUTO_LAND，
            #     而它的下降速度指令是**内置减速曲线** —— 从 8 m 一路降下来，
            #     速度在 0.5 m 附近已经被吃到 0 ⇒ 落地检测判静止 ⇒ 判落地。
            #     此时我们发的 `cmd_vel_ned` **完全无效**（AUTO_LAND 下不执行外部
            #     velocity setpoint），只能眼看着它卡住。
            #   ⇒ 改成：先**留在 OFFBOARD**，用自己的 cmd_vel_ned 以
            #     `--land-down-speed` 主动下降到 `LAND_HANDOVER_ALT`，
            #     再交给 AUTO_LAND 完成最后一段（低空交给 PX4 更稳，且它能正常判落地）。
            alt = -self.pos.z if self.pos is not None else None
            vz = self.pos.vz if self.pos is not None else None
            handover = (alt is not None and alt <= LAND_HANDOVER_ALT)
            if not handover:
                # —— 阶段 1：OFFBOARD 主动下降（可控）——
                self.publish_descend_vel()
                if alt is not None and vz is not None:
                    # 卡住检测：高度几乎不变 + 垂直速度为 0 ⇒ 没在下降
                    if (self.land_stuck_t is None
                            or abs(alt - self.land_last_alt) > 0.05):
                        self.land_stuck_t = now
                        self.land_last_alt = alt
                if (self.land_stuck_t is not None
                        and now - self.land_stuck_t > LAND_STUCK_SEC):
                    self.warn(f"⚠ 主动下降卡住了（{LAND_STUCK_SEC:.1f}s 内高度几乎不变，"
                              f"alt={alt:.2f} m）⇒ 提前交给 PX4 的 AUTO_LAND 兜底")
                    handover = True
                if not handover:
                    self.cur_a_enu = (0.0, 0.0, 0.0)   # 本段不发加速度（只发速度）

            # —— 阶段 2：交给 PX4 的 AUTO_LAND（兜底 / 最后一段）——
            if handover:
                if not self.land_auto_sent:
                    self.land_auto_sent = True
                    self.log(f"      → 在 {('?' if alt is None else '%.2f m' % alt)} "
                             f"交给 PX4 的 AUTO_LAND 完成最后下降"
                             f"（≤{LAND_HANDOVER_ALT:.1f} m）")
                self.publish_descend_vel()
                if now - self.last_land_cmd >= LAND_RESEND_SEC:
                    self.last_land_cmd = now
                    self.publish_land()
                    if self.nav_state != NAV_STATE_AUTO_LAND:
                        self.log(f"      LAND 已发（nav_state={self.nav_state}，"
                                 f"期望 {NAV_STATE_AUTO_LAND}=AUTO_LAND）")
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
                    # ★ 2026-10-19：PX4 自己停桨且高度已到位 ⇒ 视为**已落地**。
                    #   原来只置 disarmed、不置 landed ⇒ 报告出现
                    #   `landed:False disarmed:True` 的自相矛盾（实测踩到）。
                    #   ⚠ 高度高于判据时走的是上面那条 warn 分支，那里**不**置 landed。
                    self.landed = True
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
                # ★ 落地停桨后再慢速回放交会（这样画面不会被降落动作干扰）
                self.gz_launch_replay()
                self.should_exit = True
            return

        self.log_row()

    # ---------------------------------------------------------------- 段
    # ------------------------------------------------- Gazebo 可视化（可选）
    def gz_pad_z(self):
        """弹丸"停在发射架顶面"时的球心高度 (m)。

        ★ 2026-10-17 审查：这个值原来在 `gz_setup` 和 `gz_finish` 里**各算一遍**，
          而 `gz_finish` 那遍**漏了 `--gz-proj-pad-h` 覆盖** ⇒ 用户一旦指定自定义高度，
          跑完球会被"park"到一个和初始位置不同的 z（看起来像位置跳变）。
          ⇒ 唯一出处就是这里，两处都用它。
        """
        if self.args.gz_proj_pad_h is not None:
            return float(self.args.gz_proj_pad_h)
        return (self.gz_launcher_size + self.args.gz_proj_radius + PROJ_PAD_GAP)

    def gz_setup(self):
        """在 Gazebo 里摆好防御塔 + 敌方发射架 + 弹丸（弹丸停在发射架顶面）。

        ⚠ 只在 `--gz-viz` 且成功接上 Gazebo 时才有动作；任何失败都只警告不中断
          （可视化是"锦上添花"，绝不能因为它挂掉而影响制导回路）。

        ★★ **调用时机**（2026-10-17 审查改动）：只能在"解锁之前 / 地面"调用
          （见 `do_precheck` 里的详细理由）。函数内部全同步阻塞，最坏十几秒，
          放在飞行段会触发 PX4 的 offboard 失联超时 ⇒ 自由落体。
          本函数**幂等**：`self.gz_ready` 为真时直接返回。
        """
        if self.gz is None or not self.gz.available:
            if self.gz is not None:
                self.warn("⚠ Gazebo 可视化不可用（见上面的原因）—— 继续纯控制台运行")
            return False
        if self.gz_ready:
            return True
        # ⚠ 全程 try/except：可视化是"锦上添花"，**任何**异常都不得中断制导回路
        try:
            self.gz.start()
            # ★★ 2026-10-17 实跑根因：**进程内第一条服务请求要花时间做发现/握手**，
            #   1 s 的默认超时不够 ⇒ 开场两次 set_pose 各超时 1 s（主回路空洞 2028 ms、
            #   发射架和弹丸留在旧位置），而同一进程后面 30 Hz 的 set_pose 全部成功。
            #   ⇒ 先做一次长超时的**无害往返**把连接养起来，再摆实体。
            self.gz.warmup()
            # ★ 先读一次"世界真实状态"，让后面的 ensure_* 能判断"已存在否"。
            #   这样 `gz_setup` 在**起飞后**重跑时，已存在的塔/发射架/球只会被
            #   **复用或移动**，绝不会"先删后建"把它们删掉
            #   （2026-10-17 用户实测"起飞后球和发射塔又没有了"就是这个原因）。
            self.gz.refresh_snapshot(timeout=1.5)
            ok_t = self.gz.ensure_tower(self.gz_tower_r, self.gz_tower_h,
                                        center=self.gz_tower_xy)
            # ⚠ 原来这行**忽略了返回值** ⇒ 发射架建失败也没人知道
            #   （2026-10-17 实测世界里根本没有 enemy_launcher，日志却毫无提示）
            ok_l = self.gz.ensure_launcher((self.target_p0[0], self.target_p0[1], 0.0),
                                           self.gz_launcher_size)
            # ⚠ 一致性自检：本文件里的 GZ_LAUNCHER_SIZE 必须与 gz_scene_sync 的一致，
            #   否则"球心 = 边长 + 半径"算出来的高度会不对（球悬空或嵌进方块）。
            try:
                import gz_scene_sync as _gzs
                if abs(float(_gzs.LAUNCHER_SIZE) - self.gz_launcher_size) > 1e-6:
                    self.warn(f"⚠ 发射架边长不一致：节点 {self.gz_launcher_size} vs "
                              f"gz_scene_sync {_gzs.LAUNCHER_SIZE} "
                              f"⇒ 弹丸「贴顶面」的高度会算错")
            except Exception:                          # noqa: BLE001
                pass
            # ★ 弹丸初始**显示位置**（2026-10-17 用户要求：停在发射架顶面中心）
            #   与"弹道起点" target_p0 是**两个概念**，必须分开：
            #     · 显示位置：发射架顶面中心 —— 用户一进 Gazebo 就能看见这个球
            #     · 弹道起点：target_p0（默认 (-14,0,10)）—— TargetModel 算弹道用的真值
            #   ⚠ GUIDED 一开始，`gz_update` 会把球逐帧写到 TargetModel 的位置，
            #     所以球会**从发射架顶面瞬移到弹道起点**（默认 10 m 高）。
            #     这是预期行为：它模拟"发射瞬间"，不是 bug。
            #   发射架是边长 `LAUNCHER_SIZE` 的方块，中心在 z=size/2 ⇒ **顶面在 z=size**；
            #   球底面贴顶面 ⇒ 球心 z = size + radius。
            # ★ 固定装甲板（撞板任务）：幂等实体，起飞前摆好；失败只告警不中断
            if self.plate is not None:
                ok_pl = self.gz.ensure_armor_plate(
                    self.plate.center, self.plate.width, self.plate.height,
                    self.plate.thickness, self.plate.yaw_deg,
                    self.plate.pitch_deg)
                self.log(f"  ── Gazebo 固定装甲板：{'✓ 已就位' if ok_pl else '⚠ 创建失败'}"
                         f"  中心=({self.plate.center[0]:.2f},{self.plate.center[1]:.2f},"
                         f"{self.plate.center[2]:.2f})  宽×高×厚="
                         f"{self.plate.width:.2f}×{self.plate.height:.2f}×"
                         f"{self.plate.thickness:.3f} m  yaw={self.plate.yaw_deg:+.1f}°"
                         f"  pitch={self.plate.pitch_deg:+.1f}°")
            pad_z = self.gz_pad_z()          # 唯一出处，与收尾 park 用同一个值
            # ★ 撞板模式：目标不是弹丸 ⇒ 球**藏起来**（否则它会和板重叠，画面误导）
            at = ((self.target_p0[0], self.target_p0[1], pad_z)
                  if (self.args.gz_proj_preset and self.plate is None) else None)
            ok_p = self.gz.ensure_projectile(self.args.gz_proj_radius, at=at)
            # ★★ 弹丸可见性：创建时就放在**发射架顶面** + 再 set_pose 一次做**双保险**
            #   旧流程：创建在 HIDDEN_Z(-500) → set_pose 移上来
            #           ⇒ 两步都可能静默失败，且发射前球在地下 500 m，压根不在视野
            #   新流程：① create 时 pose 就给"发射架顶面"（不依赖 set_pose）
            #           ② 再 set_projectile 一次（有些 gz 版本会忽略 EntityFactory.pose，
            #              这一步兜住那种情况）—— 注意给的是**同一个位置**，
            #              不能给 target_p0，否则球又跑到 10 m 高去了。
            #   ⚠ **零风险**：不改变几何、不等待、不卡控制回路 ——
            #     弹丸位置在 GUIDED 段仍由 TargetModel 逐帧写入（见 gz_update）。
            if ok_p and self.args.gz_proj_preset and self.plate is None:
                self.gz.set_projectile((self.target_p0[0], self.target_p0[1], pad_z))
                self.log(f"  ── 弹丸已**停在发射架顶面中心** "
                         f"({self.target_p0[0]:.2f},{self.target_p0[1]:.2f},"
                         f"{pad_z:.2f})  r={self.args.gz_proj_radius:.2f} m "
                         f"（发射架顶面 z={self.gz_launcher_size:.2f} + 球半径）")
                self.log(f"     ⚠ 发射瞬间（GUIDED 开始）球会**跳到弹道起点** "
                         f"z={self.target_p0[2]:.1f} m —— 这是模拟发射，不是 bug")
                self.log(f"     （--no-gz-proj-preset 可改成藏在视野外；"
                         f"--gz-proj-pad-h=1.2 可指定球心高度）")
        except Exception as e:                          # noqa: BLE001
            self.warn(f"⚠ Gazebo 初始化抛异常（{e}）—— 跳过可视化，制导不受影响")
            self.gz = None
            return False
        # ★ 逐个报告三样实体的创建结果（不再是"要么全好要么只报一句失败"）
        _items = [("防御塔", ok_t), ("发射架", ok_l), ("弹丸", ok_p)]
        _bad = [nm for nm, o in _items if not o]
        if not _bad:
            self.log(f"  ── Gazebo 可视化已就绪：防御塔 r={self.gz_tower_r} m / "
                     f"h={self.gz_tower_h} m @ 原点；敌方发射架 @ "
                     f"({self.target_p0[0]:.1f},{self.target_p0[1]:.1f},0)；"
                     f"弹丸 r={self.args.gz_proj_radius:.2f} m "
                     f"{'@发射点' if self.args.gz_proj_preset else '@视野外'}")
        else:
            self.warn(f"⚠ Gazebo 实体创建失败：{_bad}"
                      f"（成功的是 {[nm for nm, o in _items if o] or '无'}）"
                      f" —— 制导不受影响，继续运行。"
                      f"自查：python3 gz_scene_sync.py --rebuild-scene")
        # ★ 无论成败都置位：重跑一次既不会变好，又要再阻塞十几秒 ——
        #   宁可只做一次，"失败就失败"（可视化不影响飞行）。
        self.gz_ready = True
        # ★ 视觉冲突自检：塔在原点、无人机也从原点起飞 ⇒ 起飞段无人机会**穿模**
        #   （塔只是 visual、没有 collision，所以物理上没事，但画面上很难看，
        #     也会让人误以为"无人机不见了/模型没了"）。这里只提示，不擅自挪位置。
        p_now, _ = self.state_enu()
        if p_now is not None:
            h_sep = math.hypot(p_now[0] - self.gz_tower_xy[0],
                               p_now[1] - self.gz_tower_xy[1])
            if h_sep < self.gz_tower_r + 0.5:
                self.warn(f"ℹ 防御塔在原点、半径 {self.gz_tower_r:.1f} m，"
                          f"而起飞点离原点仅 {h_sep:.2f} m ⇒ 起飞段无人机会"
                          f"**穿过塔身**（纯视觉，无碰撞）。想挪塔："
                          f"`--gz-tower-xy=0,-6`")
        return bool(ok_t and ok_p)

    def gz_update(self, p_T_enu):
        """每帧把**弹丸（目标）**位置推给 Gazebo（**非阻塞**，失败静默）。同时记轨迹点。

        ⚠ 传的是 `p_T`（弹丸），不是 `p_M`（无人机）—— 无人机本身由 PX4/Gazebo 自己渲染。
        ⚠ 包 try/except：绝不允许可视化把制导循环搞挂。
        """
        if self.gz is None or not self.gz.available:
            return
        # ★ 撞板模式：目标静止在板心。**不要**再把"弹丸球"推到板心上——
        #   那会变成"球嵌在板里"，画面误导（板才是目标，弹丸球已经不存在了）。
        #   ⇒ 撞板模式下弹丸球保持隐藏（gz_setup 里 `at=None`），这里也不推位置。
        if self.plate is not None:
            return
        try:
            self.gz.set_projectile(p_T_enu)
            self.gz_proj_pos_req += 1
            # ★ 每帧都记"最后一帧"（1 个 3 元组，开销可忽略）——
            #   命中标记不能依赖 `--gz-trail`，否则只开 --gz-viz 时黄球也不见了。
            self.gz_last_p_T = tuple(p_T_enu)
            if self.args.gz_trail:
                self.gz_trail.append(tuple(p_T_enu))
        except Exception:                               # noqa: BLE001
            self.gz = None          # 一次失败就彻底停掉可视化，避免每帧刷屏
            self.warn("⚠ Gazebo 更新失败 —— 已停用可视化（制导不受影响）")

    def gz_finish(self, hit):
        """收尾：命中画爆点 + 可选轨迹线。

        ★★ **必须在别的进程里做**（2026-10-16 实飞事故，两轮才定位到真因）：
          第 1 轮：这些调用直接写在主回路上 ⇒ 两次各 0.50 s ⇒ 主回路卡 1.0 s。
          第 2 轮：改成"放进后台线程队列" ⇒ **仍然卡 1.0 s**。
                  原因：`gz.transport` 的阻塞请求和本节点**共用同一个 Python
                  解释器** ⇒ 后台线程发请求一样抢 GIL。
          事故后果（第 2 轮实飞实测）：命中瞬间主回路空洞 **1015 ms**，
          那 1 s 里 PX4 收不到 offboard setpoint ⇒ 无人机**自由落体**
          （垂直：6.87 m / 1.149 s 仿真时间，a≈7.8 m/s²≈重力；水平：匀速漂 5.8 m）
          ⇒ 最终落点漂 **12.9 m**、最低掉到离地 **0.44 m**。
          ⇒ 现在改为**子进程**（`subprocess.Popen` 立即返回，子进程自己去连 gz 画图）：
            既不阻塞、也不抢 GIL、绘制时机还是命中瞬间。
          子进程起不来时**降级**为线程队列（有风险，会明确警告）。
        """
        if self.gz is None or not self.gz.available:
            return
        # ★ 2026-10-17 审查：命中标记原来**只能**从 `gz_trail` 里取最后一帧，
        #   而 `gz_trail` 只有加 `--gz-trail` 才记录 ⇒ 用户只开 `--gz-viz` 时
        #   "命中黄球"和"轨迹线"会**一起消失**，而且日志不提。
        #   ⇒ 命中点单独用 `gz_last_p_T`（每帧都记，零成本），与轨迹开关解耦。
        hit_pt = (self.gz_trail[-1] if self.gz_trail else
                  (self.gz_last_p_T if self.gz_last_p_T is not None else None))
        job = {
            "world": self.args.gz_world,
            "trail": [[float(p[0]), float(p[1]), float(p[2])]
                      for p in (self.gz_trail or [])],
            "mark": ([float(x) for x in hit_pt]
                     if (hit and hit_pt is not None) else None),
            # ★ 跑完后**不要把弹丸藏起来**（2026-10-17 用户反馈"弹丸消失了"）
            #   原来这里 `"hide": True` ⇒ 每次跑完球被移到 z=-500（地下）⇒
            #   用户在 Gazebo 里看到的就是"球不见了"。
            #   ⇒ 改成 `"park"`：把球**放回发射架顶面**，场景保持完整、随时能看见。
            "hide": False,
            # ★ park 高度与 gz_setup 用**同一个函数**算（含 --gz-proj-pad-h 覆盖）
            "park": [self.target_p0[0], self.target_p0[1], self.gz_pad_z()]
                    if self.args.gz_proj_preset else None,
            # 子进程画完自己删掉任务文件（原来 %TEMP% 里每轮多一个 json）
            "delete_job": True,
        }
        n_pts = len(job["trail"])
        if n_pts < 2:
            job.pop("trail", None)
        # ★ 慢速回放任务：**必须**同时带上无人机轨迹，否则看不出"怎么撞上的"
        if self.args.gz_replay:
            job["uav"] = [[float(p[0]), float(p[1]), float(p[2])]
                          for p in (self.gz_uav_trail or [])]
            job["seconds"] = self.args.replay_seconds
            job["hz"] = self.args.replay_hz
            job["hold"] = self.args.replay_hold
            job["lead_in"] = self.args.replay_lead_in
        try:
            path = os.path.join(tempfile.gettempdir(),
                                f"gz_draw_{os.getpid()}_{int(time.time() * 1000)}.json")
            with open(path, "w", encoding="utf-8") as f:
                json.dump(job, f)
            script = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                 "gz_scene_sync.py")
            # ★★ 2026-10-17 审查：原来这里是 `stdout=DEVNULL, stderr=DEVNULL`
            #   ⇒ 子进程崩了**一点痕迹都没有**（回放那条路已在同一天因此被坑过）。
            #   ⇒ 统一改成写日志文件；父进程开完立刻关句柄（Popen 已 dup 给子进程）。
            drawlog = path.replace(".json", ".log")
            with open(drawlog, "w", encoding="utf-8") as logf:
                subprocess.Popen([sys.executable, "-u", script, "--draw-json", path,
                                  "--world", self.args.gz_world],
                                 stdout=logf, stderr=subprocess.STDOUT,
                                 start_new_session=True)
            self.log(f"  ── Gazebo 收尾绘制已交给**独立子进程**"
                     f"（轨迹 {n_pts} 点{' + 命中标记' if job.get('mark') else ''}）"
                     f"—— 不占用控制回路，也不会抢 GIL")
            self.log(f"     子进程日志：{drawlog}"
                     f"（若画面上没出现轨迹/命中点，把这里的尾部发我）")
        except Exception as e:             # noqa: BLE001
            self.warn(f"⚠ 子进程绘制启动失败（{e}）—— 降级为后台线程"
                      f"（⚠ 线程仍可能抢 GIL 卡住主回路，仅作兜底）")
            try:
                if job.get("trail"):
                    self.gz.queue_trail(job["trail"])
                if job.get("mark"):
                    self.gz.queue_mark(job["mark"], ns="hit_mark", mid=1,
                                       size=0.6, color=(1.0, 0.95, 0.1),
                                       ttl_ms=15000)
                self.gz.queue_hide_projectile()
            except Exception as e2:        # noqa: BLE001
                self.warn(f"⚠ 兜底入队也失败（{e2}）—— 忽略")

        # ★ 回放任务**只写文件、先不启动** —— 等到落地停桨（DONE）再放，
        #   否则回放的球会在无人机刹车/降落时乱飞，画面很混乱。
        if self.args.gz_replay and job.get("trail"):
            try:
                rpath = os.path.join(
                    tempfile.gettempdir(),
                    f"gz_replay_{os.getpid()}_{int(time.time() * 1000)}.json")
                rjob = dict(job)
                rjob["hide_after"] = False
                rjob["clear"] = True          # 回放前清掉实时那次的轨迹/标记
                with open(rpath, "w", encoding="utf-8") as f:
                    json.dump(rjob, f)
                self.gz_replay_job = rpath
            except Exception as e:             # noqa: BLE001
                self.warn(f"⚠ 回放任务写入失败（{e}）—— 跳过回放")

    def gz_launch_replay(self):
        """★ 在**独立子进程**里慢速回放一次交会（跑完、已落地后再放）。

        弹丸交会只有 **~0.5 s**、球又只有十几厘米 ⇒ 实时里人眼看不见。
        回放把它拉长到 `--replay-seconds`（默认 6 s）、在相遇帧停顿，
        并画出**弹丸 + 无人机两条轨迹**，这样"怎么飞、什么时候撞上"一目了然。

        ⚠ 用 `print()` 而不是 `self.log()`：这几行是给用户**看画面时对照**的，
          ROS logger 会加 `[INFO][时间][节点]:` 前缀、还混在退出日志里，
          很容易被忽略。裸打印能保证它出现在最显眼的位置。
        """
        if not self.gz_replay_job or self.gz is None or not self.gz.available:
            return
        try:
            script = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                                  "gz_scene_sync.py")
            # ★★ 子进程输出**必须留痕**（2026-10-17 实飞踩到）
            #   原先把 stdout/stderr 丢进 DEVNULL ⇒ 子进程崩了也看不见，
            #   现象就是"提示说回放已启动，但 Gazebo 里什么都没动"，无从排查。
            #   ⇒ 改写到日志文件，并在 1.5 s 后**检查子进程是否还活着**：
            #     · 挂了 ⇒ 直接把日志尾巴打出来（错误立刻可见）
            #     · 活着 ⇒ 提示日志路径，供事后排查
            logpath = self.gz_replay_job.replace(".json", ".log")
            # ⚠ 开完就要**立刻关掉父进程这一侧的句柄**：Popen 会把 fd dup 给子进程，
            #   子进程照样能写；父进程不关会泄漏（实测：后续 os.remove 报 WinError 32）。
            with open(logpath, "w", encoding="utf-8") as logf:
                proc = subprocess.Popen([sys.executable, "-u", script,
                                         "--replay-json", self.gz_replay_job,
                                         "--world", self.args.gz_world],
                                        stdout=logf, stderr=subprocess.STDOUT,
                                        start_new_session=True)
            p = print
            p("")
            p("=" * 70)
            p("  ▶▶ 慢速回放已启动（独立进程，Gazebo 里现在应该开始动了）")
            p("=" * 70)
            p("     %.1f s 放完 %.3f s 的交会（%d 帧），相遇帧停顿 %.1f s"
              % (self.args.replay_seconds, self.gz_flight_secs or 0.0,
                 len(self.gz_trail), self.args.replay_hold))
            p("     先停 %.1f s 展示完整轨迹，然后开始移动" % self.args.replay_lead_in)
            p("")
            p("     颜色对照：")
            p("       橙红 轨迹    = 弹丸飞过的路径（连续线；marker 不可用时为虚线珠子）")
            p("       绿   轨迹    = 无人机飞过的路径（同上）")
            p("       橙红 小球    = 弹丸（沿轨迹慢速移动）")
            p("       绿   小球    = 无人机重放位置")
            p("       黄   大球    = 命中点（保留 60 s，便于截图）")
            p("")
            p("     ℹ 若 `/marker` 服务不可用，轨迹会自动降级成「一串静态小球」，")
            p("       视觉上是虚线 —— 这不影响回放本身，两个小球照样会动。")
            p("")
            p("     日志：%s" % logpath)
            p("     想再慢/停更久：bash run_pilot_compare.sh "
              "--replay-seconds=15 --replay-hold=4")
            p("     错过/想重看：python3 gz_scene_sync.py --replay-json %s"
              % self.gz_replay_job)
            p("=" * 70)
            p("")
            sys.stdout.flush()

            # ★ 1.5 s 后查活：子进程若已退出，说明回放根本没跑起来 —— 立刻报出来。
            #   用 time.sleep 会阻塞控制回路（此时已是 DONE 之后，无控制任务，
            #   但仍尽量短，并且只在收尾阶段执行）。
            time.sleep(1.5)
            # ⚠ 防御：测试桩 / 非标准 Popen 可能没有 poll()
            rc = proc.poll() if hasattr(proc, "poll") else None
            if rc is not None:
                p("  ⚠⚠ 回放子进程**已退出**（exit=%s）—— 回放没跑起来。"
                  % rc)
                p("     下面是它的输出（这就是原因）：")
                p("-" * 70)
                try:
                    with open(logpath, encoding="utf-8", errors="replace") as f:
                        for line in f.read().splitlines()[-25:]:
                            p("     " + line)
                except Exception as e:                     # noqa: BLE001
                    p(f"     （读日志失败：{e}）")
                p("-" * 70)
                p("     常见原因：")
                p("       ① `/marker` 服务不可用 ⇒ 轨迹线/命中点画不出来")
                p("          （它由 Gazebo **GUI** 的 MarkerManager 插件提供，"
                  "headless 就没有）")
                p("       ② 世界名不对（--gz-world）或仿真没在跑")
                p("     自检一下：python3 gz_scene_sync.py --check")
                p("")
            else:
                p("  ✓ 回放子进程存活（pid=%s），画面应在 1 s 内开始动" % proc.pid)
                p("    若 Gazebo 里仍没东西：多半是 `/marker` 不通 ⇒ 跑 "
                  "`python3 gz_scene_sync.py --check` 确认")
                p("")
            sys.stdout.flush()
        except Exception as e:             # noqa: BLE001
            self.warn(f"⚠ 回放启动失败（{e}）—— 忽略（不影响结果）")

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
        # ★★ 起飞安全门限（第三道闸）：巡航段的 a_boost 是 6 m/s²（倾角 ~31°），
        #   贴地时给出去必打桨。理论上进 CRUISE 前 SETTLE 已保证够高，
        #   但如果中途掉高（lost control / 遥测跳变），这里要能兜住。
        if alt < TAKEOFF_MIN_AGL:
            if not self.low_xy_warned:
                self.low_xy_warned = True
                self.warn(f"⚠ CRUISE 段离地仅 {alt:.2f} m（< 门限 "
                          f"{TAKEOFF_MIN_AGL:.1f} m）⇒ **撤掉水平加速**，只拉高度")
            a[0] = 0.0
            a[1] = 0.0
            a[2] = max(a[2], ALT_ADJ_A_MAX)     # 优先爬升
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
        # 记下"制导开始时已发过多少次弹丸位置" ⇒ 之后算**实际**有效帧率
        if self.gz is not None and self.gz.available:
            self.gz_setpose_t0 = int(getattr(self.gz, "stats", {}).get("set_pose", 0))
        law_name = ["PPN", "TPN", "APN"][int(self.args.law)] \
            if 0 <= int(self.args.law) <= 2 else "?"
        self.log(f"[6/6] ── 目标发射！进入 GUIDED（{law_name} 闭环）")
        self.log(f"       发射时刻 PX4 t={self.t_launch_us} µs；"
                 f"目标 {self.args.target_speed:.1f} m/s 仰角 {self.elev_deg:+.2f}° "
                 f"方位 {self.azim_deg:+.2f}° "
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
        # 同时记无人机轨迹（回放要把两条线一起画出来，才看得出"怎么撞上的"）
        if self.args.gz_trail and p_enu is not None:
            self.gz_uav_trail.append(tuple(p_enu))

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
        if self.plate is not None:
            # ★ 撞板判据（板形，2026-10-18）：**穿越板面** > **贴板容差** > 未命中。
            #   用上一拍位置做线段穿越检测 ⇒ 高速穿板也不会"两帧跨过去漏判"。
            vd = self.plate.verdict(self._plate_prev, p_enu,
                                    tol=self.plate_hit_tol,
                                    body_radius=self.plate_body_radius)
            self._plate_prev = tuple(p_enu) if p_enu is not None else None
            if self.plate_min_dist is None or vd["dist"] < self.plate_min_dist:
                self.plate_min_dist = float(vd["dist"])
                self.plate_min_t = self.t_rel
                self.plate_verdict_rec = vd
            if vd["kind"] in ("THROUGH", "HIT") and not self.plate_hit:
                self.plate_hit = True
                self.plate_verdict_rec = vd
                self.log("")
                self.log("★" * 36)
                if vd["kind"] == "THROUGH":
                    self.log(f"★ 撞板成功！（THROUGH 真穿板）距板心 R_true={R_true:.3f} m  "
                             f"板面残差 Δu={vd['du']:+.3f} Δv={vd['dv']:+.3f} m  "
                             f"@ t_rel={self.t_rel:.3f}s")
                else:
                    self.log(f"★ 撞板命中（HIT 容差命中，未穿板）距板心 R_true={R_true:.3f} m  "
                             f"板面残差 Δu={vd['du']:+.3f} Δv={vd['dv']:+.3f} m  "
                             f"@ t_rel={self.t_rel:.3f}s")
                    # ★★ 2026-10-25：HIT ≠ 真穿板。实测（板 -20,-6,4 / yaw45 / pitch30）
                    #   真穿越发生在"最后一个 GUIDED 拍 → 第一个 BRAKE 拍"之间（s=0.46），
                    #   原逻辑在 HIT 就 finish ⇒ 永远看不到 THROUGH。
                    #   ⇒ 再开一个短判定尾窗，只判定、不改飞行指令。
                    self.plate_tail_until = now + PLATE_TAIL_SEC
                    self.plate_tail_prev = tuple(p_enu) if p_enu is not None else None
                    self.plate_tail_confirmed = False
                    self.log(f"   ↳ 进入 {PLATE_TAIL_SEC:.1f} s 判定尾窗：继续检测是否真穿板…")
                self.log("★" * 36)
                self.finish("撞板")
                return
        elif R_true < self.hit_radius and not self.hit:
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
        # ★ 交会段结束 ⇒ 结算"弹丸实际被更新了几帧"（回答"为什么看不见球"）
        if self.gz is not None and self.gz.available:
            self.gz_setpose_t1 = int(getattr(self.gz, "stats", {}).get("set_pose", 0))
            self.gz_flight_secs = self.t_rel
        self.gz_finish(self.hit or self.plate_hit)   # 命中画爆点 / 画轨迹（--gz-viz 才动作）
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
        self.land_auto_sent = False        # 降落两阶段：重置"是否已交给 AUTO_LAND"
        self.land_stuck_t = None
        self.land_last_alt = None
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
            # ★ 锁定刹车期间的高度目标 = 进入时的高度（但不低于 MIN_ALTITUDE，
            #   否则失控贴地时会被"保持"在地面高度上）
            p_now, _ = self.state_enu()
            alt_now = p_now[2] if p_now is not None else self.target_height
            self.brake_alt_tgt = max(alt_now, MIN_ALTITUDE)
            self.log(f"     刹车期间高度目标锁定为 {self.brake_alt_tgt:.2f} m"
                     f"（进入时 {alt_now:.2f} m，下限 {MIN_ALTITUDE:.1f} m）")
            self.state = "BRAKE"
            self.brake_t0 = now
            self.t_state = now
            return

        self.log("→ 转入自动降落：LAND（AUTO_LAND 下降 + cmd_vel_ned）→ 落地后 DISARM 停桨。")
        self.landing_phase = True
        self.alt_hold = False
        # ★ 2026-10-17 实跑发现：报告里的"着陆漂移"是从**收尾起点**（GUIDED 结束）
        #   一路算到停桨点的，而 BRAKE 段现在**包含了"归位"**（有意飞回起点 7 m）
        #   ⇒ 输出成了"着陆漂移 6.92 m / 归位 ✓"，两个数字互相打脸。
        #   ⇒ 单独记"进入 LAND 时的位置"：真正的着陆漂移 = LAND 起点 → 停桨点。
        p_land, _ = self.state_enu()
        self.land_pos = p_land
        self.state = "LAND"
        self.t_state = now

    def plate_tail_step(self, now):
        """★ 撞板判定尾窗（2026-10-25）：HIT 之后继续检测是否**真穿板**。

        动机（实测 + 目视确认）：撞板模式下一旦 `verdict` 返回 HIT 就 `finish("撞板")`，
        而真穿越可能发生在"最后一个 GUIDED 拍 → 第一个 BRAKE 拍"之间
        （板 -20,-6,4 / yaw45 / pitch30 那组：s=0.46，穿点 Δu/Δv 仅 7 mm / 1 mm）。
        原逻辑因此会把 THROUGH 误报成 HIT。

        本方法只在收尾段做**判定**，不改变任何飞行指令；尾窗内检测到穿越就把
        `plate_verdict_rec` 升级为 THROUGH，回传报告随之改成「真穿板」。
        """
        if self.plate is None or self.plate_tail_until is None:
            return
        if now > self.plate_tail_until:
            self.plate_tail_until = None
            if not self.plate_tail_confirmed:
                self.log("   ↳ 判定尾窗结束：未检测到穿越板面 ⇒ 维持 HIT（容差命中）")
            return
        p_enu, _ = self.state_enu()
        if p_enu is None or self.plate_tail_prev is None:
            return
        vd = self.plate.verdict(self.plate_tail_prev, p_enu,
                                tol=self.plate_hit_tol,
                                body_radius=self.plate_body_radius)
        self.plate_tail_prev = tuple(p_enu)
        if vd["kind"] == "THROUGH":
            self.plate_verdict_rec = vd
            self.plate_tail_confirmed = True
            self.plate_tail_until = None
            self.plate_hit = True
            self.log("")
            self.log("★" * 36)
            self.log(f"★★ 判定尾窗确认：THROUGH（真穿板）"
                     f" Δu={vd['du']:+.3f} Δv={vd['dv']:+.3f} m"
                     f"  @ t_rel={self.t_rel:.3f}s")
            self.log("★" * 36)
        elif self.plate_min_dist is None or vd["dist"] < self.plate_min_dist:
            self.plate_min_dist = float(vd["dist"])
            self.plate_min_t = self.t_rel

    def plate_kind_text(self):
        """撞板判定的**可读文字**（★ 2026-10-19：区分"真穿板"与"容差命中"）。

        为什么必须区分：`HIT` 是"机心到板盒 ≤ tol + 机身半径"，可能机身边缘还差十几厘米
        （实测一例：距板 0.318 m、判 HIT，但边缘离板还有 0.118 m）——那是**近失**，
        不是真撞上。`THROUGH` 才是"真的穿过板面矩形"。
        """
        kind = (self.plate_verdict_rec or {}).get("kind", "MISS")
        if kind == "HIT" and self.plate_tail_until is not None:
            return "HIT（容差命中，未穿板；尾窗确认中…）"
        return {"THROUGH": "THROUGH（**真穿板**）",
                "HIT": "HIT（容差命中，未穿板）",
                "MISS": "MISS"}.get(kind, kind)

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
            self.log(f"  命中         : {'是 ✓' if (self.hit or self.plate_hit) else '否 ✗'}"
                     + ("（撞板判据）" if self.plate is not None else ""))
            if self.plate is not None:
                _md = ("—" if self.plate_min_dist is None
                       else "%.3f m @ t_rel=%.3fs" % (self.plate_min_dist,
                                                       self.plate_min_t or 0.0))
                self.log(f"  撞板判据     : {self.plate_kind_text()}"
                         f"  距板最小 {_md}  容差 {self.plate_hit_tol} m + 机身 "
                         f"{self.plate_body_radius} m")
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
        if self.self_takeoff:
            self.log(f"  起飞(自写)   : 爬升 {self.climb_rate:.2f} m/s，耗时 "
                     + ("—" if self.self_takeoff_secs is None
                        else f"{self.self_takeoff_secs:.2f} s"))
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
            # ★ 2026-10-18：`t_go = R / max(Vc, 1e-6)`，一旦弹丸已飞过（Vc ≤ 0）
            #   就会爆成 1e6 量级（实测 `t_go=3030320.77`）—— 那是**退化值**，
            #   不是数据。它和"非制导段留空"是同一类噪声，按同样口径**留空**。
            _tgo = c[7]
            if c[10] <= 0.0 or not (0.0 <= _tgo < 1.0e4):
                _tgo = ""
            else:
                _tgo = round(_tgo, 4)
            row += [round(c[1], 5), round(c[2], 5), round(c[3], 5),
                    round(c[4], 5), int(c[5]), int(c[8]), _tgo,
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
          "仰角=%+.2f°%s  方位=%+.2f°%s  t_flight=%.2fs  hits_ground=%s"
          % (self.args.target_type, self.target_p0[0], self.target_p0[1],
             self.target_p0[2], self.args.target_speed, self.elev_deg,
             "(auto)" if self.aim_elev_auto else "",
             self.azim_deg, "(auto)" if self.aim_azim_auto else "",
             self.t_flight, self.t_hits_ground))
        # ★ 自动瞄准的结果 / 撞板目标（写进回传报告，便于事后核对）
        _ar = self.aim_res or {}
        if self.plate is not None:
            p("[撞板] 目标=固定装甲板「%s」 中心 ENU=(%.2f,%.2f,%.2f)  "
              "宽×高×厚=%.2f×%.2f×%.3f m  yaw=%+.1f°  pitch=%+.1f°"
              % (self.plate.name, self.plate.center[0], self.plate.center[1],
                 self.plate.center[2], self.plate.width, self.plate.height,
                 self.plate.thickness, self.plate.yaw_deg,
                 self.plate.pitch_deg))
            p("[撞板] 判据=穿越板面矩形 / 距板 ≤ %.2f m + 机身 %.2f m"
              "（★ 我们的假设：规则表 5-1 对『空中机器人冲击』写的是『待定』）"
              % (self.plate_hit_tol, self.plate_body_radius))
            p("[撞板] 结果=%s  距板最小=%s  判定=%s"
              % ("命中 ✓" if self.plate_hit else "未命中 ✗",
                 "—" if self.plate_min_dist is None
                 else "%.3f m @ t_rel=%.3fs" % (self.plate_min_dist,
                                                self.plate_min_t or 0.0),
                 self.plate_kind_text()))
        else:
            p("[瞄准] 塔 @(%+.2f,%+.2f) r=%.2f h=%.2f 安全垫=%.2f  水平距离=%.2f m"
              % (self.tower.center[0], self.tower.center[1], self.tower.radius,
                 self.tower.height, self.tower.safety_margin,
                 math.hypot(self.tower.center[0] - self.target_p0[0],
                            self.tower.center[1] - self.target_p0[1])))
            p("[瞄准] 弹道离塔最近 %.3f m @ t_rel=%.3fs（穿塔采样 %d 点）⇒ %s"
              % (_ar.get("dist", float("nan")), _ar.get("t_rel", 0.0),
                 int(_ar.get("inside", 0)),
                 "✅ **会打中塔**" if _ar.get("ok") else
                 ("⚠ 擦边（安全垫内，算威胁）"
                  if _ar.get("dist", 9e9) < self.tower.safety_margin
                  else "❌ **打不到塔**（这发拦截没有意义）")))
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
        # ★ 可见性诊断（回答"为什么看不见球"）—— 2026-10-16 新增
        if self.gz is not None and self.gz.available:
            p("[可见性] 弹丸球半径 = %.2f m（直径 %.2f m）；"
              "真实弹径只有 0.06 m ⇒ 默认已**放大**便于观察"
              % (self.args.gz_proj_radius, self.args.gz_proj_radius * 2))
            p("[可见性] 轨迹线：%s%s"
              % ("ON" if self.args.gz_trail else "OFF（加 --gz-trail 才记轨迹）",
                 "" if not self.args.gz_replay else
                 "   慢速回放：ON %.0fs / 相遇停顿 %.1fs"
                 % (self.args.replay_seconds, self.args.replay_hold)))
            if self.plate is not None:
                # ★ 撞板模式：目标不是弹丸 ⇒ 我们**故意不推弹丸球**（球会嵌在板里）。
                #   原来这里照样统计"弹丸位置发出次数"⇒ 输出"有效帧率 0.0 Hz ⚠"，
                #   纯属误导（2026-10-19 修正）。
                p("[可见性] 撞板模式：目标=静止装甲板，弹丸球**已隐藏**"
                  "（不推位置、也不统计弹丸帧率）")
            elif (self.gz_setpose_t0 is not None and self.gz_setpose_t1 is not None
                    and self.gz_flight_secs):
                n_sent = max(self.gz_setpose_t1 - self.gz_setpose_t0, 0)
                hz_eff = n_sent / self.gz_flight_secs
                p("[可见性] ★ 交会段 %.3f s 内，弹丸位置**实际只发出 %d 次**"
                  " ⇒ 有效帧率 ≈ **%.1f Hz**（请求 %d 次 / 名义 %g Hz）"
                  % (self.gz_flight_secs, n_sent, hz_eff,
                     self.gz_proj_pos_req, self.args.gz_hz))
                if hz_eff < 10.0:
                    p("[可见性] ⚠ 有效帧率偏低 ⇒ 球看起来会「跳」而不是「飞」。"
                      "原因：每次 set_pose 是**阻塞**服务调用（可能几十~几百 ms），"
                      "把帧率拖低了。")
        nominal_ms = 1000.0 / self.rate
        gap_ms = self.loop_gap_max * 1000.0
        if self.loop_gap_max_at is None:
            p("[健康] 控制周期: 名义 %.1f ms；未采到相邻 tick 间隔（运行太短）"
              % nominal_ms)
        else:
            # ★ 判据只看**飞行段**（offboard 义务期）—— 地面段的阻塞无害，
            #   不能拿它给整轮判"有问题"（2026-10-17 实测就被这个误报带偏过）。
            tag = ("✅ 正常" if self.loop_gap_flight_max < LOOP_GAP_WARN
                   else "⚠ 飞行段有空洞")
            p("[健康] 控制周期: 名义 %.1f ms；**实测最大间隔 %.1f ms**"
              " @ t_wall=%.3fs (state=%s)  %s"
              % (nominal_ms, gap_ms, self.loop_gap_max_at,
                 self.loop_gap_max_state, tag))
            if self.loop_gap_flight_max > 0.0:
                p("[健康] 飞行段(offboard 义务期)最大间隔 %.0f ms（累计 %.0f ms）"
                  " ← **这个才影响飞行**"
                  % (self.loop_gap_flight_max * 1000.0,
                     self.loop_gap_flight_total * 1000.0))
            if self.loop_gaps:
                n_f = sum(1 for it in self.loop_gaps if it[4])
                n_g = len(self.loop_gaps) - n_f
                tot = sum(g for it in self.loop_gaps for g in (it[1],))
                p("[健康] 空洞(≥%d ms) 共 %d 次（飞行段 %d / 地面段 %d）、累计 %.0f ms："
                  % (int(LOOP_GAP_WARN * 1000), len(self.loop_gaps), n_f, n_g,
                     tot * 1000))
                for item in self.loop_gaps[:8]:
                    tw, gp, st, sdt, duty = (list(item) + [None])[:5]
                    sdt_txt = ("" if sdt is None
                               else "  仿真推进 %.3fs (RTF %.2f)" % (sdt, sdt / gp))
                    p("         t_wall=%.3f  %5.0f ms  state=%-16s%s  %s"
                      % (tw, gp * 1000, st, sdt_txt,
                         "⚠ 飞行段" if duty else "地面段(无害)"))
                if len(self.loop_gaps) > 8:
                    p("         …（另有 %d 次，见完整 CSV）" % (len(self.loop_gaps) - 8))
        # ★ 仿真实时率（RTF）：解释"位置跳变"的关键 —— 墙钟 0.1 s 里仿真可能推进 1.1 s
        if self.rtf_samples:
            rs = sorted(self.rtf_samples)
            n = len(rs)
            p("[实时率] 仿真Δt/墙钟Δt：中位 %.2f  最小 %.2f  最大 %.2f  （%d 个采样%s）"
              % (rs[n // 2], rs[0], rs[-1], n,
                 "" if not self.rtf_stale else
                 "，已剔除 %d 个「话题陈旧」样本" % self.rtf_stale))
            if rs[-1] > 2.0:
                p("[实时率] ⚠ 最大 %.2f ⇒ 某一拍墙钟只走了很短时间但**仿真推进很多**，"
                  "此时看到的位置跳变是**真的**（不是遥测坏了）" % rs[-1])
        # ★ 起飞段低空姿态（桨叶打地的直接征兆）
        if self.takeoff_tilt_max > 0.0:
            p("[起飞] 低空(<%.1fm) 最大姿态角 = %.1f°%s"
              % (TAKEOFF_MIN_AGL, self.takeoff_tilt_max,
                 "   ✅ 正常（<%.0f°）" % TAKEOFF_MAX_TILT
                 if self.takeoff_tilt_max < TAKEOFF_MAX_TILT
                 else "   ⚠ 偏大（≥%.0f°）⇒ 有桨叶打地风险" % TAKEOFF_MAX_TILT))
        if self.pad_checked:
            p("[自检] 已通过（%s）"
              % ("地面启动，在原点附近" if self.on_ground_start
                 else "空中接管（不是从地面起飞）"))
        if self.refused:
            p("[自检] ✗ **拒绝启动**：%s" % self.refuse_reason)
        # ★ 实测落点 / 漂移（不是估算）
        dr = self.landing_drift()
        if dr is None:
            p("[落点] (未采到收尾起点或停桨点)")
        else:
            d, ddx, ddy = dr
            p("[落点] 收尾起点 ENU=(%+.2f,%+.2f) → 停桨点 ENU=(%+.2f,%+.2f)"
              % (self.ending_pos[0], self.ending_pos[1],
                 self.stop_pos[0], self.stop_pos[1]))
            sp = self.return_span()
            if sp is not None and sp > 0.05:
                p("[落点] 刹车+归位段位移 = %.2f m"
                  "（收尾起点 → 进入 LAND；含「飞回起点」的有意机动）" % sp)
            p("[落点] **实测**着陆漂移 = %.2f m（Δx=%+.2f Δy=%+.2f）"
              "   ← 进入 LAND 之后才发生的漂移"
              % (d, ddx, ddy))
            p("[落点] 全程最大水平位移 = %.1f m" % self.max_h_disp)
        if self.min_alt is not None:
            p("[落点] 飞行段最低离地 %.2f m @ t_wall=%.3fs (state=%s)%s"
              % (self.min_alt, self.min_alt_at or 0.0, self.min_alt_state,
                 "   ⚠ 制导/刹车段曾贴地 ⇒ 疑失控"
                 if self.hard_ground_warned else ""))
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
            p("最近距离 = %.3f m @ t_rel=%.3f s   (阈值 %.2f m)  命中 = %s%s"
              % (self.min_R, self.t_min_R, self.hit_radius,
                 "是" if (self.hit or self.plate_hit) else "否",
                 ("（撞板判据）" if self.plate is not None else "")))
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
        if self.args.return_to_start:
            p("归位 = %s（目标 ENU=(%+.2f,%+.2f)，容差 %.2f m）"
              % ("✓ 已回到起点 ⇒ 下一轮可**不重启仿真**" if self.return_ok
                 else "✗ 未到位/超时 ⇒ **下一轮请重启仿真**",
                 self.return_xy[0], self.return_xy[1], self.args.return_tol))
        else:
            p("归位 = 已关闭（--no-return）⇒ 落点在拦截点，下一轮需重启仿真")
        if self.min_alt is not None:
            p("飞行段最低离地 = %.2f m @ state=%s" % (self.min_alt, self.min_alt_state))
        p("控制回路最大间隔 = %.0f ms  (空洞 %d 次)%s"
          % (self.loop_gap_max * 1000.0, len(self.loop_gaps),
             "" if not self.rtf_samples else
             "   RTF max %.2f" % max(self.rtf_samples)))
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
    ap.add_argument("--target-elev", type=parse_angle_or_auto, default=AIM_AUTO,
                    help="仰角 (deg)；`auto`（默认）= 打靶法求解，使弹道**穿入塔体**。"
                         "⚠ 弹道受重力+阻力，距离/速度一变，手填的固定角就不再配平")
    ap.add_argument("--target-azim", type=parse_angle_or_auto, default=AIM_AUTO,
                    help="方位角 (deg)；`auto`（默认）= 解析解，指向塔轴心。"
                         "手填时务必配合 `--tower-xy`，否则弹丸会平行错开发射点")
    # ★ 被守护目标（防御塔）—— 也是自动瞄准的靶心，与可视化共用一套坐标
    ap.add_argument("--tower-xy", default="0,0", metavar="X,Y",
                    help="被守护目标（防御塔）的水平位置 ENU。"
                         "自动瞄准以它为靶心；Gazebo 里的塔也放这里")
    ap.add_argument("--tower-radius", type=float, default=1.2, help="塔身半径 (m)")
    ap.add_argument("--tower-height", type=float, default=3.0, help="塔高 (m)")
    ap.add_argument("--tower-margin", type=float, default=1.0,
                    help="安全垫 (m)：弹道离塔体小于它就判『构成威胁』")
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
    # ★ 固定装甲板（撞板任务，2026-10-18）
    #   默认**关** ⇒ 老的"拦弹丸"流程一行不改（零行为变化）。
    ap.add_argument("--armor-plate", action="store_true",
                    help="★ 目标改成**固定装甲板**（撞板）：目标=板心（静止点），"
                         "命中判据=穿越板面 / 贴近板面，不再用球形 R_true。默认关")
    ap.add_argument("--plate-center", type=parse_vec3, default=(-14.0, 0.0, 5.0),
                    metavar="X,Y,Z",
                    help="装甲板**几何中心** ENU (m)。⚠ 默认 z=5.0：安全围栏是 "
                         f"MIN_ALTITUDE={3.0:.0f} m，板放太低会被围栏 abort")
    ap.add_argument("--plate-size", default="0.60,0.60,0.05", metavar="W,H,T",
                    help="装甲板 宽,高,厚 (m)。⚠ 默认值是**我们的假设**："
                         "规则 V1.0.0 表 5-1 对『空中机器人冲击』的判据写的是『待定』")
    ap.add_argument("--plate-yaw", type=float, default=0.0,
                    help="板面**法向**方位角 (deg)：0=法向 +x（正对来向），90=法向 +y")
    ap.add_argument("--plate-pitch", type=float, default=0.0,
                    help="板面**俯仰角** (deg)：正=法向抬高（板面朝上仰），负=低头。"
                         "⚠ 只倾斜板面，不改变宽度/高度尺寸；几何判定与 Gazebo 实体同步倾斜")
    ap.add_argument("--plate-hit-tol", type=float, default=0.15,
                    help="撞板距离容差 (m)：机心到板盒距离 ≤ tol+机身半径 判 HIT")
    ap.add_argument("--plate-body-radius", type=float, default=0.20,
                    help="机体等效半径 (m)：把板面矩形外扩，机身边缘擦到也算撞上")
    # 预加速
    ap.add_argument("--a-boost", type=float, default=6.0)
    ap.add_argument("--v-boost-min", type=float, default=5.0)
    ap.add_argument("--boost-dir", type=parse_vec3_or_auto, default=None,
                    help="巡航迎击方向（默认 `auto` = 从原点指向**弹丸发射点**）。"
                         "发射点在 x 轴上时结果就是 (-1,0,0)，与旧默认完全一致；"
                         "离轴发射（y≠0）时必须指向发射点，否则交班横向偏差"
                         "会吃掉全部修正预算")
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
    ap.add_argument("--gz-tower-xy", default=None, metavar="X,Y",
                    help="**覆盖**Gazebo 里防御塔的显示位置（默认跟随 `--tower-xy`）。"
                         "⚠ 只是画面：弹丸瞄的是 `--tower-xy`，两者不一致会告警。"
                         "⚠ 无人机从原点起飞 ⇒ 塔在原点时起飞段会**穿过塔身**"
                         "（塔只有 visual、无碰撞，纯视觉问题）。"
                         "想看得清楚可挪开，例如 --gz-tower-xy=0,-6")
    ap.add_argument("--gz-proj-radius", type=float, default=0.25,
                    help="★ Gazebo 里弹丸球的半径 (m)。默认 0.25（直径 0.5 m）——"
                         "**远大于**真实弹径 0.06 m：真弹只有 12 cm，"
                         "在十几米外 + 0.5 s 的交会里人眼根本看不见")
    ap.add_argument("--gz-proj-preset", action="store_true", default=True,
                    help="★ 创建弹丸后**停在发射架顶面中心**（默认开），"
                         "这样一进 Gazebo 就能看见这个球，确认渲染没问题")
    ap.add_argument("--no-gz-proj-preset", dest="gz_proj_preset",
                    action="store_false",
                    help="弹丸创建后藏到视野外（旧行为：只在 GUIDED 段 0.5 s 内出现）")
    ap.add_argument("--gz-proj-pad-h", type=float, default=None,
                    help="弹丸停在发射架时**球心的高度** (m)。默认自动 = "
                         "发射架顶面(0.6) + 球半径 + 0.02，即球底刚好贴着顶面。"
                         "想让它悬高一点就指定一个大值（如 1.5）")
    # ★ 慢速回放（2026-10-17 用户明确要求**默认关闭**）
    #   代码保留（--gz-replay 可手动打开），但不再自动触发、也不再往控制台打印提示。
    #   背景：用户环境的 `/marker` 服务不可用（Gazebo GUI 的 MarkerManager 插件缺失），
    #   轨迹画不出来；改用"珠子"降级后体验仍不理想 ⇒ 用户决定先不要这个功能。
    ap.add_argument("--gz-replay", action="store_true", default=False,
                    help="★ 跑完在独立子进程里**慢速回放**一次交会（把 0.5 s 拉长到几秒）。"
                         "**默认关闭**；需要时手动加此开关")
    ap.add_argument("--no-gz-replay", dest="gz_replay", action="store_false",
                    help="不慢速回放（默认行为，保留此开关仅为兼容旧脚本）")
    ap.add_argument("--replay-seconds", type=float, default=6.0,
                    help="回放总时长（墙钟秒）")
    ap.add_argument("--replay-hz", type=float, default=25.0, help="回放帧率 (Hz)")
    ap.add_argument("--replay-hold", type=float, default=1.5,
                    help="在「相遇帧」停顿多久 (s)，便于看清撞击瞬间")
    ap.add_argument("--replay-lead-in", type=float, default=1.0,
                    help="回放前先展示完整轨迹线的停顿 (s)")
    # 飞行
    ap.add_argument("--target-height", type=float, default=10.0, help="起飞高度 (m)")
    ap.add_argument("--takeoff-tol", type=float, default=1.0)
    # ★ 2026-10-19 **自写起飞**：不再调 XTDrone2 的 TAKEOFF（它把高度写死 10 m）。
    #   流程改为 ARM → OFFBOARD → 用 `cmd_vel_ned` 以 --climb-rate 爬到 --target-height。
    ap.add_argument("--self-takeoff", action="store_true",
                    help="★ 自己起飞：ARM→OFFBOARD→cmd_vel_ned 爬升到 --target-height，"
                         "跳过 XTDrone2 TAKEOFF（写死 10 m）。建议先 --target-height=5 验证")
    ap.add_argument("--climb-rate", type=float, default=1.5,
                    help="自写起飞的爬升速度 (m/s)；越大越快，低空姿态风险也越大")
    ap.add_argument("--climb-tol", type=float, default=0.30,
                    help="自写起飞的到位容差 (m)：到 target−tol 就交 SETTLE 收敛")
    ap.add_argument("--no-precheck", action="store_true",
                    help="★ 跳过起飞前自检（默认**会检查**：无人机是否在原点、"
                         "是否已解锁/已在 OFFBOARD、姿态是否水平）。"
                         "在错误位置起飞快会导致低空 50~70° 姿态、桨叶打地")
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
    # ★ 收尾归位（用户 2026-10-17 提出：飞回起点再降落 ⇒ 下一轮不必重启仿真）
    ap.add_argument("--return-to-start", action="store_true", default=True,
                    help="★ 收尾时先**飞回起飞点**再降落（默认开）。"
                         "这样落点回到起点附近，下一轮 --run 的起飞前自检能直接通过，"
                         "不用每次重启仿真")
    ap.add_argument("--no-return", dest="return_to_start", action="store_false",
                    help="关闭归位（落点就在拦截点；⚠ 下一轮必须重启仿真，"
                         "否则会在远离原点处起飞导致桨叶打地）")
    ap.add_argument("--return-tol", type=float, default=RETURN_XY_TOL,
                    help="归位到位容差 (m)")
    ap.add_argument("--return-vmax", type=float, default=RETURN_V_MAX,
                    help="归位最大水平速度 (m/s)")
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
