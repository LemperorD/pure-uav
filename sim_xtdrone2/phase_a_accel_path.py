#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Phase A —— XTDrone2 「指令回写通路」验证节点
================================================================================

对应《制导讲义/XTDrone2接入方案.md》§7 阶段 A：
    「无制导（悬停）→ 发常值 a_cmd → 无人机按预期移动」

本节点**不含制导算法**，只回答三件事：

  ① 链路通不通？   ARM → TAKEOFF → OFFBOARD → /xtdrone2/<ns>/cmd_accel_ned → PX4
  ② 方向对不对？   cmd_accel_ned 的三个分量对应哪个物理方向、符号如何
  ③ 坐标换算对不对？ PX4 的 NED 与 Gazebo 的 ENU 真值是否满足 p_ENU=(y,x,-z)

--------------------------------------------------------------------------------
实测已确认（2026-10-14 第 3 轮）
--------------------------------------------------------------------------------
  ✅ ARM / TAKEOFF / OFFBOARD 三个服务全 success=True，且发 cmd_accel_ned 后真的会动。
  ✅ 轴向：a_NED=(-0.5,0,0) → Δvx=-0.799 m/s（2 s，理论 -1.0）⇒ linear.x = NED 北，负=南。
  ✅ accel-only 是**纯前馈、无阻尼**：脉冲后 hold 段 Δvx=0.000，速度原样保持。
  ✅ 坐标换算 p_ENU=(y,x,-z) 用 odom 的**位置增量**交叉验证，三项全吻合。

--------------------------------------------------------------------------------
第 7 轮：核心目标达成 ✅ + 发现坑 8
--------------------------------------------------------------------------------
  ✅ `结论：**通路 OK** — 4/4 个脉冲段主轴响应正确，cmd_accel_ned 可用。`
     4 个脉冲段 Δv：south -0.855 / north_brake +0.929 / west -0.818 / east_brake +0.999
     （理论 ±1.0 m/s，差值来自加速度指令生效延迟）
  ✅ `[1/5] ✓ 已解锁` → `[4/5] ✓ PX4 已确认进入 OFFBOARD（nav_state=14，耗时 0.65s）`
     ⇒ 坑 5 / 坑 6 的修法都验证有效。
  ✅ 坐标自检 max|err|=0.061 m → 一致 OK。
  ⚠ 航向仍被转到正北（89.9° → 0.0°，Δ=-89.8°）—— 坑 4 未修，阶段 C 前必须打补丁。
  🔴 降落成功（t=43.70 触地，alt≈0.20 m）但 **DISARM 被拒 20 次** → 见【坑 8】。

--------------------------------------------------------------------------------
第 4 轮异常（已定位为坑 5）
--------------------------------------------------------------------------------
  现象：9 段全跑完，但 4 个脉冲段 Δv 全 ≈0.00x m/s，航向也没转（Δ=-4.3°）。
  CSV 实证：t=16.4→31.8 s 高度稳定 10.11~10.19 m、速度 <0.06 m/s、位置 ±0.07 m
  ⇒ 无人机在**用 PX4 自身位置环悬停**，根本没进 OFFBOARD。
  定位：见下【坑 5】。修法已内置：`CONFIRM_OFFBOARD` 状态 + 重发 DO_SET_MODE + nav_state 确认。

--------------------------------------------------------------------------------
⚠ 八个坑（本节点已绕过的 / 需要外部修的）
--------------------------------------------------------------------------------
【坑 1｜已绕过】回调里不能调 `rclpy.spin_until_future_complete`
  ROS2 Jazzy 的 `SingleThreadedExecutor.spin_once()` = `_enter_spin()` → 回调体同步执行
  → `_exit_spin()`，而 `_enter_spin()` 内有 `if self._is_spinning: raise RuntimeError(
  'Executor is already spinning')`。⇒ 官方 `xtd2_test/accel_test.py` 一到 OFFBOARD 就崩。
  本节点用**完全非阻塞**服务调用（call_async + 定时器轮询 future.done() + 状态机）。

【坑 2｜已绕过】起飞"到位"判据不能只认 |alt-target|<0.5
  实测 PX4 收到 NAV_TAKEOFF(param7=10) 后可能停在 ~9.4 m；且若无人机**本来就在空中**，
  TAKEOFF 不会让它再爬。判据放宽为：① |alt-target|<=tol  或
  ② alt>0.75*target 且 |vz|<0.4 m/s 连续 1.5 s。

【坑 3｜已绕过】accel-only 模式**没有垂直反馈**，a_z=0 会持续掉高
  实测进入 OFFBOARD 后 vz≈+0.5 m/s 且以 ~0.065 m/s² 加速下沉，8.3 m → 2.98 m。
  本节点默认开启**垂直高度保持辅助**（对 vehicle_local_position 做 PD，仅补 a_z 一项），
  用 `--no-alt-hold` 可关。注意：这是**测试夹具**，不是制导 —— 阶段 B 需要制导律自己
  处理垂直分量（或改用 cmd_vel_ned）。

【坑 4｜需外部修】XTDrone2 的 `cmd_accel_ned` 会把无人机转到正北
  `xtd2_communication/multirotor_communication.py` 的 `cmd_accel_ned_callback` 没设
  `cmd.yaw`（源码留着 `#TODO: How about yaw?`），ROS2 里 `float32 yaw` 默认 0.0（不是 NaN）
  ⇒ PX4 收到"航向=正北" ⇒ 实测 heading 85.5° → 0.6° 狂转。
  修法：在该回调里补 `cmd.yaw = math.nan; cmd.yawspeed = math.nan`。
  脚本见 `patch_xtdrone2_yaw.sh`。阶段 A/B 用世界系加速度，不修也能跑；阶段 C（相机）必须修。

【坑 5｜本节点已修】OFFBOARD 模式切换是"一次性、不排队"的 → 不确认就会静默失败
  PX4 侧证据链：
    · `HealthAndArmingChecks/checks/offboardCheck.cpp:49`  ⇒ `offboard_control_signal_lost`
      只有在收到**近期**（< COM_OF_LOSS_T，默认 1 s）且**至少一个控制位为 true** 的
      `OffboardControlMode` 时才为 false；
    · `ModeUtil/mode_requirements.cpp:132` ⇒ OFFBOARD 带 `mode_req_offboard_signal` 要求；
    · `failsafe/framework.cpp:677` ⇒ `modeCanRun()` 因此要求 `!offboard_control_signal_lost`；
    · `UserModeIntention.cpp:59-85` ⇒ 已解锁时若 `canRun()` 为假，`change()` **直接返回 false，
      不排队、不重试**，用户意图停留在旧模式。
  XTDrone2 的时序又恰好踩中：`cmd_callback("OFFBOARD")` 在**同一瞬间**既把
  `OFFBOARD_STATE` 翻成 ENABLED、又发出唯一一次 `DO_SET_MODE`，而此时 `self.cmd` 仍为
  `None`，`timer_callback` 因 `if not self.cmd: return` **还没发过任何 `OffboardControlMode`**。
  ⇒ PX4 处理模式切换时信号为"丢失" ⇒ **永久拒绝** ⇒ 无人机继续用 PX4 自身位置环悬停，
     我们的 setpoint 全被忽略（实测：4 个脉冲段 Δv≈0.00x m/s，高度 10.11~10.19 m 纹丝不动）。
  修法（本节点）：新增 `CONFIRM_OFFBOARD` 状态 —— 发完 `OFFBOARD` 服务后**持续流 setpoint**
  把信号养起来，**每 0.5 s 直接向 `/<ns>/fmu/in/vehicle_command` 重发 `DO_SET_MODE(OFFBOARD)`**，
  并**订阅 `vehicle_status` 直到 `nav_state==14`** 才进入测试段；超时则明确报错。
  注意：`OFFBOARD` 服务返回的 `success=True` **不代表** PX4 真进了 OFFBOARD（XTDrone2 恒返回 True）。

【坑 6｜本节点已修】`ARM` 服务成功 ≠ 真解锁 → 无人机会"卡在 AUTO_TAKEOFF 不涨高度"
  实测（2026-10-14 19:22 那次）：`nav_state` 全程 17（AUTO_TAKEOFF）、高度 0.56→0.62 m 纹丝不动、
  33 s 后被 Ctrl-C。这是典型的"**指令到了 PX4，但车没解锁**"——AUTO_TAKEOFF + DISARMED
  不会产生推力（`VEHICLE_CMD_NAV_TAKEOFF` 在 PX4 侧也只做 `change(AUTO_TAKEOFF)`，
  和 NAV_LAND 一样没有额外护栏，所以"接受"不代表"能飞"）。
  根因：和坑 5 同一类 —— 服务返回 `success=True` 只说明**通信节点收到了**，
  完全不代表 PX4 接受了（XTDrone2 无条件 `response.success = True`）。
  修法：新增 `CONFIRM_ARM` 状态 —— 发 ARM 后**以 `vehicle_status.arming_state == 2(ARMED)` 为准**，
  未解锁则每 1 s 重发，15 s 未确认就明确报错（提示去查 QGC 预解锁检查项）。
  另外 `WAIT_HEIGHT` 期间每 3 s 打印 `nav_state / arming_state`，并在
  `arming_state==DISARMED` 或 `nav_state≠17` 时主动告警；CSV 也加了这两列。
  ⇒ 以后"卡住不起飞"能一眼看出是不是没解锁。

【通用原则】**服务返回 success ≠ 生效。凡是关键状态，一律订阅 `vehicle_status` 确认。**
  已按此确认的三处：`arming_state`（解锁）、`nav_state==14`（OFFBOARD）、`arming_state==1`（停桨）。

【坑 7｜本节点已修】`SignalHandlerOptions` 的成员名跨版本不一样
  为了让 Ctrl-C 能走"降落 + 停桨"收尾，必须让 rclpy **不要**装它自己的 SIGINT
  handler。写法是 `rclpy.init(signal_handler_options=SignalHandlerOptions.???` ——
  而这个"不装"成员的名字**各版本不同**：**ROS2 Jazzy 上是 `NO`，没有 `NONE`**。
  写死 `NONE` 会：
      AttributeError: type object 'rclpy._rclpy_pybind11.SignalHandlerOptions'
      has no attribute 'NONE'
  ⇒ **节点一启动就崩，连解锁都不会开始**（2026-10-14 实测踩过）。
  修法：`resolve_no_signal_handler_option()` 按名字动态探测（`NO`/`NONE`/`None`/`No`…），
  并且注意 **`NO` 的值是 0**，判断必须用 `is not None`（不能用真值判断）；
  解析不到就退回 `rclpy.init()` + `uninstall_signal_handlers()`。

【坑 8｜本节点已修】落得了地，却停不了桨 —— XTDrone2 的 NaN 速度 setpoint 把落地检测打死
  实测（2026-10-14 第 7 轮）：无人机**确实落在地上**（t=43.70 vz 从 +0.66 瞬间变 -0.03、
  高度稳在 0.198~0.229 m 达 20 s），但 PX4 的 land detector 死活不认，
  于是 `DISARM` 被连续拒绝 20 次（`Disarming denied: not landed`），最后带着桨退出。
  PX4 侧的硬性前置（`MulticopterLandDetector.cpp:222-240`）：
      if (_flag_control_climb_rate_enabled) {
          if (_trajectory_setpoint_sub.update(&ts)) {
              _in_descend = PX4_ISFINITE(ts.velocity[2])
                            && (ts.velocity[2] >= 1.1f * LNDMC_Z_VEL_MAX);
          }
          if (!_maybe_landed_hysteresis.get_state() && !_landed_hysteresis.get_state()) {
              ground_contact &= _in_descend;      // ← 必须"正在被命令下降"
          }
      }
  即要求 `trajectory_setpoint.velocity[2]` 是**有限值**且 ≥ 1.1 × `LNDMC_Z_VEL_MAX`
  （默认 0.25 → 阈值 0.275 m/s）。而 XTDrone2 在 accel 模式下会以 20 Hz **一直**灌
  `velocity=[NaN,NaN,NaN]` 的 `TrajectorySetpoint`（`self.cmd` 一旦被设置就永久重发），
  与 `FlightModeManager` 发的 `velocity[2]=MPC_LAND_SPEED(0.7)` 抢同一个 topic，
  把该判据打成抖动 ⇒ `ground_contact` 永远凑不满 hysteresis ⇒ `landed` 永远为假。
  修法：**降落阶段改发 `cmd_vel_ned`**（`linear.z = +LAND_DESCEND_VEL`，NED 正 = 向下），
  让两个发布者的信号一致，PX4 就能正常判定落地并自行停桨。
  兜底：若普通 `DISARM` 在 `FORCE_DISARM_AFTER`(10 s) 内仍无效，
  且我们的落地判据（alt ≤ 0.30 m 且 |vz| ≤ 0.08 m/s）**持续满足**，
  就发一次**强制停桨**（`param2=21196`，跳过落地检查）。用 `--no-force-disarm` 可禁用。
  副作用（好的）：`cmd_vel_ned_callback` 里已经写了 `cmd.yaw = math.nan`，
  所以切过来之后机头不再被强制拧向正北（坑 4）。

--------------------------------------------------------------------------------
用法
--------------------------------------------------------------------------------
  source /opt/ros/jazzy/setup.bash
  source ~/XTDrone2/install/setup.bash
  python3 phase_a_accel_path.py --namespace x500_0

--------------------------------------------------------------------------------
跑完自动降落 + 停桨（默认开启）
--------------------------------------------------------------------------------
  测试段跑完（或安全中止、或按一次 Ctrl-C）后，节点会自己收尾：

    RUN ──► LAND（发 NAV_LAND + 改发 cmd_vel_ned 下降速度）
        ──► DISARM（落地后发 ARM_DISARM=0 → 停桨）
        ──► DONE（退出）

  为什么这样做：accel-only 是**纯前馈无阻尼**，只把加速度归零的话无人机会
  **保持原速度一直漂**，非常危险。所以必须真正落下来并停桨。

  关键设计（都有 PX4 源码依据）：
  · `NAV_LAND`（command=21）在 PX4 侧只走 `_user_mode_intention.change(AUTO_LAND)`，
    **没有 offboard 信号护栏**（`Commander.cpp:1064`）⇒ 在 OFFBOARD 里也能切出去。
  · `AUTO_LAND` 是内部模式、`replaces_nav_state == NONE`，不会被 ModeManagement 替换。
  · **降落阶段必须改发 `cmd_vel_ned` 的下降速度**（见坑 8），否则 PX4 不认落地。
  · `DISARM` 在**未落地时会被 PX4 直接拒绝**（`Commander.cpp:641-650`
    "Disarming denied: not landed"，除非 param2=21196 强制）⇒ 可以放心周期重发，
    落地前不会误停桨。落地后 PX4 自己也会 2 s 自动停桨（`COM_DISARM_LAND`），
    我们主动发一条只是"确保"。
  · 与 OFFBOARD 一样，NAV_LAND / DISARM 都**直接发到 `/<ns>/fmu/in/vehicle_command`**，
    并且**以 `vehicle_status` 为准**确认（`nav_state` / `arming_state`）。

  "已落地"判据（我们自己算的，用于决定何时开始发 DISARM）：
    alt ≤ `LANDED_ALT`(0.30 m)  且  |vz| ≤ `LANDED_VZ`(0.08 m/s)  连续 `LANDED_HOLD`(1.0 s)
    —— 比 PX4 自己的判据宽松，但要求"速度也小"，能可靠识别"触地那一下的突变"。

  开关：`--no-auto-land`（不降落，保持 0 加速度悬停）、`--no-disarm`（落地后不主动停桨）、
        `--no-force-disarm`（禁用兜底的强制停桨）、
        `--land-timeout`（默认 90 s，超时放弃并提示去 QGC 手动处理）。
  退出：收尾完成后 `--linger`（默认 3 s）秒自动退出；**再按一次 Ctrl-C 可随时强制退出**。

--------------------------------------------------------------------------------
安全
--------------------------------------------------------------------------------
  * 只发小加速度脉冲（默认 0.5 m/s² × 2 s），最大水平位移约 3 m。
  * 脉冲方向默认取 **南 / 西**（NED x-、y-）。walls 世界里从原点出发，Gazebo +x/+y
    方向 4.5 m 处就是墙（box1/box2），-x/-y 净空。PX4 NED x=北、y=东。
  * 保护：离地高度 < 3 m，或水平位移 > 8 m → 立即中止并转入降落。
  * 默认跑完自动 LAND + DISARM（见上）。若降落超时，节点会明确告警并提示去 QGC 手动 LAND。
  * **强制停桨（param2=21196）是安全兜底，不是常规路径**：它只在
    "普通 DISARM 连续 10 s 无效" 且 "alt ≤ 0.30 m 且 |vz| ≤ 0.08 m/s（已停在地面）"
    时才触发；一旦离开该判据会立刻停止发送。不放心可用 `--no-force-disarm` 关掉。
"""

import argparse
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
    _HAS_ODOM = True
except ImportError:
    _HAS_ODOM = False


# ==============================================================================
# 默认参数
# ==============================================================================
DEFAULT_NS = "x500_0"
DEFAULT_TARGET_HEIGHT = 10.0

# —— 起飞"到位"判据（坑 2）——
TAKEOFF_TOL = 1.0
AIRBORNE_RATIO = 0.75
STABLE_VZ = 0.4
STABLE_SEC = 1.5
TAKEOFF_TIMEOUT = 60.0

# —— 垂直高度保持辅助（坑 3）——
# a_z_cmd = a_z_base + kp*(alt - alt_target) - kd*vz     （NED：a_z 正 = 向下）
# 闭环 ω=sqrt(kp)=1 rad/s、ζ=kd/(2*sqrt(kp))=1.0（临界阻尼）
ALT_KP = 1.0
ALT_KD = 2.0

A_MAG = 0.5
PULSE_SEC = 2.0
OFFBOARD_SETTLE = 1.0
POST_ARM_DELAY = 0.5
SRV_TIMEOUT = 3.0

# —— OFFBOARD 进入确认（坑 5）——
# PX4 的 OFFBOARD 模式切换是**一次性、不排队**的：若处理 DO_SET_MODE 的瞬间
# offboard_control_signal_lost==true，切换被永久拒绝。XTDrone2 又恰好在这个瞬间
# 才刚把 OFFBOARD_STATE 翻成 ENABLED（self.cmd 还是 None，OffboardControlMode 尚未发出）
# ⇒ 必然踩中竞态。修法：流起 setpoint 后**周期重发 DO_SET_MODE**，并订阅 vehicle_status
# 直到确认 nav_state==OFFBOARD(14) 才继续。
NAV_STATE_OFFBOARD = 14            # vehicle_status_s::NAVIGATION_STATE_OFFBOARD
NAV_STATE_AUTO_LOITER = 4          # vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER
NAV_STATE_AUTO_TAKEOFF = 17        # vehicle_status_s::NAVIGATION_STATE_AUTO_TAKEOFF
NAV_STATE_AUTO_LAND = 18           # vehicle_status_s::NAVIGATION_STATE_AUTO_LAND
# 起飞后处于这些状态都算"正常在空中"（PX4 完成 AUTO_TAKEOFF 后会自动转 AUTO_LOITER）
NAV_STATES_AIRBORNE_OK = (NAV_STATE_AUTO_TAKEOFF, NAV_STATE_AUTO_LOITER)
ARMING_STATE_DISARMED = 1          # vehicle_status_s::ARMING_STATE_DISARMED
ARMING_STATE_ARMED = 2             # vehicle_status_s::ARMING_STATE_ARMED
VEHICLE_CMD_DO_SET_MODE = 176      # MAV_CMD_DO_SET_MODE
VEHICLE_CMD_NAV_LAND = 21          # MAV_CMD_NAV_LAND
VEHICLE_CMD_COMPONENT_ARM_DISARM = 400  # MAV_CMD_COMPONENT_ARM_DISARM
PX4_CUSTOM_MAIN_MODE_OFFBOARD = 6  # PX4_CUSTOM_MAIN_MODE_OFFBOARD
ARM_RESEND_SEC = 1.0               # 重发 ARM 的周期
ARM_CONFIRM_TIMEOUT = 15.0         # 解锁确认超时（秒）
OFFBOARD_RESEND_SEC = 0.5          # 重发模式切换的周期
OFFBOARD_CONFIRM_TIMEOUT = 10.0    # 确认超时（秒）

# —— 跑完自动降落 + 停桨 ——
LAND_RESEND_SEC = 1.0              # 重发 NAV_LAND 的周期
DISARM_RETRY_SEC = 1.0             # 重发 DISARM 的周期
LAND_TIMEOUT = 90.0                # 降落总超时（秒）
DISARM_TIMEOUT = 30.0              # 停桨超时（秒）
# 降落时给 XTDrone2 的"下降速度"指令（cmd_vel_ned，NED 正 = 向下）。
# 为什么要发它：PX4 的 land detector 要求 trajectory_setpoint.velocity[2]
# 必须是**有限值**且 >= 1.1*LNDMC_Z_VEL_MAX（=0.275 m/s）才认"正在下降"。
# 而 XTDrone2 在 accel 模式下会一直灌 velocity=NaN 的 setpoint，把该判据打掉，
# 于是 PX4 永远不认落地 → DISARM 被拒（见坑 8）。
LAND_DESCEND_VEL = 0.5             # m/s（NED，正 = 下降）
# "已落地"判据（比 PX4 自己宽松，但要稳）：高度低 + 速度小 + 持续一段时间
LANDED_ALT = 0.30                  # 离地高度低于此值（m）
LANDED_VZ = 0.08                   # |vz| 低于此值（m/s）
LANDED_HOLD = 1.0                  # 上述条件需连续保持的秒数
# 普通 DISARM 无效多久后改用"强制停桨"（param2=21196）
FORCE_DISARM_AFTER = 10.0          # 秒
FORCE_DISARM_HOLD = 2.0            # 强制前"已落地判据"需持续满足的秒数（安全闸）

MIN_ALTITUDE = 3.0
MAX_H_DISP = 8.0


def build_segments(a_mag, pulse_sec):
    """(标签, a_NED=(ax,ay,az), 持续秒数)。a_NED 的 z 是**向下**。"""
    return [
        ("hold",        (0.0,     0.0,     0.0), 2.0),
        ("south",       (-a_mag,  0.0,     0.0), pulse_sec),   # NED x- → 南
        ("hold",        (0.0,     0.0,     0.0), 1.0),         # 证明无阻尼
        ("north_brake", (+a_mag,  0.0,     0.0), pulse_sec),   # 反向抵消
        ("hold",        (0.0,     0.0,     0.0), 1.0),
        ("west",        (0.0,     -a_mag,  0.0), pulse_sec),   # NED y- → 西
        ("hold",        (0.0,     0.0,     0.0), 1.0),
        ("east_brake",  (0.0,     +a_mag,  0.0), pulse_sec),   # 反向抵消
        ("hold",        (0.0,     0.0,     0.0), 2.0),
    ]


EXPECTED_AXIS = {
    "south":       (0, -1),
    "north_brake": (0, +1),
    "west":        (1, -1),
    "east_brake":  (1, +1),
}


# ==============================================================================
# rclpy 兼容小工具
# ==============================================================================
# `SignalHandlerOptions` 里表示"不装信号处理器"的那个成员，各版本名字不一样：
#   · ROS2 Jazzy 实测是 **`NO`**，**没有 `NONE`** ；
#   · 写死 `NONE` 会 `AttributeError: type object 'SignalHandlerOptions'
#     has no attribute 'NONE'` —— 节点**一启动就崩**（2026-10-14 实测踩过）。
# ⇒ 按名字动态探测，不要硬编码。
_SIG_OPTION_NAMES = ("NO", "NONE", "None", "none", "No", "NONE_OPTION")


def resolve_no_signal_handler_option():
    """返回 `SignalHandlerOptions` 中"不装 handler"的成员值；解析不到返回 None。

    ⚠ 注意该成员的值可能是 **0**（Jazzy 的 `NO` 就是 0），所以判断必须用
      `is not None`，不能用真值判断。
    返回 None 表示该版本的 rclpy 不具备此能力 → 调用方走
      `uninstall_signal_handlers()` 退路。
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


class PhaseAAccelPath(Node):

    def __init__(self, args):
        super().__init__("phase_a_accel_path")

        self.ns = args.namespace
        self.target_height = args.target_height
        self.takeoff_tol = args.takeoff_tol
        self.takeoff_timeout = args.takeoff_timeout
        self.a_mag = args.a_mag
        self.pulse_sec = args.pulse_sec
        self.segments = build_segments(self.a_mag, self.pulse_sec)
        self.linger = args.linger
        self.auto_land = not args.no_auto_land
        self.do_disarm = not args.no_disarm
        self.land_timeout = args.land_timeout
        self.force_disarm_ok = not args.no_force_disarm
        self.csv_path = args.csv
        self.rate_hz = args.rate_hz
        self.srv_timeout = args.srv_timeout

        # 垂直保持辅助
        self.alt_hold = not args.no_alt_hold
        self.alt_kp = args.alt_kp
        self.alt_kd = args.alt_kd
        self.alt_target = None

        # ---- 状态机 ----
        # ARM → CONFIRM_ARM → POST_ARM → TAKEOFF → WAIT_HEIGHT → OFFBOARD
        #     → CONFIRM_OFFBOARD → SETTLE → RUN → LAND → DISARM → DONE
        self.state = "ARM"
        self.seg_idx = -1
        self.seg_t0 = None
        self.seg_records = []
        self.cur_a = (0.0, 0.0, 0.0)
        self.aborted = False
        self.abort_reason = ""
        self.t_state = time.monotonic()
        self.t_start = time.monotonic()
        self.done_t = None
        self._exit_logged = False
        self.should_exit = False
        self.force_exit_requested = False
        self.stable_t = None
        self.last_alt_log = 0.0

        # 降落 / 停桨
        self.land_t0 = 0.0
        self.last_land_cmd = 0.0
        self.landed = False
        self.disarmed = False
        self.land_stable_t = None      # "已落地判据"连续满足的起始时刻
        self.landing_phase = False     # True 之后改发 cmd_vel_ned（不再发 accel）
        self.disarm_t0 = 0.0
        self.force_disarm_sent = False
        self.off_ground_warned = False
        # 注意：force_disarm_ok 已由命令行参数设定（见上），这里不要覆盖！

        # 非阻塞服务调用
        self.fut = None
        self.fut_cmd = None
        self.fut_t = 0.0
        self.srv_warned = False
        self.no_pos_warned = False

        # OFFBOARD 进入确认（坑 5）
        self.nav_state = None          # 来自 vehicle_status.nav_state
        self.arming_state = None       # 来自 vehicle_status.arming_state
        self.last_resend = 0.0
        self.offboard_confirmed = False
        self.last_arm_cmd = 0.0

        self.pos = None
        self.odom = None
        self.odom_ok = False
        self.p0 = None
        self.odom0 = None
        self.heading0 = None
        self.pos_end = None
        self.odom_end = None
        self.heading_end = None

        # ---- 接口 ----
        self.pub_accel = self.create_publisher(
            Twist, f"/xtdrone2/{self.ns}/cmd_accel_ned", 10)
        # 降落阶段改用速度指令：既让 PX4 的 land detector 认"正在下降"（坑 8），
        # 又顺带避开 cmd_accel_ned 的 yaw 强制正北问题（cmd_vel_ned 已设 yaw=NaN）
        self.pub_vel = self.create_publisher(
            Twist, f"/xtdrone2/{self.ns}/cmd_vel_ned", 10)
        self.cli_cmd = self.create_client(XTD2Cmd, f"/xtdrone2/{self.ns}/cmd")

        # 直接对 PX4 发 vehicle_command（重发 OFFBOARD 模式切换用；不经 XTDrone2）
        self.pub_vcmd = self.create_publisher(
            VehicleCommand, f"/{self.ns}/fmu/in/vehicle_command", 10)

        self.create_subscription(
            VehicleLocalPosition,
            f"/{self.ns}/fmu/out/vehicle_local_position",
            self.pos_cb,
            QoSProfile(depth=10, reliability=qos_profile_sensor_data.reliability))

        # 订阅 vehicle_status，用于确认 PX4 是否真的进了 OFFBOARD（坑 5）
        self.create_subscription(
            VehicleStatus,
            f"/{self.ns}/fmu/out/vehicle_status",
            self.status_cb,
            QoSProfile(depth=10, reliability=qos_profile_sensor_data.reliability))

        self.sub_odom = None
        if _HAS_ODOM and not args.no_odom:
            self.sub_odom = self.create_subscription(
                Odometry, f"/{self.ns}/odometry", self.odom_cb,
                QoSProfile(depth=10, reliability=qos_profile_sensor_data.reliability))

        # ---- CSV ----
        self.csv_f = open(self.csv_path, "w", newline="")
        self.csv_w = csv.writer(self.csv_f)
        self.csv_w.writerow([
            "t_wall", "seg", "cmd_ax", "cmd_ay", "cmd_az",
            "x", "y", "z", "vx", "vy", "vz", "heading", "ax", "ay", "az",
            "t_px4_us", "odom_x", "odom_y", "odom_z", "odom_vx", "odom_vy", "odom_vz",
            "nav_state", "arming_state",
        ])

        self.timer = self.create_timer(1.0 / self.rate_hz, self.tick)

        self.log("=" * 72)
        self.log("Phase A · 指令回写通路验证")
        self.log(f"  ns={self.ns}   脉冲 |a|={self.a_mag} m/s² × {self.pulse_sec} s   "
                 f"段数={len(self.segments)}")
        self.log(f"  起飞判据: |alt-{self.target_height}|<={self.takeoff_tol} m 或 "
                 f"(alt>{AIRBORNE_RATIO * self.target_height:.1f} m 且 |vz|<{STABLE_VZ} "
                 f"连续 {STABLE_SEC} s)  超时 {self.takeoff_timeout:.0f} s")
        self.log(f"  垂直保持辅助: {'ON' if self.alt_hold else 'OFF'}"
                 f"（kp={self.alt_kp}, kd={self.alt_kd}）")
        self.log(f"  收尾: {'跑完自动 LAND + 停桨' if self.auto_land else '不自动降落'}"
                 f"（land_timeout={self.land_timeout:.0f}s, "
                 f"disarm={'ON' if self.do_disarm else 'OFF'}, "
                 f"强制停桨兜底={'ON' if self.force_disarm_ok else 'OFF'}）"
                 f"   Ctrl-C 一次=降落收尾，两次=强制退出")
        self.log(f"  CSV={os.path.abspath(self.csv_path)}")
        self.log("=" * 72)

    # ---------------------------------------------------------------- 工具
    def log(self, m):
        self.get_logger().info(m)

    def warn(self, m):
        self.get_logger().warn(m)

    # ---------------------------------------------------- 非阻塞服务调用
    def issue(self, command):
        if not self.cli_cmd.service_is_ready():
            if not self.srv_warned:
                self.warn(f"服务 /xtdrone2/{self.ns}/cmd 未就绪，重试中…（{command}）")
                self.srv_warned = True
            return False
        self.srv_warned = False
        req = XTD2Cmd.Request()
        req.command = command
        try:
            self.fut = self.cli_cmd.call_async(req)
        except Exception as e:
            self.warn(f"call_async({command}) 失败：{e!r}")
            self.fut = None
            return False
        self.fut_cmd = command
        self.fut_t = time.monotonic()
        self.log(f"  → 发出 {command}")
        return True

    def poll_service(self, now):
        if self.fut is None:
            return False
        if not self.fut.done():
            if now - self.fut_t > self.srv_timeout:
                cmd = self.fut_cmd
                self.warn(f"  service {cmd} 超时（{self.srv_timeout:.1f}s）")
                self.fut = None
                self.fut_cmd = None
                self.on_ack(cmd, None)
            return True
        cmd = self.fut_cmd
        ok = None
        try:
            exc = self.fut.exception()
            if exc is not None:
                self.warn(f"  service {cmd} 异常：{exc!r}")
            else:
                res = self.fut.result()
                ok = bool(res.success) if res is not None else None
        except Exception as e:
            self.warn(f"  service {cmd} 取结果异常：{e!r}")
        self.fut = None
        self.fut_cmd = None
        self.log(f"  service {cmd:<9} → success={ok}")
        self.on_ack(cmd, ok)
        return True

    def on_ack(self, cmd, ok):
        if self.state == "ARM" and cmd == "ARM":
            self.state = "CONFIRM_ARM"
            self.t_state = time.monotonic()
            self.last_arm_cmd = time.monotonic()   # 1 s 后再重发
            self.log("      （ARM 服务已回；是否真解锁以 vehicle_status.arming_state 为准）")
        elif self.state == "TAKEOFF" and cmd == "TAKEOFF":
            self.state = "WAIT_HEIGHT"
            self.t_state = time.monotonic()
            self.stable_t = None
        elif self.state == "OFFBOARD" and cmd == "OFFBOARD":
            self.cur_a = (0.0, 0.0, 0.0)
            self.state = "CONFIRM_OFFBOARD"
            self.t_state = time.monotonic()
            self.last_resend = 0.0   # 立刻重发一次 DO_SET_MODE
            self.log("      （服务 success 只说明通信节点收到了；是否真进 OFFBOARD "
                     "以 vehicle_status 为准）")

    # ---------------------------------------------------------------- 回调
    def pos_cb(self, msg):
        self.pos = msg

    def odom_cb(self, msg):
        self.odom = msg
        self.odom_ok = True

    def status_cb(self, msg):
        # PX4 当前导航状态；NAV_STATE_OFFBOARD(14) 才算真的进了 OFFBOARD
        self.nav_state = int(msg.nav_state)
        self.arming_state = int(msg.arming_state)

    def now_us(self):
        """与 XTDrone2 通信节点一致的 PX4 微秒时间戳。"""
        s, ns = self.get_clock().now().seconds_nanoseconds()
        return int(s * 1e6 + ns / 1000)

    def _vcmd(self, command, **params):
        m = VehicleCommand()
        m.timestamp = self.now_us()
        m.command = command
        for k, v in params.items():
            setattr(m, k, float(v))
        m.target_system = 1
        m.target_component = 1
        m.source_system = 1
        m.source_component = 1
        m.from_external = True
        self.pub_vcmd.publish(m)

    def publish_offboard_mode(self):
        """直接对 PX4 重发 DO_SET_MODE(OFFBOARD)。

        绕过 XTDrone2 的 OFFBOARD 服务：那个服务只在调用瞬间发一次 DO_SET_MODE，
        而那一刻 offboard 信号还没建立，会被 PX4 一次性拒绝且不重试（坑 5）。
        """
        self._vcmd(VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=PX4_CUSTOM_MAIN_MODE_OFFBOARD)

    def publish_land(self):
        """直接对 PX4 发 NAV_LAND（→ AUTO_LAND 下降）。

        PX4 的 NAV_LAND 分支只做 `change(AUTO_LAND)`，没有 offboard 信号护栏
        （`Commander.cpp:1064`），所以从 OFFBOARD 也能切出去。
        """
        self._vcmd(VEHICLE_CMD_NAV_LAND, param1=0.0, param4=0.0, param5=0.0, param6=0.0, param7=0.0)

    def publish_descend_vel(self):
        """降落阶段发"下降速度"指令到 `cmd_vel_ned`（NED，正 = 向下）。

        为什么必须这么做（坑 8）——PX4 的落地检测里有一条硬性前置：

            // MulticopterLandDetector.cpp:222-240
            if (_flag_control_climb_rate_enabled) {
                if (_trajectory_setpoint_sub.update(&trajectory_setpoint)) {
                    _in_descend = PX4_ISFINITE(trajectory_setpoint.velocity[2])
                                  && (trajectory_setpoint.velocity[2] >= 1.1f * LNDMC_Z_VEL_MAX);
                }
                if (!_maybe_landed_hysteresis.get_state() && !_landed_hysteresis.get_state()) {
                    ground_contact &= _in_descend;    // ← 必须"正在被命令下降"
                }
            }

        即 `ground_contact` 要求 `trajectory_setpoint.velocity[2]` 是**有限值**且
        ≥ 1.1 × `LNDMC_Z_VEL_MAX`（默认 0.25 → 阈值 0.275 m/s）。
        而 XTDrone2 在 accel 模式下会以 20 Hz 一直灌 `velocity=[NaN,NaN,NaN]`，
        把这个判据打掉 ⇒ `ground_contact` 永远不成立 ⇒ `landed` 永远为假
        ⇒ **DISARM 被拒**（实测连拒 20 次，最后无人机停在地面却停不了桨）。

        改发速度和 FlightModeManager 的信号一致（它在降落时发 MPC_LAND_SPEED≈0.7），
        两个发布者就不再打架了。

        副作用（好的）：XTDrone2 的 `cmd_vel_ned_callback` 里已经写了 `cmd.yaw = math.nan`，
        所以切过来之后机头不再被强制拧向正北（见坑 4）。
        """
        m = Twist()
        m.linear.x = 0.0
        m.linear.y = 0.0
        m.linear.z = LAND_DESCEND_VEL      # NED：正 = 向下
        self.pub_vel.publish(m)

    def publish_disarm(self, force=False):
        """发 ARM_DISARM=0（停桨）。

        force=False：param2 = 0 —— PX4 在**未落地时会拒绝**（`Commander.cpp:641`
                     "Disarming denied: not landed"），落地后才生效 ⇒ 可安全周期重发。
        force=True ：param2 = 21196（MAVLink 的强制魔数）—— 跳过落地检查。
                     只在"我们自己的落地判据已连续满足"时才使用（见 FORCE_DISARM_HOLD）。
        """
        self._vcmd(VEHICLE_CMD_COMPONENT_ARM_DISARM,
                   param1=0.0, param2=(21196.0 if force else 0.0))

    # ---------------------------------------------------------------- 主循环
    def tick(self):
        now = time.monotonic()

        if self.poll_service(now):
            self.log_row()
            return

        if self.state == "ARM":
            if self.fut is None and now - self.last_arm_cmd >= ARM_RESEND_SEC:
                self.last_arm_cmd = now
                self.issue("ARM")

        elif self.state == "CONFIRM_ARM":
            # 坑 5 的同类问题：ARM 服务返回 success 只说明通信节点收到了，
            # PX4 可能因为预解锁检查不过而**静默拒绝**。以 arming_state 为准。
            if self.arming_state == ARMING_STATE_ARMED:
                self.log("[1/5] ✓ 已解锁（arming_state=ARMED）")
                self.state = "POST_ARM"
                self.t_state = now
            elif now - self.t_state > ARM_CONFIRM_TIMEOUT:
                self.abort(f"解锁确认超时（{ARM_CONFIRM_TIMEOUT:.0f}s）："
                           f"arming_state={self.arming_state}（期望 {ARMING_STATE_ARMED}=ARMED）"
                           f" —— PX4 拒绝了 ARM。检查 QGC 的预解锁检查项"
                           f"（EKF / 磁力计 / 遥控 / 电池）。")
            elif self.fut is None and now - self.last_arm_cmd >= ARM_RESEND_SEC:
                self.last_arm_cmd = now
                self.issue("ARM")
                self.log(f"      重发 ARM（arming_state={self.arming_state}）")

        elif self.state == "POST_ARM":
            if now - self.t_state >= POST_ARM_DELAY:
                self.state = "TAKEOFF"

        elif self.state == "TAKEOFF":
            if self.issue("TAKEOFF"):
                self.log(f"[2/5] TAKEOFF …（目标 {self.target_height} m）")

        elif self.state == "WAIT_HEIGHT":
            if self.pos is None:
                if not self.no_pos_warned and now - self.t_state > 10.0:
                    self.no_pos_warned = True
                    self.warn("还没收到 vehicle_local_position —— 检查仿真是否在跑？")
            else:
                alt = -self.pos.z
                vz = self.pos.vz
                near = abs(alt - self.target_height) <= self.takeoff_tol
                airborne = alt > AIRBORNE_RATIO * self.target_height
                if abs(vz) < STABLE_VZ and (near or airborne):
                    if self.stable_t is None:
                        self.stable_t = now
                    if now - self.stable_t >= STABLE_SEC:
                        self.log(f"[3/5] 起飞到位：alt={alt:.2f} m, vz={vz:+.2f} m/s"
                                 f"（目标 {self.target_height} m）→ 请求 OFFBOARD …")
                        self.p0 = (self.pos.x, self.pos.y, self.pos.z)
                        self.alt_target = alt
                        self.odom0 = self.odom_pose()
                        self.heading0 = self.pos.heading
                        self.state = "OFFBOARD"
                else:
                    self.stable_t = None
                if now - self.last_alt_log > 3.0:
                    self.last_alt_log = now
                    self.log(f"     爬升中… alt={alt:.2f} m  vz={vz:+.2f} m/s  "
                             f"nav_state={self.nav_state}  arming_state={self.arming_state}")
                    if self.arming_state == ARMING_STATE_DISARMED:
                        self.warn("     ⚠ 车辆处于 DISARMED —— 不可能起飞。"
                                  "检查 QGC 预解锁检查项（EKF / 磁力计 / 遥控 / 电池）。")
                    elif self.nav_state is not None \
                            and self.nav_state not in NAV_STATES_AIRBORNE_OK:
                        self.warn(f"     ⚠ nav_state={self.nav_state} 不在"
                                  f" {NAV_STATES_AIRBORNE_OK}"
                                  f"（{NAV_STATE_AUTO_TAKEOFF}=AUTO_TAKEOFF，"
                                  f"{NAV_STATE_AUTO_LOITER}=AUTO_LOITER）—— TAKEOFF 未生效？")
                if now - self.t_state > self.takeoff_timeout:
                    self.abort(f"起飞超时（{self.takeoff_timeout:.0f}s，当前 alt={alt:.2f} m,"
                               f" vz={vz:+.2f} m/s）")

        elif self.state == "OFFBOARD":
            # 先把 setpoint 流起来：一旦通信节点把 OFFBOARD_STATE 翻成 ENABLED，
            # OffboardControlMode 就会开始发，从而把 PX4 的 offboard 信号"养"活。
            self.publish_a()
            if self.fut is None:
                self.issue("OFFBOARD")

        elif self.state == "CONFIRM_OFFBOARD":
            # 持续发 setpoint 维持 offboard 信号，并周期重发 DO_SET_MODE，
            # 直到 PX4 真正切进 OFFBOARD（坑 5：不确认就会静默失败）。
            self.publish_a()
            if now - self.last_resend >= OFFBOARD_RESEND_SEC:
                self.last_resend = now
                self.publish_offboard_mode()
            if self.nav_state == NAV_STATE_OFFBOARD:
                self.offboard_confirmed = True
                self.log(f"[4/5] ✓ PX4 已确认进入 OFFBOARD（nav_state=14，"
                         f"耗时 {now - self.t_state:.2f}s）")
                self.state = "SETTLE"
                self.t_state = now
            elif now - self.t_state > OFFBOARD_CONFIRM_TIMEOUT:
                self.abort(f"OFFBOARD 确认超时（{OFFBOARD_CONFIRM_TIMEOUT:.0f}s）："
                           f"PX4 nav_state={self.nav_state}（期望 {NAV_STATE_OFFBOARD}=OFFBOARD）"
                           f" —— 模式切换被拒，检查 offboard 信号 / 解锁状态")

        elif self.state == "SETTLE":
            self.publish_a()
            if now - self.t_state >= OFFBOARD_SETTLE:
                self.log("[5/5] 稳定完毕，开始测试段")
                self.state = "RUN"
                self.next_segment()

        elif self.state == "RUN":
            if self.seg_t0 is not None and \
                    now - self.seg_t0 >= self.segments[self.seg_idx][2]:
                self.end_segment()
                if not self.next_segment():
                    self.finish()
            self.publish_a()
            self.check_safety()

        elif self.state == "LAND":
            # 降落阶段：发"下降速度"指令（不再发 accel，见坑 8），并周期重发 NAV_LAND，
            # 直到 PX4 认落地（或我们自己的判据成立）。
            self.publish_descend_vel()
            if now - self.last_land_cmd >= LAND_RESEND_SEC:
                self.last_land_cmd = now
                self.publish_land()
                if self.nav_state != NAV_STATE_AUTO_LAND:
                    self.log(f"      LAND 已发（nav_state={self.nav_state}，"
                             f"期望 {NAV_STATE_AUTO_LAND}=AUTO_LAND）")

            alt = -self.pos.z if self.pos is not None else None
            vz = self.pos.vz if self.pos is not None else None
            landed_now = (alt is not None and vz is not None
                          and alt <= LANDED_ALT and abs(vz) <= LANDED_VZ)
            if landed_now:
                if self.land_stable_t is None:
                    self.land_stable_t = now
            else:
                self.land_stable_t = None

            if self.arming_state == ARMING_STATE_DISARMED:
                # PX4 自己判定落地并停桨了（最理想的情况）
                self.landed = True
                self.disarmed = True
                self.log("✓ PX4 已自行判定落地并停桨（arming_state=DISARMED）。")
                self.state = "DONE"
                self.done_t = now
            elif landed_now and now - self.land_stable_t >= LANDED_HOLD:
                alt_txt = f"{alt:.2f}"
                self.log(f"✓ 已落地：alt={alt_txt} m, |vz|={abs(vz):.3f} m/s"
                         f"（判据：alt≤{LANDED_ALT} m 且 |vz|≤{LANDED_VZ} m/s 持续 "
                         f"{LANDED_HOLD:.0f}s）→ 停桨 DISARM")
                self.landed = True
                self.state = "DISARM"
                self.t_state = now
                self.disarm_t0 = now
                self.last_land_cmd = 0.0        # 立刻发一次 DISARM
            elif self.pos is None and self.arming_state is None and now - self.land_t0 > 5.0:
                self.warn("⚠ 收不到任何遥测（vehicle_local_position / vehicle_status），"
                          "无法确认降落 —— 直接退出。")
                self.state = "DONE"
                self.done_t = now
            elif now - self.land_t0 > self.land_timeout:
                self.warn(f"⚠ 降落超时（{self.land_timeout:.0f}s）：alt={alt}，"
                          f"arming_state={self.arming_state}，nav_state={self.nav_state}")
                self.warn("   放弃自动收尾 —— 请到 QGC 里手动 LAND；"
                          "或再按一次 Ctrl-C 强制退出。")
                self.state = "DONE"
                self.done_t = now

        elif self.state == "DISARM":
            self.publish_descend_vel()
            alt = -self.pos.z if self.pos is not None else None
            vz = self.pos.vz if self.pos is not None else None
            # 安全闸：必须仍然满足"已落地判据"，否则不再发停桨指令
            still_on_ground = (alt is not None and vz is not None
                               and alt <= LANDED_ALT and abs(vz) <= LANDED_VZ)
            if still_on_ground:
                self.off_ground_warned = False
            else:
                self.land_stable_t = None

            if self.arming_state == ARMING_STATE_DISARMED:
                self.disarmed = True
                self.log("✓ 已停桨（arming_state=DISARMED）。")
                self.state = "DONE"
                self.done_t = now
            elif now - self.disarm_t0 > DISARM_TIMEOUT:
                self.warn(f"⚠ 停桨超时（{DISARM_TIMEOUT:.0f}s）：arming_state="
                          f"{self.arming_state}。请在 QGC 里确认。")
                self.state = "DONE"
                self.done_t = now
            elif not self.do_disarm:
                if now - self.last_land_cmd >= DISARM_RETRY_SEC:
                    self.last_land_cmd = now
                    self.log(f"      等待 PX4 自动停桨（arming_state={self.arming_state}）")
            elif not still_on_ground:
                if not self.off_ground_warned:
                    self.off_ground_warned = True
                    self.warn(f"⚠ 已离开落地判据（alt={alt}, vz={vz}）"
                              f"—— 暂停发停桨指令，等回到地面再继续。")
            elif now - self.last_land_cmd >= DISARM_RETRY_SEC:
                self.last_land_cmd = now
                if (self.force_disarm_ok and not self.force_disarm_sent
                        and now - self.disarm_t0 >= FORCE_DISARM_AFTER):
                    # 普通 DISARM 一直没用：PX4 的 land detector 不肯松口（坑 8）。
                    # 我们的判据已连续满足 —— 用强制停桨兜底（param2=21196）。
                    self.force_disarm_sent = True
                    self.warn("⚠ 普通 DISARM 无效 → 改用【强制停桨】（param2=21196）。")
                    self.warn(f"   安全前提：alt={alt:.2f} m ≤ {LANDED_ALT} m，"
                              f"|vz|={abs(vz):.3f} m/s ≤ {LANDED_VZ} m/s。")
                    self.publish_disarm(force=True)
                else:
                    self.publish_disarm(force=False)
                    self.log(f"      DISARM 已发（arming_state={self.arming_state}）")

        elif self.state == "DONE":
            if self.landing_phase:
                self.publish_descend_vel()   # 维持"正在下降"，别停
            else:
                self.publish_a()
            if not self._exit_logged and now - self.done_t > self.linger:
                self._exit_logged = True
                if self.disarmed:
                    self.log("收尾完成：已落地并停桨。节点退出。")
                elif self.landed:
                    self.log("收尾完成：已落地（未确认停桨）。节点退出。")
                elif self.aborted:
                    self.log("收尾结束（中止）。节点退出。")
                else:
                    self.log("阶段 A 结束。节点退出。")
                self.should_exit = True
            return

        self.log_row()

    # ---------------------------------------------------------------- 分段
    def next_segment(self):
        self.seg_idx += 1
        if self.seg_idx >= len(self.segments):
            return False
        label, a, dur = self.segments[self.seg_idx]
        self.cur_a = a
        self.seg_t0 = time.monotonic()
        self.seg_records.append({"label": label, "a": a, "dur": dur, "v0": self.vel()})
        self.log(f"  ── 段 {self.seg_idx + 1}/{len(self.segments)}  {label:<12}"
                 f" a_NED=({a[0]:+.2f},{a[1]:+.2f},{a[2]:+.2f})  {dur:.1f}s")
        return True

    def end_segment(self):
        self.seg_records[-1]["v1"] = self.vel()

    def finish(self):
        self.cur_a = (0.0, 0.0, 0.0)
        self.snapshot_end()
        self.print_summary()
        self.start_landing()

    def abort(self, reason):
        self.aborted = True
        self.abort_reason = reason
        self.cur_a = (0.0, 0.0, 0.0)
        self.snapshot_end()
        self.warn("!" * 72)
        self.warn(f"!! 安全中止：{reason}")
        self.warn("!" * 72)
        self.print_summary()
        self.start_landing()

    # ------------------------------------------------------------ 降落 / 停桨
    def start_landing(self, note=""):
        """转入降落流程：停脉冲 → LAND（切 AUTO_LAND 下降）→ 落地后 DISARM 停桨。"""
        if self.state in ("LAND", "DISARM", "DONE"):
            return
        self.cur_a = (0.0, 0.0, 0.0)
        self.alt_hold = False            # 降落阶段不要再做高度保持
        now = time.monotonic()
        self.landed = False
        self.disarmed = False
        self.land_stable_t = None
        self.force_disarm_sent = False
        if note:
            self.log(note)
        if not self.auto_land:
            self.log("→ --no-auto-land：不做自动降落，保持 0 加速度悬停。")
            self.log("   ⚠ accel-only 无阻尼，无人机会保持原速度继续漂；请尽快在 QGC 里 LAND。")
            self.state = "DONE"
            self.done_t = now
            return
        # 从这里开始才是真的降落：改发 cmd_vel_ned（不再发 accel，见坑 8）
        self.landing_phase = True
        self.land_t0 = now
        self.last_land_cmd = 0.0         # 立刻发一次
        self.disarm_t0 = now
        self.log("→ 转入自动降落：LAND（切 AUTO_LAND 下降）→ 落地后 DISARM 停桨。")
        self.state = "LAND"
        self.t_state = now

    def request_landing(self, reason):
        """外部（Ctrl-C）请求收尾：降落 + 停桨，然后退出。"""
        if self.state == "DONE":
            self.log(f"收到 {reason} → 立即退出。")
            self.should_exit = True
            return
        if self.state in ("LAND", "DISARM"):
            self.log(f"已在降落/停桨流程中（{reason}），忽略重复请求。")
            return
        self.aborted = True
        self.abort_reason = f"外部请求：{reason}"
        self.cur_a = (0.0, 0.0, 0.0)
        self.snapshot_end()
        self.print_summary()
        self.start_landing(note=f"收到 {reason} → 转入自动降落。")

    def snapshot_end(self):
        if self.pos is not None:
            self.pos_end = (self.pos.x, self.pos.y, self.pos.z)
            self.heading_end = self.pos.heading
        self.odom_end = self.odom_pose()

    # ---------------------------------------------------------------- 安全
    def check_safety(self):
        if self.pos is None:
            return
        alt = -self.pos.z
        if alt < MIN_ALTITUDE:
            self.abort(f"离地高度 {alt:.2f} m < {MIN_ALTITUDE} m")
            return
        if self.p0 is not None:
            d = math.hypot(self.pos.x - self.p0[0], self.pos.y - self.p0[1])
            if d > MAX_H_DISP:
                self.abort(f"水平位移 {d:.2f} m > {MAX_H_DISP} m")

    # ---------------------------------------------------------------- 发布/记录
    def vel(self):
        return (0.0, 0.0, 0.0) if self.pos is None else (self.pos.vx, self.pos.vy, self.pos.vz)

    def odom_pose(self):
        o = self.odom
        if o is None:
            return None
        return (o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z)

    def publish_a(self):
        ax, ay, az = (float(v) for v in self.cur_a)
        # 垂直保持辅助（仅补 a_z 一项；a_z 正 = 向下）
        if self.alt_hold and self.alt_target is not None and self.pos is not None:
            az += self.alt_kp * ((-self.pos.z) - self.alt_target) - self.alt_kd * self.pos.vz
        m = Twist()
        m.linear.x, m.linear.y, m.linear.z = ax, ay, az
        self.pub_accel.publish(m)

    def log_row(self):
        p = self.pos
        if p is None:
            return
        seg = self.segments[self.seg_idx][0] if 0 <= self.seg_idx < len(self.segments) else "pre"
        row = [f"{time.monotonic() - self.t_start:.3f}", seg,
               self.cur_a[0], self.cur_a[1], self.cur_a[2],
               p.x, p.y, p.z, p.vx, p.vy, p.vz, p.heading, p.ax, p.ay, p.az,
               p.timestamp]
        o = self.odom
        if o is not None:
            row += [o.pose.pose.position.x, o.pose.pose.position.y, o.pose.pose.position.z,
                    o.twist.twist.linear.x, o.twist.twist.linear.y, o.twist.twist.linear.z]
        else:
            row += [""] * 6
        row.append("" if self.nav_state is None else self.nav_state)
        row.append("" if self.arming_state is None else self.arming_state)
        self.csv_w.writerow(row)

    # ---------------------------------------------------------------- 汇总
    def print_summary(self):
        self.log("")
        self.log("=" * 72)
        self.log("阶段 A 响应汇总  (Δv = 段末速度 − 段初速度, NED, m/s)")
        self.log("-" * 72)
        self.log(f"{'段':<14}{'指令 a_NED':<24}{'Δvx':>8}{'Δvy':>8}{'Δvz':>8}   判读")
        self.log("-" * 72)
        all_ok, n = True, 0
        for rec in self.seg_records:
            if "v1" not in rec:
                continue
            a = rec["a"]
            dv = tuple(rec["v1"][i] - rec["v0"][i] for i in range(3))
            note = ""
            if rec["label"] in EXPECTED_AXIS:
                axis, sign = EXPECTED_AXIS[rec["label"]]
                n += 1
                good = (math.copysign(1.0, dv[axis]) == sign) and abs(dv[axis]) > 0.15
                note = (f"OK  主轴 v{axis} {'增' if sign > 0 else '减'}") if good else "✗ 主轴响应不符"
                all_ok &= good
            self.log(f"{rec['label']:<14}({a[0]:+.2f},{a[1]:+.2f},{a[2]:+.2f})        "
                     f"{dv[0]:>8.3f}{dv[1]:>8.3f}{dv[2]:>8.3f}   {note}")
        self.log("-" * 72)

        # —— 坐标自检：用 odom 的**位置增量**（pose 在世界系=ENU）——
        # 注意：odom.twist 按 nav_msgs 约定在**机体系 FLU**，不能直接与世界系比。
        if self.odom0 is not None and self.odom_end is not None and \
                self.p0 is not None and self.pos_end is not None:
            do = tuple(self.odom_end[i] - self.odom0[i] for i in range(3))
            dn = tuple(self.pos_end[i] - self.p0[i] for i in range(3))
            pred = (dn[1], dn[0], -dn[2])          # 方案 §5：p_ENU=(y,x,-z)
            err = max(abs(do[i] - pred[i]) for i in range(3))
            self.log(f"坐标自检  Δp_ENU(odom) =({do[0]:+.3f},{do[1]:+.3f},{do[2]:+.3f})")
            self.log(f"          Δp_ENU(换算)=({pred[0]:+.3f},{pred[1]:+.3f},{pred[2]:+.3f})"
                     f"  max|err|={err:.3f} m  →  {'一致 OK' if err < 0.5 else '不一致 ✗'}")
            self.log("          （用 pose 增量比对；odom.twist 是机体系 FLU，不能直接比）")
        else:
            self.log("坐标自检  跳过（未收到 /odometry）")

        # —— 航向漂移（XTDrone2 的 cmd_accel_ned 会强制正北，见坑 4）——
        if self.heading0 is not None and self.heading_end is not None:
            dh = math.degrees(self.heading_end - self.heading0)
            self.log(f"航向      start={math.degrees(self.heading0):6.1f}°  "
                     f"end={math.degrees(self.heading_end):6.1f}°  Δ={dh:+6.1f}°"
                     + ("   ← 被转到正北（XTDrone2 未设 yaw，见坑 4）"
                        if abs(dh) > 20 else ""))

        # —— OFFBOARD 是否真的进去了（坑 5）——
        ns_txt = "?" if self.nav_state is None else str(self.nav_state)
        self.log(f"OFFBOARD  确认={'是' if self.offboard_confirmed else '否'}   "
                 f"末态 nav_state={ns_txt}（14=OFFBOARD）")

        if self.aborted:
            self.log(f"结论：**中止** — {self.abort_reason}")
        elif n == 0:
            self.log("结论：未采到有效脉冲段数据")
        elif all_ok:
            self.log(f"结论：**通路 OK** — {n}/{n} 个脉冲段主轴响应正确，cmd_accel_ned 可用。")
        elif not self.offboard_confirmed:
            self.log("结论：**PX4 未进入 OFFBOARD** — setpoint 全被忽略（悬停在 PX4 自身位置环）。")
            self.log("      查 offboard 信号：OffboardControlMode 是否在发 / COM_OF_LOSS_T 是否过期。")
        else:
            self.log("结论：**存在主轴响应不符的脉冲段** — 见上表，重点查 NED/ENU 是否搞反。")
        self.log("=" * 72)
        if self.auto_land:
            self.log("→ 进入自动降落流程：LAND（AUTO_LAND 下降 + cmd_vel_ned）→ 落地后 DISARM 停桨…")
        else:
            self.log("⚠ --no-auto-land：无人机保持 0 加速度（无阻尼，会保持原速度漂移）。")
            self.log("  请到 QGC 里 LAND。")

    def close(self):
        try:
            self.csv_f.flush()
            self.csv_f.close()
        except Exception:
            pass


def main(argv=None):
    ap = argparse.ArgumentParser(
        description="XTDrone2 Phase A · 指令回写通路验证",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    ap.add_argument("--namespace", default=DEFAULT_NS)
    ap.add_argument("--target-height", type=float, default=DEFAULT_TARGET_HEIGHT)
    ap.add_argument("--takeoff-tol", type=float, default=TAKEOFF_TOL)
    ap.add_argument("--takeoff-timeout", type=float, default=TAKEOFF_TIMEOUT)
    ap.add_argument("--a-mag", type=float, default=A_MAG)
    ap.add_argument("--pulse-sec", type=float, default=PULSE_SEC)
    ap.add_argument("--rate-hz", type=float, default=20.0)
    ap.add_argument("--linger", type=float, default=3.0,
                    help="收尾完成后多少秒自动退出")
    ap.add_argument("--srv-timeout", type=float, default=SRV_TIMEOUT)
    ap.add_argument("--no-alt-hold", action="store_true", help="关闭垂直高度保持辅助")
    ap.add_argument("--alt-kp", type=float, default=ALT_KP)
    ap.add_argument("--alt-kd", type=float, default=ALT_KD)
    ap.add_argument("--no-auto-land", action="store_true",
                    help="跑完后不自动降落（默认会自动 LAND + 停桨）")
    ap.add_argument("--no-disarm", action="store_true",
                    help="落地后不主动发 DISARM（PX4 自己也会 2s 自动停桨）")
    ap.add_argument("--land-timeout", type=float, default=LAND_TIMEOUT,
                    help="降落总超时（秒）")
    ap.add_argument("--no-force-disarm", action="store_true",
                    help="禁止兜底的强制停桨（param2=21196）；默认允许")
    ap.add_argument("--no-odom", action="store_true")
    ap.add_argument("--csv", default="phase_a_log.csv")
    args, _ = ap.parse_known_args(argv)

    # 自己接管 SIGINT：Ctrl-C 不再直接退出，而是先走"降落 + 停桨"流程再退出；
    # 再按一次 Ctrl-C 才强制退出。
    #
    # 关键：必须让 rclpy **不要**装它自己的 SIGINT handler，否则 Ctrl-C 会在
    # `rclpy.spin_once()` 内部被 rclpy 处理掉，我们拿不到控制权。
    _no_opt = resolve_no_signal_handler_option()
    if _no_opt is not None:
        rclpy.init(args=argv, signal_handler_options=_no_opt)
        own_sigint = True
    else:
        # 退路：先按默认方式 init（会装 rclpy 自己的 handler），再把它卸掉
        rclpy.init(args=argv)
        own_sigint = False
        try:
            from rclpy.signals import uninstall_signal_handlers
            uninstall_signal_handlers()
            own_sigint = True
        except Exception as _e:
            print(f"[warn] 无法接管 SIGINT（{_e!r}）："
                  f"Ctrl-C 将直接退出，不会自动降落。", file=sys.stderr)

    node = PhaseAAccelPath(args)

    if own_sigint:
        def _on_sigint(signum, frame):
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
        node.cur_a = (0.0, 0.0, 0.0)
        try:
            node.publish_a()
        except Exception:
            pass
    finally:
        node.close()
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            try:
                # 我们已接管 SIGINT，就别让 rclpy 在退出时再动它。
                # （老版本可能不认这个入参 → 退回无参调用）
                rclpy.shutdown(uninstall_handlers=False)
            except TypeError:
                try:
                    rclpy.shutdown()
                except Exception:
                    pass
            except Exception:
                pass


if __name__ == "__main__":
    main()
