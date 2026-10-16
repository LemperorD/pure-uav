#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""离线状态机回归测试：把 rclpy / ROS 消息全替换成桩，直接驱动 phase_a_accel_path 的状态机。

目标：验证「跑完 → LAND → 落地 → DISARM → DONE → 退出」这条**安全相关**路径，
以及 Ctrl-C 请求收尾（request_landing）、OFFBOARD 确认失败后仍收尾等分支。
**不需要 ROS，不需要仿真**，Windows 侧也能直接跑：

    python3 test_phase_a_states.py        # 退出码 0 = 全过

覆盖 9 个用例：
  1. 正常跑完 → 自动 LAND → 落地 → DISARM → DONE → 退出
  2. 测试段中途 Ctrl-C（request_landing）→ 降落 + 停桨
  3. DONE 状态下 Ctrl-C → 立即退出
  4. --no-auto-land → 不降落，直接 DONE
  5. 降落超时 → 放弃并提示（不卡死）
  6. OFFBOARD 确认失败 → abort → 仍然走降落流程
  7. ARM 未被 PX4 确认（坑 6）→ 明确报错，不会盲目前进
  8. SIGINT 枚举名探测（钉住 "SignalHandlerOptions.NONE 不存在" 这个 bug）
  9. main() 启动路径冒烟测试（钉住 "一启动就崩" 这个 bug）
"""
import sys, types, math, os, tempfile

# ----------------------------------------------------------------- 桩
class _Logger:
    def __init__(self): self.lines = []
    def info(self, m): self.lines.append(("I", m))
    def warn(self, m): self.lines.append(("W", m))

class _Clock:
    def now(self):
        class _T:
            def seconds_nanoseconds(self): return (12345, 678000000)
        return _T()

class _Pub:
    def __init__(self, sink, topic): self.sink, self.topic = sink, topic
    def publish(self, m): self.sink.append((self.topic, m))

class _Node:
    def __init__(self, name): self._name = name; self.logger = _Logger()
    def create_publisher(self, msgtype, topic, qos):
        PUB_SINK.append((topic, None)); return _Pub(PUB_SINK, topic)
    def create_client(self, srv, topic): return None
    def create_subscription(self, *a, **k): return None
    def create_timer(self, *a, **k): return None
    def get_logger(self): return self.logger
    def get_clock(self): return _Clock()
    def destroy_node(self): pass

def _mkmod(name):
    m = types.ModuleType(name); sys.modules[name] = m; return m

PUB_SINK = []

rclpy = _mkmod("rclpy")
rclpy.init = lambda *a, **k: None
rclpy.ok = lambda: True
rclpy.shutdown = lambda *a, **k: None
rclpy.spin_once = lambda *a, **k: None
rclpy_node = _mkmod("rclpy.node"); rclpy_node.Node = _Node
rclpy_qos = _mkmod("rclpy.qos")
class _QoS:
    def __init__(self, **k): pass
rclpy_qos.QoSProfile = _QoS
rclpy_qos.qos_profile_sensor_data = types.SimpleNamespace(reliability=1)
rclpy_signals = _mkmod("rclpy.signals")
# ⚠ 真实 Jazzy 里"不装 handler"的成员叫 NO（**没有 NONE**）。
# 这里故意只提供 NO，用来防止代码里再写死 SignalHandlerOptions.NONE。
class _SHO:
    NO = 0
    SIGINT = 1
    SIGTERM = 2
    ALL = 3
rclpy_signals.SignalHandlerOptions = _SHO
rclpy_signals.uninstall_signal_handlers = lambda: None

gm = _mkmod("geometry_msgs"); gm_msg = _mkmod("geometry_msgs.msg")
class Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)
gm_msg.Twist = Twist

px4 = _mkmod("px4_msgs"); px4_msg = _mkmod("px4_msgs.msg")
class VehicleLocalPosition: pass
class VehicleStatus: pass
class VehicleCommand:
    def __init__(self):
        for f in ("timestamp","command","param1","param2","param3","param4",
                  "param5","param6","param7","target_system","target_component",
                  "source_system","source_component","from_external"):
            setattr(self, f, 0)
px4_msg.VehicleLocalPosition = VehicleLocalPosition
px4_msg.VehicleStatus = VehicleStatus
px4_msg.VehicleCommand = VehicleCommand

xtd2 = _mkmod("xtd2_msgs"); xtd2_srv = _mkmod("xtd2_msgs.srv")
class XTD2Cmd:
    class Request:
        def __init__(self): self.command = ""
xtd2_srv.XTD2Cmd = XTD2Cmd
# 注意：故意不建 nav_msgs ⇒ 模块里 _HAS_ODOM = False

# ----------------------------------------------------------------- 虚拟时钟
T = [1000.0]
import time as _time
_time.monotonic = lambda: T[0]

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import phase_a_accel_path as M

class P:
    def __init__(self, x=0.0, y=0.0, z=-10.0, vx=0.0, vy=0.0, vz=0.0, heading=0.0):
        self.x, self.y, self.z = x, y, z
        self.vx, self.vy, self.vz = vx, vy, vz
        self.heading = heading
        self.ax = self.ay = self.az = 0.0
        self.timestamp = int(T[0] * 1e6)

def mk_args(**kw):
    d = dict(namespace="x500_0", target_height=10.0, takeoff_tol=1.0, takeoff_timeout=60.0,
             a_mag=0.5, pulse_sec=2.0, rate_hz=20.0, linger=3.0, srv_timeout=3.0,
             no_alt_hold=False, alt_kp=1.0, alt_kd=2.0, no_auto_land=False,
             no_disarm=False, no_force_disarm=False, land_timeout=90.0, no_odom=True,
             csv=os.path.join(tempfile.gettempdir(), "sm_test.csv"))
    d.update(kw)
    return types.SimpleNamespace(**d)

def new_node(**kw):
    n = M.PhaseAAccelPath(mk_args(**kw))
    n.calls = []
    def fake_issue(cmd):
        n.calls.append(cmd); n.on_ack(cmd, True); return True
    n.issue = fake_issue
    n.land_cmds = []
    n.disarm_cmds = []        # [(时刻, 是否强制)]
    n.descend_vel_calls = 0
    n.publish_land = lambda: n.land_cmds.append(T[0])
    n.publish_disarm = lambda force=False: n.disarm_cmds.append((T[0], force))
    n.publish_descend_vel = lambda: setattr(
        n, "descend_vel_calls", n.descend_vel_calls + 1)
    n.offb_cmds = []
    n.publish_offboard_mode = lambda: n.offb_cmds.append(T[0])
    n.log_row = lambda: None
    return n

def step(n, dt=0.05, n_iter=1):
    for _ in range(n_iter):
        T[0] += dt
        n.tick()

def drive_to_run(n, alt=9.4):
    """推进到 RUN 段开始。"""
    n.pos = P(z=-alt)
    step(n, n_iter=3)                       # ARM 服务
    n.arming_state = M.ARMING_STATE_ARMED   # PX4 确认解锁
    step(n, n_iter=3)                       # CONFIRM_ARM → POST_ARM
    step(n, 0.6 / 0.05)                     # POST_ARM → TAKEOFF
    step(n, 0.2 / 0.05)                     # TAKEOFF
    for _ in range(80):                     # WAIT_HEIGHT（vz=0 保持 1.5s）
        n.pos.vz = 0.0
        step(n, 0.05)
    step(n, 0.1 / 0.05)                     # OFFBOARD 服务
    n.nav_state = M.NAV_STATE_OFFBOARD      # PX4 确认
    step(n, n_iter=3)                       # CONFIRM_OFFBOARD → SETTLE
    step(n, 1.2 / 0.05)                     # SETTLE
    return n

FAIL = []
def check(cond, msg):
    print(("  PASS  " if cond else "  FAIL  ") + msg)
    if not cond: FAIL.append(msg)

print("=" * 70)
print("用例 1：正常跑完 → 自动 LAND → 落地 → DISARM → DONE → 退出")
print("=" * 70)
n = new_node()
drive_to_run(n)
check(n.state == "RUN", f"已进入 RUN（当前 {n.state}）")

# 跑完 9 段（总时长 15s；浮点累积使每段可能多要 1 拍，故多给 40 拍余量）
total = sum(s[2] for s in n.segments)
step(n, 0.05, int(total / 0.05) + 40)
check(n.state == "LAND", f"跑完自动进入 LAND（当前 {n.state}）")
check(len(n.land_cmds) >= 1, f"已发 NAV_LAND（{len(n.land_cmds)} 次）")

# 下降
n.nav_state = M.NAV_STATE_AUTO_LAND
for i in range(400):
    n.pos.z = -max(0.0, 9.4 - 0.06 * i)
    step(n, 0.05)
    if n.state == "DISARM": break
check(n.state == "DISARM", f"落地后进入 DISARM（当前 {n.state}，alt={-n.pos.z:.2f}）")
step(n, 0.1 / 0.05)                          # 下一拍才真正发 DISARM
check(len(n.disarm_cmds) >= 1, f"已发 DISARM（{len(n.disarm_cmds)} 次）")

n.arming_state = M.ARMING_STATE_DISARMED
step(n, 0.1 / 0.05)
check(n.state == "DONE", f"停桨后进入 DONE（当前 {n.state}）")
step(n, 3.5 / 0.05)
check(n.should_exit, "linger 后 should_exit=True")
check(n.disarmed, "disarmed 标记已置位")
print(f"  调用序列: {n.calls}")

print()
print("=" * 70)
print("用例 2：测试段中途 Ctrl-C（request_landing）→ 降落 + 停桨")
print("=" * 70)
n = new_node()
drive_to_run(n)
step(n, 3.0 / 0.05)                        # 跑了几段
check(n.state == "RUN", "仍在 RUN")
n.request_landing("Ctrl-C")
check(n.state == "LAND", f"Ctrl-C 后转入 LAND（当前 {n.state}）")
check(n.aborted, "已标记 aborted")
check(n.alt_hold is False, "降落时已关闭高度保持辅助")
for i in range(300):
    n.pos.z = -max(0.0, 9.4 - 0.06 * i)
    step(n, 0.05)
    if n.state == "DISARM": break
check(n.state == "DISARM", f"落地 → DISARM（当前 {n.state}）")
n.arming_state = M.ARMING_STATE_DISARMED
step(n, 0.1 / 0.05)
step(n, 3.5 / 0.05)
check(n.should_exit, "最终 should_exit=True")

print()
print("=" * 70)
print("用例 3：DONE 状态下 Ctrl-C → 立即退出")
print("=" * 70)
n = new_node()
drive_to_run(n)
n.state = "DONE"; n.done_t = T[0]
n.request_landing("Ctrl-C")
check(n.should_exit, "DONE 下 Ctrl-C 立即 should_exit")

print()
print("=" * 70)
print("用例 4：--no-auto-land → 不降落，直接 DONE")
print("=" * 70)
n = new_node(no_auto_land=True)
drive_to_run(n)
step(n, 0.05, int(sum(s[2] for s in n.segments) / 0.05) + 40)
check(n.state == "DONE", f"直接进入 DONE（当前 {n.state}）")
check(len(n.land_cmds) == 0, "没有发 NAV_LAND")

print()
print("=" * 70)
print("用例 5：降落超时 → 放弃并提示（不卡死）")
print("=" * 70)
n = new_node(land_timeout=10.0)
drive_to_run(n)
n.request_landing("Ctrl-C")
check(n.state == "LAND", "进入 LAND")
n.pos.z = -9.0                              # 一直不掉高
step(n, 0.05, int(12.0 / 0.05))
check(n.state == "DONE", f"超时后进入 DONE（当前 {n.state}）")
check(not n.landed, "landed 未置位（确实没落地）")

print()
print("=" * 70)
print("用例 6：OFFBOARD 确认失败 → abort → 仍然走降落流程")
print("=" * 70)
n = new_node()
n.pos = P(z=-9.4)
step(n, n_iter=3)
n.arming_state = M.ARMING_STATE_ARMED
step(n, n_iter=3)
step(n, 0.6 / 0.05); step(n, 0.2 / 0.05)
for _ in range(80):
    n.pos.vz = 0.0; step(n, 0.05)
step(n, 0.1 / 0.05)                          # 发 OFFBOARD 服务
n.nav_state = 2                              # PX4 停在 POSCTL，不确认
step(n, 0.05, int(11.0 / 0.05))
check(n.aborted, "OFFBOARD 确认超时 → aborted")
check(n.state == "LAND", f"abort 后进入 LAND（当前 {n.state}）")
check(len(n.land_cmds) >= 1, "abort 后仍然发 NAV_LAND")

print()
print("=" * 70)
print("用例 7：ARM 未被 PX4 确认（坑 6）→ 明确报错，不会盲目前进")
print("=" * 70)
n = new_node()
n.pos = P(z=-9.4)
n.arming_state = M.ARMING_STATE_DISARMED     # 一直没解锁
step(n, 0.05, int(20.0 / 0.05))              # 超过 ARM_CONFIRM_TIMEOUT(15s)
check(n.aborted, "解锁确认超时 → aborted")
check(n.state in ("LAND", "DISARM", "DONE"),
      f"进入收尾流程（当前 {n.state}；已 DISARMED 时 LAND 会立即判定落地）")
check("解锁确认超时" in n.abort_reason, f"报错文案明确：{n.abort_reason}")
check(len(n.calls) >= 2, f"期间重发过 ARM（{len(n.calls)} 次）")

print()
print("=" * 70)
print("用例 8：SIGINT 枚举名探测（钉住 \"SignalHandlerOptions.NONE 不存在\" 这个 bug）")
print("=" * 70)
# 8a) 模拟真实 Jazzy：只有 NO（值为 0），没有 NONE
opt = M.resolve_no_signal_handler_option()
check(opt is not None, f"能解析出'不装 handler'成员（得到 {opt!r}）")
check(opt == 0, "解析到的是 NO=0（值为 0，不能用真值判断）")

# 8b) 模拟一个连 NO 都没有的版本 → 必须返回 None（走 uninstall 退路），而不是抛异常
_saved = rclpy_signals.SignalHandlerOptions
class _Empty: pass
rclpy_signals.SignalHandlerOptions = _Empty
try:
    opt2 = M.resolve_no_signal_handler_option()
    check(opt2 is None, "成员名全都不匹配时返回 None（不抛异常）")
finally:
    rclpy_signals.SignalHandlerOptions = _saved

# 8c) 文档/注释里不能再出现写死的 SignalHandlerOptions.NONE
src = open(os.path.join(os.path.dirname(os.path.abspath(__file__)),
                        "phase_a_accel_path.py"), encoding="utf-8").read()
code_lines = [l for l in src.splitlines() if not l.strip().startswith("#")]
hard = [l for l in code_lines if "SignalHandlerOptions.NONE" in l]
check(not hard, f"代码里没有写死的 SignalHandlerOptions.NONE（{len(hard)} 处）")

print()
print("=" * 70)
print("用例 9：main() 启动路径冒烟测试（钉住 \"一启动就崩\" 这个 bug）")
print("=" * 70)
import signal as _signal
_saved_sigint = _signal.getsignal(_signal.SIGINT)
_saved_spin = rclpy.spin_once
# 让 spin_once 跑一圈就置 should_exit，从而 main() 立刻走 finally 退出
rclpy.spin_once = lambda node, **kw: setattr(node, "should_exit", True)
try:
    rc = "ok"
    # ⚠ 必须显式指定 --csv，否则 main() 会用默认的 phase_a_log.csv
    #   （相对路径）在当前目录留一个副产品
    _csv_tmp = os.path.join(tempfile.gettempdir(), "sm_test_main.csv")
    try:
        M.main(["--csv", _csv_tmp])
    except BaseException as e:          # noqa: BLE001
        rc = f"{type(e).__name__}: {e}"
    check(rc == "ok", f"main() 正常跑完并退出（{rc}）")
finally:
    rclpy.spin_once = _saved_spin
    _signal.signal(_signal.SIGINT, _saved_sigint)

print()
print("=" * 70)
print("用例 10：坑 8 —— 降落阶段必须改发 cmd_vel_ned 的下降速度")
print("=" * 70)
# 背景：PX4 的 land detector 要求 trajectory_setpoint.velocity[2] 是有限值且
# >= 1.1*LNDMC_Z_VEL_MAX(0.275)，否则 ground_contact 永远不成立 → DISARM 被拒。
# XTDrone2 在 accel 模式下一直灌 velocity=NaN，所以降落必须切到 cmd_vel_ned。
n = new_node()
drive_to_run(n)
step(n, 0.05, int(sum(s[2] for s in n.segments) / 0.05) + 40)
check(n.state == "LAND", f"进入 LAND（当前 {n.state}）")
check(n.landing_phase, "landing_phase 已置位（之后发 vel 不发 accel）")
check(n.descend_vel_calls >= 1,
      f"降落阶段已在发下降速度指令（{n.descend_vel_calls} 次）")
check(M.LAND_DESCEND_VEL >= 1.1 * 0.25,
      f"LAND_DESCEND_VEL={M.LAND_DESCEND_VEL} ≥ 1.1×LNDMC_Z_VEL_MAX(0.25)=0.275")

# 降到地面
for i in range(400):
    n.pos.z = -max(0.0, 9.4 - 0.06 * i)
    step(n, 0.05)
    if n.state == "DISARM": break
check(n.state == "DISARM", f"落到地面后进入 DISARM（当前 {n.state}，alt={-n.pos.z:.2f}）")
check(n.landing_phase and n.descend_vel_calls > 20,
      f"降落期间持续发下降速度（累计 {n.descend_vel_calls} 次）")

print()
print("=" * 70)
print("用例 11：PX4 拒不停桨（复现坑 8）→ 兜底强制停桨生效")
print("=" * 70)
# 让 arming_state 一直保持 ARMED，模拟"PX4 的 land detector 不认落地"
disarm_states = []
for i in range(800):
    step(n, 0.05)
    disarm_states = [f for (_, f) in n.disarm_cmds]
    if any(disarm_states):
        break
check(any(f for (_, f) in n.disarm_cmds),
      "超时后发出了强制停桨（param2=21196）")
n_force = sum(1 for (_, f) in n.disarm_cmds if f)
n_normal = sum(1 for (_, f) in n.disarm_cmds if not f)
check(n_normal >= 1 and n_force >= 1,
      f"先发普通 DISARM（{n_normal} 次）再升级为强制（{n_force} 次）")

# 安全闸：强制停桨只在"仍在落地判据内"时发
_n_saved = len(n.disarm_cmds)
n.pos = P(z=-9.0)                 # 突然"飞起"（模拟判据失效）
step(n, 0.05, 40)
check(len(n.disarm_cmds) == _n_saved,
      "离开落地判据后不再发任何停桨指令（安全闸生效）")

# 回到地面 → PX4 最终停桨
n.pos = P(z=-0.20)
n.arming_state = M.ARMING_STATE_DISARMED
step(n, 0.05, 5)
check(n.state == "DONE" and n.disarmed, f"最终确认停桨并 DONE（{n.state}）")

print()
print("=" * 70)
print("用例 12：--no-force-disarm → 永不发强制停桨")
print("=" * 70)
n = new_node(no_force_disarm=True)
drive_to_run(n)
step(n, 0.05, int(sum(s[2] for s in n.segments) / 0.05) + 40)
for i in range(400):
    n.pos.z = -max(0.0, 9.4 - 0.06 * i)
    step(n, 0.05)
    if n.state == "DISARM": break
check(n.state == "DISARM", f"进入 DISARM（当前 {n.state}）")
step(n, 0.05, int(15.0 / 0.05))       # 超过 FORCE_DISARM_AFTER
check(not any(f for (_, f) in n.disarm_cmds),
      f"没有发过强制停桨（共 {len(n.disarm_cmds)} 次普通 DISARM）")
check(n.force_disarm_ok is False, "force_disarm_ok 已按参数关闭")

print()
print("=" * 70)
print(("全部通过 ✅" if not FAIL else f"失败 {len(FAIL)} 项 ❌"))
for f in FAIL: print("   -", f)
print("=" * 70)
sys.exit(1 if FAIL else 0)
