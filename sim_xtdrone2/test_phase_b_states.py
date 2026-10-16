#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Phase B 离线闭环回归测试：桩替 ROS，但**制导核是真代码**（真 .so/.dll）。

它把「假 PX4（质点 + 一阶加速度响应）+ 真制导核 + 真 DART 目标模型」接在一起，
驱动 phase_b_guidance 的**完整状态机**跑一遍，验证：

  · 状态机：ARM→…→CRUISE→GUIDED→命中→LAND→DISARM→DONE 全链路
  · **坐标方向**：ENU 的 a_cmd 是否被正确翻成 NED 发出去（最危险的一环）
  · **PPN 零速死区**：--no-cruise 时应该脱靶（这是设计 CRUISE 段的理由）
  · 安全围栏：掉高/越界 → abort → 仍然走降落收尾
  · 命中判定与统计量

**不需要 ROS、不需要仿真、不需要 Gazebo**，Windows 侧也能直接跑：

    python3 test_phase_b_states.py                       # 自动找同目录的制导核
    python3 test_phase_b_states.py path/to/libpure_guidance.so

退出码 0 = 全过。
"""
import math
import os
import sys
import tempfile
import types

# ----------------------------------------------------------------- 桩
class _Logger:
    def __init__(self):
        self.lines = []

    def info(self, m):
        self.lines.append(("I", m))

    def warn(self, m):
        self.lines.append(("W", m))

    def text(self):
        return "\n".join(f"[{k}] {m}" for k, m in self.lines)


class _Clock:
    def now(self):
        class _T:
            def seconds_nanoseconds(self):
                return (12345, 678000000)
        return _T()


class _Pub:
    def __init__(self, sink, topic):
        self.sink, self.topic = sink, topic

    def publish(self, m):
        self.sink.append((self.topic, m))


class _Node:
    def __init__(self, name):
        self._name = name
        self.logger = _Logger()

    def create_publisher(self, msgtype, topic, qos):
        return _Pub(PUB_SINK, topic)

    def create_client(self, srv, topic):
        return None

    def create_subscription(self, *a, **k):
        return None

    def create_timer(self, *a, **k):
        return None

    def get_logger(self):
        return self.logger

    def get_clock(self):
        return _Clock()

    def destroy_node(self):
        pass


def _mkmod(name):
    m = types.ModuleType(name)
    sys.modules[name] = m
    return m


PUB_SINK = []

rclpy = _mkmod("rclpy")
rclpy.init = lambda *a, **k: None
rclpy.ok = lambda: True
rclpy.shutdown = lambda *a, **k: None
rclpy.spin_once = lambda *a, **k: None
rclpy_node = _mkmod("rclpy.node")
rclpy_node.Node = _Node
rclpy_qos = _mkmod("rclpy.qos")


class _QoS:
    def __init__(self, **k):
        pass


rclpy_qos.QoSProfile = _QoS
rclpy_qos.qos_profile_sensor_data = types.SimpleNamespace(reliability=1)
rclpy_signals = _mkmod("rclpy.signals")


class _SHO:            # 故意只有 NO（Jazzy 的真实成员名），没有 NONE
    NO = 0
    SIGINT = 1
    SIGTERM = 2
    ALL = 3


rclpy_signals.SignalHandlerOptions = _SHO
rclpy_signals.uninstall_signal_handlers = lambda: None

gm = _mkmod("geometry_msgs")
gm_msg = _mkmod("geometry_msgs.msg")


class Twist:
    def __init__(self):
        self.linear = types.SimpleNamespace(x=0.0, y=0.0, z=0.0)


gm_msg.Twist = Twist

px4 = _mkmod("px4_msgs")
px4_msg = _mkmod("px4_msgs.msg")


class VehicleLocalPosition:
    def __init__(self):
        self.x = self.y = self.z = 0.0
        self.vx = self.vy = self.vz = 0.0
        self.heading = 0.0
        self.timestamp = 0


class VehicleStatus:
    def __init__(self):
        self.nav_state = 0
        self.arming_state = 1


class VehicleCommand:
    def __init__(self):
        for f in ("timestamp", "command", "param1", "param2", "param3", "param4",
                  "param5", "param6", "param7", "target_system", "target_component",
                  "source_system", "source_component", "from_external"):
            setattr(self, f, 0)


px4_msg.VehicleLocalPosition = VehicleLocalPosition
px4_msg.VehicleStatus = VehicleStatus
px4_msg.VehicleCommand = VehicleCommand

xtd2 = _mkmod("xtd2_msgs")
xtd2_srv = _mkmod("xtd2_msgs.srv")


class XTD2Cmd:
    class Request:
        def __init__(self):
            self.command = ""


xtd2_srv.XTD2Cmd = XTD2Cmd
# 故意不建 nav_msgs ⇒ 模块里 HAVE_ODOM = False

# ----------------------------------------------------------------- 虚拟时钟
VT = [1000.0]
import time as _time          # noqa: E402
_time.monotonic = lambda: VT[0]

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import phase_b_guidance as M   # noqa: E402

FAIL = []


def check(ok, msg):
    print(("  PASS  " if ok else "  FAIL  ") + msg)
    if not ok:
        FAIL.append(msg)


# ==========================================================================
#  假 PX4：无人机质点 + 一阶加速度响应，状态量按 NED 写回 node.pos
# ==========================================================================
class FakePX4:
    def __init__(self, node, p_enu=(0.0, 0.0, 10.0), tau=0.2, dt=0.02):
        self.n = node
        self.p = list(p_enu)                 # ENU（默认已悬停在 10 m）
        self.v = [0.0, 0.0, 0.0]             # ENU
        self.a = [0.0, 0.0, 0.0]             # ENU（实际）
        self.tau = tau
        self.dt = dt
        self.allow_offboard = True           # PX4 是否认可 OFFBOARD 切换（用例 5 会关掉）
        self.on_ground = False               # 是否已触地
        self.n.pos = VehicleLocalPosition()
        self.n.status = VehicleStatus()
        self.launched_land = False
        self.sync()

    def sync(self):
        """ENU → NED 写回 node.pos。"""
        p = self.n.pos
        p.x, p.y, p.z = self.p[1], self.p[0], -self.p[2]
        p.vx, p.vy, p.vz = self.v[1], self.v[0], -self.v[2]
        p.timestamp = int(VT[0] * 1e6)       # PX4 仿真时间戳（µs）

    def advance(self):
        """按当前指令推进一个 dt，并同步到 node.pos。"""
        # 降落阶段：节点改发 cmd_vel_ned ⇒ 按给定下降速度垂直下降，触地后速度归零
        if getattr(self.n, "landing_phase", False):
            if not self.on_ground:
                vz = self.n.args.land_down_speed
                self.p[2] -= vz * self.dt
                self.v = [0.0, 0.0, -vz]
                if self.p[2] <= 0.20:        # 起落架高度，视为触地
                    self.p[2] = 0.20
                    self.on_ground = True
            else:
                self.p[2] = 0.20
                self.v = [0.0, 0.0, 0.0]
            self.a = [0.0, 0.0, 0.0]
            self.sync()
            return
        a_cmd = self.n.cur_a_enu             # 节点本周期要发的加速度（ENU）
        for k in range(3):
            if self.tau > 1e-9:
                self.a[k] += (a_cmd[k] - self.a[k]) * (self.dt / self.tau)
            else:
                self.a[k] = a_cmd[k]
            self.v[k] += self.a[k] * self.dt
            self.p[k] += self.v[k] * self.dt
        self.sync()


def mk_args(lib, **kw):
    d = dict(
        namespace="x500_0", lib=lib,
        target_p0=(-12.0, 0.0, 11.0), target_type=2,
        target_speed=15.0, target_elev=5.0, target_azim=0.0,
        target_mass=0.40, target_cd=0.50, target_area=2.83e-3, target_rho=1.225,
        law=0, N=6.0, a_max=10.0, t_go_freeze=0.10, hit_radius=0.5,
        a_boost=6.0, v_boost_min=5.0, boost_dir=(-1.0, 0.0, 0.0), no_cruise=False,
        cruise_x=None, v_kp=1.5, cruise_timeout=8.0,
        target_height=10.0, takeoff_tol=1.0, takeoff_timeout=60.0, rate=50.0,
        no_auto_land=False, no_disarm=False, no_force_disarm=False,
        land_timeout=90.0, land_down_speed=0.5, linger=3.0,
        no_alt_hold=False, alt_kp=1.0, alt_kd=2.0, srv_timeout=3.0,
        no_odom=True,
        csv=os.path.join(tempfile.gettempdir(), "sm_test_b.csv"),
    )
    d.update(kw)
    return types.SimpleNamespace(**d)


def new_node(lib, **kw):
    n = M.PhaseBGuidance(mk_args(lib, **kw))
    # 服务调用走桩：直接 on_ack
    def fake_issue(cmd):
        n.on_ack(cmd, True)
        return True
    n.issue = fake_issue
    # 桩：PX4 默认已解锁（用例可改）
    n.arming_state = 2
    n.nav_state = 17
    return n


def step(n, fx, n_iter=1, dt=0.02, px4_update=True):
    """推进 n_iter 个控制周期（同时模拟 PX4 侧的 nav_state / arming_state 变化）。"""
    for _ in range(n_iter):
        # ---- 假 PX4 的"模式管理" ----
        if n.state == "CONFIRM_OFFBOARD" and fx.allow_offboard:
            n.nav_state = M.NAV_STATE_OFFBOARD      # 认可切换
        elif n.state in ("TAKEOFF", "WAIT_HEIGHT"):
            n.nav_state = (M.NAV_STATE_AUTO_TAKEOFF if fx.p[2] < 1.0 else 4)  # 4=AUTO_LOITER
        if fx.launched_land:
            n.nav_state = M.NAV_STATE_AUTO_LAND
        # ---- 假 PX4 的"停桨响应"：落地后收到 DISARM 就接受 ----
        if n.state == "DISARM" and fx.on_ground:
            n.arming_state = M.ARMING_STATE_DISARMED
        n.tick()
        if px4_update:
            fx.advance()
        VT[0] += dt


# ==========================================================================
def find_lib(explicit=None):
    if explicit and os.path.exists(explicit):
        return os.path.abspath(explicit)
    here = os.path.dirname(os.path.abspath(__file__))
    for d in (here, os.getcwd()):
        for nm in ("libpure_guidance.so", "pure_guidance.dll",
                   "libpure_guidance.dll", "libpure_guidance.dylib"):
            p = os.path.join(d, nm)
            if os.path.exists(p):
                return p
    raise SystemExit("找不到制导核动态库。先跑 build_guidance_shim.sh，"
                     "或把库路径作为第一个参数传入。")


def drive_to_guided(n, fx, alt=10.0, timeout_steps=2000):
    """把状态机推到 GUIDED 段起点。"""
    fx.p = [0.0, 0.0, alt]
    fx.v = [0.0, 0.0, 0.0]
    fx.sync()
    for _ in range(timeout_steps):
        if n.state == "GUIDED":
            return True
        step(n, fx, 1)
    return False


# ==========================================================================
def main():
    lib = find_lib(sys.argv[1] if len(sys.argv) > 1 else None)
    print("=" * 74)
    print("Phase B 离线回归测试（桩替 ROS / 真制导核 / 假 PX4 质点）")
    print(f"制导核: {lib}")
    print("=" * 74)

    # ---------------------------------------------------------------- 用例 1
    print()
    print("=" * 74)
    print("用例 1：ENU → NED 指令方向（最危险的一环）")
    print("=" * 74)
    n = new_node(lib)
    PUB_SINK.clear()
    n.cur_a_enu = (1.0, 2.0, 3.0)          # ENU: x=东, y=北, z=上
    n.publish_a()
    twists = [m for (t, m) in PUB_SINK if t.endswith("cmd_accel_ned")]
    check(len(twists) == 1, f"发布了 1 条 cmd_accel_ned（实际 {len(twists)}）")
    if twists:
        m = twists[0]
        ok = (abs(m.linear.x - 2.0) < 1e-9 and      # NED 北 = ENU y
              abs(m.linear.y - 1.0) < 1e-9 and      # NED 东 = ENU x
              abs(m.linear.z + 3.0) < 1e-9)         # NED 下 = −ENU z
        check(ok, f"换算正确：ENU(1,2,3) → NED({m.linear.x},{m.linear.y},{m.linear.z})"
                  f"，期望 (2.0,1.0,-3.0)")
    n.close()

    # ---------------------------------------------------------------- 用例 2
    print()
    print("=" * 74)
    print("用例 2：完整闭环（巡航 → 制导 → 命中 → 自动降落收尾）")
    print("=" * 74)
    n = new_node(lib)
    fx = FakePX4(n)
    ok = drive_to_guided(n, fx)
    check(ok, f"进入 GUIDED（当前 {n.state}）")
    check(n.cruise_t0 is not None, "CRUISE 段跑过")
    v = math.sqrt(sum(x * x for x in fx.v))
    check(v >= n.v_boost_min - 0.3, f"预加速达标 |v|={v:.2f} ≥ {n.v_boost_min}")

    for _ in range(1200):
        step(n, fx, 1)
        if n.state in ("LAND", "DONE"):
            break
    check(n.hit, f"命中（最近 {n.min_R if n.min_R is None else round(n.min_R, 3)} m）")
    check(n.state in ("LAND", "DISARM", "DONE"), f"命中后进入降落收尾（当前 {n.state}）")
    # 继续驱动：下降 → 触地 → DISARM → DONE（落地的模拟在 fx.advance 里）
    for _ in range(2000):
        step(n, fx, 1)
        if n.should_exit:
            break
    check(n.landed, f"判定落地（alt={fx.p[2]:.2f} m）")
    check(n.state == "DONE", f"收尾到 DONE（当前 {n.state}）")
    check(n.should_exit, "最终 should_exit=True（节点会退出）")
    check(n.disarmed, "已确认停桨（arming_state=DISARMED）")
    if n.min_R is None:
        print("  统计: （没有制导段数据）")
        check(False, "采到了制导段数据（min_R 不为 None）")
    else:
        print(f"  统计: 最近 {n.min_R:.3f} m @ t_rel={n.t_min_R:.3f}s  "
              f"峰值 {n.peak_a_cmd:.2f} m/s²  饱和 {n.sat_steps}/{n.guided_steps}  "
              f"模式 {n.modes}")
        check(n.min_R < 1.0, f"最近距离 < 1.0 m（实际 {n.min_R:.3f}）")
    check(n.peak_a_cmd <= n.args.a_max + 1e-9,
          f"峰值指令 ≤ a_max（{n.peak_a_cmd:.2f} ≤ {n.args.a_max}）")
    n.close()

    # ---------------------------------------------------------------- 用例 3
    print()
    print("=" * 74)
    print("用例 3：--no-cruise（悬停直接制导）→ 复现 PPN 零速死区，应当脱靶")
    print("=" * 74)
    n = new_node(lib, no_cruise=True)
    fx = FakePX4(n)
    ok = drive_to_guided(n, fx)
    check(ok, f"进入 GUIDED（当前 {n.state}）")
    check(math.sqrt(sum(x * x for x in fx.v)) < 0.5,
          f"确认是从悬停开始（|v|={math.sqrt(sum(x * x for x in fx.v)):.3f}）")
    for _ in range(1200):
        step(n, fx, 1)
        if n.state in ("LAND", "DONE"):
            break
    print(f"  结果: 最近 {n.min_R:.3f} m  命中={n.hit}  模式分布 {n.modes}")
    check(not n.hit, f"如预期**未命中**（最近 {n.min_R:.3f} m）—— 证明 CRUISE 段是必要的")
    check(n.modes.get(M.GMODE_SEARCH, 0) == 0, "没有进 SEARCH（目标一直是可见的）")
    n.close()

    # ---------------------------------------------------------------- 用例 4
    print()
    print("=" * 74)
    print("用例 4：安全围栏 —— 掉高到 3 m 以下 → 中止 → 仍然走降落收尾")
    print("=" * 74)
    n = new_node(lib, no_cruise=True)
    fx = FakePX4(n)
    drive_to_guided(n, fx)
    # 强行把无人机拉到 2 m 高
    fx.p[2] = 2.0
    fx.sync()
    step(n, fx, 1)
    check(n.aborted, f"触发中止（原因：{n.abort_reason}）")
    check("离地高度" in (n.abort_reason or ""), f"原因文案明确：{n.abort_reason}")
    check(n.landing_phase, "转入降落收尾流程")
    n.close()

    # ---------------------------------------------------------------- 用例 5
    print()
    print("=" * 74)
    print("用例 5：OFFBOARD 确认超时 → 中止 + 降落（不会盲目前进）")
    print("=" * 74)
    n = new_node(lib)
    fx = FakePX4(n)
    fx.p = [0.0, 0.0, 10.0]
    fx.sync()
    fx.allow_offboard = False           # PX4 不认可切换 → 节点应超时中止
    # 让它走到 CONFIRM_OFFBOARD
    for _ in range(2000):
        if n.state == "CONFIRM_OFFBOARD":
            break
        step(n, fx, 1)
    check(n.state == "CONFIRM_OFFBOARD", f"到达 CONFIRM_OFFBOARD（当前 {n.state}）")
    for _ in range(800):
        n.tick()
        fx.advance()
        VT[0] += 0.02
        if n.aborted:
            break
    check(n.aborted, "OFFBOARD 确认超时 → aborted")
    check("OFFBOARD 确认超时" in (n.abort_reason or ""),
          f"原因文案明确：{n.abort_reason}")
    check(n.landing_phase or n.state == "DONE", f"转入收尾（当前 {n.state}）")
    n.close()

    # ---------------------------------------------------------------- 用例 6
    print()
    print("=" * 74)
    print("用例 6：制导核参数确实生效（换 N / a_max 后指令应变化）")
    print("=" * 74)
    res = {}
    for N in (4.0, 8.0):
        n = new_node(lib, N=N)
        fx = FakePX4(n)
        drive_to_guided(n, fx)
        for _ in range(1200):
            step(n, fx, 1)
            if n.state in ("LAND", "DONE"):
                break
        res[N] = (n.min_R, n.peak_a_cmd, n.hit)
        n.close()
    print(f"  N=4 → 最近 {res[4.0][0]:.3f} m，峰值 {res[4.0][1]:.2f}")
    print(f"  N=8 → 最近 {res[8.0][0]:.3f} m，峰值 {res[8.0][1]:.2f}")
    check(res[4.0][2] and res[8.0][2], "N=4 与 N=8 都能命中")
    # 两条都饱和到 a_max，所以比"峰值"没意义 —— 比 **最近距离**（N 确实进了制导核）
    check(abs(res[4.0][0] - res[8.0][0]) > 1e-4,
          f"N 改变了制导结果（最近 {res[4.0][0]:.3f} vs {res[8.0][0]:.3f} m）")

    # ---------------------------------------------------------------- 用例 7
    print()
    print("=" * 74)
    print("用例 7：守备场景巡航到位判据 --cruise-x（PPN/APN 实飞对比的前提）")
    print("=" * 74)

    def _vnorm(v):
        return (v[0] * v[0] + v[1] * v[1] + v[2] * v[2]) ** 0.5

    # --- 7.1 指定 cruise_x：速度达标但**未到位** ⇒ 不应发射 ---
    n = new_node(lib, cruise_x=-4.0)
    fx = FakePX4(n, p_enu=(0.0, 0.0, 10.0))
    x_speed_ok, state_speed_ok, fire_x = None, None, None
    for _ in range(6000):
        step(n, fx, 1)
        if n.state == "GUIDED":
            fire_x = fx.p[0]
            break
        if n.state == "CRUISE" and x_speed_ok is None and _vnorm(fx.v) >= 5.0:
            x_speed_ok, state_speed_ok = fx.p[0], n.state
    check(state_speed_ok == "CRUISE",
          f"速度达标瞬间（|v|≥5 @ x={x_speed_ok:+.2f} m）仍停在 CRUISE —— 未提前发射")
    check(fire_x is not None and fire_x <= -4.0,
          f"最终在 x={fire_x:+.2f} m（≤ −4.0）才发射 —— 位置判据生效")
    n.close()

    # --- 7.2 不指定 cruise_x（旧行为）：速度达标即发射 ---
    n = new_node(lib, cruise_x=None)
    fx = FakePX4(n, p_enu=(0.0, 0.0, 10.0))
    fire_x_old = None
    for _ in range(6000):
        step(n, fx, 1)
        if n.state == "GUIDED":
            fire_x_old = fx.p[0]
            break
    check(fire_x_old is not None and fire_x_old > -3.0,
          f"未指定 --cruise-x ⇒ 速度达标即发射（x={fire_x_old:+.2f} m，远未到 −4）"
          f" —— 向后兼容")
    n.close()

    # --- 7.3 起点已在阈值之外 ⇒ 必须**显式告警**，不静默 ---
    n = new_node(lib, cruise_x=-4.0)
    warns = []
    n.warn = lambda s: warns.append(s)
    fx = FakePX4(n, p_enu=(-6.0, 0.0, 10.0))      # 起点已在 −4 以西
    for _ in range(3000):
        step(n, fx, 1)
        if n.state == "GUIDED":
            break
    check(any("已在巡航阈值" in w for w in warns),
          "起点在阈值之外时给出显式告警（提示重启仿真，不静默出不可比结果）")
    n.close()

    # --- 7.4 巡航段速度被**保持**住（不再一路顶上去） ---
    n = new_node(lib, cruise_x=-4.0)
    fx = FakePX4(n, p_enu=(0.0, 0.0, 10.0))
    v_peak = 0.0
    for _ in range(6000):
        step(n, fx, 1)
        if n.state == "GUIDED":
            break
        if n.state == "CRUISE":
            v_peak = max(v_peak, _vnorm(fx.v))
    check(5.0 <= v_peak < 6.0,
          f"巡航段速度保持有效（峰值 {v_peak:.2f} m/s，目标 5.0、上限 6.0）")
    n.close()

    # --- 7.5 ★ pilot 守备场景离线对比：PPN vs APN ---
    print()
    print("  7.5 pilot 守备场景离线对比"
          "（8 m 高 / 5 m/s 迎击；弹丸 (−14,0,10) 14 m/s 俯角 6°）")
    pilot = dict(target_p0=(-14.0, 0.0, 10.0), target_speed=14.0, target_elev=-6.0,
                 target_height=8.0, v_boost_min=5.0, cruise_x=-4.0)
    pres = {}
    for law in (0, 2):
        n = new_node(lib, law=law, **pilot)
        n.log = lambda s: None          # 静音，保持测试输出整洁
        n.warn = lambda s: None
        fx = FakePX4(n, p_enu=(0.0, 0.0, 8.0))
        for _ in range(8000):
            step(n, fx, 1)
            if n.state in ("LAND", "DONE"):
                break
        mr = n.min_R if n.min_R is not None else float("nan")
        pres[law] = (mr, bool(n.hit))
        n.close()
    for law, nm in ((0, "PPN"), (2, "APN")):
        print(f"    {nm}(law {law}) → 最近 {pres[law][0]:.3f} m  "
              f"{'命中' if pres[law][1] else '未命中'}")
    check(pres[0][1] and pres[2][1], "pilot 场景下 PPN / APN 均能命中")
    check(pres[2][0] < pres[0][0],
          f"APN 脱靶量小于 PPN（{pres[2][0]:.3f} vs {pres[0][0]:.3f} m）"
          f" —— 证明 a_T⊥ 真进了制导核")

    # ---------------------------------------------------------------- 汇总
    print()
    print("=" * 74)
    print("全部通过 ✅" if not FAIL else f"失败 {len(FAIL)} 项 ❌")
    for f in FAIL:
        print("  -", f)
    print("=" * 74)
    return 0 if not FAIL else 1


if __name__ == "__main__":
    sys.exit(main())
