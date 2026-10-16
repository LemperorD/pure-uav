# XTDrone2 接入 · 阶段 A（指令回写通路）✅ + 阶段 B（制导闭环）

> 对应《制导讲义/XTDrone2接入方案.md》§7。
> 本目录是**仿真接入脚本**的家；算法仍在 `src/guidance/`，二者解耦。
>
> · **阶段 A**（§0–§3.6）：指令回写通路 —— **已验收通过**（2026-10-14 第 8 轮全绿）
> · **阶段 B**（§3.7–§3.8）：制导闭环 —— 制导核已接进来，节点已就绪（待实跑）

---

## 0. 这一步要回答什么

| 问题 | 判据 |
| --- | --- |
| 链路通不通？ | `ARM` / `TAKEOFF` / `OFFBOARD` 三次服务都 `success=True`，**且 `vehicle_status.nav_state == 14`（真进了 OFFBOARD）**，发 `cmd_accel_ned` 后无人机真的动 |
| 方向对不对？ | 每个脉冲段的速度增量落在**指令对应的那个轴**上，符号正确 |
| 坐标换算对不对？ | `/odometry`（ENU 真值）与 PX4 的 NED 状态满足 `v_ENU = (vy, vx, -vz)` |

> ⚠ **`OFFBOARD` 服务返回 `success=True` 并不代表 PX4 真进了 OFFBOARD**（XTDrone2 恒返回 True）。
> 唯一可信的判据是订阅 `/<ns>/fmu/out/vehicle_status` 看 `nav_state`（14 = OFFBOARD）。见坑 5。

**注意：本阶段完全不接制导。** 目的是把"环境/接口"问题和"制导"问题彻底分开。

---

## 1. 先起仿真（终端 1）

```bash
source /opt/ros/jazzy/setup.bash
source ~/XTDrone2/install/setup.bash
ros2 launch xtd2_launch ros2_single_vehicle_demo_launch.py
```

等到日志出现 `[commander] Ready for takeoff!`。**另开 QGC**（语言必须英文，否则不能解锁）。

> 只跑这一条 launch —— 不要再单独开 MicroXRCEAgent，也不要跑 `make px4_sitl gz_x500_depth`。

---

## 2. 步骤 A-0：先跑官方样例（可选，30 秒）

官方自带一个加速度测试节点，可用它先确认"路是通的"：

```bash
# 终端 2
source /opt/ros/jazzy/setup.bash
source ~/XTDrone2/install/setup.bash
ros2 run xtd2_test accel_test
```

它会：解锁 → 起飞到 10 m → 切 OFFBOARD → 持续发 `a_NED=(1,0,0)`。
**预期**：无人机向北（Gazebo +y）加速飘走，速度线性增长。

> ⚠️ **但这个官方样例在 ROS2 Jazzy 上大概率会崩。**
> 它在 `position_callback` 里调用了 `rclpy.spin_until_future_complete(...)`，
> 而 Jazzy 的 `SingleThreadedExecutor._enter_spin()` 里有
> `if self._is_spinning: raise RuntimeError('Executor is already spinning')`
> —— 回调体本身就是在该 executor 的 spin 窗口内同步执行的，所以**一到 10 m 切 OFFBOARD
> 就会抛 `RuntimeError: Executor is already spinning`**，节点直接退出。
> 这不是你的环境问题，是官方样例写在更早的 rclpy 上。
>
> 所以：**如果它崩了，直接跳过，跑我们的 A-1 即可**（A-1 已改成完全非阻塞的服务调用）。

按 `Ctrl-C` 结束（无人机此后会保持最后的指令，需在 QGC 里 LAND）。

---

## 3. 步骤 A-1：跑我们的验证节点（定量 + 方向自检）

```bash
# 终端 2
source /opt/ros/jazzy/setup.bash
source ~/XTDrone2/install/setup.bash
cd ~/rm_guidance_sim

# 首次需要：脚本是从 Windows 侧写进 WSL 的，没有执行位（Windows 侧 chmod 对 WSL 无效）
chmod +x run_phase_a.sh phase_a_accel_path.py

./run_phase_a.sh --namespace x500_0
```

> 若报 `Permission denied`，两条等价解法任选：
> ① `chmod +x run_phase_a.sh` 后再 `./run_phase_a.sh`；
> ② 直接 `bash run_phase_a.sh --namespace x500_0`（不需要执行位）。
>
> 也可以完全不用 wrapper：`python3 phase_a_accel_path.py --namespace x500_0`

### 它做了什么

解锁（**并确认 `arming_state==ARMED`**）→ 等**起飞到位** → 请求 OFFBOARD → **确认 PX4 真进了 OFFBOARD（`nav_state==14`）** → 按下面 9 段发指令 → **自动 LAND + 停桨**：

> **解锁确认（坑 6）**：发 ARM 后节点会等到 `vehicle_status.arming_state == 2(ARMED)` 才继续；
> 每 1 s 重发一次，15 s 未确认 → 明确报错（提示查 QGC 预解锁检查项）。
> 日志会打印 `[1/5] ✓ 已解锁（arming_state=ARMED）`。

> **OFFBOARD 确认（坑 5）**：请求 OFFBOARD 后节点会持续流 setpoint 把 offboard 信号养起来，
> 每 0.5 s 重发一次 `DO_SET_MODE(OFFBOARD)`，直到 `vehicle_status.nav_state == 14` 才继续。
> 日志会打印 `[4/5] ✓ PX4 已确认进入 OFFBOARD（nav_state=14，耗时 x.xxs）`。
> 若 10 s 内没进去 → 明确报错中止（不会再像第 4 轮那样"假成功"）。

> **「起飞到位」判据是鲁棒的**（满足任一即可）：
> ① `|alt − 10| ≤ 1.0 m`；② `alt > 7.5 m` 且 `|vz| < 0.4 m/s` 连续保持 1.5 s。
>
> 为什么不能只认 ±0.5 m：实测 PX4 收到 `NAV_TAKEOFF(param7=10)` 后常稳定在 **~9.4 m**
> （不严格等于 10 m）；而且如果仿真里无人机**本来就在空中**（上一轮留下的），
> TAKEOFF 不会让它再爬升。只认 `|alt−10|<0.5` 会**永远不满足 → 超时中止**。
> 用 `--takeoff-tol` 可调容差。

| 段 | a_NED (m/s²) | 时长 | 期望现象 |
| --- | --- | --- | --- |
| hold | (0, 0, 0) | 3 s | 静止（基准） |
| **south** | (−0.5, 0, 0) | 2 s | **vx 变负**（NED x = 北，故这是往南） |
| hold | (0, 0, 0) | 1 s | 速度**保持**（accel-only 无阻尼，这本身是重要结论） |
| north_brake | (+0.5, 0, 0) | 2 s | vx 回到 0 |
| hold | (0, 0, 0) | 2 s | |
| **west** | (0, −0.5, 0) | 2 s | **vy 变负**（NED y = 东，故这是往西） |
| hold | (0, 0, 0) | 1 s | |
| east_brake | (0, +0.5, 0) | 2 s | vy 回到 0 |
| hold | (0, 0, 0) | 3 s | 收尾 |

**为什么用南/西方向**：`walls` 世界里从原点出发，Gazebo 的 `+x` / `+y` 方向
4.5 m 处就有墙（`box1` / `box2`）；而 `−x` / `−y` 方向是净空的。
PX4 NED 的 x=北、y=东，所以 NED 的 `x−` / `y−` 正好对应 Gazebo 的 `−y` / `−x`，安全。

### 预期输出（关键几行 + 末尾汇总表）

```
[1/5] ✓ 已解锁（arming_state=ARMED）
[2/5] TAKEOFF …（目标 10.0 m）
[3/5] 起飞到位：alt=9.4x m, vz=+0.0x m/s（目标 10.0 m）→ 请求 OFFBOARD …
  → 发出 OFFBOARD
  service OFFBOARD  → success=True
      （服务 success 只说明通信节点收到了；是否真进 OFFBOARD 以 vehicle_status 为准）
[4/5] ✓ PX4 已确认进入 OFFBOARD（nav_state=14，耗时 0.5x s）
[5/5] 稳定完毕，开始测试段
  ── 段 1/9  hold ...

阶段 A 响应汇总  (Δv = 段末速度 − 段初速度, NED, m/s)
--------------------------------------------------------------------------------
段            指令 a_NED               Δvx     Δvy     Δvz   判读
--------------------------------------------------------------------------------
hold          (+0.00,+0.00,+0.00)    0.00    0.00    0.00
south         (-0.50,+0.00,+0.00)   -0.9x   0.0x    0.0x   OK  主轴 v0 减
hold          (+0.00,+0.00,+0.00)    0.0x    0.00    0.00
north_brake   (+0.50,+0.00,+0.00)   +0.9x   0.0x    0.0x   OK  主轴 v0 增
...
坐标自检  Δp_ENU(odom) =(...)  Δp_ENU(换算)=(...)  max|err|=0.0xx m  →  一致 OK
航向      start=  8x.x°  end=  8x.x°  Δ= +0.0°
OFFBOARD  确认=是   末态 nav_state=14（14=OFFBOARD）
结论：**通路 OK** — 4/4 个脉冲段主轴响应正确，cmd_accel_ned 可用。
→ 进入自动降落流程：LAND（AUTO_LAND 下降 + cmd_vel_ned）→ 落地后 DISARM 停桨…
✓ 已落地：alt=0.20 m, |vz|=0.034 m/s（判据：alt≤0.30 m 且 |vz|≤0.08 m/s 持续 1s）→ 停桨 DISARM
      DISARM 已发（arming_state=2）
✓ 已停桨（arming_state=DISARMED）。
收尾完成：已落地并停桨。节点退出。
```

> 若看到 `OFFBOARD 确认=否` 或 `结论：**PX4 未进入 OFFBOARD**`，说明坑 5 又回来了 ——
> 去查 `ros2 topic echo /x500_0/fmu/out/vehicle_status --once` 的 `nav_state`。
> 若看到 `爬升中… arming_state=1`，说明坑 6（没解锁）。
> 若看到 `⚠ 普通 DISARM 无效 → 改用【强制停桨】`，说明坑 8 又回来了（但兜底已生效）。

同时会在当前目录生成 `phase_a_log.csv`（20 Hz，含 NED 状态 + ENU 真值 + 指令）。

### 安全

- 脉冲很小（0.5 m/s² × 2 s），最大水平位移约 3 m。
- 自动保护：离地高度 < 3 m、或水平位移 > 8 m → **立即中止并转入降落流程**。
- 跑完（或按一次 Ctrl-C）会自动 **LAND + 停桨**，见 §3.6。
- **强制停桨（param2=21196）只是安全兜底**：仅在"普通 DISARM 连续 10 s 无效"
  且"`alt ≤ 0.30 m` 且 `|vz| ≤ 0.08 m/s`（已停在地面）"时才触发；
  一旦离开该判据立刻停止发送。不放心可用 `--no-force-disarm` 关掉。

---

## 3.5 实测结论与八个坑（2026-10-14）

> 编号与 `phase_a_accel_path.py` 顶部文档**完全一致**：
> 坑 1 = 回调里 spin、坑 2 = 起飞判据、坑 3 = 掉高、坑 4 = yaw 转正北、
> 坑 5 = OFFBOARD 静默失败、坑 6 = ARM 未确认（卡在 AUTO_TAKEOFF）、
> 坑 7 = `SignalHandlerOptions` 成员名跨版本不同（一启动就崩）、
> 坑 8 = 落得了地却停不了桨（NaN 速度 setpoint 打死落地检测）。

### ✅ 阶段 A 验收通过（第 8 轮 · 全绿 · 2026-10-14）

```
[1/5] ✓ 已解锁（arming_state=ARMED）
[2/5] TAKEOFF …（目标 10.0 m）
[3/5] 起飞到位：alt=9.61 m → 请求 OFFBOARD …
[4/5] ✓ PX4 已确认进入 OFFBOARD（nav_state=14，耗时 0.05s）
[5/5] 稳定完毕，开始测试段
 ...
结论：**通路 OK** — 4/4 个脉冲段主轴响应正确，cmd_accel_ned 可用。
```

| 段 | 指令 a_NED | Δv（实测） | 理论 | 判读 |
| --- | --- | --- | --- | --- |
| south | (−0.5, 0, 0) | Δvx = **−0.839** | −1.0 | OK |
| north_brake | (+0.5, 0, 0) | Δvx = **+0.917** | +1.0 | OK |
| west | (0, −0.5, 0) | Δvy = **−0.817** | −1.0 | OK |
| east_brake | (0, +0.5, 0) | Δvy = **+1.013** | +1.0 | OK |

差值来自加速度指令生效延迟。**坐标自检 `max|err| = 0.025 m` → 一致 OK。**
`hold` 段 Δv ≈ 0 再次确认 **accel-only 无阻尼**。

**自动收尾一次通过（本轮新验证）**：9.61 m → 0.25 m 下降约 12 s（`vz ≈ 0.70 m/s`，全程 `nav_state=18`）；
落地判据一次触发、`DISARM` 第 2 次即被接受 —— **强制兜底未动用**（坑 8 修法验证有效）。全程 ~50 s，无人工干预。

> ⚠ 遗留：航向仍被转到正北（`60.3° → 0.0°`，Δ=−60.2°）—— 坑 4 未修，
> **阶段 C（相机）前必须打补丁**。阶段 A/B 用世界系指令，不受影响。

### 阶段 A 四条结论

- ✅ `ARM` / `TAKEOFF` / `OFFBOARD` 三个服务全 `success=True`，且发 `cmd_accel_ned` 后**真的会动**。
- ✅ **轴向**：`a_NED=(-0.5,0,0)` → `Δvx = -0.839 m/s`（2 s × 0.5，理论 −1.0）⇒ `linear.x` = NED 北，负 = 南。
- ✅ **`accel-only` 是纯前馈、无阻尼**：脉冲后紧跟的 `hold` 段 `Δvx ≈ 0`，速度原样保持。
- ✅ **坐标换算 `p_ENU=(y,x,-z)`** 用 odom 的**位置增量**交叉验证，`max|err| = 0.025 m`。

### 坑 8：落得了地，却停不了桨 —— NaN 速度 setpoint 打死落地检测 🔴🔴

**现象（第 7 轮）**：无人机**确实落在地面**，但 PX4 死活不认，`DISARM` 被连拒 20 次
（`Disarming denied: not landed`），最后带着桨退出。

CSV 剖面是铁证：

```
t=43.70  alt=0.201  vz=+0.6626   ← 正在以 0.66 m/s 下降（MPC_LAND_SPEED）
t=43.75  alt=0.198  vz=-0.0343   ← 速度瞬间被"吃掉" = 触地！
t=43.75→63.69  alt 0.198~0.229  |vz|<0.04   ← 在地面待了 20 秒
```

**根因**（`MulticopterLandDetector.cpp:222-240`）：

```cpp
if (_flag_control_climb_rate_enabled) {
    if (_trajectory_setpoint_sub.update(&ts)) {
        _in_descend = PX4_ISFINITE(ts.velocity[2])
                      && (ts.velocity[2] >= 1.1f * LNDMC_Z_VEL_MAX);   // 阈值 0.275 m/s
    }
    if (!_maybe_landed_hysteresis.get_state() && !_landed_hysteresis.get_state()) {
        ground_contact &= _in_descend;      // ← 必须"正在被命令下降"
    }
}
```

两个发布者在抢同一个 `trajectory_setpoint` topic：

| 发布者 | 内容 | 频率 |
| --- | --- | --- |
| `FlightModeManager`（AUTO_LAND） | `velocity[2] = MPC_LAND_SPEED ≈ 0.7` ✅ | 高频 |
| **XTDrone2**（accel 模式） | `velocity = [NaN, NaN, NaN]` ❌ | **20 Hz，永不停止** |

`self.cmd` 一旦被设置，XTDrone2 就**永久重发**它 ⇒ 判据被打成抖动
⇒ `ground_contact` 凑不满 hysteresis（`LNDMC_TRIG_TIME/3 = 0.33 s`）
⇒ `maybe_landed` 永不成立 ⇒ `landed` 永不成立 ⇒ **`DISARM` 被拒**。

**修法（本节点已内置）**：

1. **降落阶段改发 `cmd_vel_ned`**（`linear.z = +0.5`，NED 正 = 向下），
   让两个发布者的信号一致。副作用（好的）：`cmd_vel_ned_callback` 里本来就有
   `cmd.yaw = math.nan`，所以切过来后机头不再被强制拧向正北。
2. **兜底强制停桨**：若普通 `DISARM` 在 `FORCE_DISARM_AFTER`(10 s) 内仍无效，
   且我们的落地判据持续满足，就发一次 `param2 = 21196`（跳过落地检查）。
   用 `--no-force-disarm` 可禁用。
3. 全程有**安全闸**：一旦离开"已落地判据"（`alt ≤ 0.30 m` 且 `|vz| ≤ 0.08 m/s`），
   立刻停止发送任何停桨指令。

> **对阶段 B/C 的启示**：`cmd_accel_ned` 模式的 `velocity=NaN` setpoint 会**永久污染
> `trajectory_setpoint` topic**。凡是依赖该 topic 的 PX4 功能（落地检测、部分 failsafe）
> 都可能受影响。阶段 B 若仍用 accel 模式，收尾时务必照此处理；或者干脆改用 `cmd_vel_ned`。

### 坑 4：`cmd_accel_ned` 会把无人机转到正北（XTDrone2 的 bug）🔴

`xtd2_communication/multirotor_communication.py` 的两个 accel 回调**没设 `cmd.yaw`**
（源码里留着 `#TODO: How about yaw?`），而 ROS2 里 `float32 yaw` 的默认值是 **0.0**（不是 NaN）
⇒ PX4 收到"航向 = 正北"。实测 `heading 85.5° → 8.2° → 0.6°`，机头被强行拧到正北。

**修法**（脚本已备好）：

```bash
python3 ~/rm_guidance_sim/patch_xtdrone2_yaw.py --dry-run   # 先看 diff
python3 ~/rm_guidance_sim/patch_xtdrone2_yaw.py             # 写入（自动备份 .bak）
```

会同时修源码和**已安装副本**（后者才是实际加载的），不需要重新 `colcon build`。
阶段 A/B 用世界系加速度，**不修也能跑**；**阶段 C（相机闭环）必须修** —— 机头恒指正北就没法看目标。

### 坑 3：`accel-only` 没有垂直反馈，`a_z=0` 会持续掉高 🔴

实测进入 OFFBOARD 后 `vz≈+0.5 m/s`，并以约 **0.065 m/s²** 加速下沉：8.3 m → 7.0 → 5.2 → 2.98 m（触发安全中止）。

本节点因此默认开启**垂直高度保持辅助**：对 `vehicle_local_position` 做 PD，只补 `a_z` 一项
（`a_z += kp*(alt-alt_target) - kd*vz`，默认 `kp=1.0, kd=2.0`，临界阻尼）。
用 `--no-alt-hold` 可关。

> ⚠ 这是**测试夹具，不是制导**。它说明了一件对阶段 B 很重要的事：
> `cmd_accel_ned` 模式下垂直方向没有闭环，**制导律必须自己补垂直分量**，否则会掉高。

### 坑 5：OFFBOARD 模式切换是「一次性、不排队」的 → 不确认就静默失败 🔴🔴

**现象（第 4 轮）**：9 段全跑完，但 **4 个脉冲段 Δv 全 ≈ 0.00x m/s**，航向也没转（Δ=−4.3°）。
CSV 实证 `t=16.4→31.8 s`：高度稳定 10.11~10.19 m、速度 < 0.06 m/s、位置 ±0.07 m
⇒ 无人机在**用 PX4 自身位置环悬停**，`cmd_accel_ned` 被**完全忽略**。

**PX4 侧源码证据链**：

| 文件:行 | 事实 |
| --- | --- |
| `HealthAndArmingChecks/checks/offboardCheck.cpp:49` | `offboard_control_signal_lost` 只有在收到**近期**（< `COM_OF_LOSS_T`，默认 1 s）且**至少一个控制位为 true** 的 `OffboardControlMode` 时才为 false |
| `ModeUtil/mode_requirements.cpp:132` | OFFBOARD 带 `mode_req_offboard_signal` 要求 |
| `failsafe/framework.cpp:677` | `modeCanRun()` 因此要求 `!offboard_control_signal_lost` |
| `UserModeIntention.cpp:59-85` | 已解锁时若 `canRun()` 为假 → `change()` **直接返回 false，不排队、不重试** |

**XTDrone2 的时序恰好踩中**：`cmd_callback("OFFBOARD")` 在**同一瞬间**既把 `OFFBOARD_STATE`
翻成 `ENABLED`、又发出唯一一次 `DO_SET_MODE`；而此时 `self.cmd` 仍为 `None`，
`timer_callback` 因 `if not self.cmd: return` **还没发过任何 `OffboardControlMode`**。
⇒ PX4 处理模式切换时信号为"丢失" ⇒ **永久拒绝**。

**修法（本节点已内置）**：新增 `CONFIRM_OFFBOARD` 状态 ——

1. 发完 `OFFBOARD` 服务后**持续流 setpoint**，把 offboard 信号养起来；
2. **每 0.5 s 直接向 `/<ns>/fmu/in/vehicle_command` 重发 `DO_SET_MODE(OFFBOARD)`**（绕过 XTDrone2，不动它的状态机）；
3. **订阅 `vehicle_status` 直到 `nav_state == 14`** 才进入测试段；超过 10 s 未确认则明确报错中止。

> ⚠ `OFFBOARD` 服务返回的 `success=True` **不代表** PX4 真进了 OFFBOARD（XTDrone2 恒返回 True）。
> 以后一律以 `vehicle_status.nav_state` 为准。

### 坑 6：`ARM` 服务成功 ≠ 真解锁 → 无人机「卡在 AUTO_TAKEOFF 不涨高度」🔴🔴

**现象（2026-10-14 19:22 那次）**：CSV 显示 `nav_state` 全程 **17（AUTO_TAKEOFF）**、
高度 **0.56 → 0.62 m 纹丝不动**、`seg` 一直是 `pre`，33 s 后被 Ctrl-C。
即节点停在 `WAIT_HEIGHT`，起飞永远不"到位"。

**根因**：和坑 5 是同一类 —— **服务返回 `success=True` 只说明通信节点收到了**，
完全不代表 PX4 接受（XTDrone2 的 `cmd_callback` 无条件 `response.success = True`）。
`VEHICLE_CMD_NAV_TAKEOFF` 在 PX4 侧同样只做 `change(AUTO_TAKEOFF)`（`Commander.cpp:1035`，
没有额外护栏），所以"模式切过去了"**不代表车能飞**：
**AUTO_TAKEOFF + DISARMED 不会产生推力** —— 也就是根本没解锁。

**修法（本节点已内置）**：

1. 新增 `CONFIRM_ARM` 状态：发 ARM 后**以 `vehicle_status.arming_state == 2(ARMED)` 为准**，
   未解锁则每 1 s 重发，15 s 未确认 → 明确报错（提示去查 QGC 预解锁检查项）。
2. `WAIT_HEIGHT` 期间每 3 s 打印 `nav_state / arming_state`；若 `arming_state==DISARMED`
   或 `nav_state≠17` 会**主动告警**。
3. CSV 新增 `nav_state` + `arming_state` 两列。

> **通用原则：服务返回 success ≠ 生效。凡是关键状态，一律订阅 `vehicle_status` 确认。**
> 已按此确认的三处：`arming_state==2`（解锁）、`nav_state==14`（OFFBOARD）、`arming_state==1`（停桨）。

---

## 3.6 跑完自动降落 + 停桨（默认开启）

测试段跑完、或安全中止、或**按一次 Ctrl-C**，节点都会自己收尾：

```
RUN ──► LAND（发 NAV_LAND → PX4 切 AUTO_LAND 下降；同时改发 cmd_vel_ned 的下降速度）
    ──► DISARM（落地后发 ARM_DISARM=0 → 停桨）
    ──► DONE（linger 3 s 后自动退出）
```

**为什么必须这样做**：`accel-only` 是**纯前馈、无阻尼** —— 只把加速度归零的话，
无人机会**保持原速度一直漂**，非常危险。

**PX4 侧依据**：

| 事实 | 出处 |
| --- | --- |
| `NAV_LAND` 只做 `change(AUTO_LAND)`，**无 offboard 信号护栏** ⇒ 在 OFFBOARD 里也能切出去 | `Commander.cpp:1064` |
| `AUTO_LAND` 是内部模式、`replaces_nav_state == NONE`，不会被 ModeManagement 替换 | `ModeManagement.cpp:468` |
| **降落阶段必须改发 `cmd_vel_ned` 的下降速度**，否则 PX4 不认落地 | `MulticopterLandDetector.cpp:222-240`（见坑 8） |
| `DISARM` 在**未落地时被 PX4 直接拒绝**（除非 param2=21196 强制）⇒ 可放心周期重发 | `Commander.cpp:641-650` |
| 落地后 PX4 自己也会 2 s 自动停桨（`COM_DISARM_LAND`），我们主动发只是"确保" | — |

> ✅ **本流程已于 2026-10-14 第 8 轮实测通过**：9.61 m → 0.25 m 下降约 12 s（`vz ≈ 0.70 m/s`），
> 落地判据一次触发，`DISARM` 第 2 次即被接受（**强制兜底未动用**），全程 ~50 s 无人工干预。

**"已落地"判据**（我们自己算的，用于决定何时开始发 DISARM）：

```
alt ≤ LANDED_ALT(0.30 m)  且  |vz| ≤ LANDED_VZ(0.08 m/s)  连续 LANDED_HOLD(1.0 s)
```

比 PX4 自己的判据宽松，但要求"速度也小"，能可靠识别**触地那一下的突变**
（实测：`vz` 从 +0.662 瞬间变成 −0.034）。

**开关**：

| 参数 | 作用 |
| --- | --- |
| `--no-auto-land` | 跑完不降落（保持 0 加速度；⚠ 会继续漂，需 QGC 手动 LAND） |
| `--no-disarm` | 落地后不主动发 DISARM（靠 PX4 自动停桨） |
| `--no-force-disarm` | 禁用兜底的**强制停桨**（默认允许，见坑 8） |
| `--land-timeout` | 降落总超时，默认 90 s；超时会明确告警并提示去 QGC 处理 |
| `--linger` | 收尾完成后多少秒自动退出，默认 3 s |

**Ctrl-C 语义**：**按一次 = 降落 + 停桨再退出；连按两次 = 强制退出（跳过收尾）**。

### 离线回归测试（不需要 ROS / 仿真）

```bash
python3 test_phase_a_states.py     # 退出码 0 = 全过
```

用桩替换 rclpy / PX4 消息，直接驱动状态机，覆盖 9 个用例：正常收尾、Ctrl-C 收尾、
DONE 下 Ctrl-C、`--no-auto-land`、降落超时、OFFBOARD 确认失败、**ARM 未确认（坑 6）**、
**SIGINT 枚举名探测（坑 7）**、**`main()` 启动路径冒烟**。
Windows 侧也能直接跑。

### 坑 7：`SignalHandlerOptions` 成员名跨版本不同 → **一启动就崩** 🔴

为了让 Ctrl-C 走"降落 + 停桨"收尾，得让 rclpy **不要**装它自己的 SIGINT handler。
写法是 `rclpy.init(signal_handler_options=SignalHandlerOptions.???)` ——
而这个"不装"成员的名字**各版本不一样**：

```python
# rclpy/__init__.py（Jazzy）第 82 行：
signal_handler_options = SignalHandlerOptions.NO     # ← 是 NO，不是 NONE
```

写死 `NONE` 会直接崩在启动阶段（连解锁都不会开始）：

```
AttributeError: type object 'rclpy._rclpy_pybind11.SignalHandlerOptions'
has no attribute 'NONE'
```

**修法**：`resolve_no_signal_handler_option()` 按名字动态探测
（`NO` / `NONE` / `None` / `none` / `No`），**注意 `NO` 的值是 `0`，
判断必须用 `is not None`**；解析不到就退回 `rclpy.init()` + `uninstall_signal_handlers()`。

> 这个坑已被 `test_phase_a_states.py` 的**用例 8 + 用例 9** 永久钉住：
> 用例 8 验证枚举探测逻辑（含"值为 0 不能用真值判断"），
> 用例 9 直接跑 `main()` 启动路径。把 `NONE` 写回去，用例 9 会立刻报出那条 `AttributeError`。

### 自检为什么之前报"不一致"

`nav_msgs/Odometry` 的 `twist` 按约定在 `child_frame_id`（**机体系 FLU**），只有 `pose` 在世界系。
在 heading≈0° 处实测 `odom.twist = (-0.878, 0.014, -1.077)` ≈ 机体 FLU 预测 `(-0.885, -0.023, -1.067)` —— 完全对上。
所以换算公式没错，是自检用错了字段。现已改成用 **pose 增量**比对，并额外打印**航向漂移**。

---

## 3.7 阶段 B 预备：制导核 shim（C++ → Python，ctypes）

阶段 B 要在 ROS 节点里跑**真** `guidance_libs`。做法：把它包一层薄 C ABI
（`pure_guidance_shim.cpp`，只做数据搬运、**不含任何算法**），编成动态库，
Python 用 ctypes 调用。制导 / 传感器墙 / 目标模型全部是仓库里的真代码。

| 文件 | 作用 |
| --- | --- |
| `pure_guidance_shim.cpp` | C ABI 封装（数组布局见文件头注释） |
| `build_guidance_shim.sh` | WSL 侧一键编译 → `libpure_guidance.so` + 自检 |
| `test_guidance_shim.py` | 冒烟 + **等价性测试**（不需要 ROS / 仿真） |
| `build_guidance_shim_mingw.sh` | Windows/MSYS2 侧本地回归（改过 `guidance_libs` 先在本机验） |

**本机已验证（2026-10-14）**：复刻 `pn_sim` 默认场景（PPN N=6 · DART 迎头 · IDEAL）
逐行对比 —— **1219 行 × 25 列最大偏差 = 0**，即 shim 链路与仓库离线仿真**逐位一致**；
命中（最近距离 **0.012 m** @ t=1.21 s）、饱和 5/1218 步、末端 `HOLD` 冻结正常、
SEARCH 分支零指令。

**WSL 侧使用**：

```bash
cd ~/rm_guidance_sim
bash build_guidance_shim.sh     # 编译 .so 并自动跑自检（应打印“全部通过 ✅”）
```

> `guidance_libs/` 是源码编译副本：脚本会优先从 `/mnt/c` 仓库刷新；
> 读不到时用助手同步过来的本地副本（同样与仓库一致）。

---

## 3.8 阶段 B：制导闭环（PPN + 真 DART 目标）

**阶段 A 验证了"指令能写下去"；阶段 B 验证"制导律真的能飞"。**

链路变了：不再是发常值脉冲，而是

```
PX4/Gazebo 真值（无人机，真动力学）
      ↓  NED → ENU
  真 guidance_step（仓库的 C++ 制导核，经 .so 调用）
      ↓  a_cmd（ENU）
  ENU → NED → cmd_accel_ned → PX4 → 无人机
```

### 两个关键设计

**① 目标是数学模型，不是 Gazebo 实体。**
目标轨迹由仓库的 `TargetModel`（真 C++，含 DART 气动）按时间推进——它只依赖 `t`，
确定性、可复现。**无人机则是真的在 PX4 里飞**（真姿态环、真限幅）。
⇒ 制导闭环是真的，只是"目标"是虚拟的；命中判定用数学距离 `R_true`。
好处：不用写 Gazebo 插件、飞镖动力学直接复用仓库代码、跑一次就复现。
**阶段 C 引入相机后，目标才会变成 Gazebo 里的真实体。**

**② 必须先"预加速"（CRUISE 段），不能从悬停直接开制导。**
PPN 的指令是 `a = N·(ω × v_M)`——**`v_M = 0` 时恒为零**（零速死区）。
而且悬停起步时无人机没有横向速度可用于修正，交会时间又只有零点几秒 → 必脱靶。
所以像真实比赛一样：先飞起来迎击，有速度后再交给制导。
（`--no-cruise` 可关掉，用来复现这个失败案例——离线测试里已验证：悬停起步最近 1.07 m，脱靶。）

### 默认场景

全在 **y=0 走廊**里（`walls` 世界的墙都在 y=±5/±10，此走廊净空；朝 −x 迎击）：

```
无人机: 从 (0,0,10) 起飞，朝 −x 预加速到 5 m/s
飞镖  : 从 (−12,0,11) 以 15 m/s、仰角 5° 朝 +x 发射
制导  : PPN  N=6  a_max=10 m/s²（x500 真实水平过载量级）  t_go_freeze=0.10
命中判定: R_true < 0.5 m
```

> **这套参数是离线扫出来的**，而且扫参暴露了一条硬约束，务必记住：
>
> ⚠ **飞镖的飞行距离必须在 10 m 量级。** 飞镖同时受重力+气动阻力，弹道持续下坠。
> 若把发射点放到 22 m 外，飞镖要飞 1.5 s 以上，期间**下坠十几米**——
> 那时无论 PN 怎么调都够不着（实测最近距离 0.8~3.9 m，全部脱靶）。
> 真实 RM 里飞镖只需飞 5~10 m，**10 m 级才是对的尺度**。
>
> ⚠ 无人机的横向机动能力 = `½ · a_max · t_go²`。`t_go=0.4 s`、`a_max=10` 时**只能修 0.8 m**
> ⇒ 交班时的初始偏差必须控制在 ~1 m 量级，这靠**场景几何**（共线迎头）保证，不是靠算法。

### 状态机

```
ARM → CONFIRM_ARM → TAKEOFF → WAIT_HEIGHT
    → OFFBOARD → CONFIRM_OFFBOARD → SETTLE
    → CRUISE（预加速）→ GUIDED（目标发射，PPN 闭环）
    → [命中 / 脱靶 / 超时 / 安全中止] → LAND → DISARM → DONE（退出）
```

与阶段 A 同源的四条铁律（都在阶段 A 踩过坑）：
服务 success ≠ 生效（一律订阅 `vehicle_status` 确认）／回调里不许 spin／
降落阶段改发 `cmd_vel_ned`（坑 8）／`SignalHandlerOptions` 成员名动态探测（坑 7）。

### 跑起来

```bash
cd ~/rm_guidance_sim
bash build_guidance_shim.sh          # 若还没编译制导核（只需一次）
bash run_phase_b.sh --namespace x500_0
```

预期关键日志：

```
[1/6] ✓ 已解锁（arming_state=ARMED）
[4/6] ✓ PX4 已确认进入 OFFBOARD（nav_state=14，耗时 x.xxs）
[5/6] 稳定完毕 → 进入预加速
  ── CRUISE 完成：|v|=5.1x m/s（耗时 1.xx s） @ (x, y, z)
[6/6] ── 目标发射！进入 GUIDED（PPN 闭环）
★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
★ 命中！R_true=0.xxx m（阈值 0.5 m） @ t_rel=0.xxx s
★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★★
✓ 已落地：alt=0.2x m → 停桨 DISARM
✓ 已停桨（arming_state=DISARMED）。
收尾完成：已落地并停桨。节点退出。
```

### 离线工具（不需要 ROS / 仿真 / Gazebo，Windows 侧也能跑）

| 命令 | 作用 |
| --- | --- |
| `python3 test_phase_b_states.py` | **状态机 + 真制导核**的离线回归（桩替 ROS、假 PX4 质点）。6 用例 22 断言 |
| `python3 sim_phase_b_offline.py` | **纯离线弹道仿真**：扫场景参数、看轨迹（`--csv` 导出） |

```bash
# 例：试试不同场景（离线先验证，再进仿真）
python3 sim_phase_b_offline.py --p0 "-12,0,11" --target-speed 15 --target-elev 5
python3 sim_phase_b_offline.py --no-cruise          # 复现零速死区
```

**本机已验证（2026-10-14）**：6 用例 22 断言全过；完整闭环**命中 0.272 m**、
峰值指令 10.00 m/s²（未超 a_max）、末端 `HOLD` 冻结正常；
并做了**反向验证**——把 ENU→NED 换算故意写反，测试立刻 FAIL（证明该测点有效）。

### 安全

- 只发制导量级的加速度（≤ `a_max`，默认 10 m/s²，比阶段 A 的脉冲更温和）
- 安全围栏：离地 < 3 m、水平位移 > 25 m、预加速超时 → 立刻中止并转入降落
- 跑完/中止/Ctrl-C（一次）→ 自动 LAND → 停桨 → 退出；Ctrl-C 两次 = 强制退出

---

## 3.9 守备拦截：场景模型 + 巡航态初段仿真（2026-10-14 新增）

> 背景：需求已澄清 —— 真实任务不是"追一发弹丸"，而是**守住防御塔**；
> 无人机**一直在守备巡航**，弹丸从远处（~22 m）打来。
> 完整规划见《制导讲义/守备拦截系统规划.md》。
> **决策层（威胁判定 / 拦截窗口 / t_start* / 巡航策略）已挂起，列为后续任务**；
> 本阶段只做两件事：**① 把场景实体模型建好；② 制导层的初步仿真**。

### 新增文件

| 文件 | 作用 |
| --- | --- |
| `scene_model.py` | **场景实体模型**：塔 / 发射场 / 弹丸 / 无人机 / 场地（纯数据+几何，零依赖） |
| `sim_phase_b_offline.py --mode pilot` | **巡航态 + 弹丸 + 立即拦截** 的离线仿真（本次新增的第三种模式） |

### 防御塔模型（`scene_model.Tower`）

建模为**实心圆柱**，核心是"点到塔体的最近距离"（已自检 8/8）：

```python
t = Tower(center=(0,0,0), radius=1.2, height=3.0, safety_margin=1.0)
t.distance_to_body(p)            # 到塔表面的最近距离；0 = 打进塔里
t.closest_approach(traj)         # 一条弹道到塔的最近距离
t.is_threatened_by(traj)         # 最近距离 < safety_margin → 构成威胁
```

`main()` 里可以直接看预设场景摘要：

```bash
python3 scene_model.py pilot
python3 scene_model.py                 # 列出可用预设
```

### pilot 模式：无人机巡航态 + 立即拦截

对应需求原话：
> "先让无人机飞起来并以一定速度巡航的基础上，给一个较近的弹丸（考虑空气阻力，重力等）
> （速度，方向先给定）（**立马开始拦截**，假定已经判断在这个时刻开始拦截）"

**默认场景**（可通过 CLI 或场景 JSON 改）：

```
塔    : 原点，半径 1.2 m，高 3 m，安全垫 1.0 m
无人机: 从 (−4, 0, 8) 朝 −x（迎击方向）巡航，5 m/s   ← 已经在飞
弹丸  : 从 (−14, 0, 10) 以 14 m/s、俯角 6° 朝 +x 飞向塔（DART：重力+二次阻力）
制导  : t=0 弹丸出现 → **立即开始制导**
```

```bash
cd ~/rm_guidance_sim
python3 sim_phase_b_offline.py --mode pilot                 # PPN（项目默认）
python3 sim_phase_b_offline.py --mode pilot --law 2         # APN
python3 sim_phase_b_offline.py --mode pilot --csv t.csv     # 导出轨迹
```

### ★ 两个重要结论（离线扫参得出）

**① 修掉了一个统计口径 bug。** 原来"命中就立刻停"，导致统计到的"最近距离"只是
"刚跌破 `hit_radius` 的那个采样点"——真实脱靶量被**截断掩盖**（曾误以为精度卡在 0.5 m 上不去）。
现已改为**跑到真正最近点再停**，因此下面的数字是**真实脱靶量**：

| 导引律 | 真实脱靶量 | 峰值指令 | 限幅 |
| --- | --- | --- | --- |
| PPN（默认） | **0.392 m** | 10.0 m/s² | 2 步 |
| TPN | 0.544 m | 10.0 m/s² | 8 步 |
| **APN** | **0.018 m** ← 好 20 倍 | 10.0 m/s² | 1 步 |

**② 这个场景里 APN 显著优于 PPN，原因是物理的。**
弹丸是**持续加速目标**（重力 −9.8 m/s² + 气动阻力），而 PPN 的指令
`a = N·(ω × v_M)` **不含目标加速度项**；APN 的 `+(N/2)·a_T⊥` 恰好补上这一块。
在 ~0.5 s 的交会里，重力造成的位移有 ~1.2 m 量级 —— 这正是 PPN 那 0.39 m 脱靶的来源。

> 理想执行（`tau=0`）时对比更明显：APN 只需 **3.7 m/s²**（不触发限幅）就能把脱靶压到 **0.0028 m**。

⇒ **阶段 C / 真机建议评估改用 APN**（需要估计 `a_T⊥`，见《待决问题.md》）。
另：**塔威胁评估**同时确认了该弹丸"若不拦截会打到塔"（最近距塔 0.00 m @ t=1.06 s），
所以这次拦截是**有意义的**（不是拦一个本来打不中的弹丸）。

### 待办（已挂起）

按用户指示，**决策层全部列为后续任务**（见《守备拦截系统规划.md》T2~T9）：
威胁判定 / 拦截窗口 / `t_start*` 决策函数 / 守备巡航策略 / 预置位机动 / 多方向加权最优驻留。

---

## 4. 请回报给我什么

**阶段 A 已完成** ✅（第 8 轮全绿，见 §3.5）；**阶段 B 节点已就绪、待实跑**（§3.8）；
**守备拦截的场景模型 + pilot 初段仿真已完成**（§3.9）。

跑完 `bash run_phase_b.sh --namespace x500_0` 后，把这三样贴给我：

1. **节点日志**里 `[1/6]`…`[6/6]`、`★ 命中！`、末尾汇总表 + 收尾那几行；
2. `phase_b_log.csv`（直接说文件在哪就行，我去读）；
3. 如果**没命中**：把 `phase_b_log.csv` 给我就够——我要看"最近距离、饱和步数、
   mode 分布、R_true 的收敛过程"，据此判断是场景几何还是制导参数的问题。

---

## 5. 文件说明

| 文件 | 作用 |
| --- | --- |
| `phase_a_accel_path.py` | 阶段 A 验证节点（**本目录是权威副本**） |
| `test_phase_a_states.py` | 阶段 A 状态机离线回归（桩替 rclpy，**不需要 ROS/仿真**） |
| `phase_b_guidance.py` | **阶段 B 制导闭环节点**（见 §3.8） |
| `run_phase_b.sh` | 阶段 B 一键 source + 运行 |
| `test_phase_b_states.py` | 阶段 B 离线回归（桩替 ROS + **真制导核** + 假 PX4 质点） |
| `sim_phase_b_offline.py` | 纯离线弹道仿真（三种模式：cruise / hot / **pilot**；见 §3.8、§3.9） |
| `scene_model.py` | **场景实体模型**：塔 / 发射场 / 弹丸 / 无人机 / 场地（见 §3.9） |
| `apply_update.sh` | **WSL 侧一键换版**：把待启用副本换上去 + 跑离线自检 |
| `patch_xtdrone2_yaw.py` | 修 XTDrone2 的 yaw bug（坑 4） |
| `run_phase_a.sh` | 阶段 A 一键 source + 运行 |
| `pure_guidance_shim.cpp`、`build_guidance_shim.sh`、`test_guidance_shim.py` | 制导核 C ABI 封装 + 编译/自检（见 §3.7） |
| `build_guidance_shim_mingw.sh` | Windows/MSYS2 侧本地回归（可选，开发机用） |
| `README.md` | 本文 |

> WSL 里的 `~/rm_guidance_sim/` 是**运行副本**（避免中文路径）。
>
> ⚠ 从 Windows 侧写入的文件在 WSL 里**没有执行位**，每次同步后都要 `chmod +x`；
> 或者干脆用 `bash run_phase_a.sh` / `python3 phase_a_accel_path.py` 绕开执行位。
>
> ⚠ **若节点还在跑（或句柄未释放）**，从 Windows 侧覆盖会 `Permission denied`。
> 这时助手会把新版本写成另一个文件名的副本，你只需在 WSL 里跑一次：
>
> ```bash
> cd ~/rm_guidance_sim
> bash apply_update.sh
> ```
>
> 它会：清 `__pycache__` → 挑**最新**的待启用副本换到正式文件名 → 清理过期副本
> → 补执行位 → **跑离线自检** → 打印下一步命令。
>
> 等价的手工做法：
>
> ```bash
> cd ~/rm_guidance_sim
> rm -rf __pycache__
> mv -f phase_a_v6.py phase_a_accel_path.py        # 文件名以助手实际给出的为准
> chmod +x phase_a_accel_path.py
> python3 test_phase_a_states.py
> ```
>
> **同步前建议先 Ctrl-C 停掉节点**，能省掉这一整套麻烦。
