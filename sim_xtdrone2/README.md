# XTDrone2 接入 · 阶段 A（指令回写通路）✅ + 阶段 B（制导闭环）✅

> 对应《制导讲义/XTDrone2接入方案.md》§7。
> 本目录是**仿真接入脚本**的家；算法仍在 `src/guidance/`，二者解耦。
>
> · **阶段 A**（§0–§3.6）：指令回写通路 —— **已验收通过**（2026-10-14 第 8 轮全绿）
> · **阶段 B**（§3.7–§3.8）：制导闭环 —— **已验收通过**（2026-10-16 实跑命中 0.318 m）
> · **场景建模 + 可视化**（§3.9–§3.10）：塔/发射场/弹丸/场地模型 + Python 画图 —— 已完成
> · **阶段 C**（相机）：未开始，**必须先打 yaw 补丁**（坑 4）

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

### 起仿真前先预检（推荐）

```bash
cd ~/rm_guidance_sim
bash apply_update_v4.sh          # 若有待换版副本，先换版（会自动跑离线自检）
bash preflight.sh --wait         # 检查环境/文件/制导核，并等仿真就绪（最多 120 s）
```

`preflight.sh` 会逐项打 ✓/✗：ROS2 + XTDrone2 是否 source 得到、节点与离线工具是否齐全、
`libpure_guidance.so` 是否存在（缺了就顺手编译）、有没有还没换版的 `_vN` 副本，
最后轮询 `/<ns>/fmu/out/vehicle_status` 确认仿真真的起来了。

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

### ★★ 实跑验收通过（2026-10-16）

在 WSL 里实跑 `bash run_phase_b.sh --namespace x500_0`，**一次命中、全流程无人工干预**：

```
[1/6] ✓ 已解锁（arming_state=ARMED）
[2/6] TAKEOFF …（目标 10.0 m）              → 爬升到 9.63 m
[3/6] 起飞到位 → 请求 OFFBOARD …
[4/6] ✓ PX4 已确认进入 OFFBOARD（nav_state=14，耗时 0.56s）
[5/6] 稳定完毕 → 进入预加速
  ── CRUISE 完成：|v|=5.05 m/s（耗时 1.10s） @ (-2.23, 0.08, 10.19)
[6/6] ── 目标发射！进入 GUIDED（PPN 闭环）
★ 命中！R_true=0.318 m（阈值 0.5 m） @ t_rel=0.456s，命中时 |v_M|=6.02 m/s
  最近距离 0.318 m / 峰值指令 10.00 m/s² / 饱和 1÷23 步（4.3%）/ mode: PN=18, HOLD=5
✓ 已落地：alt=0.13 m, 下降率=+0.009 m/s → DISARM（第 2 次接受）→ 节点退出
```

⇒ **阶段 B（制导闭环）验收通过**：真 PX4 动力学 + 真 C++ 制导核 + 真 DART 弹丸模型，
从起飞到自动收尾全程自动。

> ⚠ **符号约定（极易踩）**：PX4 的 `vehicle_local_position` 是 **NED**（z 轴朝下），
> 所以 **`vz > 0` = 下降、`vz < 0` = 上升** —— **爬升时 `vz` 为负是正常的**，不是 bug。
> 节点日志现已统一成"上为正"打印（`alt` / `上升率` / `下降率`），启动横幅里也标了约定；
> CSV 里则同时给 `px4_*`（NED）与 `enu_*`（ENU）两套，避免歧义。

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

## 3.10 场景复现 + 可视化（2026-10-16 新增）

> 需求原话："继续场景建模，先用 Python 可视化复现场景（核心代码仍为 C++），
> 然后在 Python 中可视化。"
>
> 分工：**算法一律在 C++**（`guidance_libs` 的制导 + `TargetModel` 的弹丸弹道，
> 经 `pure_guidance_shim.cpp` 的 C ABI 被 Python 调用）；**Python 只做"取数 + 画图"**。

### 新增文件

| 文件 | 作用 |
| --- | --- |
| `viz_scene.py` | **场景复现 + 拦截过程可视化**（matplotlib；不含任何算法） |
| `viz_out/` | 输出目录：2 张 PNG + 可选 GIF + 每条导引律的轨迹 CSV |

### 场景复现已对齐 `walls.sdf` 真实几何

`scene_model.Field` 现在默认带上 **walls.sdf 的 4 面墙**（逐字核对
`~/XTDrone2/src/xtd2_gz_sim/worlds/walls.sdf`，2026-10-16）：

| 墙体 | pose（中心） | size | 实际范围（ENU，z 已裁到地面以上） |
| --- | --- | --- | --- |
| `box1` | (5, 0, 0) | 1 × 20 × 30 | x≈5，y∈[−10,10]，z∈[0,15] |
| `box2` | (−3, 5, 0) | 10 × 1 × 30 | y≈5，x∈[−8,2]，z∈[0,15] |
| `box3` | (13, −10, 0) | 17 × 1 × 30 | y≈−10，x∈[4.5,21.5]，z∈[0,15] |
| `box4` | (12, 0, 0) | 1 × 20 × 30 | x≈12，y∈[−10,10]，z∈[0,15] |

> ⚠ 注意：SDF 里 box 的 `pose.z = 0`，所以它们在 **z ∈ [−15, +15]**（地面以上 15 m、
> 以下 15 m）。可视化用 `Field.obstacles_above_ground()` 只画地面以上部分。
> 另有 `ground_plane`（100×100 平面，z=0）。**世界里没有可移动目标。**

### 怎么跑

```bash
cd ~/rm_guidance_sim
python3 viz_scene.py --laws 0,2 --out ./viz_out            # PPN vs APN 对比
python3 viz_scene.py --laws 0,2 --out ./viz_out --anim     # 额外产出 GIF 动画
python3 viz_scene.py --preset phase_b                      # 换成阶段 B 预设场景
python3 viz_scene.py --scene scene.json                    # 从场景文件加载
```

依赖：`numpy` + `matplotlib`（GIF 另需 `pillow`）。

### 产物说明

**`viz_scene_layout.png` —— 静态场景复现**（3 联图）
* (a) 3D 透视：塔（实心圆柱）、4 面墙、安全盒（绿色点线）、敌方发射场（紫方块）、
  弹丸发射点（三角）、无人机起点 + 巡航速度箭头、**弹丸标称弹道**（红虚线）
* (b) 俯视 XY：塔 + 保护圈（红虚线，半径 = `radius + safety_margin`）、墙体、安全盒、
  发射场、发射点、无人机巡航方向、"若不拦截"的最近点（红 ✕）
* (c) 侧视 XZ：高度剖面（看弹丸下坠与塔高）

**`viz_intercept.png` —— 拦截全过程**（2×3）
* (a)(b)(c) 3D / 俯视 / 侧视，叠加**每条导引律的无人机轨迹**与弹丸轨迹 + 最近点（★）；
  俯视与侧视各带一个**"交会区放大"内嵌子图**（交会只有 ~3 m 尺度，全景看不清）
* (d) 交会距离 `R(t)`（对数轴，含 0.5 m 命中球参考线）
* (e) 制导指令加速度 `|a_cmd|(t)`（含 `a_max` 参考线，能看出限幅）
* (f) 无人机速度 `|v_M|(t)` 与剩余时间 `t_go(t)`（双纵轴）

**`viz_intercept_anim.gif` —— 交会动画**（可选）
左俯视 / 右侧视双视图 + 实时状态文字（t / R / |v_M| / |a_cmd|）。
交会仅 ~0.5 s，默认 10 fps 播放约 1.1 s。

### 实测（本机 Windows，真 C++ 制导核 + 真 DART 弹丸）

```
[若不拦截] 弹丸最近距塔 0.00 m（t=1.06s @ (0.6,0,3.0)）→ ⚠ 构成威胁
[PPN] 最近距离 0.392 m  命中  峰值指令 10.00 m/s²  航程 2.75 m
[APN] 最近距离 0.018 m  命中  峰值指令 10.00 m/s²  航程 2.74 m
```

> 本机 Windows 侧编译 shim 的命令（MSYS2 ucrt64）见 `build_guidance_shim_mingw.sh`；
> 编译产物 `pure_guidance.dll` 放在本目录即可被 `viz_scene.py` / `sim_phase_b_offline.py` 找到。

---

## 3.11 pilot 守备场景实飞 + PPN/APN 对比（2026-10-16 新增）

### 为什么做这个

§3.10 已经能在**离线**里把"守备场景"算出来（`preset_pilot`），但那是 Python 侧的
数学模型。本节把**同一套场景搬进 PX4 里真飞**，并把 §7 的离线结论
（APN 显著优于 PPN）**拿到实飞上验证** —— 这是"离线结论可信"的最后一道关。

### 场景（对齐 `scene_model.preset_pilot`）

| 元素 | 参数 |
| --- | --- |
| 防御塔 | 原点，半径 1.2 m、高 3 m、安全垫 1.0 m |
| 无人机 | 8 m 高；朝 **−x（西，迎击方向）匀速 5 m/s** 巡航；**飞到 x = −4 m 时**弹丸发射 |
| 弹丸 | 从 **(−14, 0, 10)** 以 **14 m/s、俯角 6°** 朝 +x 飞向塔（DART，含重力 + 气动阻力） |
| 命中判据 | `R_true < 0.5 m` |
| 制导 | N = 6，`a_max` = 10 m/s²；`--law 0`（PPN）/ `--law 2`（APN） |

> 塔在原点、弹丸指向塔 ⇒ **不拦截就撞塔**（离线实测"最近距塔 0.00 m"）。

### 新增的机制：`--cruise-x`（巡航到位判据）

原 `do_cruise` 是「**速度一达标就发射**」，但守备场景要求「**巡航到位再发射**」。
现在 `do_cruise` 改成**两阶段**：

```
阶段 1（|v| < v_boost_min）→ 朝 boost_dir 恒加速 a_boost        （= 旧行为）
阶段 2（|v| ≥ v_boost_min）→ 沿 boost_dir 的 P 控制把速度**按住**（不再一路顶上去）
                              └─ 未指定 --cruise-x：速度达标即发射（旧行为）
                              └─ 指定 --cruise-x：还要 x 越过阈值才发射
```

新增开关（**不指定时行为与旧版一致**）：

| 参数 | 默认 | 含义 |
| --- | --- | --- |
| `--cruise-x` | `None` | 巡航发射位置判据（**ENU** x 坐标）。指定后需「速度达标 **且** x 已越过该值」才发射 |
| `--v-kp` | `1.5` | 巡航段速度保持增益 (1/s)：`a = v_kp·(v_boost_min − v_沿boost_dir)` |
| `--cruise-timeout` | `8.0` | 巡航/预加速段超时 (s) |

⚠ **`--cruise-x` 用 ENU 坐标**，而节点内的 `start_x` 取自 PX4 的 NED（x=北）——
两者不可混用。（这个坑在本轮实测中被回归测试抓出来过。）

### 怎么跑

```bash
# 1) 确认仿真已在另一个终端跑起来（§1）
# 2) 打包换版 + 自检（若 Windows 侧写过文件）
cd ~/rm_guidance_sim && bash apply_update.sh
# 3) 跑对比
bash run_pilot_compare.sh              # 默认 PPN(0) + APN(2)
bash run_pilot_compare.sh 0            # 只跑 PPN
bash run_pilot_compare.sh 0 2 1        # 自定义序列（0=PPN 1=TPN 2=APN）
bash run_pilot_compare.sh --help       # 看完整说明
```

★ **脚本默认在每个 law 之间暂停**，提示你重启仿真 —— 见下一节。

> ⚠ **手敲命令时必须用 `=` 等号形式**（`--opt=值`），不能写 `--opt 值`：
> `argparse` 会把「以 `-` 开头、但不是纯数字」的值误判成**选项名**。
>
> ```bash
> # ❌ 会报 error: argument --target-p0: expected one argument
> python3 phase_b_guidance.py --target-p0 -14,0,10 --boost-dir -1,0,0
> # ✅ 正确
> python3 phase_b_guidance.py --target-p0=-14,0,10 --boost-dir=-1,0,0
> ```
>
> 纯负数（`--target-elev -6`、`--cruise-x -4`）是例外，argparse 有专门的负数规则；
> 但统一用 `=` 最省心。`run_pilot_compare.sh` 里已全部使用 `=` 形式。

### ★★ 跑第二个 law 之前必须重启仿真（脚本已自动暂停提醒）

跑完一个 law 后节点会 **LAND + DISARM**，无人机停在**拦截点附近**而非原点。
紧接着跑下一个 law，无人机就从那里起飞 ⇒

1. 到 `--cruise-x` 的位移变短（甚至起点已越过阈值，一达标就立刻发射）；
2. 弹丸发射点是**绝对坐标**，与无人机的相对几何随之改变 ⇒ **两个 law 不可比**。

**两层防护**：

| 层 | 机制 |
| --- | --- |
| 脚本层 | 每个 law 之间有 `read` 暂停，打印重启步骤，等你在跑 launch 的终端里 Ctrl-C 并重跑 launch（约 20 s）后按 Enter 继续 |
| 节点层 | `SETTLE` 阶段 `check_cruise_geometry()` 自检，若起点已越过阈值则**显式 warn**（不静默给错结果） |

节点侧的告警长这样：

```
  ⚠ 起点 x=-6.00 m 已在巡航阈值 -4.00 m 的 < 侧 ⇒ 速度一达标就立刻发射，几何与守备场景不符。
     多半是上一次跑完停在拦截点、没回原点 —— 请**重启仿真**后再跑（或接受本次几何已漂移）。
```

**重启仿真** = `Ctrl-C` 关掉 launch 终端，再跑那条 launch 命令即可（约 20 s）。

> 只想快速冒烟、不在乎几何漂移时，可加 `--no-pause` 跳过暂停。

### ★★ 首次实跑暴露的两个坑（2026-10-16 已修）

第一次实跑 PPN 脱靶 **2.165 m**、APN **2.137 m**（都 100% 饱和），与离线预期（0.28 / 0.13）
完全不符。定位到**根因不在制导律，而在"高度"**：

| # | 坑 | 详情 |
| --- | --- | --- |
| **1** | **XTDrone2 的 TAKEOFF 高度写死 10 m** | `multirotor_communication.py:320` 发 `VEHICLE_CMD_NAV_TAKEOFF` 时 `param7=10.0`。⇒ 我们的 `--target-height 8` **控制不了 PX4 爬多高**，它永远爬到约 10.1 m |
| **2** | **`alt_hold` 的 PD 符号写反（正反馈）** | 原式 `a[2] += kp·(当前 − 目标) − kd·v`；ENU 里 `a_z>0` 是**向上** ⇒ "太高继续往上推"。实飞日志正是如此：起飞到位 10.13 m，发射时涨到 **10.76 m** |

**后果**：离线基准假设无人机在 **8 m**（FakePX4 直接摆在 8 m），实飞却在 **10.76 m** ——
比弹丸起点（10 m）还高 ⇒ 迎头几何变成"要往下扎" ⇒ 指令全程顶格（30/30 饱和）
⇒ **两套几何完全不同，结果不可比**。

**修复**：
1. **修正 `alt_hold` 符号** → `a[2] = kp·(目标 − 当前) − kd·v_z`
2. **SETTLE 段新增"高度收敛"**：进 OFFBOARD 后主动把高度调到 `--target-height`，
   等 `|alt−目标| ≤ 0.35 m` 且 `|v_z| ≤ 0.30 m/s`（最长等 `--alt-settle-timeout`，默认 8 s）才继续。
   ⇒ 即使 PX4 爬到 10.1 m，节点也会把它带到 8 m 再发射。
   （`--no-alt-hold` 时退回旧行为，只等固定 0.5 s）

**新增参数**：`--alt-settle-tol`(0.35) / `--alt-settle-vtol`(0.30) / `--alt-settle-timeout`(8.0)

**回归**（`test_phase_b_states.py` **用例 9**，用 FakePX4 摆在 10.13 m 复现）：
```
起点 10.13 m（模拟 PX4 的 TAKEOFF 高度）→ 发射时 alt=8.13 m（目标 8.0 m）  ✅
--no-alt-hold 时 alt=10.13 m（不调整，退回旧行为）                          ✅
高度 10.5 m 时垂直指令 a_z=-2.490 m/s²（负 = 向下，符号正确）                ✅
```

> ⚠ **上次的实飞数据（2.165 / 2.137 m）不可用** —— 那次的几何是错的。修复后请重跑。

### ★★ 第二次实跑：APN 命中 / PPN 脱靶，但暴露"横向漂移"（2026-10-16 已修）

高度修复生效后（日志出现 `[5/6] 稳定到位：alt=8.35 m`），结果：

| law | 最近距离 | 命中 | 饱和 | 发射几何（ENU） |
| --- | --- | --- | --- | --- |
| PPN | 1.113 m | ✗ | 29/30 = **96.7%** | (−4.09, **−0.99**, 8.47) |
| APN | **0.220 m** | ✓ | 22/26 = 84.6% | (−4.03, −0.29, 8.45) |

✅ **趋势对了**（APN 命中、PPN 脱靶，与离线一致），但：

⚠ **两个问题**：
1. **横向漂移**：期望 ENU y ≈ 0，实际 PPN 偏 **−0.99 m**、APN 起点偏 x=+2.39 m
2. **饱和率仍极高**（96.7%）⇒ 制导全程顶格，没有余量

**根因 = 坑 1（accel-only 纯前馈无阻尼）**：起飞残留的水平速度进 OFFBOARD 后
**永不衰减**，而 SETTLE 段**只控垂直** ⇒ 横向一路漂。
关键量化：**横向偏差 0.99 m ≈ ½·a_max·t_go² = ½·10·0.5² = 1.25 m 的全部修正预算**
⇒ PPN 无余量 ⇒ 脱靶。**这正是《MEMORY §7》那条结论的实飞验证。**

**修复**：
1. **SETTLE 段升级为三维稳定**：除了把高度调到 `--target-height`，
   还把水平位置**收敛到 `--settle-xy`（默认 0,0）**、并阻尼水平速度，
   到位判据加上 `|Δxy| ≤ 0.25 m` 且 `|v_h| ≤ 0.20 m/s`。
2. **CRUISE 段加横向保持**：只阻尼**垂直于 `boost_dir` 的水平速度分量**，保证走直线。
   ⚠ **不能**直接阻尼 `v_enu[0]/[1]` —— 巡航主方向常就是 ENU 的 ±x，
   那样会把加速一起阻尼掉（我踩过：巡航速度涨不起来、一路飞到 x=−22.6 然后超时）。
3. **发射时刻打印几何报告**（Δx/Δy/Δz + 横向速度 + 修正预算），
   并在这几项偏大时 **warn** —— 一眼看出"这次跑得可比吗"。

**新增参数**：`--settle-xy`(0,0；填 `none` 只做速度阻尼) / `--settle-kp`(1.0) /
`--settle-kd`(2.0) / `--settle-xy-tol`(0.25) / `--settle-xy-vtol`(0.20) /
`--cruise-lat-hold`（默认 ON）/ `--no-cruise-lat-hold`

**回归**（`test_phase_b_states.py` **用例 10**）：
```
起点 (1.20,−0.90,8.00) 且 v_y=+0.35
  → 发射时 (−4.12, +0.01, +8.00)，v=(−5.83, +0.00)   ✅ 横向完全收敛
巡航速度正常建立 |v_x|=5.81 m/s                        ✅ 阻尼没吃掉主方向
```
> ⚠ 用例 7.3 的语义随之变化：原来断言"起点在阈值外必须告警"，
> 现在起点会被 SETTLE **拉回** ⇒ 该告警改由 `--settle-xy none` 覆盖（用例 7.3b）。

### ★★ 着陆段数值审计：停桨前状态是否合理（2026-10-16）

用 `audit_landing.py` 自查（比测试里的 `FakePX4` 更真实：**着陆时不再瞬间归零水平速度**，
而是模拟 PX4 AUTO_LAND 的有限水平减速度 ~2 m/s²）。

**审计结果（修复前）**：

| 用例 | 着陆起点 | 停桨点 | **水平漂移** | 停桨高度 | 停桨水平速度 |
| --- | --- | --- | --- | --- | --- |
| PPN（脱靶） | (−7.09, 0) @5.87 m/s | **(−15.64, 0)** | **8.55 m** | 0.200 m ✅ | 0.000 ✅ |
| APN（命中） | — | — | ~8 m | 0.200 m ✅ | 0.000 ✅ |

⇒ **停桨瞬间的状态完全合理**（贴地 0.20 m、水平/垂直速度全为 0）。
但 **着陆过程漂了 8.55 m**，落在 ENU x=−15.6 —— 而声明安全盒是 **ENU x ≥ −9** ⇒ **超出 6.6 m**。

**根因**：`AUTO_LAND` 的水平减速度有限（~2 m/s²），脱靶时残留的 5.87 m/s 需要
`v²/(2a) = 8.6 m` 才能刹住（与实测 8.55 m 完全吻合）。

**修复：新增 `BRAKE` 段（着陆前先刹车）**
转入收尾时，若水平速度还大，**先留在 OFFBOARD 把水平速度刹到 ≤0.30 m/s 再 LAND**
（我们在 OFFBOARD 里自己可控，`a_max=10` ⇒ 刹车距离只要 `v²/(2a) ≈ 1.7 m`）。

| | 着陆漂移 | GUIDED 结束 → 最终停稳 总位移 |
| --- | --- | --- |
| 关刹车 | 8.55 m | 8.55 m |
| **开刹车（默认）** | **0.01 m** | **3.12 m**（5.4× 改善中的 3.1 m 是刹车距离本身） |

**新增参数**：`--brake`（默认 ON）/ `--no-brake` / `--brake-kd`(2.0) /
`--brake-vtol`(0.30) / `--brake-timeout`(5.0)

**顺带修一个"静默漏报"**：PX4 自己停桨时（`arming_state` 变 DISARMED），
原代码**只看 `+`/`−` 就打了 `✓ 已停桨`**，**不检查高度**。而实飞 APN 那次是在
**alt=0.73 m** 就停的 —— **那是硬摔，不是落地**。现在：
- 停桨高度 > `LANDED_ALT + 0.15` ⇒ **warn**（说明是掉下来的，并列出可能原因）
- 汇总表新增一行 `着陆质量：✓ 软着陆 / ⚠ 非软着陆`

**回归**（`test_phase_b_states.py` **用例 11**）：
```
开启刹车: LAND 起点 (-10.20) → 停桨 (-10.21)   漂移 0.01 m   停桨高度 0.20 m  |v_h|=0.00
关闭刹车: LAND 起点 ( -7.09) → 停桨 (-15.64)   漂移 8.55 m   停桨高度 0.20 m  |v_h|=0.00
```

> ℹ️ `audit_landing.py` 是一个**独立的数值审计脚本**（不属于回归套件），
> 用来随时自查"停桨前状态是否合理"，并逐帧打印着陆段。

### 预期（离线基准，看趋势不看绝对值）
| 模型来源 | PPN | APN |
| --- | --- | --- |
| `viz_scene.py`（真 DART + 真 C++ 制导核，§3.10） | 0.392 m | **0.018 m** |
| `test_phase_b_states.py` 用例 7.5（简化质点执行模型） | 0.278 m | 0.127 m |

⇒ 两套离线模型都给出 **APN 明显优于 PPN**，但绝对值不同（执行模型/滞后不同）。
**实飞应该看到同样的趋势**；绝对值会因真 PX4 动力学而再次不同 ——
**结论以趋势为准**（APN 的 `+(N/2)·a_T⊥` 补偿了弹丸持续加速这个本质）。

### 要回传什么

各 law 的：

- 节点日志：`[1/6]`…`[6/6]`、`── CRUISE 完成：|v|=… @ (x,y,z)`（含"到位判据"行）、
  `★ 命中！R_true=… @ t_rel=…`、末尾汇总表（峰值指令 / 饱和步数 / mode 分布）
- CSV：`phase_b_pilot_law0.csv` / `phase_b_pilot_law2.csv`
- ⚠ 若出现"起点已在巡航阈值之外"的 warn，请说明 —— 那次几何已漂移，需重跑

---

## 3.12 在 Gazebo 里看见拦截过程（塔 / 弹丸 / 轨迹）（2026-10-16 新增）

### 效果

跑 `run_pilot_compare.sh`（默认已带 `--gz-viz`）时，**Gazebo GUI 里会多出**：

| 元素 | 外观 | 位置 |
| --- | --- | --- |
| **防御塔** | 蓝色实心圆柱 r=1.2 m / h=3 m | 世界原点 |
| **敌方发射架** | 橙色方块（地标，指示弹丸从哪来） | 弹丸发射点正下方 |
| **弹丸** | 橙红色小球 r=0.06 m，**带自发光**（暗背景也看得见） | 沿 `TargetModel` 弹道飞向塔 |
| **弹道轨迹线** | 橙色折线（`--gz-trail`，跑完才画） | 弹丸走过的路径 |
| **命中标记** | 黄色球（15 s 后自动消失） | 命中点 |

无人机本身由 PX4/Gazebo 自己渲染，本模块不碰它。

### 零侵入实现（★ 没改 XTDrone2 任何文件）

`walls.sdf` 里**已经启用了** `gz-sim-user-commands-system` 插件，它提供
`/world/<world>/{create,set_pose,remove}` 三个运行时服务 ⇒ 可以**在运行时动态创建/移动实体**。
所以**不需要**改 `walls.sdf`、不需要 `colcon build`、不影响队友。

用的 Python 绑定：`gz.transport13` + `gz.msgs10`（随 Gazebo Harmonic 一起装在
`/usr/lib/python3/dist-packages/gz/`，**不需要 ROS** 就能 import）。

**坐标系**：Gazebo 世界是 **ENU**（x 东 / y 北 / z 上），与制导核的 `p_enu` **直接对应**
⇒ **不做任何换算**（这是本模块唯一要记的约定；别在这里"再转一次"）。

### 两个设计要点

1. **不阻塞制导回路**：`gz.transport` 的服务请求是**阻塞**的（等响应），
   所以 `GzScene` 用**后台线程**按固定频率发 `set_pose`，ROS 回调只更新缓存值
   （`set_projectile()` **立即返回**）。
2. **弹丸位置完全由我们说了算**：弹丸 SDF 用 `static=false` + **`<gravity>false</gravity>`
   + 无 collision** ⇒ 物理引擎不会把它推走 ⇒ **所见即 `TargetModel` 所算**，
   与制导用的弹道逐帧一致（而不是让 Gazebo 另算一条对不上的弹道）。

### 怎么用

```bash
# ── 只想先确认可视化链路（不用 ROS 节点）──
python3 gz_scene_sync.py --demo        # 放塔 + 让弹丸飞一条线（8 s）
python3 gz_scene_sync.py --tower       # 只放一座塔
python3 gz_scene_sync.py --clear       # 清掉塔 / 弹丸 / 发射架 / 轨迹

# ── 在实飞里看（推荐）──
bash run_pilot_compare.sh              # 默认 --gz-viz --gz-trail
bash run_pilot_compare.sh --no-gz-viz 0 2   # 关掉画面（纯控制台）
```

### ⚠ 接不上 Gazebo 会**自动降级**（不影响制导）

可视化是"锦上添花"，任何失败都**只警告不中断**：

- `gz` Python 绑定缺失 → 打一条警告，跳过可视化
- `create` / `set_pose` 失败（仿真没跑 / 世界名不对）→ **只警告一次**（不刷屏），继续跑
- `gz-msgs` 版本不兼容（字段名/枚举名变了）→ **启动自检立刻报错**并跳过，不会跑到一半 AttributeError

**回归已覆盖**：`test_phase_b_states.py` **用例 8** 专门验证降级路径 ——
"Gazebo 不可用时制导照常命中、收尾正常、`--gz-viz` 未指定时零开销"。

### 排错

| 现象 | 原因 / 处理 |
| --- | --- |
| 打了"⚠ Gazebo 未响应 /world/walls/create" | 仿真没跑，或世界名不是 `walls`。先用 `gz topic -l \| grep world` 确认；换世界用 `--gz-world <名字>` |
| 塔／弹丸看不见 | Gazebo 相机没对准原点。GUI 里按 `Ctrl+9`（或 Entity Tree 里双击 `defense_tower`）跳过去 |
| 交会一闪而过看不清 | Gazebo GUI 右下角物理面板可**实时调实时率**（`real_time_factor`），或暂停/单步（GUI 工具栏）；也可先 `--no-gz-viz` 看 CSV |
| 弹丸留在地上不动 | 正常 —— 命中/脱靶后代码会把它移到视野外（"消失"），轨迹线仍在 |
| 想清掉残留实体 | `python3 gz_scene_sync.py --clear`（或重启仿真） |

### 校验工具：`verify_gz_msgs.py`

本机（Windows）没有 Gazebo，跑不了 `--demo`；而字段名写错是**运行时**才炸。
所以这个脚本**直接读 WSL 里 `gz/msgs10/*_pb2.py` 内嵌的 protobuf 描述符**（二进制），
自己解析（手写最小 wire-format 解码器，不依赖 `google.protobuf`），核对本模块用到的
字段名 / 枚举名。**发现过一次真错**：`Marker.Action` 只有 `ADD_MODIFY`，**没有 `ADD`**。

```bash
python3 verify_gz_msgs.py      # 应打印"全部通过 ✅"
```

---

## 3.13 CSV 位姿输出 + 0.1 s 采样（2026-10-16 新增）

### 为什么改

实飞日志里**只有位置和 heading，没有 roll/pitch** —— 一旦出现"无人机姿态拧着飞"、
"相机看不到目标"这类现象，光看位置根本判不出来。另外 CSV 本来是 **50 Hz**
（每个控制周期一行），一次 30 s 的实飞就是 1500+ 行，回传效率低。

### 改了什么

| 项 | 改前 | 改后 |
| --- | --- | --- |
| 采样率 | 跟随 `--rate`（50 Hz，0.02 s） | **`--csv-hz`（默认 10 Hz = 0.1 s）** |
| 姿态列 | ❌ 只有 `heading` | ✅ **`roll_deg` / `pitch_deg` / `yaw_deg`** + `q_w`/`q_x`/`q_y`/`q_z` |
| 位置列 | ✅ `px4_*(NED)` + `enu_*(ENU)` | 不变 |
| 收尾 | 可能漏掉最后一行 | `DONE` 时 `log_row(force=True)` **补最后一行** |

> 指令仍是 `--rate`（50 Hz）—— **只有落盘被节流**，控制回路一点没变。
> 想看细节把 `--csv-hz` 给 50 即可。

### 姿态从哪来：`/odometry`（ENU 真值）

`grid` 里 `VehicleLocalPosition` **只有 `heading`，没有 roll/pitch** —— 姿态只能取
真值桥 `/<ns>/odometry` 的 `pose.pose.orientation`（ENU 四元数）。该话题**本来就在订阅**
（`odom_cb`），只是之前只用了 `position`；本轮新增 `odom_att()` 把 orientation 解出来：

```python
def odom_att(self):     # → (roll_deg, pitch_deg, yaw_deg, (w,x,y,z)) 或 None
    q = self.odom.pose.pose.orientation
    w, x, y, z = q.w, q.x, q.y, q.z      # 具名字段，与分量顺序无关
    # ZYX 欧拉角（yaw-pitch-roll）标准式
    roll  = atan2(2(wx + yz), 1 − 2(x² + y²))
    pitch = asin (2(wy − zx))
    yaw   = atan2(2(wz + xy), 1 − 2(y² + z²))
```

* **ENU 口径**：`yaw=0` 朝东(+x)、`yaw=+90°` 朝北(+y) —— 与 `--target-azim`
  （"方位角从 +x 逆时针"）**同一口径**，对日志时不用再心算换算。
* **没有 odometry 就留空**，**不用 `heading` 硬凑 roll/pitch**（那是编数据）。
* 全零四元数 → 返回 `None`，**绝不输出 NaN**。

### 怎么用

```bash
# 默认 0.1 s 一行（10 Hz），带姿态列
python3 phase_b_guidance.py --csv phase_b_log.csv

# 想看每个控制周期（调试姿态环）
python3 phase_b_guidance.py --csv-hz 50

# 只要 2 Hz（长航时省空间）
python3 phase_b_guidance.py --csv-hz 2
```

### 回传分析用哪几列

| 关心什么 | 看哪些列 |
| --- | --- |
| 位置漂移 / 偏离发射点 | `enu_x` `enu_y` `enu_z`（ENU，上为正） |
| 姿态是否稳定 | `roll_deg` `pitch_deg`（巡航/制导段应基本为 0） |
| 机头指向 | `yaw_deg`（对比 `--target-azim` 看是否对上目标方向） |
| 制导指令 | `ax_enu` `ay_enu` `az_enu` `a_norm` `sat` |
| 交会几何 | `R_true` `t_go` `Vc` `mode` |
| 收尾是否合理 | `nav_state` `arming_state` |

### 离线回归（用例 12）

`test_phase_b_states.py` 新增 **用例 12**，覆盖：

* 姿态反解正确性：单位四元数 / 纯 yaw ±90° / 纯 roll 30° / 纯 pitch −20° /
  复合 (roll30,pitch10,yaw90) —— **全部 1e-6 内**；无 odom 与全零四元数都返回 `None`。
* 采样节流**严格对时**：50 Hz 拍频下 `--csv-hz 10` ⇒ 200 拍出 **40 行**；
  `50` ⇒ **200 行**；`2` ⇒ **8 行**；`force=True` 绕过节流（0.1 Hz 下仍写出 2 行）。
* CSV 表头含全部新增姿态列与位置列。

---

## 4. 请回报给我什么

**阶段 A 已完成** ✅（第 8 轮全绿，见 §3.5）；**阶段 B 已实跑验收通过** ✅
（命中 0.318 m，见 §3.8）；**场景模型 + 可视化已完成** ✅（§3.9–§3.10）；
**pilot 守备场景 + PPN/APN 对比已就绪** 🟡（代码 / 脚本 / 离线基准齐备，见 §3.11，**待实跑**）；
**Gazebo 实体模型（塔 / 弹丸 / 轨迹）已完成** 🟡（见 §3.12，**待在 WSL 里实看**）；
**CSV 位姿输出 + 0.1 s 采样已完成** ✅（见 §3.13，离线用例 12 全绿）。

### 下一步可选方向

| 方向 | 状态 | 说明 |
| --- | --- | --- |
| **A. pilot 场景进仿真** | 🟡 **已就绪，待跑** | 已加 `--cruise-x` + `run_pilot_compare.sh`，见 **§3.11**。跑一次并把日志 / CSV 回传即可 |
| **A2. Gazebo 看拦截过程** | 🟡 **已就绪，待看** | 塔 + 弹丸 + 轨迹 + 命中标记，见 **§3.12**。跑 `run_pilot_compare.sh` 即带画面 |
| **A3. CSV 位姿回传** | ✅ **已完成** | 位置 + roll/pitch/yaw + 四元数，默认 **0.1 s 一行**，见 **§3.13**。回传 CSV 即可做姿态/漂移诊断 |
| **B. 换 APN** | 🟡 **已就绪，待跑** | 已被 §3.11 的对比脚本覆盖。`a_T⊥` 来源已确认：`sensor_cfg()` 是 **IDEAL 档（真值透传）**，仿真里能拿到真值；**真机需另估** |
| **C. 阶段 C（相机）** | ⏳ 待办 | 引入 OakD 视觉，目标从数学模型换成 Gazebo 实体。⚠ **必须先打 yaw 补丁**（坑 4）；且受《待决问题.md》**Q8 机体轴**阻塞 |
| **D. 决策层 T2~T9** | ⏸ 挂起 | 威胁判定 / 拦截窗口 / `t_start*` / 巡航策略 / 预置位（见《守备拦截系统规划.md》） |

跑仿真后如有异常，把 **节点日志**（`[1/6]`…`[6/6]`、`★ 命中！`、末尾汇总表 + 收尾几行）
和 `phase_b_log.csv` 贴给我即可 —— 我要看"最近距离、饱和步数、mode 分布、`R_true` 收敛过程"。

---

## 5. 文件说明

| 文件 | 作用 |
| --- | --- |
| `phase_a_accel_path.py` | 阶段 A 验证节点（**本目录是权威副本**） |
| `test_phase_a_states.py` | 阶段 A 状态机离线回归（桩替 rclpy，**不需要 ROS/仿真**） |
| `phase_b_guidance.py` | **阶段 B 制导闭环节点**（见 §3.8；§3.11 加了两阶段巡航 + `--cruise-x`） |
| `run_phase_b.sh` | 阶段 B 一键 source + 运行 |
| `run_pilot_compare.sh` | **pilot 守备场景 PPN/APN 对比**（一键跑多个 law；见 §3.11） |
| `gz_scene_sync.py` | **在 Gazebo 里显示塔 / 弹丸 / 轨迹**（零侵入；见 §3.12） |
| `verify_gz_msgs.py` | **校验 gz-msgs 字段/枚举名**是否与本机版本一致（见 §3.12） |
| `audit_landing.py` | **着陆段数值审计**：逐帧打印着陆过程 + 判定停桨前状态是否合理（见 §3.11） |
| `test_phase_b_states.py` | 阶段 B 离线回归（桩替 ROS + **真制导核** + 假 PX4 质点；用例 1~12） |
| `sample_csv_pose_0p1s.csv` | **CSV 列格式参照**：离线跑一遍（真制导核 + 假 PX4），0.1 s 采样、带姿态列（见 §3.13） |
| `sim_phase_b_offline.py` | 纯离线弹道仿真（三种模式：cruise / hot / **pilot**；见 §3.8、§3.9） |
| `scene_model.py` | **场景实体模型**：塔 / 发射场 / 弹丸 / 无人机 / 场地（含 walls.sdf 真实墙体；见 §3.9、§3.10） |
| `viz_scene.py` | **场景复现 + 拦截过程可视化**（Python 画图 / C++ 核心；见 §3.10） |
| `preflight.sh` | **起仿真前预检**：环境/文件/制导核自检 + 等仿真就绪（见 §1） |
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
