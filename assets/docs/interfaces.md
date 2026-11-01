# 系统接口与数据对齐规范

> 本文是跨模块的“契约”：坐标系、姿态系、时间基准、单位、端口。
> 代码里的唯一换算点是 `src/guidance/adapter/`；其他地方**不要再转一次**。

- 相关代码：`src/guidance/adapter/include/state_adapter.hpp`、`state_check.hpp`
- 相关文档：[architecture.md](architecture.md)；时间基准细节见本文 §4（ADR 0005 计划中）。

---

## 1 内部冻结口径

制导核（`GuidanceState` / `guidance_step`）内部一律：

| 项 | 约定 |
|---|---|
| 惯性系 | **ENU**：x 东 / y 北 / z 上 |
| 姿态 | `q = [w,x,y,z]`，机体 → 世界（ENU），`|q| = 1` |
| 时间 | 秒（`double`），只取差值；基准由调用方给 |
| 长度 / 速度 | m / (m/s) |
| 加速度 | m/s² |
| 角度 | rad |
| 视线 | `los_hat` = 机 → 目标单位矢量；`r = los_hat · R` |
| 相对量 | `v_rel = v_T − v_M`；`r = r_T − r_M` |

外部数据进入 `adapter::to_guidance_state()` 之后，全链路不再做任何换算。

---

## 2 世界系：NED ↔ ENU

- **NED**：x 北 / y 东 / z 下（PX4 uORB、MAVLink 惯例）。
- **ENU**：x 东 / y 北 / z 上（制导核内部）。

换算自逆：

```text
ENU = (N.y, N.x, −N.z)
NED = (E.y, E.x, −E.z)
```

位置 / 速度 / 加速度 / 目标速度都适用同一套。

### 无 GPS 时的“NED”

本项目飞控没有 GPS，飞控 local 系没有真北参考：

- x = **EKF 初始化（上电/起飞）那一刻的机头方向**；
- y = 初始机头右侧；
- z = 下（PX4 惯例不变）。

它与“真 NED”只差一个**固定偏航角 θ₀**。比例导引全是矢量运算，对惯性系整体刚性旋转不敏感；真正要注意的是：**目标数据必须与 p/v 同系**。

---

## 3 姿态系：外部机体系 → 光轴系

制导核期望“光轴系”：x 右 / y 下 / z 前（相机约定）。外部可能给：

| 枚举 | 三轴指向 | 说明 |
|---|---|---|
| `kFrd` | x 前 / y 右 / z 下 | PX4 / MAVLink 最常见，**默认** |
| `kFlu` | x 前 / y 左 / z 上 | ROS / 航空 FLU |
| `kCamFrd` | 光轴系，按 FRD 语义描述 | 相机约定 |
| `kCamFlu` | 光轴系，按 FLU 语义描述 | 相机约定 |
| `kOpticalPassthrough` | 已是光轴系 | 直通，只做世界系换算 |

- `kFrd → 光轴系` 是循环置换 `(x,y,z)_cam = (y,z,x)_frd`，`det = +1`；
- 默认取 `kFrd`，但**真实值必须由通信/视觉确认**（《待决问题.md》Q8），不要凭默认值上线；
- `kOpticalPassthrough` 只做 NED→ENU，机体轴不动。

`convert_attitude()` 的输出：

- `q_out`：光轴系 → 世界系（ENU），`|q_out| ≈ 1`；
- `world_out`：恒为 `Frame::kEnu`；
- 输入不构成正规旋转（`|q|` 偏离过大 / NaN）时返回 false，`q_out` 置单位四元数。

---

## 4 时间基准

### 4.1 单位

| 枚举 | 单位 | 典型来源 |
|---|---|---|
| `kSeconds` | s | 内部 |
| `kMillis` | ms | MAVLink `time_boot_ms`（uint32，约 49.7 天回绕） |
| `kMicros` | µs | PX4 uORB `timestamp`（uint64） |

### 4.2 基准

| 枚举 | 含义 |
|---|---|
| `kSameSource` | 与参考时标同源（**唯一安全**） |
| `kSinceBoot` | 开机以来 |
| `kUnixEpoch` | Unix epoch |
| `kUnknown` | 未知 |

**关键不变量：同一帧数据的所有时间戳必须同源。**
跨源相减不会报错，只会静默给出垃圾差值。

### 4.3 相机/视觉时钟 → 飞控 boot 时钟

视觉时间戳可能是 Unix µs 或相机自身时钟。用同帧的 `SYSTEM_TIME`：

```text
offset_us = time_unix_usec − time_boot_ms × 1000
boot_us   = 视觉时间戳 − offset_us
```

换算是**单向**的（只做相机 → boot），`offset_us` 要持续维护（`SYSTEM_TIME` 约 1 Hz）。

### 4.4 `StateTimeTracker`

不要让调用方自己写 `(now − first) * 1e-6`，原因：

- uint32 ms 回绕（约 49.7 天）会让差值变成巨大负数；
- uint64 µs 的绝对差在 uptime 很大时可能超过 2^53，double 转换丢精度。

`StateTimeTracker` 改为**累加每帧差值**（差值总是小量，恒定精度），并天然容纳回绕。

接口：

- `push_u64_us / push_u32_ms / push_s`：每帧推一个原始时间戳；
- `t_s()`：相对参考点的内部秒（未设参考点时从首帧 0 起算）；
- `set_reference_*`：把当前原始时间戳定为 t=0；
- `out_of_order()` / `last_jump_suspect()`：只提示，不影响累加。

---

## 5 单位换算

单位换算（m/mm/cm、m/s/cm/s、deg/cdeg/degE7、ms/µs）统一在 `src/common/common_libs/mavlink_units`：

- 只做“数值 + 单位”，不解析报文、不做坐标变换、不做状态机；
- 角度归一化结果一律为 rad；
- MAVLink 哨兵（UINT16_MAX 等）必须判，不能当合法值。

---

## 6 端口与外部口径

| 端口 | 输入/输出 | 口径 |
|---|---|---|
| `IInsSource` | `InsSample` | 外部 NED、m、m/s、q[wxyz]、boot µs |
| `ITargetSource` | `TargetSample` | 像素 `(u,v)` + 框 + 距离 + 曝光 boot µs |
| `ICommandSink` | `GuidanceOutput` | 内部算完后换成 NED 的 `a_cmd_ned` / `v_cmd_ned` |

`InsSample` / `TargetSample` / `GuidanceOutput` 定义在 `src/guidance/flight/flight_types.hpp`。

---

## 7 像素 → 视线（主循环内完成）

```text
d_cam  = ((u−cx)/fx, (v−cy)/fy, 1)
d_body = R_body_cam · d_cam
d_world = R(q) · d_body
los_hat = normalize(d_world)
```

- `fx/fy/cx/cy/R_body_cam` 来自 `LoopConfig::cam`；
- `q` 来自 `InsSample`（机体 → 世界）；
- `R_body_cam` 的目标系是 **PX4 FRD**；
- 目标在相机后方（`d_cam.z <= 0`）或结果零矢量 → 视作目标丢失。

---

## 8 输入体检

`state_check` 只报告、不修改：

- 四元数模长；
- 视线是否零/未归一化；
- 距离是否为负；
- 数据是否过期；
- 时间是否倒退。

## 9 维护

- 改坐标/姿态/时间约定 → 同步本文档、`state_adapter`、相关测试；
- 不要在业务代码里再做一次 NED/ENU 或 FRD/光轴换算；
- 新增外部口径 → 先加枚举 + 独立参考实现 + 测试，再接线。
