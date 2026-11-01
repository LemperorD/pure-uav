# ADR 0003：视线角速率 ω 的两套算法与时间基准

- 状态：接受
- 日期：2026-11-01（文档化）
- 相关代码：`src/guidance/guidance_libs/src/pn_law.cpp`、`src/guidance/flight/guidance_loop.cpp`
- 相关测试：`test_guidance` §1–§15、`test_state_adapter` §5

## 背景

比例导引需要视线角速率 `ω`。项目保留两套算法，用 `GuidanceParams::los_rate_source` 切换：

| 枚举 | 公式 | 需要 | 用在 |
|---|---|---|---|
| `FROM_RELATIVE_VELOCITY` | `ω = (r × v_rel) / R²` | 距离 `R` + 目标速度 `v_T` | 仿真（真值） |
| `FROM_LOS_DIFFERENCE` | `ω = û × (dû/dt)` | 只需 `û` 历史 + 自身速度 | 真机 |

## 决定 1：分母是 `R²`，不是 `R`

`ω = (r × v_rel) / R²`。

写成 `(û × v_rel) / R` 量纲看起来对，但数值会系统性偏大，导致脱靶。这是制导铁律之一。

## 决定 2：差分法的 `dt` 必须对应 `û` 的更新间隔

`FROM_LOS_DIFFERENCE` 用：

```text
ω = û × (dû/dt)
```

`û` 只在**目标帧**更新，所以：

- `state.t` 必须取**目标帧采样时刻**（`tgt_now_.boot_us`）；
- 只在**目标帧到达**时调用 `guidance_step()`；
- 其余拍子只重发上一拍指令（PX4 offboard 要求 setpoint 持续到达）。

反例（必须避免）：

```text
用飞控帧时间喂 tracker，却按目标帧更新 û
⇒ dt = 20 ms（50 Hz），而 û 实际跨了 33 ms（30 Hz）
⇒ ω 被少算约 40% ⇒ 制导偏软
```

## 决定 3：只在目标帧更新时重算

`GuidanceLoop::run_once()` 的流程：

1. poll 飞控帧和目标帧；
2. 有新目标帧且数据新鲜 → 像素反投影 → `to_guidance_state` → `quick_check` → `guidance_step`；
3. 没有新目标帧 → 重发上一拍（`send_on_every_tick`）；
4. 数据不健康 → 按 `on_unhealthy` 保持/零速度悬停/停。

## 边界

- `FROM_LOS_DIFFERENCE` 的差分放大噪声；真机通常需要再滤波。
- 第一帧没有历史，`ω` 保持为零。
- 没有 `range` 时，`t_go`/`Vc` 会失准；默认 `BY_TGO` 冻结可能一上来就触发。此时应改用 `BY_RANGE`（有距离）或显式关闭冻结。
- `FROM_RELATIVE_VELOCITY` 只适用于仿真真值；真机拿不到 `v_T`。

## 维护

- 改 `dt` 来源或 `guidance_step` 调用时机 → 必须重跑 `test_guidance` 的 ω/制导断言；
- 改 `los_rate_source` 默认值 → 同步 `default_params()` 和真机 YAML；
- 新增 `LosRateSource` 枚举时，必须同时更新 `guidance_types.hpp` 的注释和本文档。
