# ADR 0004：末端冻结判据

- 状态：接受
- 日期：2026-11-01（文档化）
- 相关代码：`src/guidance/guidance_libs/src/guidance.cpp`、`include/guidance_types.hpp`
- 相关测试：`test_guidance` §14

## 背景

末端 `R→0` 时，`ω = (r×v_rel)/R²`（或 `û` 差分）里的 `1/R` 会把噪声放大成大指令。若不处理，最后一拍可能突然满舵。

## 决定

提前锁定最后一帧指令（`HOLD`），不再更新。判据三选一：

| 枚举 | 条件 | 适合 |
|---|---|---|
| `BY_TGO` | `t_go < t_go_freeze` | 默认；时间随 `Vc` 自动缩放 |
| `BY_RANGE` | `R < R_freeze` | `R` 可直接测（size/深度），实现最简 |
| `BY_TGO_OR_RANGE` | 任一满足 | 用 `R` 兜底，防 `Vc→0` |

`t_go = R / max(Vc, ε)`。

## 为什么默认 `BY_TGO`

- 可修正量 ∝ `½·a·t_go²`，**时间才是物理根本量**；
- `Vc` 变化时，固定 `R` 阈值会过早/过晚冻结；
- `BY_TGO` 随接近速度自动缩放。

## 等价关系

`Vc` 近似常数时：

```text
R_freeze ≈ Vc · t_go_freeze
```

例如 `Vc≈33 m/s`、`t_go_freeze=0.15 s` ⇒ `R_freeze≈5 m`。

两者并不严格等价：存在交会加速度时，`R` 与 `t_go` 会分道扬镳。

## 为什么需要 `BY_TGO_OR_RANGE`

尾追/远离时 `Vc→0`，`t_go = R/max(Vc,ε)` 会发散，`BY_TGO` **永不冻结**。此时用 `R_freeze` 兜底。

## 与 `range` 缺失的关系

- 没有 `range`：`t_go`/`Vc` 失准，默认 `BY_TGO` 可能一上来就触发 HOLD；
- 此时应改用 `BY_RANGE`（有距离）或显式关闭冻结；
- PPN 本身不需要 `range` 算加速度，但冻结/诊断需要。

## HOLD 的输出

`HOLD` 时：

- `mode = HOLD`；
- `a_cmd = memory.a_cmd_prev`（上一拍限幅后的指令）；
- 第一拍没有历史时，`a_cmd_prev` 为默认零矢量。

## 默认值

- `freeze_criterion = BY_TGO`；
- `t_go_freeze = 0.15 s`；
- `R_freeze = 5.0 m`。

## 维护

- 改冻结逻辑 → 跑 `test_guidance` §14；
- 改默认值 → 同步 `guidance_default_params()` 和真机 YAML；
- 新增判据 → 同步 `FreezeCriterion` 枚举和本文。
