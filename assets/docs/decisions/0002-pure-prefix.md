# ADR 0002：`PureGuidanceCmd` / `PureGuidanceMode` 的 `Pure` 前缀

- 状态：接受
- 日期：2026-10-20（改名）/ 2026-11-01（文档化）
- 相关代码：`src/guidance/guidance_libs/include/guidance_types.hpp`
- 相关模块：`src/driver/communicate/include/communicator.hpp`、communication 分支的 `src/common/msg.hpp`

## 背景

通信模块在**同一个 `pure_uav` 命名空间**里也定义了：

- `GuidanceCmd`：期望位置/速度/加速度的 setpoint（NED）；
- `GuidanceMode`：通信侧的模式。

制导模块原来的输出也叫 `GuidanceCmd` / `GuidanceMode`。两者同名同命名空间，任何 TU 同时 include 就会重定义。

## 决定

制导模块统一加 `Pure` 前缀：

- `PureGuidanceCmd`：我们算出来的指令（ENU 加速度 + 调试量）；
- `PureGuidanceMode`：PN / APN / HOLD / SEARCH。

通信侧的 `GuidanceCmd` 保持不变。

## 语义对照

| 类型 | 含义 | 坐标系 |
|---|---|---|
| `PureGuidanceCmd` | 导引律输出：加速度 + `t_go/Vc/R/lam_dot` | 内部 ENU |
| `GuidanceCmd` | 通信 setpoint：位置/速度/加速度 | NED |

接线方式：

```text
guidance_step() → PureGuidanceCmd
    → adapter::cmd_accel_to_ned()
    → 通信 GuidanceCmd{mode=ACCELERATION, afx,afy,afz}
```

## 理由

- 避免与通信模块同名冲突；
- `Pure` 与 `libpure_guidance.so` 命名一致；
- 机载主循环里两种类型可以并存，语义清楚；
- 将来通信模块若改名，`Pure` 前缀仍可保留，不影响语义。

## 维护

- 不要为了“好看”去掉 `Pure` 前缀；
- 若通信侧也改名，先确认两边不再同名，再评估是否保留；
- 新增输出类型时，继续用 `Pure` 前缀避免再次撞名。
