# ADR 0008：传感器墙（仿真虚拟视觉）

- 状态：现行（仿真）
- 日期：2026-11-01（文档化）
- 相关代码：`src/guidance/guidance_libs/include/sensor.hpp`、`src/sensor.cpp`
- 相关测试：`test_guidance` §8–§12

## 决定

传感器墙把仿真真值 `TruthState` 变成“看起来像视觉系统给的” `GuidanceState`：

- 接口固定；
- 切换逼真度**不改制导律一行代码**；
- 真机上这一层由 detection / VIO 实现，制导只消费 `GuidanceState`。

## 三级逼真度

| 级别 | 内容 |
|---|---|
| `IDEAL` | 真值透传（无噪、无延迟、无丢帧）——基线 |
| `SEMI_REAL` | 相机投影 + 像素噪声 + size 反推测距（λ̇ 走视线差分） |
| `FULL_REAL` | 在 SEMI_REAL 基础上再加链路延迟 + 丢帧 |

## 配对约定

非 `IDEAL` 时拿不到 `v_T`，制导必须改用：

```text
LosRateSource::FROM_LOS_DIFFERENCE
```

这是真机唯一可行的 ω 来源。仿真里由 `pn_sim` 负责设置配对。

## 使用边界

- 只用于仿真/测试；
- 不链接进 `guidance_core` / `guidance_flight`；
- 真机不要用传感器墙“补数据”。

## 维护

- 改投影/噪声/延迟模型 → 同步本文和 `test_guidance` §8–§12；
- 新增逼真度 → 加枚举 + 测试；
- 保持“接口固定、实现可换”。
