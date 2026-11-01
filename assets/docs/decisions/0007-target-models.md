# ADR 0007：目标模型

- 状态：现行（仿真）
- 日期：2026-11-01（文档化）
- 相关代码：`src/guidance/guidance_libs/include/target.hpp`、`src/target.cpp`
- 相关测试：`test_guidance` §13

## 决定

目标模型只依赖时间 `t`，输出真值 `{p, v, a}`：

```text
TargetModel::state(t) -> {p, v, a}
```

- 不依赖无人机状态；
- 可预生成、可回放、可复用；
- 输出 `a` 真值供 APN 使用（真机靠估计）。

## 支持类型

| 类型 | 物理 | 求解 |
|---|---|---|
| `STRAIGHT` | 匀速直线 | 解析 |
| `BALLISTIC` | 只受重力 | 解析（含落地时刻） |
| `DART` | 重力 + 二次气动阻力 | RK4 预积分成表 |
| `CIRCLE` | 定常盘旋 | 数值 |
| `SNAKE` | 侧向正弦机动 | 数值 |

## DART 动力学

```text
a = g + a_drag + a_maneuver
a_drag = −k · |v_rel| · v_rel
k      = ½ · ρ · Cd · A / m
v_rel  = v − v_wind
```

发射参数用 `launch_speed / launch_elev / launch_azim` 给出。

## 为什么“有加速度”关键

1. 重力使弹道成为抛物线，拦截几何完全不同；
2. 阻力 ∝ v²，速度单调衰减、弹道更早落地；
3. `a_T` 沿速度反向且随速度变化 → APN 补偿项不是常值；
4. “最近点”可能出现在下降段，而非几何中点。

## 边界

- 气动参数（质量/迎风面积/Cd）目前是**估计值**，需与飞镖组确认真实值；
- 真机没有 `v_T/a_T` 真值，APN 需要估计；
- 模型只用于仿真/测试，不上机载（`guidance_sim` 不在 `guidance_flight` 里）。

## 维护

- 改动力学/参数 → 同步本文和 `test_guidance` §13；
- 新增目标类型 → 加枚举、解析/数值实现、交叉验证测试；
- 不要把目标模型塞进 `guidance_core`（仿真专用）。
