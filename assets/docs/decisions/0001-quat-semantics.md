# ADR 0001：四元数语义与放置位置

- 状态：接受
- 日期：2026-10-16（语义实测）/ 2026-11-01（文档化）
- 相关代码：`src/guidance/guidance_libs/include/quat.hpp`、`src/guidance/guidance_libs/src/quat.cpp`
- 相关测试：`test_guidance` §6、`test_state_adapter` §4

## 背景

四元数工具最初写在 `sensor.cpp`（仿真的“虚拟视觉”）里。问题不是代码错，而是**位置错**：

- 它只是通用刚体数学，与相机、噪声、弹丸都无关；
- 真机路径的 `test_state_adapter` 为了做四元数语义交叉验证，不得不链接整个 `guidance_sim`（含 `<random>` 噪声发生器和弹丸动力学）。

## 决定

1. `quat.hpp/.cpp` 放在 `guidance_core`，不属于 `guidance_sim`。
2. 分量顺序固定为 **`[w, x, y, z]`**，`q[0] = w`，`q[1..3] = (x, y, z)`。
3. `quat_rotate_vec(q, v)` 的语义是 **`R(q)·v`（标准旋转矩阵）**，不是转置。
4. 单位四元数为 `[1, 0, 0, 0]`。
5. `guidance.cpp` / `pn_law.cpp` **不使用四元数**；PPN/TPN/APN 全是世界系矢量运算。

## 理由

- 放在 core 后，`test_state_adapter` 只需链接 `guidance_core`，不再被仿真依赖污染；
- 分量顺序与 PX4 uORB、MAVLink（ATTITUDE_QUATERNION、ODOMETRY.q）一致，上游可原样填入；
- `R(q)·v` 语义与 `guidance::adapter::rotate_vec_by_quat`、标准旋转矩阵一致，跨模块不会“转反”。

## 实测

2026-10-16 用三条独立路径（本文件、`guidance::adapter` 自带实现、标准旋转矩阵）各跑 200 例随机姿态，最大偏差 **2.7e-15**。

一致性由 `test_guidance` §6 与 `test_state_adapter` §4 双向钉住。

## 使用边界

使用 `quat_*` 的地方：

- 仿真侧 `Sensor::project/observe`（世界 → 机体投影）；
- `pn_sim`（给点质量模型补一个姿态）；
- 适配层/测试的语义交叉验证。

制导核本身不做机体系变换；真机姿态必须来自飞控/视觉的四元数，**不要用 `quat_from_forward` 替代真实姿态**。

## 维护

- 改分量顺序或 `R(q)` 语义 → 必须同步 `test_guidance` §6、`test_state_adapter` §4 和本文档；
- 不要把本文件移回 `guidance_sim`，否则 `test_state_adapter` 会重新依赖仿真。
