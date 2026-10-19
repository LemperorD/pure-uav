#pragma once
// =============================================================================
//  四元数工具（**通用数学**，属于制导核心，不属于仿真）
//
//  【为什么在核心而不是仿真里】
//    这三个函数原本放在 `sensor.cpp`（仿真"虚拟视觉"）里，但它们本身只是
//    **通用刚体数学**，与相机、噪声、弹丸都无关。放在仿真里的直接后果是：
//      真机路径的测试（`test_state_adapter`）为了交叉验证四元数语义，
//      不得不链接整个 `guidance_sim`（含 <random> 噪声发生器 + 弹丸动力学）。
//    移到核心后：`test_state_adapter` 只需链接 `guidance_core`。
//
//  【分量顺序】一律 **`[w, x, y, z]`** —— `q[0] = w`（标量），`q[1..3] = (x, y, z)`。
//    与 PX4 uORB / MAVLink（ATTITUDE_QUATERNION、ODOMETRY.q）一致。
//    单位四元数 = `[1, 0, 0, 0]`。详见《坐标系与时间基准.md》§6。
//
//  【语义】`R(q)·v` 是标准旋转矩阵语义（**不是转置**）。
//    2026-10-16 用三条独立路径实测（各 200 例随机姿态，最大偏差 2.7e-15）：
//      本文件 `quat_rotate_vec` ≡ `state_adapter::rotate_vec_by_quat` ≡ 标准旋转矩阵
//    一致性由 `test_guidance` §6 与 `test_state_adapter` §4 双向钉住。
//
//  【制导核是否需要它】⚠ 不需要 —— `guidance.cpp` / `pn_law.cpp` **完全不使用四元数**
//    （PPN/TPN/APN 全是世界系矢量运算）。本文件服务于：
//      · 仿真侧 `Sensor::project/observe`（世界→机体投影）
//      · 仿真侧 `pn_sim`（给点质量模型补一个姿态）
//      · 接口层 / 测试的语义交叉验证
//    ⇒ 真机主循环若不做机体系变换，可以不链接本文件所在的 `quat.cpp`。
// =============================================================================

#include "vec3.hpp"

namespace pure_uav {

// 四元数旋转矢量：v' = R(q)·v（q = [w,x,y,z]，机体 → 世界）
Vec3 quat_rotate_vec(const double q[4], const Vec3 &v);

// 逆旋转：世界 → 机体（= 用共轭四元数旋转，只需把矢量部分取反）
Vec3 quat_rotate_vec_inv(const double q[4], const Vec3 &v);

// 构造"机头朝 forward"的最小旋转四元数（**制导核的光轴约定：机体前向 = +z**）。
// ⚠ 只用它把 +z 对齐 forward，**绕前向轴的 roll 是"最小旋转"的副产物**，
//   不是一个物理可实现的机体系（详见《待决问题.md》Q8）。
// ⇒ 仿真里用来给点质量模型补姿态够用；**真机姿态必须来自飞控/视觉的四元数**。
void quat_from_forward(const Vec3 &forward, double q[4]);

} // namespace pure_uav
