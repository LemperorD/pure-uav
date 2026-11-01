#pragma once
// =============================================================================
//  quat.hpp —— 四元数工具（通用刚体数学，属于 guidance_core）
//
//  约定：q = [w,x,y,z]；R(q)·v 是标准旋转（不是转置）；单位四元数 [1,0,0,0]。
//  设计/历史/实测见 assets/docs/decisions/0001-quat-semantics.md。
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
