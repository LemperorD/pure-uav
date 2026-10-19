#pragma once

#include "vec3.hpp"

// 比例导引族（PN family）与几何量计算。
// 详细推导见《比例导引讲义.html》。

namespace pure_uav {

// ---- ω 的两套算法（见《比例导引讲义.html》第 12 节）----

// A）真值法：ω = (r × v_rel) / R²
// 注意：分母是 R²，不是 R。写成 (û × v_rel)/R 量纲对但会脱靶。
// 需要相对位置 r 与相对速度 v_rel（即需要目标速度 v_T）。
Vec3 los_rate_vector(const Vec3 &r, const Vec3 &v_rel, double R);

// B）真机法：ω = û × (dû/dt)，用视线方向时序差分。
// 只需要 û 与上一帧 û（不需要 v_T / R）。
// 注意：差分会放大噪声，真机通常需再滤波。
Vec3 los_rate_from_los_difference(const Vec3 &los_hat, const Vec3 &los_hat_prev, double dt);

// 接近速度 Vc = -(r · v_rel) / R
// Vc > 0 表示正在接近；迎头为正且大，追击可能为负。
double closing_velocity(const Vec3 &r, const Vec3 &v_rel, double R);

// 真比例导引 TPN：a = N · Vc · (ω × û)
Vec3 pn_tpn(const Vec3 &omega, const Vec3 &los_hat, double Vc, double N);

// 纯比例导引 PPN：a = N · (ω × v_M)
// 不含距离项，对测距误差免疫，无人机常用。
Vec3 pn_ppn(const Vec3 &omega, const Vec3 &v_M, double N);

// 增广比例导引 APN：a = N · Vc · (ω × û) + (N/2) · a_T⊥
// 1/2 来自匀加速位移 s = ½at²。
// a_T 传完整目标加速度即可，函数内部自动取垂直于视线的分量 a_T⊥。
Vec3 pn_apn(const Vec3 &omega, const Vec3 &los_hat, double Vc, double N,
            const Vec3 &a_T);

} // namespace pure_uav
