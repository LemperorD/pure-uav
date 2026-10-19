// 四元数工具实现（见 quat.hpp 的说明：通用数学，属于核心，不属于仿真）
//
// 本文件内容原在 `sensor.cpp`，2026-10-16 为"制导核心 / 仿真"解耦而移出。
// 移动**只改文件归属，不改一行算法**（回归：三个单测全绿 + 结果逐位不变）。

#include "quat.hpp"

#include <algorithm>
#include <cmath>

namespace pure_uav {

// v' = R(q)·v（q = [w,x,y,z]，机体→世界）
// ⚠ 分量顺序 [w,x,y,z]：q[0]=w（标量），q[1..3]=(x,y,z)。与 PX4 / MAVLink 一致。
Vec3 quat_rotate_vec(const double q[4], const Vec3 &v) {
    const Vec3   u(q[1], q[2], q[3]);
    const double w = q[0];
    const Vec3   t = cross(u, v) * 2.0;
    return v + t * w + cross(u, t);
}

// 逆旋转：世界→机体（= 用共轭四元数旋转，只需把矢量部分取反）
Vec3 quat_rotate_vec_inv(const double q[4], const Vec3 &v) {
    const double qc[4] = {q[0], -q[1], -q[2], -q[3]};
    return quat_rotate_vec(qc, v);
}

// 构造"机头朝 forward"的最小旋转四元数（机体前向 = +z）
void quat_from_forward(const Vec3 &forward, double q[4]) {
    const Vec3 f = normalized(forward);
    if (norm(f) < 1e-12) { q[0] = 1.0; q[1] = q[2] = q[3] = 0.0; return; }

    const Vec3   from(0.0, 0.0, 1.0);          // 机体前向 = +z
    const double d = std::max(-1.0, std::min(1.0, dot(from, f)));

    if (d > 1.0 - 1e-9) {                       // 已同向 → 单位四元数 [w,x,y,z]
        q[0] = 1.0; q[1] = q[2] = q[3] = 0.0;
        return;
    }
    if (d < -1.0 + 1e-9) {                      // 反向：绕 x 轴 180° → w=cos90°=0
        q[0] = 0.0; q[1] = 1.0; q[2] = 0.0; q[3] = 0.0;
        return;
    }
    const Vec3   axis  = normalized(cross(from, f));
    const double angle = std::acos(d);
    const double s     = std::sin(angle * 0.5);
    q[0] = std::cos(angle * 0.5);               // w
    q[1] = axis.x * s;                          // x
    q[2] = axis.y * s;                          // y
    q[3] = axis.z * s;                          // z
}

} // namespace pure_uav
