#include "pn_law.hpp"

namespace pure_uav {

// ============================================================
//  ω 的两套算法（见《比例导引讲义.html》第 12 节）
// ============================================================

// A）真值法：ω = (r × v_rel) / R²
Vec3 los_rate_vector(const Vec3 &r, const Vec3 &v_rel, double R) {
    if (R < 1e-9) return Vec3{};        // 防除零
    return cross(r, v_rel) / (R * R);   // 分母是 R²，不是 R
}

// B）真机法：ω = û × (dû/dt)
Vec3 los_rate_from_los_difference(const Vec3 &los_hat, const Vec3 &los_hat_prev, double dt) {
    if (dt < 1e-9) return Vec3{};                     // 防除零
    const Vec3 d_los = (los_hat - los_hat_prev) / dt; // dû/dt 的差分近似
    return cross(los_hat, d_los);                     // ω = û × (dû/dt)
}

// ============================================================
//  几何量
// ============================================================

// 接近速度 Vc = -(r · v_rel) / R
double closing_velocity(const Vec3 &r, const Vec3 &v_rel, double R) {
    if (R < 1e-9) return 0.0;
    return -dot(r, v_rel) / R;
}

// ============================================================
//  三种导引律（返回未限幅的加速度指令，惯性系）
// ============================================================

// TPN：a = N · Vc · (ω × û)
Vec3 pn_tpn(const Vec3 &omega, const Vec3 &los_hat, double Vc, double N) {
    return cross(omega, los_hat) * (N * Vc);
}

// PPN：a = N · (ω × v_M)
Vec3 pn_ppn(const Vec3 &omega, const Vec3 &v_M, double N) {
    return cross(omega, v_M) * N;
}

// APN：a = N · Vc · (ω × û) + (N/2) · a_T⊥
Vec3 pn_apn(const Vec3 &omega, const Vec3 &los_hat, double Vc, double N,
            const Vec3 &a_T) {
    const Vec3 a_T_perp = a_T - los_hat * dot(a_T, los_hat);  // 取垂直分量 a_T⊥
    return cross(omega, los_hat) * (N * Vc) + a_T_perp * (N * 0.5);
}

} // namespace pure_uav
