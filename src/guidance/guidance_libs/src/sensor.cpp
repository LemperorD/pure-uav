#include "sensor.hpp"

#include <algorithm>
#include <cmath>

#include "pn_law.hpp"   // los_rate_vector / los_rate_from_los_difference
#include "quat.hpp"     // quat_rotate_vec / _inv（通用数学，见 quat.hpp；已移出本文件）

namespace pure_uav {

// ============================================================
//  Sensor
// ============================================================
Sensor::Sensor(const SensorConfig &cfg) : cfg_(cfg), rng_(cfg.seed) {}

void Sensor::reset() {
    stats_    = SensorStats{};
    buf_.clear();
    has_prev_ = false;
    los_prev_ = Vec3{};
    t_prev_   = 0.0;
}

bool Sensor::project(const TruthState &truth, double &u, double &v, double &w_px) const {
    const Vec3 r_world = truth.p_T - truth.p_M;
    const Vec3 r_cam   = quat_rotate_vec_inv(truth.q, r_world);   // 世界→机体(=相机)
    if (r_cam.z <= 1e-6) return false;                        // 在相机后方

    u = cfg_.cam.fx * r_cam.x / r_cam.z + cfg_.cam.cx;
    v = cfg_.cam.fy * r_cam.y / r_cam.z + cfg_.cam.cy;
    if (u < 0.0 || u >= cfg_.cam.width || v < 0.0 || v >= cfg_.cam.height) {
        return false;                                         // 出视场
    }
    w_px = cfg_.cam.fx * cfg_.cam.target_size_m / r_cam.z;    // 框宽(px)
    if (w_px < cfg_.cam.min_bbox_px) return false;            // 目标太小
    return true;
}

GuidanceState Sensor::observe(const TruthState &truth, double t) {
    ++stats_.frames;

    GuidanceState st;
    st.t     = t;
    st.valid = true;

    // 惯导量（本仿真假定惯导总是好的）：自身速度 / 位置 / 姿态
    st.v_M = truth.v_M;
    st.p_M = truth.p_M;
    for (int i = 0; i < 4; ++i) st.q[i] = truth.q[i];

    // ---------- IDEAL：真值透传 ----------
    if (cfg_.fidelity == SensorFidelity::IDEAL) {
        const Vec3 r = truth.p_T - truth.p_M;
        st.range    = norm(r);
        st.los_hat  = normalized(r);
        st.v_T      = truth.v_T;      // 真值法算 ω 需要
        st.a_T_perp = truth.a_T;      // APN 用（函数内部取垂直分量）
        const Vec3 w = los_rate_vector(r, truth.v_T - truth.v_M, st.range);
        st.lam_dot = norm(w);         // 真值 λ̇（仅供日志对照）
        return st;
    }

    // ---------- SEMI_REAL / FULL_REAL：走"视觉"链路 ----------
    // 目标速度 v_T 拿不到 → 置零；制导必须改用差分法算 ω
    st.v_T      = Vec3{};
    st.a_T_perp = Vec3{};

    double u = 0.0, v = 0.0, w_px = 0.0;
    if (!project(truth, u, v, w_px)) {
        ++stats_.lost;
        st.valid = false;             // 目标丢失 → 制导进 SEARCH
        return st;
    }

    // 像素噪声（检测框中心抖动）
    u += cfg_.pixel_noise_std * noise();
    v += cfg_.pixel_noise_std * noise();

    // 视线方向：像素 → 相机系单位矢量 → 世界
    const Vec3 dir_cam = normalized(Vec3((u - cfg_.cam.cx) / cfg_.cam.fx,
                                         (v - cfg_.cam.cy) / cfg_.cam.fy,
                                         1.0));
    st.los_hat = quat_rotate_vec(truth.q, dir_cam);

    // 测距：size 反推 R ≈ fx·s / w_px（再叠比例/系统误差）
    double R = cfg_.cam.fx * cfg_.cam.target_size_m / w_px;
    R = R * (1.0 + cfg_.range_scale_err) + cfg_.range_bias;
    st.range = R;

    // λ̇：视线方向时序差分（真机唯一可得的做法，见讲义第 12 节）
    if (has_prev_ && (t - t_prev_) > 1e-9) {
        const Vec3 w = los_rate_from_los_difference(st.los_hat, los_prev_, t - t_prev_);
        st.lam_dot = norm(w);
    }
    los_prev_ = st.los_hat;
    t_prev_   = t;
    has_prev_ = true;

    // ---------- FULL_REAL：再加 丢帧 + 延迟 ----------
    if (cfg_.fidelity == SensorFidelity::FULL_REAL) {
        if (cfg_.drop_rate > 0.0) {
            std::uniform_real_distribution<double> uni(0.0, 1.0);
            if (uni(rng_) < cfg_.drop_rate) {
                ++stats_.dropped;
                st.valid = false;     // 本帧丢失
            }
        }

        // 延迟：输出 latency_s 之前的那一帧
        buf_.push_back({t, st});
        const double t_target = t - cfg_.latency_s;
        GuidanceState out  = buf_.back().st;   // 预热期默认取最新（零延迟）
        bool           hit = false;
        while (!buf_.empty() && buf_.front().t <= t_target) {
            out = buf_.front().st;
            buf_.pop_front();
            hit = true;
        }
        (void)hit;
        return out;
    }

    return st;
}

} // namespace pure_uav
