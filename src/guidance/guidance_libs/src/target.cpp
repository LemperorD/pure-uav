#include "target.hpp"

#include <algorithm>
#include <cmath>

namespace pure_uav {

const char *target_type_name(TargetType t) {
    switch (t) {
        case TargetType::STRAIGHT:  return "STRAIGHT";
        case TargetType::BALLISTIC: return "BALLISTIC";
        case TargetType::DART:      return "DART";
        case TargetType::CIRCLE:    return "CIRCLE";
        case TargetType::SNAKE:     return "SNAKE";
    }
    return "?";
}

namespace {

// 由发射参数生成初速度（ENU：x 东, y 北, z 天）
//   v = speed · (cosθ·cosφ, cosθ·sinφ, sinθ)
Vec3 make_launch_velocity(double speed, double elev, double azim) {
    const double ch = std::cos(elev);
    return {speed * ch * std::cos(azim),
            speed * ch * std::sin(azim),
            speed * std::sin(elev)};
}

// 二次阻力加速度：a_drag = −k·|v_rel|·v_rel（方向与相对速度反向，大小 ∝ v²）
Vec3 drag_accel(const Vec3 &v_rel, double k) {
    return v_rel * (-k * norm(v_rel));
}

} // namespace

TargetModel::TargetModel() : TargetModel(TargetConfig{}) {}

TargetModel::TargetModel(const TargetConfig &cfg) : cfg_(cfg) {
    v0_ = (cfg_.launch_speed > 0.0)
              ? make_launch_velocity(cfg_.launch_speed, cfg_.launch_elev, cfg_.launch_azim)
              : cfg_.v0;
    build();
}

void TargetModel::build() {
    if (cfg_.type == TargetType::STRAIGHT || cfg_.type == TargetType::BALLISTIC) {
        build_analytic();
    } else {
        build_table();
    }
}

// ============================================================
//  解析式：STRAIGHT / BALLISTIC
// ============================================================
void TargetModel::build_analytic() {
    t_flight_    = cfg_.t_max;
    hits_ground_ = false;

    if (cfg_.type != TargetType::BALLISTIC) return;   // STRAIGHT：永不落地

    // 落地时刻：z0 + vz0·t − ½·g·t² = ground_z  →  ½g·t² − vz0·t + (gz − z0) = 0
    const double g = -cfg_.gravity.z;                 // >0
    if (g <= 1e-9) return;

    const double a = 0.5 * g;
    const double b = -v0_.z;
    const double c = cfg_.ground_z - cfg_.p0.z;
    const double disc = b * b - 4.0 * a * c;
    if (disc < 0.0) return;                           // 永远落不到地（往上飞）

    const double sq = std::sqrt(disc);
    const double t1 = (-b + sq) / (2.0 * a);
    const double t2 = (-b - sq) / (2.0 * a);
    const double t  = std::max(t1, t2);
    if (t > 0.0) {
        t_flight_    = std::min(t, cfg_.t_max);
        hits_ground_ = (t <= cfg_.t_max);
    }
}

// ============================================================
//  数值预积分：DART / CIRCLE / SNAKE
// ============================================================
Vec3 TargetModel::accel_at(const Vec3 &p, const Vec3 &v, double t) const {
    (void)p;   // 当前各类型的加速度都只依赖 v / t；保留 p 以便将来加位置相关力（如地磁/风场）
    switch (cfg_.type) {
        case TargetType::DART: {
            const double k = 0.5 * cfg_.rho * cfg_.drag_cd * cfg_.drag_area / cfg_.mass;
            const Vec3   v_rel = v - cfg_.wind;
            return cfg_.gravity + drag_accel(v_rel, k) + cfg_.a_maneuver;
        }
        case TargetType::CIRCLE: {
            // 定常盘旋：速度方向绕 +z 持续旋转 → 向心加速度 a = ω × v
            // （水平飞行，不计重力；|v| 恒定，a ⊥ v）
            const Vec3 omega{0.0, 0.0, cfg_.turn_rate};
            return cross(omega, v);
        }
        case TargetType::SNAKE: {
            // 沿初速度方向的直线飞行 + 侧向正弦机动加速度
            const Vec3 dir = normalized(v0_);
            Vec3 lat = cross(Vec3{0.0, 0.0, 1.0}, dir);      // 水平侧向（⊥ 航向）
            if (norm(lat) < 1e-6) lat = Vec3{1.0, 0.0, 0.0}; // 垂直上飞时的退化
            lat = normalized(lat);
            return lat * (cfg_.snake_acc * std::sin(cfg_.snake_freq * t));
        }
        default:
            return Vec3{};
    }
}

void TargetModel::build_table() {
    const double dt = std::max(1e-5, cfg_.dt_table);

    Vec3   p = cfg_.p0;
    Vec3   v = v0_;
    double t = 0.0;

    tab_.clear();
    tab_.reserve(static_cast<std::size_t>(cfg_.t_max / dt) + 2);
    tab_.push_back({t, p, v, accel_at(p, v, t)});

    t_flight_    = cfg_.t_max;
    hits_ground_ = false;

    while (t < cfg_.t_max) {
        // ---- RK4 推进一步 ----
        const Vec3 k1p = v,                                        k1v = accel_at(p, v, t);
        const Vec3 k2p = v + k1v * (dt * 0.5),                     k2v = accel_at(p + k1p * (dt * 0.5), v + k1v * (dt * 0.5), t + dt * 0.5);
        const Vec3 k3p = v + k2v * (dt * 0.5),                     k3v = accel_at(p + k2p * (dt * 0.5), v + k2v * (dt * 0.5), t + dt * 0.5);
        const Vec3 k4p = v + k3v * dt,                             k4v = accel_at(p + k3p * dt,         v + k3v * dt,         t + dt);

        const Vec3 p_new = p + (k1p + k2p * 2.0 + k3p * 2.0 + k4p) * (dt / 6.0);
        const Vec3 v_new = v + (k1v + k2v * 2.0 + k3v * 2.0 + k4v) * (dt / 6.0);
        const double t_new = t + dt;

        // ---- 落地判据（下降段穿过地面）----
        if (p_new.z <= cfg_.ground_z && v_new.z < 0.0) {
            tab_.push_back({t_new, p_new, v_new, accel_at(p_new, v_new, t_new)});
            t_flight_    = t_new;
            hits_ground_ = true;
            return;
        }

        p = p_new; v = v_new; t = t_new;
        tab_.push_back({t, p, v, accel_at(p, v, t)});
    }
}

// ============================================================
//  查询：t → {p, v, a}
// ============================================================
TargetState TargetModel::state(double t) const {
    // 夹到有效飞行区间（超出则停在末态）
    if (t < 0.0) t = 0.0;
    if (t > t_flight_) t = t_flight_;

    if (cfg_.type == TargetType::STRAIGHT) {
        return {cfg_.p0 + v0_ * t, v0_, Vec3{0, 0, 0}};
    }
    if (cfg_.type == TargetType::BALLISTIC) {
        return {cfg_.p0 + v0_ * t + cfg_.gravity * (0.5 * t * t),
                v0_ + cfg_.gravity * t,
                cfg_.gravity};
    }

    // ---- 数值型：查表 + 线性插值 ----
    if (tab_.empty()) return {cfg_.p0, v0_, accel_at(cfg_.p0, v0_, 0.0)};

    const auto it = std::upper_bound(
        tab_.begin(), tab_.end(), t,
        [](double val, const Sample &s) { return val < s.t; });

    if (it == tab_.begin()) return {tab_.front().p, tab_.front().v, tab_.front().a};
    if (it == tab_.end())   return {tab_.back().p,  tab_.back().v,  tab_.back().a};

    const Sample &b = *it;
    const Sample &a = *(it - 1);
    const double span = b.t - a.t;
    const double u    = (span > 1e-12) ? (t - a.t) / span : 0.0;

    return {a.p + (b.p - a.p) * u,
            a.v + (b.v - a.v) * u,
            a.a + (b.a - a.a) * u};
}

} // namespace pure_uav
