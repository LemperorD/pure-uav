#pragma once

#include <vector>

#include "vec3.hpp"

// 目标模型：给定时间 t 返回 {p,v,a}；类型/动力学/参数见
// assets/docs/decisions/0007-target-models.md。

namespace pure_uav {

// 目标真值（仿真内部使用）
struct TargetState {
    Vec3 p, v, a;
};

enum class TargetType {
    STRAIGHT,    // 匀速直线
    BALLISTIC,   // 抛体：只受重力
    DART,        // 飞镖：重力 + 气动阻力
    CIRCLE,      // 定常盘旋
    SNAKE,       // 正弦机动
};

const char *target_type_name(TargetType t);

struct TargetConfig {
    TargetType type = TargetType::DART;

    // ---- 初值 ----
    Vec3   p0{0, 0, 0};
    Vec3   v0{0, 0, 0};          // 直接给速度矢量（launch_speed <= 0 时用）
    double launch_speed = 0.0;   // 发射速率 (m/s)；>0 时用它 + 角度生成 v0
    double launch_elev  = 0.0;   // 发射仰角 (rad)，水平为 0，+ 上抬
    double launch_azim  = 0.0;   // 发射方位角 (rad)，从 +x 绕 +z 逆时针

    // ---- 环境 ----
    Vec3 gravity{0, 0, -9.81};   // 重力加速度（ENU：z 向上，故 −z）
    Vec3 wind{0, 0, 0};          // 风矢量 (m/s)（相对速度用，0 = 无风）

    // ---- 气动（仅 DART 用）——  ⚠ 估计值，待飞镖组确认 ----
    double mass      = 0.40;     // 质量 (kg)
    double drag_cd   = 0.50;     // 阻力系数 Cd
    double drag_area = 2.83e-3;  // 迎风面积 (m²)（≈ φ60mm 圆截面）
    double rho       = 1.225;    // 空气密度 (kg/m³)

    // ---- 机动（可选）----
    Vec3   a_maneuver{0, 0, 0};  // 常值机动加速度 (m/s²)，叠加在 DART 上（模拟规避/制导弹）
    double turn_rate  = 0.6;     // CIRCLE：转弯角速率 (rad/s)
    double snake_acc  = 5.0;     // SNAKE：侧向加速度幅值 (m/s²)
    double snake_freq = 3.0;     // SNAKE：正弦角频率 (rad/s)

    // ---- 数值 ----
    double dt_table = 0.0005;    // 预积分步长 (s)
    double t_max    = 10.0;      // 预积分时长上限 (s)
    double ground_z = 0.0;       // 落地判据高度 (m)
};

class TargetModel {
public:
    TargetModel();
    explicit TargetModel(const TargetConfig &cfg);

    // 唯一对外接口：t → {p, v, a}（t 会被夹到 [0, flight_time]）
    TargetState state(double t) const;

    // 元信息
    double flight_time() const { return t_flight_; }    // 落地/超时时刻 (s)
    bool   hits_ground() const { return hits_ground_; } // 是否在 t_max 内落地
    Vec3   launch_velocity() const { return v0_; }      // 解析后的初速度
    const TargetConfig &config() const { return cfg_; }

private:
    void build();
    void build_analytic();                              // STRAIGHT / BALLISTIC
    void build_table();                                 // DART / CIRCLE / SNAKE
    Vec3 accel_at(const Vec3 &p, const Vec3 &v, double t) const;

    TargetConfig cfg_;
    Vec3   v0_{0, 0, 0};
    double t_flight_    = 0.0;
    bool   hits_ground_ = false;

    struct Sample { double t; Vec3 p, v, a; };
    std::vector<Sample> tab_;    // 数值型目标的预积分轨迹
};

} // namespace pure_uav
