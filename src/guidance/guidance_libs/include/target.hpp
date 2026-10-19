#pragma once

#include <vector>

#include "vec3.hpp"

// ============================================================
//  目标模型（设计文档 M2）
//
//  职责：给定时间 t，返回目标真值 {p, v, a}。
//   · **只依赖 t**（不依赖无人机状态）→ 可预生成、可回放、可复用
//   · 解析式优先（STRAIGHT / BALLISTIC）；气动型（DART）用数值预积分成表
//   · 输出 a 的真值，供 APN 使用（真机里靠估计，仿真里先给真值）
//
//  支持类型（按拦截场景优先级）：
//    STRAIGHT   匀速直线（基准对照）            解析
//    BALLISTIC  抛体：只受重力 g                解析
//    DART       飞镖：重力 + 气动阻力            数值 ★
//    CIRCLE     定常盘旋（打地面机器人）          数值
//    SNAKE      正弦机动（鲁棒性压力测试）         数值
//
//  ── 飞镖（DART）动力学（点质量 + 二次阻力）────────────────
//      a = g + a_drag + a_maneuver
//      a_drag = −k · |v_rel| · v_rel
//      k      = ½ · ρ · Cd · A / m          （阻力/惯量系数，单位 1/m）
//      v_rel  = v − v_wind                  （有风时用相对速度）
//
//  为什么"有加速度"很关键：
//    ① 重力使弹道成为**抛物线**（不再是直线）→ 拦截几何完全不同；
//    ② 阻力与 v² 成正比 → 速度单调衰减、弹道比理想抛物线**更早落地**，
//       且 a_T 沿速度反向、大小随速度变化 → APN 的补偿项不再是常值；
//    ③ 于是"最近点"出现在弹道下降段，而非几何中点。
//
//  ⚠ 飞镖的气动参数（质量 / 迎风面积 / Cd）当前为**估计值**（见 TargetConfig
//     注释），需与飞镖组确认后填真实值。已记入《待决问题.md》。
// ============================================================

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
