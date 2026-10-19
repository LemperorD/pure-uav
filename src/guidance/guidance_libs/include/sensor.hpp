#pragma once

#include <cstdint>
#include <deque>
#include <random>

#include "guidance_types.hpp"
#include "vec3.hpp"

// 姿态 / 四元数工具已移到**核心**（`quat.hpp`）—— 它们语义上是通用刚体数学，
// 不是仿真。这里保留 include 只为**向后兼容**（老代码 `#include "sensor.hpp"`
// 仍能用到 quat_*）。⚠ 新代码建议直接 include "quat.hpp"。
#include "quat.hpp"

// ============================================================
//  传感器墙（设计文档 M4）—— 仿真侧的"虚拟视觉"
//
//  作用：把仿真真值 TruthState 变成"看起来像视觉系统给的" GuidanceState。
//  接口固定：切换 fidelity（逼真度）**不改制导律一行代码**。
//
//  真机上这一层由 detection / vio 组实现，制导只消费 GuidanceState。
//  设计见《仿真代码设计思路.md》M4（三级逼真度）。
//
//  三级逼真度：
//    IDEAL     真值透传（无噪、无延迟、无丢帧）—— 基线
//    SEMI_REAL 相机投影 + 像素噪声 + size 反推测距（λ̇ 走视线差分）
//    FULL_REAL 在 SEMI_REAL 基础上再加 链路延迟 + 丢帧
//
//  ⚠ 配对约定：非 IDEAL 时传感器拿不到目标速度 v_T，制导必须改用
//     LosRateSource::FROM_LOS_DIFFERENCE（真机唯一可行的 ω 来源）。
//     仿真里这一配对由 pn_sim 负责设置。
// ============================================================

namespace pure_uav {

// 真值（仿真内部使用；真机上不存在这个结构）
struct TruthState {
    double t = 0.0;
    Vec3   p_M, v_M;             // 无人机 位置/速度
    Vec3   p_T, v_T, a_T;        // 目标   位置/速度/加速度
    double q[4] = {1, 0, 0, 0};  // 机体姿态四元数 [w,x,y,z]（机体→世界；与 PX4/MAVLink 一致）
};

// 相机内参 + 目标特征尺寸（size 反推测距用）
struct CameraModel {
    double fx = 1200.0;          // 焦距 x (像素)
    double fy = 1200.0;          // 焦距 y (像素)
    double cx = 320.0;           // 主点 x (像素)
    double cy = 240.0;           // 主点 y (像素)
    int    width  = 640;
    int    height = 480;
    double target_size_m = 0.5;  // 目标特征尺寸(m)（飞镖长 / 弹丸直径）
    double min_bbox_px   = 1.0;  // 最小可用框(px)，低于此判为丢失
};

// 逼真度等级
enum class SensorFidelity {
    IDEAL,      // 真值透传
    SEMI_REAL,  // 投影 + 像素噪声 + size 反推测距
    FULL_REAL   // 再加 延迟 + 丢帧
};

struct SensorConfig {
    SensorFidelity fidelity = SensorFidelity::IDEAL;
    CameraModel    cam;

    // --- 噪声（SEMI_REAL / FULL_REAL）---
    double pixel_noise_std = 0.1;   // 检测框中心像素噪声 std (px)
    double range_scale_err = 0.0;   // size 反推测距的比例误差（0.05 = +5%）
    double range_bias      = 0.0;   // size 反推测距的系统偏差(m)

    // --- 延迟 / 丢帧（仅 FULL_REAL）---
    double latency_s = 0.05;        // 图像采集+处理链路延迟(s)
    double drop_rate = 0.05;        // 每帧丢失概率 0~1（5%）

    std::uint32_t seed = 12345;     // 随机种子（可复现）
};

// 观测统计（写报告 / 画图用）
struct SensorStats {
    long long frames  = 0;   // observe() 调用次数
    long long dropped = 0;   // 丢帧次数
    long long lost    = 0;   // 目标丢失（出视场 / 太小 / 在身后）
};

class Sensor {
public:
    explicit Sensor(const SensorConfig &cfg);

    // 真值 → 制导输入（含噪声 / 延迟 / 丢帧）
    GuidanceState observe(const TruthState &truth, double t);

    const SensorStats &stats() const { return stats_; }
    void reset();

private:
    // 世界 → 像素投影。false = 本帧不可用（在相机后方 / 出视场 / 目标太小）
    bool project(const TruthState &truth, double &u, double &v, double &w_px) const;
    double noise() { return gauss_(rng_); }   // N(0,1)

    SensorConfig cfg_;
    SensorStats  stats_;
    std::mt19937 rng_;
    std::normal_distribution<double> gauss_{0.0, 1.0};

    // 延迟缓冲（FULL_REAL）
    struct Sample { double t; GuidanceState st; };
    std::deque<Sample> buf_;

    // 视线差分（算 λ̇ 用）
    bool   has_prev_ = false;
    Vec3   los_prev_;
    double t_prev_ = 0.0;
};

// （四元数 / 姿态工具已移到 `quat.hpp`，见文件头的 include 说明。）

} // namespace pure_uav
