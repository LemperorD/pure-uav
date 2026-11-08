#pragma once

#include <cstdint>
#include <deque>
#include <random>

#include "guidance_types.hpp"
#include "vec3.hpp"

// 姿态/四元数工具已移到 quat.hpp；这里保留 include 只为向后兼容。
#include "quat.hpp"

// 传感器墙（仿真虚拟视觉）：真值 → 带噪声/延迟/丢帧的 GuidanceState。
// 三级逼真度与配对约定见 assets/docs/decisions/0008-sensor-wall.md。

namespace pure_uav {

// 真值（仿真内部使用；真机上不存在这个结构）
struct TruthState {
    double t = 0.0;
    Vec3 p_M, v_M;               // 无人机 位置/速度
    Vec3 p_T, v_T, a_T;          // 目标   位置/速度/加速度
    double q[4] = {1, 0, 0, 0};  // 机体姿态四元数 [w,x,y,z]（机体→世界；与 PX4/MAVLink 一致）
};

// 相机内参 + 目标特征尺寸（size 反推测距用；仿真专用）
struct GuidanceSimCameraModel {
    double fx = 1200.0;  // 焦距 x (像素)
    double fy = 1200.0;  // 焦距 y (像素)
    double cx = 320.0;   // 主点 x (像素)
    double cy = 240.0;   // 主点 y (像素)
    int width = 640;
    int height = 480;
    double target_size_m = 0.5;  // 目标特征尺寸(m)（飞镖长 / 弹丸直径）
    double min_bbox_px = 1.0;    // 最小可用框(px)，低于此判为丢失
};

// 逼真度等级
enum class SensorFidelity {
    IDEAL,      // 真值透传
    SEMI_REAL,  // 投影 + 像素噪声 + size 反推测距
    FULL_REAL   // 再加 延迟 + 丢帧
};

struct SensorConfig {
    SensorFidelity fidelity = SensorFidelity::IDEAL;
    GuidanceSimCameraModel cam;

    // --- 噪声（SEMI_REAL / FULL_REAL）---
    double pixel_noise_std = 0.1;  // 检测框中心像素噪声 std (px)
    double range_scale_err = 0.0;  // size 反推测距的比例误差（0.05 = +5%）
    double range_bias = 0.0;       // size 反推测距的系统偏差(m)

    // --- 延迟 / 丢帧（仅 FULL_REAL）---
    double latency_s = 0.05;  // 图像采集+处理链路延迟(s)
    double drop_rate = 0.05;  // 每帧丢失概率 0~1（5%）

    std::uint32_t seed = 12345;  // 随机种子（可复现）
};

// 观测统计（写报告 / 画图用）
struct SensorStats {
    long long frames = 0;   // observe() 调用次数
    long long dropped = 0;  // 丢帧次数
    long long lost = 0;     // 目标丢失（出视场 / 太小 / 在身后）
};

class Sensor {
  public:
    explicit Sensor(const SensorConfig& cfg);

    // 真值 → 制导输入（含噪声 / 延迟 / 丢帧）
    GuidanceState observe(const TruthState& truth, double t);

    [[nodiscard]] const SensorStats& stats() const {
        return stats_;
    }
    void reset();

  private:
    // 世界 → 像素投影。false = 本帧不可用（在相机后方 / 出视场 / 目标太小）
    bool project(const TruthState& truth, double& u, double& v, double& w_px) const;
    double noise() {
        return gauss_(rng_);
    }  // N(0,1)

    SensorConfig cfg_;
    SensorStats stats_;
    std::mt19937 rng_;
    std::normal_distribution<double> gauss_{0.0, 1.0};

    // 延迟缓冲（FULL_REAL）
    struct Sample {
        double t;
        GuidanceState st;
    };
    std::deque<Sample> buf_;

    // 视线差分（算 λ̇ 用）
    bool has_prev_ = false;
    Vec3 los_prev_;
    double t_prev_ = 0.0;
};

// （四元数 / 姿态工具已移到 `quat.hpp`，见文件头的 include 说明。）

}  // namespace pure_uav
