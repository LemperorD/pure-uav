// 无飞控/无相机联调工具的公共核心（header-only，可单元测试）。
//
// 两个工具：
//   tools/odometry_sender.cpp  按脚本合成一条 LOCAL_FRD/BODY_FRD 轨迹，30 Hz 发 ODOMETRY。
//                              用途：不接相机、不跑 VIO，单独验证"飞控能不能收到、轴向/单位对不对"。
//   tools/odometry_probe.cpp   从串口/文件解码 ODOMETRY，打印 + 统计 + 异常告警。
//                              用途：把"上位机发送成功 / 飞控收到 / 飞控融合"三件事分开取证。
//
// 这里只放纯逻辑（轨迹、统计），I/O 留在工具里，测试见 tests/test_odom_tool_core.cpp。

#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <vector>

#include "common/mavlink.h"

namespace pure_uav {
namespace odom_tool {

// ---------------------------------------------------------------------------
// 合成轨迹
// ---------------------------------------------------------------------------
struct PoseSample {
    float x = 0.0f, y = 0.0f, z = 0.0f;  // m，LOCAL_FRD（z 向下为正）
    float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};  // w x y z
    float vx = 0.0f, vy = 0.0f, vz = 0.0f;  // m/s，BODY_FRD
};

struct ScriptStep {
    double begin_s;
    double end_s;
    const char* name;    // 给操作员看的阶段名
    const char* expect;  // 这个阶段飞控端应该看到什么
};

inline const std::vector<ScriptStep>& scriptSteps() {
    static const std::vector<ScriptStep> steps = {
        {0.0, 3.0, "静止", "位置保持 (0,0,0)，姿态不变"},
        {3.0, 6.0, "前移 1 m", "x 从 0 增到 +1，y/z 不变"},
        {6.0, 7.0, "停住", "保持 x=+1"},
        {7.0, 8.0, "退回原点", "x 从 +1 减到 0"},
        {8.0, 9.0, "静止", "保持 (0,0,0)"},
        {9.0, 12.0, "右移 0.6 m", "y 从 0 增到 +0.6"},
        {12.0, 13.0, "停住", "保持 y=+0.6"},
        {13.0, 14.0, "退回原点", "y 从 +0.6 减到 0"},
        {14.0, 15.0, "静止", "保持 (0,0,0)"},
        {15.0, 18.0, "上抬 0.5 m", "z 从 0 变到 -0.5（FRD 向下为正）"},
        {18.0, 19.0, "停住", "保持 z=-0.5"},
        {19.0, 20.0, "落回原点", "z 从 -0.5 回到 0"},
        {20.0, 21.0, "静止", "保持 (0,0,0)"},
        {21.0, 25.0, "偏航 +90°", "航向顺时针转 90°（FRD：绕 z 正向）"},
        {25.0, 27.0, "保持 90°", "位置不变，航向 90°"},
    };
    return steps;
}

inline double scriptDuration() { return scriptSteps().back().end_s; }

// 线性插值 + 机身速度（与位置变化一致）。t 超出脚本长度时返回最后一帧。
inline PoseSample sampleScript(double t_s) {
    PoseSample p;
    auto setYaw = [&](double yaw_rad) {
        p.q[0] = static_cast<float>(std::cos(yaw_rad * 0.5));
        p.q[1] = 0.0f;
        p.q[2] = 0.0f;
        p.q[3] = static_cast<float>(std::sin(yaw_rad * 0.5));
    };
    auto ramp = [](double t, double t0, double t1) {
        if (t <= t0) return 0.0;
        if (t >= t1) return 1.0;
        return (t - t0) / (t1 - t0);
    };

    if (t_s < 3.0) {
        return p;
    } else if (t_s < 6.0) {                       // 前移 0→1
        const double s = ramp(t_s, 3.0, 6.0);
        p.x = static_cast<float>(s * 1.0);
        p.vx = 1.0f / 3.0f;
    } else if (t_s < 7.0) {
        p.x = 1.0f;
    } else if (t_s < 8.0) {                       // 退回 1→0
        const double s = ramp(t_s, 7.0, 8.0);
        p.x = static_cast<float>(1.0 - s);
        p.vx = -1.0f;
    } else if (t_s < 9.0) {
        // 原点
    } else if (t_s < 12.0) {                      // 右移 0→0.6
        const double s = ramp(t_s, 9.0, 12.0);
        p.y = static_cast<float>(s * 0.6);
        p.vy = 0.2f;
    } else if (t_s < 13.0) {
        p.y = 0.6f;
    } else if (t_s < 14.0) {                      // 退回
        const double s = ramp(t_s, 13.0, 14.0);
        p.y = static_cast<float>(0.6 - s * 0.6);
        p.vy = -0.6f;
    } else if (t_s < 15.0) {
        // 原点
    } else if (t_s < 18.0) {                      // 上抬 0→-0.5
        const double s = ramp(t_s, 15.0, 18.0);
        p.z = static_cast<float>(-s * 0.5);
        p.vz = -0.5f / 3.0f;
    } else if (t_s < 19.0) {
        p.z = -0.5f;
    } else if (t_s < 20.0) {                      // 落回
        const double s = ramp(t_s, 19.0, 20.0);
        p.z = static_cast<float>(-0.5 + s * 0.5);
        p.vz = 0.5f;
    } else if (t_s < 21.0) {
        // 原点
    } else {
        const double s = ramp(t_s, 21.0, 25.0);
        setYaw(s * M_PI / 2.0);
    }
    return p;
}

// t 所在阶段的序号；超出返回最后一个
inline int scriptPhaseIndex(double t_s) {
    const auto& steps = scriptSteps();
    for (size_t i = 0; i < steps.size(); ++i) {
        if (t_s < steps[i].end_s) return static_cast<int>(i);
    }
    return static_cast<int>(steps.size()) - 1;
}

// ---------------------------------------------------------------------------
// ODOMETRY 解码统计（probe 用）
// ---------------------------------------------------------------------------
struct ProbeStats {
    uint64_t odometry = 0;
    uint64_t heartbeat = 0;
    uint64_t imu = 0;
    uint64_t other = 0;
    uint64_t time_backwards = 0;   // time_usec 倒退次数
    uint64_t reset_changes = 0;    // reset_counter 变化次数
    uint64_t bad_quality = 0;      // quality < 50（ArduPilot 常见门限）
    uint64_t bad_quat = 0;         // |q|-1 超过 1e-2
    uint64_t bad_frame = 0;        // frame_id/child_frame_id 不是 20/12
    uint64_t gaps = 0;             // 与上一帧间隔 > 3× 中位间隔
    int64_t first_time_us = -1;
    int64_t last_time_us = -1;
    int64_t prev_time_us = -1;
    uint8_t last_reset = 0;
    bool have_reset = false;
    int8_t min_quality = 127;
    int8_t max_quality = -128;
    double interval_sum_us = 0.0;
    uint64_t interval_n = 0;
    std::vector<double> intervals_us;

    void add(const mavlink_odometry_t& o, int64_t host_ns) {
        (void)host_ns;
        ++odometry;
        const int64_t t = static_cast<int64_t>(o.time_usec);
        if (last_time_us >= 0 && t < last_time_us) ++time_backwards;
        if (prev_time_us >= 0) {
            const double dt = static_cast<double>(t - prev_time_us);
            if (dt > 0) {
                intervals_us.push_back(dt);
                interval_sum_us += dt;
                ++interval_n;
                if (intervals_us.size() > 600) {
                    intervals_us.erase(intervals_us.begin());
                }
            }
        }
        prev_time_us = t;
        if (first_time_us < 0) first_time_us = t;
        last_time_us = t;

        if (have_reset && o.reset_counter != last_reset) ++reset_changes;
        last_reset = o.reset_counter;
        have_reset = true;

        if (o.quality < min_quality) min_quality = o.quality;
        if (o.quality > max_quality) max_quality = o.quality;
        if (o.quality < 50) ++bad_quality;

        const double qn = std::sqrt(static_cast<double>(o.q[0]) * o.q[0] +
                                    static_cast<double>(o.q[1]) * o.q[1] +
                                    static_cast<double>(o.q[2]) * o.q[2] +
                                    static_cast<double>(o.q[3]) * o.q[3]);
        if (std::fabs(qn - 1.0) > 1e-2) ++bad_quat;

        if (o.frame_id != 20 || o.child_frame_id != 12) ++bad_frame;
    }

    double medianIntervalUs() const {
        if (intervals_us.empty()) return 0.0;
        std::vector<double> v = intervals_us;
        std::sort(v.begin(), v.end());
        return v[v.size() / 2];
    }

    double rateHz() const {
        const double med = medianIntervalUs();
        return med > 1.0 ? 1e6 / med : 0.0;
    }

    // 检查"明显不对"的间隔：>3×中位
    void countGaps() {
        const double med = medianIntervalUs();
        if (med <= 1.0) return;
        for (double dt : intervals_us) {
            if (dt > 3.0 * med) ++gaps;
        }
    }
};

}  // namespace odom_tool
}  // namespace pure_uav
