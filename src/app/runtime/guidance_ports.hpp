#pragma once

#include <algorithm>
#include <cmath>

#include "communicator.hpp"
#include "guidance_flight_types.hpp"

namespace pure_uav::app {
namespace flight = guidance::flight;

// 飞控姿态和位置分别到达；只有两者都更新且时间差满足窗口才交付。
inline bool makeGuidanceIns(const Attitude& attitude, const LocalPosition& position, double max_skew_s, flight::GuidanceInsSample& out) {
    if (attitude.time_boot_ms == 0 || position.time_boot_ms == 0) { return false;
}
    const auto delta = static_cast<int64_t>(attitude.time_boot_ms) - position.time_boot_ms;
    if (std::abs(static_cast<double>(delta)) > max_skew_s * 1000.0) { return false;
}
    out.boot_us = static_cast<uint64_t>(std::min(attitude.time_boot_ms, position.time_boot_ms)) * 1000;
    out.p = Vec3{position.x, position.y, position.z};
    out.v = Vec3{position.vx, position.vy, position.vz};
    for (int i = 0; i < 4; ++i) { out.q[i] = attitude.q[i];
}
    out.has_position = out.has_attitude = true;
    return true;
}

inline GuidanceCmd makeGuidanceCmd(const flight::GuidanceOutput& output) {
    GuidanceCmd cmd;
    if (output.ctrl == flight::GuidanceControlMode::kAcceleration) {
        cmd.mode = GuidanceMode::ACCELERATION;
        cmd.afx = output.a_cmd_ned.x;
        cmd.afy = output.a_cmd_ned.y;
        cmd.afz = output.a_cmd_ned.z;
    } else {
        cmd.mode = GuidanceMode::VELOCITY;
        cmd.vx = output.v_cmd_ned.x;
        cmd.vy = output.v_cmd_ned.y;
        cmd.vz = output.v_cmd_ned.z;
    }
    return cmd;
}

}  // namespace pure_uav::app
