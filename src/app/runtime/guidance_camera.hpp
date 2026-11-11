#pragma once

#include <algorithm>
#include <cmath>
#include <string>

#include "camera_frame.hpp"
#include "guidance_flight_types.hpp"

namespace pure_uav::app {
// 测距是否可用：检测进程用同一帧的原生深度 + 深度内参 + D2C 外参 + 彩色标定解算距离。
// 缺任一项时检测仍能出框，但 has_range 恒为 false，任务不会进入 strike。
// 返回空字符串表示可用，否则给出面向操作者的中文原因。
inline std::string rangeMeasurementError(const camera::StreamMeta& stream) {
    if (stream.has_depth != 1) return "camera.yaml 的 use_depth=false 或深度流未开启";
    if (stream.range_calibration_valid != 1) return "相机服务未提供完整的深度内参/D2C 外参/彩色标定";
    return {};
}

// 检测测距与制导反投影必须使用同一原始彩色像素模型。
inline bool guidanceCameraMatchesStream(const guidance::flight::GuidanceCameraModel& cam, const camera::StreamMeta& stream, std::string& error) {
    error.clear();
    // 制导未标定时不会运行；相机服务无完整测距标定时 color_fx/color_distortion 也不可信，
    // 无法核对彩色内参。两种情况都不影响任务安全：制导未标定不会进 strike，
    // 测距不可用则由 rangeMeasurementError 在启动期报错。
    if (!cam.valid() || stream.range_calibration_valid != 1) return true;
    const auto equal = [](double a, double b) { return std::isfinite(a) && std::isfinite(b) && std::abs(a - b) <= 1e-9 * std::max({1.0, std::abs(a), std::abs(b)}); };
    bool matches = cam.width == static_cast<int>(stream.width) && cam.height == static_cast<int>(stream.height) && equal(cam.fx, stream.color_fx) && equal(cam.fy, stream.color_fy) && equal(cam.cx, stream.color_cx) && equal(cam.cy, stream.color_cy);
    for (int i = 0; i < 8; ++i) matches = matches && equal(i < 5 ? cam.distortion[i] : 0.0, stream.color_distortion[i]);
    if (!matches) error = "guidance-camera.yaml 与 camera_server 的彩色内参/畸变/分辨率不一致；请统一原始彩色标定";
    return matches;
}
}  // namespace pure_uav::app
