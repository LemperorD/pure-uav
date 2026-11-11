#pragma once

#include <algorithm>
#include <cmath>
#include <string>

#include "camera_frame.hpp"
#include "guidance_flight_types.hpp"

namespace pure_uav::app {
// 检测测距与制导反投影必须使用同一原始彩色像素模型。
inline bool guidanceCameraMatchesStream(const guidance::flight::GuidanceCameraModel& cam, const camera::StreamMeta& stream, std::string& error) {
    error.clear();
    // 未安装标定的制导不会运行；无完整测距标定的检测只能发布 has_range=false。
    if (!cam.valid() || stream.range_calibration_valid != 1) return true;
    const auto equal = [](double a, double b) { return std::isfinite(a) && std::isfinite(b) && std::abs(a - b) <= 1e-9 * std::max({1.0, std::abs(a), std::abs(b)}); };
    bool matches = cam.width == static_cast<int>(stream.width) && cam.height == static_cast<int>(stream.height) && equal(cam.fx, stream.color_fx) && equal(cam.fy, stream.color_fy) && equal(cam.cx, stream.color_cx) && equal(cam.cy, stream.color_cy);
    for (int i = 0; i < 8; ++i) matches = matches && equal(i < 5 ? cam.distortion[i] : 0.0, stream.color_distortion[i]);
    if (!matches) error = "guidance-camera.yaml 与 camera_server 的彩色内参/畸变/分辨率不一致；请统一原始彩色标定";
    return matches;
}
}  // namespace pure_uav::app
