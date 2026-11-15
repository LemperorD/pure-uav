#pragma once

#include <algorithm>

#include "camera_frame.hpp"
#include "pure_uav/depth_range.hpp"

namespace pure_uav::camera {
// IPC 保存 C2D（VIO 口径），测距使用 D2C；只在完整标定有效时交付。
inline DepthRangeCalibration rangeCalibration(const StreamMeta& stream) {
    if (stream.range_calibration_valid != 1 || stream.has_depth != 1) return {};
    DepthRangeCalibration c;
    c.depth.width = static_cast<int>(stream.depth_width);
    c.depth.height = static_cast<int>(stream.depth_height);
    c.depth.fx = stream.fx;
    c.depth.fy = stream.fy;
    c.depth.cx = stream.cx;
    c.depth.cy = stream.cy;
    std::copy(std::begin(stream.depth_distortion), std::end(stream.depth_distortion), c.depth.distortion.begin());
    c.color.width = static_cast<int>(stream.width);
    c.color.height = static_cast<int>(stream.height);
    c.color.fx = stream.color_fx;
    c.color.fy = stream.color_fy;
    c.color.cx = stream.color_cx;
    c.color.cy = stream.color_cy;
    std::copy(std::begin(stream.color_distortion), std::end(stream.color_distortion), c.color.distortion.begin());
    const cv::Matx33d R_c2d(stream.R_c2d);
    c.R_depth_to_color = R_c2d.t();
    c.t_depth_to_color_m = -(c.R_depth_to_color * cv::Vec3d(stream.t_mm[0], stream.t_mm[1], stream.t_mm[2])) * 1e-3;
    return c.valid() ? c : DepthRangeCalibration{};
}
}  // namespace pure_uav::camera
