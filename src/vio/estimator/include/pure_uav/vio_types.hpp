// 传感器数据的内部表示（与具体数据源解耦）。
//
// 在线运行时 ImuSample.timestamp_us 是飞控 boot 时间微秒；
// 离线回放时是数据集里的时间戳微秒（session05 bag 里就是 Unix 微秒）。

#pragma once

#include <cstdint>

namespace pure_uav {

struct ImuSample {
    uint64_t timestamp_us = 0;  // us
    double gx = 0.0, gy = 0.0, gz = 0.0;  // rad/s，机体 FRD
    double ax = 0.0, ay = 0.0, az = 0.0;  // m/s²，比力（含重力），机体 FRD
};

struct CameraFrame {
    uint64_t timestamp_us = 0;  // us
    // 由数据源保证是单通道 8 位灰度图（OpenVINS 的 KLT 前端要求）。
    // 这里不直接包含 cv::Mat，避免 vio_types 依赖 OpenCV；
    // 需要图像时用 cv::Mat 单独传递。
};

}  // namespace pure_uav
