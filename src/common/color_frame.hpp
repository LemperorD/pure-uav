#pragma once

#include <cstdint>
#include <opencv2/core.hpp>

namespace pure_uav {

enum class ColorEncoding { jpeg, rgb, yuyv, bgr };

struct EncodedColorFrame {
    cv::Mat bytes;
    ColorEncoding encoding = ColorEncoding::jpeg;
    int width = 0;
    int height = 0;
    int64_t host_ns = 0;
    int64_t device_us = 0;
    uint64_t seq = 0;
    // 与彩色属于同一帧集的原生深度，单位毫米；异步消费者须一起复制。
    cv::Mat depth_mm;
    int64_t depth_device_us = 0;
};

}  // namespace pure_uav
