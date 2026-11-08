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
};

}  // namespace pure_uav
