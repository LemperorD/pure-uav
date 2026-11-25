#pragma once

#include <string>
#include <vector>

#include "camera_frame.hpp"
#include "color_frame.hpp"

namespace pure_uav::detection {
// 返回的 Mat 引用 payload；submit() 会将彩色和深度一起复制到工作线程。
bool unpackColorFrame(const camera::StreamMeta& stream, const camera::FrameMeta& meta, std::vector<uint8_t>& payload, EncodedColorFrame& out, std::string& error);
}  // namespace pure_uav::detection
