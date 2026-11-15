#include "shm_color_frame.hpp"

#include <limits>

namespace pure_uav::detection {
bool unpackColorFrame(const camera::StreamMeta& stream, const camera::FrameMeta& meta, std::vector<uint8_t>& payload, EncodedColorFrame& out, std::string& error) {
    out = {};
    const uint64_t total = static_cast<uint64_t>(meta.depth_bytes) + meta.color_bytes;
    const uint64_t depth_bytes = static_cast<uint64_t>(stream.depth_width) * stream.depth_height * sizeof(uint16_t);
    const bool has_depth = (meta.flags & camera::kFrameHasDepth) != 0;
    if (meta.gray_bytes != 0 || total != payload.size() || meta.color_bytes == 0 || meta.color_bytes > static_cast<uint32_t>(std::numeric_limits<int>::max()) || meta.width != stream.width || meta.height != stream.height || meta.width == 0 || meta.height == 0 || meta.width > 4096 ||
        meta.height > 2160 || meta.host_ns <= 0 || meta.seq == 0 || meta.color_encoding > static_cast<uint32_t>(ColorEncoding::bgr) || has_depth != (meta.depth_bytes != 0) ||
        (has_depth && (stream.has_depth != 1 || stream.depth_width == 0 || stream.depth_height == 0 || stream.depth_width > 8192 || stream.depth_height > 8192 || meta.depth_bytes != depth_bytes))) {
        error = "Malformed camera_server color/depth frame dimensions, lengths or flags";
        return false;
    }
    out.bytes = cv::Mat(1, static_cast<int>(meta.color_bytes), CV_8UC1, payload.data() + meta.depth_bytes);
    if (has_depth) out.depth_mm = cv::Mat(static_cast<int>(stream.depth_height), static_cast<int>(stream.depth_width), CV_16UC1, payload.data());
    out.encoding = static_cast<ColorEncoding>(meta.color_encoding);
    out.width = static_cast<int>(meta.width);
    out.height = static_cast<int>(meta.height);
    out.host_ns = meta.host_ns;
    out.device_us = meta.device_us;
    out.depth_device_us = meta.depth_device_us;
    out.seq = meta.seq;
    error.clear();
    return true;
}
}  // namespace pure_uav::detection
