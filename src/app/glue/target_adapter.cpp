#include "glue/target_adapter.hpp"

#include <cmath>
#include <utility>

#include "glue/camera_undistort.hpp"

namespace pure_uav::app {

TargetAdapter::TargetAdapter(CameraCalibration camera) : camera_(std::move(camera)) {
}

bool TargetAdapter::make(const MissionTargetPacket& packet, std::uint64_t boot_us, guidance::flight::GuidanceTargetSample& out) const {
    out = guidance::flight::GuidanceTargetSample{};
    out.boot_us = boot_us;

    if (packet.present == 0) {
        return true;
    }
    if (!std::isfinite(packet.u_px) || !std::isfinite(packet.v_px) || !std::isfinite(packet.w_px) || !std::isfinite(packet.h_px) || !std::isfinite(packet.confidence) || packet.w_px <= 0.0 || packet.h_px <= 0.0 || packet.confidence < 0.0 || packet.confidence > 1.0 || packet.class_id < 0) {
        return false;
    }

    double u_px = packet.u_px;
    double v_px = packet.v_px;
    if (camera_.calibrated) {
        if (!undistortPixel(camera_, u_px, v_px, u_px, v_px)) {
            return false;
        }
    }

    out.present = true;
    out.has_box = true;
    out.u_px = u_px;
    out.v_px = v_px;
    out.w_px = packet.w_px;
    out.h_px = packet.h_px;
    out.conf = packet.confidence;
    out.class_id = packet.class_id;

    // 测距由检测进程用同帧原生深度投影得到；这里只透传有效 range，不补算。
    if (packet.has_range != 0 && std::isfinite(packet.range_m) && packet.range_m > 0.0) {
        out.has_range = true;
        out.range_m = packet.range_m;
    }
    return true;
}

}  // namespace pure_uav::app
