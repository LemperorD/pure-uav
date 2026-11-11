// FrameResult → GuidanceTargetSample 桥接实现。
#include "target_bridge.hpp"

#include <cmath>

namespace pure_uav::detection {

bool to_target_sample(const FrameResult& r, uint64_t boot_us, pure_uav::guidance::flight::GuidanceTargetSample& out) {
    // 先置成"无目标"的降级状态，保证 present=false 时主循环可直接进 SEARCH。
    out.boot_us = boot_us;
    out.u_px = 0.0;
    out.v_px = 0.0;
    out.has_box = false;
    out.w_px = 0.0;
    out.h_px = 0.0;
    out.conf = 0.0;
    out.class_id = -1;
    out.present = false;
    out.has_range = false;
    out.range_m = 0.0;
    out.has_v_T = false;
    out.v_T = Vec3{};

    if (!r.present || r.count <= 0 || r.count > kMaxDetections || !std::isfinite(r.u_px) || !std::isfinite(r.v_px) || !std::isfinite(r.w_px) || !std::isfinite(r.h_px) || r.w_px <= 0 || r.h_px <= 0 || !std::isfinite(r.conf) || r.conf < 0 || r.conf > 1 || r.class_id < 0) {
        return false;
    }

    out.u_px = r.u_px;
    out.v_px = r.v_px;
    out.has_box = true;
    out.w_px = r.w_px;
    out.h_px = r.h_px;
    out.conf = r.conf;
    out.class_id = r.class_id;
    out.present = true;
    out.has_range = r.has_range && std::isfinite(r.range_m) && r.range_m > 0;
    out.range_m = out.has_range ? r.range_m : 0.0;
    return true;
}

app::MissionTargetPacket to_mission_target_packet(const FrameResult& result, int64_t captured_ns) {
    guidance::flight::GuidanceTargetSample target;
    to_target_sample(result, 0, target);
    app::MissionTargetPacket packet;
    packet.captured_ns = captured_ns;
    packet.sequence = result.seq;
    packet.present = target.present ? 1U : 0U;
    packet.u_px = target.u_px;
    packet.v_px = target.v_px;
    packet.w_px = target.w_px;
    packet.h_px = target.h_px;
    packet.confidence = target.conf;
    packet.class_id = target.class_id;
    packet.has_range = target.has_range ? 1U : 0U;
    packet.range_m = target.range_m;
    return packet;
}

}  // namespace pure_uav::detection
