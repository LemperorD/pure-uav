#include "pure_uav/detection/target_bridge.hpp"

#include <cmath>

namespace pure_uav {
namespace detection {

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
    out.has_range = false;  // 本模块只输出检测框
    out.range_m = 0.0;
    out.has_v_T = false;
    out.v_T = Vec3{};

    if (!r.present || r.count <= 0 || r.count > kMaxDetections || !std::isfinite(r.u_px) || !std::isfinite(r.v_px) || !std::isfinite(r.w_px) || !std::isfinite(r.h_px) || !std::isfinite(r.conf) || r.w_px <= 0 || r.h_px <= 0 || r.conf < 0 || r.conf > 1 || r.class_id < 0) return false;

    out.u_px = r.u_px;
    out.v_px = r.v_px;
    out.has_box = true;
    out.w_px = r.w_px;
    out.h_px = r.h_px;
    out.conf = r.conf;
    out.class_id = r.class_id;
    out.present = true;
    return true;
}

}  // namespace detection
}  // namespace pure_uav
