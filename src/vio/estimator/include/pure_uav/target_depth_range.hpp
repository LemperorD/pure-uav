#pragma once

#include "pure_uav/depth_range.hpp"
#include "sample.hpp"

namespace pure_uav {

// 检测先填目标像素/曝光时间/是否存在，再调用。只写距离，失败时清除上帧残留。
// 投影与邻域查询在视觉处理线程完成，不放进 GuidanceLoop 的 250Hz poll() 路径。
inline bool fillTargetDepthRange(app::TargetSample& target, const DepthRangeProjector& depth) {
    target.has_range = false;
    target.range_m = 0.0;
    if (!target.present) return false;
    const auto range = depth.sample(target.u_px, target.v_px, target.boot_us);
    target.has_range = range.has_range;
    target.range_m = range.range_m;
    return target.has_range;
}

}  // namespace pure_uav
