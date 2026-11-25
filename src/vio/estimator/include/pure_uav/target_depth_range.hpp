#pragma once

#include "pure_uav/depth_range.hpp"

namespace pure_uav {
// 目标像素与深度必须使用同一个时钟域。模板兼容检测结果及制导目标类型。
// 在视觉线程调用；每次先清除旧距离，不把测距计算放进制导 poll()。
template <class Target>
bool fillTargetDepthRange(Target& target, const DepthRangeProjector& depth, uint64_t color_timestamp_us) {
    target.has_range = false;
    target.range_m = 0.0;
    if (!target.present) return false;
    const auto range = depth.sample(target.u_px, target.v_px, color_timestamp_us);
    target.has_range = range.has_range;
    target.range_m = range.range_m;
    return target.has_range;
}
}  // namespace pure_uav
