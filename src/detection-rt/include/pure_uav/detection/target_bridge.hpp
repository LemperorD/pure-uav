#pragma once
// FrameResult → flight::GuidanceTargetSample 桥接；不读时钟、不做坐标换算。
// 时间戳语义见 assets/docs/interfaces.md §4。

#include <cstdint>

#include "pure_uav/detection/detection_types.hpp"
#include "guidance_flight_types.hpp"

namespace pure_uav {
namespace detection {

/// 把检测结果填成制导目标样本。
/// @param[in] r 检测结果
/// @param[in] boot_us 已换算到飞控 boot 时钟的采样时刻（µs）
/// @param[out] out 无目标时也写入 present=false 的降级内容
/// @return 有目标为 true。
bool to_target_sample(const FrameResult& r, uint64_t boot_us,
                      pure_uav::guidance::flight::GuidanceTargetSample& out);

}  // namespace detection
}  // namespace pure_uav
