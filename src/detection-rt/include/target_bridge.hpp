#pragma once
// FrameResult → flight::GuidanceTargetSample 桥接；不读时钟、不做坐标换算。
// 时间戳语义见 assets/docs/interfaces.md §4。

#include <cstdint>

#include "detection_types.hpp"
#include "guidance_flight_types.hpp"
#include "mission_target_channel.hpp"

namespace pure_uav::detection {

/// 把检测结果填成制导目标样本。
/// @param[in] r 检测结果
/// @param[in] boot_us 已换算到飞控 boot 时钟的采样时刻（µs）
/// @param[out] out 无目标时也写入 present=false 的降级内容
/// @return 有目标为 true。
bool to_target_sample(const FrameResult& r, uint64_t boot_us, pure_uav::guidance::flight::GuidanceTargetSample& out);

// 同机父子进程传输仍使用采集时的 CLOCK_MONOTONIC，不提前换算飞控时间。
app::MissionTargetPacket to_mission_target_packet(const FrameResult& result, int64_t captured_ns);

}  // namespace pure_uav::detection
