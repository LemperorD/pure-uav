#pragma once
// =============================================================================
//  FrameResult → pure_uav::guidance::flight::GuidanceTargetSample 桥接
//
//  【时间戳语义（必须看清）】
//    FrameResult::host_us 是**帧到达主机的时刻**（Unix 微秒，system_clock），
//      · 不是曝光中点；
//      · 不是飞控 boot 时钟；
//      · 曝光→到达的链路时延本次**不补偿**。
//    调用方负责用通信模块的 SystemTime（time_unix_usec + time_boot_ms）把它
//    换算成飞控 boot 微秒，再作为 boot_us 传进来；本函数不读时钟、不换算。
//
//  【映射表（与 spec 一致）】
//    GuidanceTargetSample.boot_us ← 调用方传入（已换算到飞控 boot 时钟）
//    u_px / v_px            ← 主目标框中心
//    has_box                ← present
//    w_px / h_px            ← 主目标框宽高
//    conf                   ← 主目标置信度
//    class_id               ← 主目标类别
//    present                ← count > 0
//    has_range / range_m    ← 恒为 false / 0（本模块只输出检测框）
//    has_v_T / v_T          ← 恒为 false / 零向量（真机拿不到目标速度）
//
//  返回 false 表示没有可用目标（present=false 或字段非法），此时 out 仍会被
//  置成“无目标”的降级内容（present=false），主循环可据此进入 SEARCH。
// =============================================================================

#include <cstdint>

#include "pure_uav/detection/detection_types.hpp"
#include "guidance_flight_types.hpp"   // flight::GuidanceTargetSample

namespace pure_uav {
namespace detection {

bool to_target_sample(const FrameResult& r, uint64_t boot_us,
                      pure_uav::guidance::flight::GuidanceTargetSample& out);

}  // namespace detection
}  // namespace pure_uav
