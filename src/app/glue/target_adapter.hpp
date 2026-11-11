#pragma once

#include <cstdint>

#include "glue/guidance_session.hpp"
#include "guidance_flight_types.hpp"
#include "mission_target_channel.hpp"

namespace pure_uav::app {

/// 把检测目标包映射成制导目标样本。
/// 当前只做像素校正；range 仍由后续的 VIO 测距适配器填充。
class TargetAdapter {
  public:
    explicit TargetAdapter(CameraCalibration camera);

    /// @param[in] boot_us 已换算到飞控 boot 时钟的目标采样时刻
    /// @return 目标包内容合法时为 true；present=false 也返回 true。
    bool make(const MissionTargetPacket& packet, std::uint64_t boot_us, guidance::flight::GuidanceTargetSample& out) const;

  private:
    CameraCalibration camera_;
};

}  // namespace pure_uav::app
