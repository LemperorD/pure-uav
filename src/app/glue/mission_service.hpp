#pragma once

#include "glue/guidance_session.hpp"
#include "glue/mission_ports.hpp"
#include "glue/target_adapter.hpp"
#include "mission_target_channel.hpp"
#include "pure_uav/vio_runtime.hpp"
#include "startup/preflight_config.hpp"

namespace pure_uav::app {

// main 的任务门面：封装 VIO 发布锁、目标 IPC、时间映射与 Runtime 的组合。
// 生命周期由 main 管理；必须先停止检测监督，再析构本对象的目标通道。
class MissionService {
  public:
    MissionService(VioRuntime& vio, Communicator& comm, mission::RuntimeConfig config, GuidanceAppParams guidance, CameraCalibration camera, PreflightConfig preflight, int heartbeat_max_age_ms, int navigation_max_age_ms, bool detection_enabled);
    ~MissionService();
    int targetPublisherFd() const;
    const mission::RuntimeStatus& tick(bool return_requested, bool manual_takeover = false, bool strike_confirmed = false);
    void stop();
    bool guidanceReady() const;

  private:
    VioRuntime& vio_;
    Communicator& comm_;
    mission::RuntimeConfig config_;
    int navigation_max_age_ms_;
    MissionTargetChannel channel_;
    MissionTargetPacket target_;
    TargetAdapter target_adapter_;
    MavlinkMissionPort flight_;
    MissionGuidanceRunner guidance_;
    mission::MissionRuntime runtime_;
};

}  // namespace pure_uav::app
