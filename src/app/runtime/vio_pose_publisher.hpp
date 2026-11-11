#pragma once

#include <array>
#include <cstdint>

#include "camera_service.hpp"
#include "communicator.hpp"
#include "pose_gate.hpp"
#include "pure_uav/vio_runtime.hpp"

namespace pure_uav::app {

// VIO 发布状态与统计：会话/时间基变化时重置去重时间戳，累计计数保留。
struct VioPosePublishState {
    uint64_t last_sent_us = 0;
    uint64_t session = 0;
    uint64_t generation = 0;
    uint64_t sent = 0, send_errors = 0, would_send = 0;
    std::array<uint64_t, static_cast<size_t>(PoseGateReason::count)> blocked{};
};

// VIO 在内部线程处理数据；这里仅门控并发布最新位姿，不等待新数据。
bool publishVioPose(VioRuntime& vio, Communicator& comm, bool send, int max_age_ms, VioPosePublishState& publish_state);
void logAppStatus(const Communicator& comm, const VioRuntime& vio, const CameraService& camera, const VioPosePublishState& publish_state, uint64_t overruns);

void logAppStopped(const VioRuntime& vio, const VioPosePublishState& publish_state);

}  // namespace pure_uav::app
