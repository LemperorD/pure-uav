#include "glue/mission_service.hpp"

#include <chrono>
#include <stdexcept>
#include <utility>

#include "pose_gate.hpp"

namespace pure_uav::app {
MissionService::MissionService(VioRuntime& vio, Communicator& comm, mission::RuntimeConfig config, GuidanceAppParams guidance, int heartbeat_max_age_ms, int navigation_max_age_ms, bool detection_enabled)
    : vio_(vio), comm_(comm), config_(config), navigation_max_age_ms_(navigation_max_age_ms), flight_(comm, config.mission.pose_max_age_s, heartbeat_max_age_ms), guidance_(std::move(guidance)), runtime_(config, flight_, &guidance_) {
    if (detection_enabled && !channel_.open()) {
        throw std::runtime_error("cannot open mission target channel");
    }
}
MissionService::~MissionService() {
    stop();
}
int MissionService::targetPublisherFd() const {
    return channel_.publisherFd();
}
bool MissionService::guidanceReady() const {
    return guidance_.ready();
}
void MissionService::stop() {
    runtime_.stop();
}
const mission::RuntimeStatus& MissionService::tick(bool return_requested, bool manual_takeover, bool strike_confirmed) {
    // 持锁到 Runtime 最终发送，VIO 重建不能越过本拍的检查和串口写。
    auto publication = vio_.publication();
    const auto& snapshot = publication.snapshot();
    mission::RuntimeSignals signals;
    const auto now_ns = std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
    signals.now_s = static_cast<double>(now_ns) * 1e-9;
    signals.navigation_generation = publication.generation();
    PoseGateContext gate;
    gate.publishable = publication.publishable();
    gate.clock_valid = publication.clockValid();
    gate.timebase_generation = publication.generation();
    gate.connected = comm_.isConnected();
    gate.session = comm_.reconnectCount();
    gate.now_ns = now_ns;
    gate.max_age_ms = navigation_max_age_ms_;
    signals.navigation_ready = vio_.running() && poseGateReason(snapshot, gate) == PoseGateReason::allow;
    signals.start_requested = signals.navigation_ready;
    signals.return_requested = return_requested;
    signals.manual_takeover = manual_takeover;
    signals.strike_confirmed = strike_confirmed;
    const auto target_max_age_ns = static_cast<int64_t>(config_.mission.target_max_age_s * 1e9);
    channel_.latest(target_, now_ns, target_max_age_ns);
    uint64_t target_boot_us = 0;
    if (target_.captured_ns > 0 && target_.captured_ns <= now_ns && now_ns - target_.captured_ns <= target_max_age_ns && publication.hostToFcuUs(target_.captured_ns, target_boot_us)) {
        auto& target = signals.target;
        target.boot_us = target_boot_us;
        target.present = target_.present != 0;
        target.has_box = target.present;
        target.u_px = target_.u_px;
        target.v_px = target_.v_px;
        target.w_px = target_.w_px;
        target.h_px = target_.h_px;
        target.conf = target_.confidence;
        target.class_id = target_.class_id;
        target.has_range = target_.has_range != 0;
        target.range_m = target_.range_m;
        signals.target_time_s = static_cast<double>(target_.captured_ns) * 1e-9;
    }
    return runtime_.tick(signals);
}
}  // namespace pure_uav::app
