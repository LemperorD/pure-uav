#include "runtime/vio_pose_publisher.hpp"

#include <chrono>
#include <cstdio>

#include "live_timing.hpp"

namespace pure_uav::app {
namespace {
using Clock = std::chrono::steady_clock;
int64_t nowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
}

}  // namespace

// 公共主循环里 VIO 的一拍：取受保护快照 -> 门控 -> 填质量/重置计数 -> 经同一个
// Communicator 发送。IMU/图像的喂数由 VioRuntime 内部线程按数据到达处理，这里不 feed。
// 返回 false 表示 VIO 工作线程已停止，上层应结束本拍并退出。
bool publishVioPose(VioRuntime& vio, Communicator& comm, bool send, int max_age_ms, VioPosePublishState& publish_state) {
    if (!vio.running()) {
        return false;
    }
    // publication guard 只覆盖最终检查与发送，不包含休眠或日志。
    auto publication = vio.publication();
    const auto& snapshot = publication.snapshot();
    const auto session = comm.reconnectCount();
    const auto generation = publication.generation();
    if (session != publish_state.session || generation != publish_state.generation) {
        publish_state.session = session;
        publish_state.generation = generation;
        publish_state.last_sent_us = 0;  // 重连或时间基变化后同步重置去重状态
    }
    PoseGateContext gate;
    gate.last_sent_us = publish_state.last_sent_us;
    gate.publishable = publication.publishable();
    gate.clock_valid = publication.clockValid();
    gate.timebase_generation = publish_state.generation;
    gate.connected = comm.isConnected();
    gate.session = publish_state.session;
    gate.now_ns = nowNs();
    gate.max_age_ms = max_age_ms;
    const auto reason = poseGateReason(snapshot, gate);
    if (reason != PoseGateReason::allow) {
        ++publish_state.blocked[static_cast<size_t>(reason)];
        return true;
    }
    if (!send) {
        ++publish_state.would_send;  // --no-send：门控照跑，只跳过串口发送
        return true;
    }
    // snapshot.pose.timestamp_us 已是主机 Unix 微秒（VioRuntime 经 FcuClock 映射），
    // 发布前不再用 SYSTEM_TIME 做一次时间转换。
    VisionPose pose = snapshot.pose;
    pose.quality = static_cast<int8_t>(snapshot.quality);
    pose.reset_counter = odometryResetCounter(static_cast<uint64_t>(snapshot.rebuilds));
    pose.pos_sigma_m = snapshot.pos_sigma_m;
    pose.vel_sigma_mps = snapshot.vel_sigma_mps;
    // Communicator 在实际写入前会再复核会话，会话变化不会越过这次发送。
    if (comm.pushVisionPoseForSession(pose, publish_state.session)) {
        publish_state.last_sent_us = pose.timestamp_us;  // 只有发送成功才推进去重
        ++publish_state.sent;
    } else {
        ++publish_state.send_errors;
    }
    return true;
}

void logAppStatus(const Communicator& comm, const VioRuntime& vio, const CameraService& camera, const VioPosePublishState& publish_state, uint64_t overruns) {
    const auto stats = vio.stats();
    VioPoseSnapshot snapshot;
    vio.latestPose(snapshot);
    const auto& blocked = publish_state.blocked;
    const double capture_age_ms = snapshot.sample_host_ns > 0 ? static_cast<double>(nowNs() - snapshot.sample_host_ns) * 1e-6 : -1.0;
    std::printf(
        "[MAIN] connected=%d firmware=%s mode=%s session=%llu generation=%llu "
        "imu=%.1fHz health=%s features=%d zupt=%d "
        "sent=%llu send_errors=%llu would_send=%llu "
        "blocked(stale/duplicate/lost/timebase/session/disconnected)=%llu/%llu/%llu/%llu/%llu/%llu "
        "frames=%llu dropped=%llu imu_dropped=%llu camera_dropped=%llu poses=%llu resets=%llu "
        "capture_age_ms=%.1f overruns=%llu error=%s\n",
        static_cast<int>(comm.isConnected()), comm.getAutopilotNameCn(), comm.getFlightModeNameCn(), static_cast<unsigned long long>(publish_state.session), static_cast<unsigned long long>(publish_state.generation), stats.imu_hz, toString(static_cast<VioHealthState>(snapshot.health)),
        snapshot.features, stats.zupt_enabled ? 1 : 0, static_cast<unsigned long long>(publish_state.sent), static_cast<unsigned long long>(publish_state.send_errors), static_cast<unsigned long long>(publish_state.would_send),
        static_cast<unsigned long long>(blocked[static_cast<size_t>(PoseGateReason::stale)]), static_cast<unsigned long long>(blocked[static_cast<size_t>(PoseGateReason::duplicate)]), static_cast<unsigned long long>(blocked[static_cast<size_t>(PoseGateReason::lost)]),
        static_cast<unsigned long long>(blocked[static_cast<size_t>(PoseGateReason::timebase)]), static_cast<unsigned long long>(blocked[static_cast<size_t>(PoseGateReason::session)]), static_cast<unsigned long long>(blocked[static_cast<size_t>(PoseGateReason::disconnected)]),
        static_cast<unsigned long long>(stats.frames_fed), static_cast<unsigned long long>(stats.frames_dropped), static_cast<unsigned long long>(stats.imu_dropped), static_cast<unsigned long long>(camera.dropped()), static_cast<unsigned long long>(stats.poses),
        static_cast<unsigned long long>(stats.timebase_resets), capture_age_ms, static_cast<unsigned long long>(overruns), stats.last_error.empty() ? "-" : stats.last_error.c_str());
}

void logAppStopped(const VioRuntime& vio, const VioPosePublishState& publish_state) {
    const auto stats = vio.stats();
    std::printf("[MAIN] 已停止：sent=%llu send_errors=%llu would_send=%llu frames=%llu poses=%llu resets=%llu error=%s\n", static_cast<unsigned long long>(publish_state.sent), static_cast<unsigned long long>(publish_state.send_errors), static_cast<unsigned long long>(publish_state.would_send),
                static_cast<unsigned long long>(stats.frames_fed), static_cast<unsigned long long>(stats.poses), static_cast<unsigned long long>(stats.timebase_resets), stats.last_error.empty() ? "-" : stats.last_error.c_str());
}

}  // namespace pure_uav::app
