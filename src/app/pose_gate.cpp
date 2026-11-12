#include "pose_gate.hpp"

#include <cmath>

namespace pure_uav::app {
PoseGateReason poseGateReason(const VioPoseSnapshot& s, const PoseGateContext& c) {
    if (!c.connected) {
        return PoseGateReason::disconnected;
    }
    if (s.session != c.session) {
        return PoseGateReason::session;
    }
    if (!c.clock_valid || s.timebase_generation != c.timebase_generation) {
        return PoseGateReason::timebase;
    }
    if (!c.publishable || (s.health != static_cast<int>(VioHealthState::OK) && s.health != static_cast<int>(VioHealthState::DEGRADED))) {
        return PoseGateReason::lost;
    }
    if (!s.valid || s.pose.timestamp_us == 0) {
        return PoseGateReason::invalid;
    }
    if (s.pose.timestamp_us <= c.last_sent_us) {
        return PoseGateReason::duplicate;
    }
    const int64_t limit_ns = static_cast<int64_t>(c.max_age_ms) * 1000000LL;
    const auto fresh = [&](int64_t stamp) { return stamp > 0 && c.now_ns >= stamp && c.now_ns - stamp <= limit_ns; };
    if (c.max_age_ms <= 0 || !fresh(s.produced_host_ns) || !fresh(s.sample_host_ns)) {
        return PoseGateReason::stale;
    }
    const auto& p = s.pose;
    if (p.frame_id != VisionFrame::LOCAL_FRD || p.child_frame_id != VisionFrame::BODY_FRD) {
        return PoseGateReason::invalid_pose;
    }
    for (float value : {p.x, p.y, p.z, p.vx, p.vy, p.vz, p.q[0], p.q[1], p.q[2], p.q[3]}) {
        if (!std::isfinite(value)) {
            return PoseGateReason::invalid_pose;
        }
    }
    double norm2 = 0;
    for (float value : p.q) {
        norm2 += static_cast<double>(value) * value;
    }
    if (std::abs(norm2 - 1.0) > 0.02) {
        return PoseGateReason::invalid_pose;
    }
    if (s.quality < 1 || s.quality > 100 || !std::isfinite(s.pos_sigma_m) || !std::isfinite(s.vel_sigma_mps) || s.pos_sigma_m <= 0 || s.vel_sigma_mps <= 0) {
        return PoseGateReason::invalid_pose;
    }
    return PoseGateReason::allow;
}
bool shouldPublish(const VioPoseSnapshot& s, const PoseGateContext& c) {
    return poseGateReason(s, c) == PoseGateReason::allow;
}
const char* toString(PoseGateReason reason) {
    switch (reason) {
        case PoseGateReason::allow:
            return "allow";
        case PoseGateReason::invalid:
            return "invalid";
        case PoseGateReason::duplicate:
            return "duplicate";
        case PoseGateReason::lost:
            return "lost";
        case PoseGateReason::timebase:
            return "timebase";
        case PoseGateReason::session:
            return "session";
        case PoseGateReason::disconnected:
            return "disconnected";
        case PoseGateReason::stale:
            return "stale";
        case PoseGateReason::invalid_pose:
            return "invalid_pose";
        default:
            return "unknown";
    }
}
}  // namespace pure_uav::app
