#include <cmath>
#include <cstdio>
#include <stdexcept>

#include "live_timing.hpp"
#include "pose_gate.hpp"

using namespace pure_uav;
using namespace pure_uav::app;
namespace {
int checks = 0;
void expect(bool value, const char* reason) {
    ++checks;
    if (!value) throw std::runtime_error(reason);
}
VioPoseSnapshot snapshot() {
    VioPoseSnapshot s;
    s.valid = true;
    s.session = 2;
    s.timebase_generation = 7;
    s.pose.timestamp_us = 100;
    s.produced_host_ns = s.sample_host_ns = 1900000000LL;
    s.health = static_cast<int>(VioHealthState::OK);
    s.quality = 90;
    s.pos_sigma_m = 0.15f;
    s.vel_sigma_mps = 0.25f;
    return s;
}
PoseGateContext context() {
    return {99, true, true, 7, true, 2, 2000000000LL, 150};
}
}  // namespace
int main() {
    try {
        auto s = snapshot();
        auto c = context();
        expect(shouldPublish(s, c), "new healthy pose should pass");
        c.last_sent_us = 0;
        expect(shouldPublish(s, c), "first pose should pass");
        c.last_sent_us = 100;
        expect(poseGateReason(s, c) == PoseGateReason::duplicate, "duplicate was accepted");
        c.last_sent_us = 101;
        expect(!shouldPublish(s, c), "older pose was accepted");
        c = context();
        s.session = 1;
        expect(poseGateReason(s, c) == PoseGateReason::session, "old session was accepted");
        s = snapshot();
        ++s.timebase_generation;
        expect(poseGateReason(s, c) == PoseGateReason::timebase, "old generation was accepted");
        s = snapshot();
        c.publishable = false;
        expect(poseGateReason(s, c) == PoseGateReason::lost, "unpublishable pose was accepted");
        c = context();
        c.clock_valid = false;
        expect(poseGateReason(s, c) == PoseGateReason::timebase, "invalid clock was accepted");
        c = context();
        c.connected = false;
        expect(poseGateReason(s, c) == PoseGateReason::disconnected, "disconnected pose was accepted");
        c = context();
        s.valid = false;
        expect(!shouldPublish(s, c), "invalid snapshot was accepted");
        s = snapshot();
        s.pose.timestamp_us = 0;
        expect(!shouldPublish(s, c), "zero timestamp was accepted");
        s = snapshot();
        s.produced_host_ns = s.sample_host_ns = 1850000000LL;
        expect(shouldPublish(s, c), "age == max_age_ms should pass");
        --s.produced_host_ns;
        expect(poseGateReason(s, c) == PoseGateReason::stale, "age just beyond limit should fail");
        s = snapshot();
        s.sample_host_ns = 1700000000LL;
        expect(poseGateReason(s, c) == PoseGateReason::stale, "old capture with fresh output should fail");
        s = snapshot();
        s.produced_host_ns = c.now_ns + 1;
        expect(!shouldPublish(s, c), "future output timestamp should fail");
        s = snapshot();
        s.sample_host_ns = 0;
        expect(!shouldPublish(s, c), "unknown capture time should fail");
        s = snapshot();
        c.max_age_ms = 0;
        expect(!shouldPublish(s, c), "invalid age limit should fail");
        c = context();
        s.health = static_cast<int>(VioHealthState::LOST);
        expect(!shouldPublish(s, c), "LOST snapshot should fail even if atomics say otherwise");
        s.health = static_cast<int>(VioHealthState::UNKNOWN);
        expect(!shouldPublish(s, c), "UNKNOWN snapshot should fail");
        s.health = static_cast<int>(VioHealthState::DEGRADED);
        expect(shouldPublish(s, c), "valid DEGRADED snapshot should pass before takeoff as well");
        s = snapshot();
        s.pose.x = NAN;
        expect(!shouldPublish(s, c), "nonfinite position should fail");
        s = snapshot();
        s.pose.q[0] = 0;
        expect(!shouldPublish(s, c), "invalid quaternion should fail");
        s = snapshot();
        s.pose.frame_id = VisionFrame::LOCAL_NED;
        expect(!shouldPublish(s, c), "wrong reference frame should fail");
        s = snapshot();
        s.pos_sigma_m = INFINITY;
        expect(!shouldPublish(s, c), "infinite sigma should fail");
        s = snapshot();
        s.quality = 0;
        expect(!shouldPublish(s, c), "invalid quality should fail");
        expect(odometryResetCounter(255) == 255 && odometryResetCounter(256) == 0 && odometryResetCounter(257) == 1, "reset counter must wrap");
        std::printf("PASS: %d publication gate checks\n", checks);
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FAIL: %s\n", e.what());
        return 1;
    }
}
