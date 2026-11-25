#pragma once
#include <cstdint>

#include "pure_uav/vio_runtime.hpp"

namespace pure_uav::app {
enum class PoseGateReason { allow, invalid, duplicate, lost, timebase, session, disconnected, stale, invalid_pose, count };
struct PoseGateContext {
    uint64_t last_sent_us = 0;
    bool publishable = false;
    bool clock_valid = false;
    uint64_t timebase_generation = 0;
    bool connected = false;
    uint64_t session = 0;
    int64_t now_ns = 0;
    int max_age_ms = 150;
};
PoseGateReason poseGateReason(const VioPoseSnapshot& s, const PoseGateContext& c);
bool shouldPublish(const VioPoseSnapshot& s, const PoseGateContext& c);
const char* toString(PoseGateReason reason);
}  // namespace pure_uav::app
