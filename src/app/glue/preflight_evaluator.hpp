#pragma once

#include <cstdint>
#include <string>

#include "msg.hpp"
#include "startup/preflight_config.hpp"

namespace pure_uav::app {

/// 任务 Preflight 需要的全部输入；由 MavlinkMissionPort 从 Communicator 填充。
struct PreflightInput {
    bool connected = false;
    bool target_locked = false;
    bool autopilot_px4 = false;
    bool session_stable = true;
    bool armed = false;
    bool landed = false;
    std::uint32_t mode = 0;

    std::int64_t heartbeat_age_ms = -1;
    bool attitude_seen = false;
    std::int64_t attitude_age_ms = -1;
    Attitude attitude;
    bool position_seen = false;
    std::int64_t position_age_ms = -1;
    LocalPosition position;
    bool battery_seen = false;
    Battery battery;
    bool system_time_seen = false;
    std::int64_t system_time_age_ms = -1;
};

struct PreflightResult {
    bool ok = false;
    std::string reason;
};

/// 纯函数式检查器：不读设备、不等待、不分配后台资源。
class PreflightEvaluator {
  public:
    explicit PreflightEvaluator(PreflightConfig config);

    [[nodiscard]] PreflightResult evaluate(const PreflightInput& input) const;
    [[nodiscard]] const PreflightConfig& config() const {
        return config_;
    }

  private:
    PreflightConfig config_;
};

}  // namespace pure_uav::app
