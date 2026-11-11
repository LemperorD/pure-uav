#pragma once

#include <optional>
#include <string>

#include "communicator.hpp"
#include "glue/preflight_evaluator.hpp"
#include "guidance_loop.hpp"
#include "mission/mission_runtime.hpp"
#include "startup/guidance_app_params.hpp"
#include "startup/preflight_config.hpp"

namespace pure_uav::app {

// 共用 main 的 Communicator；不拥有设备、不启动线程。
class MavlinkMissionPort final : public mission::MissionFlightPort {
  public:
    MavlinkMissionPort(Communicator& comm, double pose_max_age_s, int heartbeat_max_age_ms, PreflightConfig preflight);
    mission::FlightFeedback feedback(double now_s) override;
    bool send(const GuidanceCmd& command, std::uint64_t session) override;
    bool request(mission::FlightRequest request, std::uint64_t session) override;

  private:
    Communicator& comm_;
    int pose_max_age_ms_;
    int heartbeat_max_age_ms_;
    PreflightEvaluator preflight_;
    std::uint64_t session_ = 0;
    std::uint64_t preflight_session_ = 0;
    bool has_preflight_session_ = false;
    std::uint64_t pose_boot_us_ = 0;
    double pose_time_s_ = -1.0;
    std::string last_preflight_reason_;
};

// GuidanceLoop 的发送出口仅缓存结果；真实发送仍只发生在 MissionRuntime。
class MissionGuidanceRunner final : public mission::MissionGuidancePort {
  public:
    explicit MissionGuidanceRunner(GuidanceAppParams params);
    bool ready() const override;
    bool reset() override;
    std::optional<GuidanceCmd> step(const mission::MissionInput& input) override;

  private:
    struct InsSource final : flight::GuidanceInsSource {
        flight::GuidanceInsSample value;
        std::uint64_t delivered_us = 0;
        bool poll(flight::GuidanceInsSample& out) override;
        bool alive() const override;
    } ins_;
    struct TargetSource final : flight::GuidanceTargetSource {
        flight::GuidanceTargetSample value;
        std::uint64_t delivered_us = 0;
        bool poll(flight::GuidanceTargetSample& out) override;
        bool alive() const override;
    } target_;
    flight::GuidanceNullSink sink_;
    GuidanceAppParams params_;
    std::optional<flight::GuidanceLoop> loop_;
};

}  // namespace pure_uav::app
