#pragma once

#include <cstdint>
#include <optional>

#include "mission/mission_controller.hpp"

namespace pure_uav::app::mission {

struct FlightFeedback {
    MissionInput input;  // 只使用飞控字段；控制事件由 RuntimeSignals 提供。
    std::uint64_t session = 0;
    bool supported = false;
};

// 组合根注入真实通信或假飞控。所有函数均不等待 ACK，不启动线程。
class MissionFlightPort {
  public:
    virtual ~MissionFlightPort() = default;
    virtual FlightFeedback feedback(double now_s) = 0;
    virtual bool send(const GuidanceCmd& command, std::uint64_t session) = 0;
    virtual bool request(FlightRequest request, std::uint64_t session) = 0;
};

class MissionGuidancePort {
  public:
    virtual ~MissionGuidancePort() = default;
    virtual bool ready() const = 0;
    virtual bool reset() = 0;
    // 仅计算，不能自行向飞控发送。Runtime 统一仲裁指令出口。
    virtual std::optional<GuidanceCmd> step(const MissionInput& input) = 0;
};

// main 与任务层的唯一输入契约。后续检测/裁判适配器也只填此结构。
struct RuntimeSignals {
    double now_s = 0.0;  // steady_clock::time_since_epoch，秒。
    bool start_requested = false;
    bool return_requested = false;
    bool manual_takeover = false;
    bool navigation_ready = false;
    std::uint64_t navigation_generation = 0;
    bool strike_confirmed = false;
    guidance::flight::GuidanceTargetSample target;
    double target_time_s = -1.0;
};

struct RuntimeConfig {
    MissionConfig mission;
    bool send_enabled = false;
    double prestream_s = 1.5;
    double max_setpoint_gap_s = 0.20;
};

struct RuntimeStatus {
    MissionOutput mission;
    std::uint64_t setpoints_sent = 0;
    std::uint64_t requests_sent = 0;
    std::uint64_t would_send = 0;
};

class MissionRuntime {
  public:
    MissionRuntime(RuntimeConfig config, MissionFlightPort& flight, MissionGuidancePort* guidance = nullptr);
    const RuntimeStatus& tick(const RuntimeSignals& signals);
    // 停止只有释放控制权；无后台线程或排队命令。调用后 tick 不会恢复发送。
    void stop();
    [[nodiscard]] const RuntimeStatus& status() const;

  private:
    void fail(MissionReason reason, double now_s);
    void clearStream();
    RuntimeConfig config_;
    MissionController controller_;
    MissionFlightPort& flight_;
    MissionGuidancePort* guidance_;
    RuntimeStatus status_;
    std::optional<std::uint64_t> session_;
    std::uint64_t navigation_generation_ = 0;
    double stream_started_s_ = -1.0;
    double last_sent_s_ = -1.0;
    double last_now_s_ = 0.0;
    bool stopped_ = false;
};

}  // namespace pure_uav::app::mission
