#pragma once

#include <atomic>
#include <chrono>
#include <cstdint>
#include <string>
#include <thread>

namespace pure_uav {
class Communicator;
}

namespace pure_uav::app {

class Logger;
struct FlightState;
struct MainConfig;

/// OFFBOARD 零速度保活线程；只负责 setpoint 不断流，不参与制导/状态机。
class OffboardZeroStreamer {
  public:
    OffboardZeroStreamer(Communicator& comm, int rate_hz);
    ~OffboardZeroStreamer();

    OffboardZeroStreamer(const OffboardZeroStreamer&) = delete;
    OffboardZeroStreamer& operator=(const OffboardZeroStreamer&) = delete;

    bool start(std::string& error);
    void stop();

    [[nodiscard]] std::uint64_t sent() const;
    [[nodiscard]] std::uint64_t failed() const;
    [[nodiscard]] bool running() const;

  private:
    void run();

    Communicator& comm_;
    std::chrono::microseconds period_us_;
    std::atomic<bool> running_{false};
    std::thread thread_;
    std::atomic<std::uint64_t> sent_{0};
    std::atomic<std::uint64_t> failed_{0};
};

/// 进入 AUTO_LAND/NAV_LAND，等落地并 disarm；失败时保持解锁状态交给上层。
bool landAndDisarm(Communicator& comm, const MainConfig& cfg, Logger& log, FlightState& state, OffboardZeroStreamer& streamer);

/// 链路/模式/阶段失败时的安全退出：先切降落，再停 setpoint。
void safeAbort(Communicator& comm, const MainConfig& cfg, Logger& log, FlightState& state, OffboardZeroStreamer* streamer);

}  // namespace pure_uav::app
