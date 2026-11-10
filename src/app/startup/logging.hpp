#pragma once

#include <chrono>
#include <fstream>
#include <string>
#include <thread>

namespace pure_uav {
class Communicator;
}

namespace pure_uav::app {

constexpr int kMainLoopPeriodMs = 50;
constexpr int kStatusPeriodMs = 1000;

enum class Phase {
    kConfig,
    kWaitLink,
    kGuidance,
    kPreflight,
    kGroundReference,
    kArm,
    kTakeoffMode,
    kTakeoffCommand,
    kClimb,
    kHover,
    kOffboard,
    kLand,
    kDisarm,
    kAbort,
    kDone,
};

const char* phaseName(Phase phase);

/// 正常运行时同时输出到控制台与 Log/main_*.log。
class Logger {
  public:
    bool open(std::string& error);
    void info(Phase phase, const std::string& message);
    void warn(Phase phase, const std::string& message);
    void error(Phase phase, const std::string& message);

  private:
    void write(const char* level, Phase phase, const std::string& message);
    std::ofstream file_;
};

std::string fixed(double value, int precision = 2);
std::string modeText(const Communicator& comm);

/// 安装 SIGINT/SIGTERM；处理函数只置退出标志。
void installSignalHandlers();
bool interrupted();
bool sleepInterruptible(int milliseconds);

template <typename Predicate>
bool waitUntil(std::chrono::milliseconds timeout, Predicate predicate) {
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
        if (interrupted()) {
            return false;
        }
        if (predicate()) {
            return true;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(kMainLoopPeriodMs));
    }
    return !interrupted() && predicate();
}

}  // namespace pure_uav::app
