#include "startup/logging.hpp"

#include <csignal>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <iomanip>
#include <sstream>

#include "communicator.hpp"

namespace pure_uav::app {
namespace {

volatile std::sig_atomic_t g_interrupted = 0;

void onSignal(int /*unused*/) {
    g_interrupted = 1;
}

}  // namespace

const char* phaseName(Phase phase) {
    switch (phase) {
        case Phase::kConfig:
            return "CONFIG";
        case Phase::kWaitLink:
            return "WAIT_LINK";
        case Phase::kGuidance:
            return "GUIDANCE";
        case Phase::kPreflight:
            return "PREFLIGHT";
        case Phase::kGroundReference:
            return "GROUND_REF";
        case Phase::kArm:
            return "ARM";
        case Phase::kTakeoffMode:
            return "TAKEOFF_MODE";
        case Phase::kTakeoffCommand:
            return "TAKEOFF_CMD";
        case Phase::kClimb:
            return "CLIMB";
        case Phase::kHover:
            return "HOVER";
        case Phase::kOffboard:
            return "OFFBOARD";
        case Phase::kLand:
            return "LAND";
        case Phase::kDisarm:
            return "DISARM";
        case Phase::kAbort:
            return "ABORT";
        case Phase::kDone:
            return "DONE";
    }
    return "UNKNOWN";
}

bool Logger::open(std::string& error) {
    std::error_code ec;
    std::filesystem::create_directories("Log", ec);
    if (ec) {
        error = "创建 Log 目录失败: " + ec.message();
        return false;
    }

    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    char name[64] = {};
    std::strftime(name, sizeof(name), "Log/main_%Y%m%d_%H%M%S.log", &tm);

    file_.open(name, std::ios::out | std::ios::trunc);
    if (!file_.is_open()) {
        error = std::string("打开日志文件失败: ") + name;
        return false;
    }
    return true;
}

void Logger::info(Phase phase, const std::string& message) {
    write("INFO", phase, message);
}

void Logger::warn(Phase phase, const std::string& message) {
    write("WARN", phase, message);
}

void Logger::error(Phase phase, const std::string& message) {
    write("ERROR", phase, message);
}

void Logger::write(const char* level, Phase phase, const std::string& message) {
    const auto now = std::chrono::system_clock::now();
    const std::time_t t = std::chrono::system_clock::to_time_t(now);
    const auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(now.time_since_epoch()) % 1000;
    std::tm tm{};
#if defined(_WIN32)
    localtime_s(&tm, &t);
#else
    localtime_r(&t, &tm);
#endif
    char clock[16] = {};
    std::strftime(clock, sizeof(clock), "%H:%M:%S", &tm);

    std::ostringstream line;
    line << "[" << clock << "." << std::setw(3) << std::setfill('0') << ms.count() << "][" << phaseName(phase) << "][" << level << "] " << message;
    const std::string text = line.str();
    std::printf("%s\n", text.c_str());
    std::fflush(stdout);
    if (file_.is_open()) {
        file_ << text << '\n';
        file_.flush();
    }
}

std::string fixed(double value, int precision) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(precision) << value;
    return os.str();
}

std::string modeText(const Communicator& comm) {
    std::ostringstream os;
    os << comm.getFlightModeNameCn() << " (0x" << std::hex << std::uppercase << std::setw(8) << std::setfill('0') << comm.getCustomMode() << ")";
    return os.str();
}

void installSignalHandlers() {
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
}

bool interrupted() {
    return g_interrupted != 0;
}

bool sleepInterruptible(int milliseconds) {
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(milliseconds);
    while (std::chrono::steady_clock::now() < deadline) {
        if (interrupted()) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return !interrupted();
}

}  // namespace pure_uav::app
