// test_drone：机载真机流程测试。
//
// 只验证飞控侧链路与飞行模式流程，不启动相机、VIO、检测：
//   wait_link → preflight → arm → AUTO_TAKEOFF → 爬升
//   → OFFBOARD 零速度悬停 → AUTO_LAND → disarm
//
// 参数见 config/test_drone.yaml；绝对安全上限仍由本文件的 constexpr 限制。

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <ctime>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include "common/mavlink.h"
#include "communicator.hpp"

namespace {

using namespace pure_uav;
using Clock = std::chrono::steady_clock;

// 代码中的硬上限：YAML 只能在这个范围内调参。
constexpr float kHardMaxTakeoffAltitudeM = 3.0F;
constexpr int kHardMaxHoverSeconds = 60;
constexpr int kHardMinOffboardHz = 10;
constexpr int kHardMaxOffboardHz = 50;
constexpr int kHardMaxTimeoutSeconds = 120;
constexpr double kMaxGroundReferenceStddevM = 0.05;
constexpr int kGroundReferenceSamples = 20;
constexpr int kGroundReferenceIntervalMs = 50;
constexpr int kMainLoopPeriodMs = 50;
constexpr int kStatusPeriodMs = 1000;
constexpr int kBatteryAlarmPercent = 20;
constexpr double kLandAltitudeM = 0.10;
constexpr double kLandClimbMps = 0.10;
constexpr double kRadToDeg = 57.29577951308232;
constexpr double kQuatNormTolerance = 1e-3;

enum class Phase {
    kConfig,
    kWaitLink,
    kPreflight,
    kGroundReference,
    kArm,
    kTakeoffMode,
    kTakeoffCommand,
    kClimb,
    kOffboard,
    kHover,
    kLand,
    kDisarm,
    kAbort,
    kDone,
};

const char* phaseName(Phase phase) {
    switch (phase) {
        case Phase::kConfig:
            return "CONFIG";
        case Phase::kWaitLink:
            return "WAIT_LINK";
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
        case Phase::kOffboard:
            return "OFFBOARD";
        case Phase::kHover:
            return "HOVER";
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

volatile std::sig_atomic_t g_interrupted = 0;

void onSignal(int /*unused*/) {
    g_interrupted = 1;
}

bool interrupted() {
    return g_interrupted != 0;
}

// ============================================================================
//  配置
// ============================================================================

struct TestDroneConfig {
    float takeoff_altitude_m = 1.0F;
    int hover_s = 10;

    int link_timeout_s = 10;
    int heartbeat_max_age_ms = 3000;
    int attitude_max_age_ms = 200;
    int local_position_max_age_ms = 200;
    int system_time_max_age_ms = 2000;

    int ack_timeout_ms = 2000;
    int arm_timeout_s = 5;
    int mode_timeout_s = 5;

    int min_battery_percent = 30;
    double max_tilt_deg = 10.0;
    double max_vertical_speed_mps = 0.2;

    int takeoff_timeout_s = 30;
    double height_tolerance_m = 0.15;
    double height_stable_s = 1.0;

    int offboard_rate_hz = 20;
    double offboard_prestream_s = 1.0;
    double hover_altitude_tolerance_m = 0.3;

    int land_timeout_s = 30;
};

bool checkKeys(const YAML::Node& node, std::initializer_list<const char*> allowed, const char* section, std::string& error) {
    if (!node) {
        return true;
    }
    if (!node.IsMap()) {
        error = std::string(section) + " 必须是映射";
        return false;
    }
    for (const auto& item : node) {
        if (!item.first.IsScalar()) {
            error = std::string(section) + " 存在非字符串键";
            return false;
        }
        const std::string key = item.first.as<std::string>();
        bool known = false;
        for (const char* name : allowed) {
            if (key == name) {
                known = true;
                break;
            }
        }
        if (!known) {
            error = std::string(section) + " 存在未知键: " + key;
            return false;
        }
    }
    return true;
}

bool readInt(const YAML::Node& node, const char* key, int& out, std::string& error) {
    if (!node || !node[key]) {
        return true;
    }
    try {
        out = node[key].as<int>();
    } catch (const YAML::Exception&) {
        error = std::string("键 ") + key + " 不是整数";
        return false;
    }
    return true;
}

bool readDouble(const YAML::Node& node, const char* key, double& out, std::string& error) {
    if (!node || !node[key]) {
        return true;
    }
    try {
        out = node[key].as<double>();
    } catch (const YAML::Exception&) {
        error = std::string("键 ") + key + " 不是数字";
        return false;
    }
    return true;
}

bool readFloat(const YAML::Node& node, const char* key, float& out, std::string& error) {
    double value = static_cast<double>(out);
    if (!readDouble(node, key, value, error)) {
        return false;
    }
    out = static_cast<float>(value);
    return true;
}

bool rangeError(const char* field, double value, const char* range, std::string& error) {
    std::ostringstream os;
    os << field << "=" << value << " 超出允许范围 " << range;
    error = os.str();
    return false;
}

bool validateConfig(const TestDroneConfig& cfg, std::string& error) {
    if (cfg.takeoff_altitude_m < 0.3F || cfg.takeoff_altitude_m > kHardMaxTakeoffAltitudeM) {
        return rangeError("takeoff_altitude_m", cfg.takeoff_altitude_m, "[0.3, 3.0]", error);
    }
    if (cfg.hover_s < 1 || cfg.hover_s > kHardMaxHoverSeconds) {
        return rangeError("hover_s", cfg.hover_s, "[1, 60]", error);
    }
    if (cfg.link_timeout_s < 1 || cfg.link_timeout_s > 60) {
        return rangeError("link.timeout_s", cfg.link_timeout_s, "[1, 60]", error);
    }
    if (cfg.heartbeat_max_age_ms < 500 || cfg.heartbeat_max_age_ms > 10000) {
        return rangeError("link.heartbeat_max_age_ms", cfg.heartbeat_max_age_ms, "[500, 10000]", error);
    }
    if (cfg.attitude_max_age_ms < 50 || cfg.attitude_max_age_ms > 1000) {
        return rangeError("link.attitude_max_age_ms", cfg.attitude_max_age_ms, "[50, 1000]", error);
    }
    if (cfg.local_position_max_age_ms < 50 || cfg.local_position_max_age_ms > 1000) {
        return rangeError("link.local_position_max_age_ms", cfg.local_position_max_age_ms, "[50, 1000]", error);
    }
    if (cfg.system_time_max_age_ms < 500 || cfg.system_time_max_age_ms > 5000) {
        return rangeError("link.system_time_max_age_ms", cfg.system_time_max_age_ms, "[500, 5000]", error);
    }
    if (cfg.ack_timeout_ms < 200 || cfg.ack_timeout_ms > 10000) {
        return rangeError("command.ack_timeout_ms", cfg.ack_timeout_ms, "[200, 10000]", error);
    }
    if (cfg.arm_timeout_s < 1 || cfg.arm_timeout_s > 30) {
        return rangeError("command.arm_timeout_s", cfg.arm_timeout_s, "[1, 30]", error);
    }
    if (cfg.mode_timeout_s < 1 || cfg.mode_timeout_s > 30) {
        return rangeError("command.mode_timeout_s", cfg.mode_timeout_s, "[1, 30]", error);
    }
    if (cfg.min_battery_percent < 10 || cfg.min_battery_percent > 100) {
        return rangeError("preflight.min_battery_percent", cfg.min_battery_percent, "[10, 100]", error);
    }
    if (cfg.max_tilt_deg < 1.0 || cfg.max_tilt_deg > 30.0) {
        return rangeError("preflight.max_tilt_deg", cfg.max_tilt_deg, "[1, 30]", error);
    }
    if (cfg.max_vertical_speed_mps < 0.0 || cfg.max_vertical_speed_mps > 1.0) {
        return rangeError("preflight.max_vertical_speed_mps", cfg.max_vertical_speed_mps, "[0, 1]", error);
    }
    if (cfg.takeoff_timeout_s < 5 || cfg.takeoff_timeout_s > kHardMaxTimeoutSeconds) {
        return rangeError("takeoff.timeout_s", cfg.takeoff_timeout_s, "[5, 120]", error);
    }
    if (cfg.height_tolerance_m < 0.05 || cfg.height_tolerance_m > 0.5) {
        return rangeError("takeoff.height_tolerance_m", cfg.height_tolerance_m, "[0.05, 0.5]", error);
    }
    if (cfg.height_stable_s < 0.2 || cfg.height_stable_s > 5.0) {
        return rangeError("takeoff.height_stable_s", cfg.height_stable_s, "[0.2, 5.0]", error);
    }
    if (cfg.offboard_rate_hz < kHardMinOffboardHz || cfg.offboard_rate_hz > kHardMaxOffboardHz) {
        return rangeError("offboard.rate_hz", cfg.offboard_rate_hz, "[10, 50]", error);
    }
    if (cfg.offboard_prestream_s < 0.2 || cfg.offboard_prestream_s > 5.0) {
        return rangeError("offboard.prestream_s", cfg.offboard_prestream_s, "[0.2, 5.0]", error);
    }
    if (cfg.hover_altitude_tolerance_m < 0.1 || cfg.hover_altitude_tolerance_m > 1.0) {
        return rangeError("offboard.hover_altitude_tolerance_m", cfg.hover_altitude_tolerance_m, "[0.1, 1.0]", error);
    }
    if (cfg.land_timeout_s < 5 || cfg.land_timeout_s > kHardMaxTimeoutSeconds) {
        return rangeError("land.timeout_s", cfg.land_timeout_s, "[5, 120]", error);
    }
    return true;
}

bool loadConfig(const std::string& path, TestDroneConfig& cfg, std::string& error) {
    YAML::Node root;
    try {
        root = YAML::LoadFile(path);
    } catch (const YAML::Exception& e) {
        error = "打不开或解析失败: " + path + " (" + e.what() + ")";
        return false;
    }
    if (!root.IsMap()) {
        error = "配置根节点必须是映射";
        return false;
    }
    if (!checkKeys(root, {"test_drone", "link", "preflight", "command", "takeoff", "offboard", "land"}, "root", error)) {
        return false;
    }

    const YAML::Node td = root["test_drone"];
    if (!td || !td.IsMap()) {
        error = "缺少 test_drone 段";
        return false;
    }
    if (!checkKeys(td, {"takeoff_altitude_m", "hover_s"}, "test_drone", error)) {
        return false;
    }
    if (!td["takeoff_altitude_m"] || !td["hover_s"]) {
        error = "test_drone 必须包含 takeoff_altitude_m 和 hover_s";
        return false;
    }
    if (!readFloat(td, "takeoff_altitude_m", cfg.takeoff_altitude_m, error) || !readInt(td, "hover_s", cfg.hover_s, error)) {
        return false;
    }

    const YAML::Node link = root["link"];
    if (!checkKeys(link, {"timeout_s", "heartbeat_max_age_ms", "attitude_max_age_ms", "local_position_max_age_ms", "system_time_max_age_ms"}, "link", error)) {
        return false;
    }
    if (!readInt(link, "timeout_s", cfg.link_timeout_s, error) || !readInt(link, "heartbeat_max_age_ms", cfg.heartbeat_max_age_ms, error) || !readInt(link, "attitude_max_age_ms", cfg.attitude_max_age_ms, error) || !readInt(link, "local_position_max_age_ms", cfg.local_position_max_age_ms, error) ||
        !readInt(link, "system_time_max_age_ms", cfg.system_time_max_age_ms, error)) {
        return false;
    }

    const YAML::Node command = root["command"];
    if (!checkKeys(command, {"ack_timeout_ms", "arm_timeout_s", "mode_timeout_s"}, "command", error)) {
        return false;
    }
    if (!readInt(command, "ack_timeout_ms", cfg.ack_timeout_ms, error) || !readInt(command, "arm_timeout_s", cfg.arm_timeout_s, error) || !readInt(command, "mode_timeout_s", cfg.mode_timeout_s, error)) {
        return false;
    }

    const YAML::Node preflight = root["preflight"];
    if (!checkKeys(preflight, {"min_battery_percent", "max_tilt_deg", "max_vertical_speed_mps"}, "preflight", error)) {
        return false;
    }
    if (!readInt(preflight, "min_battery_percent", cfg.min_battery_percent, error) || !readDouble(preflight, "max_tilt_deg", cfg.max_tilt_deg, error) || !readDouble(preflight, "max_vertical_speed_mps", cfg.max_vertical_speed_mps, error)) {
        return false;
    }

    const YAML::Node takeoff = root["takeoff"];
    if (!checkKeys(takeoff, {"timeout_s", "height_tolerance_m", "height_stable_s"}, "takeoff", error)) {
        return false;
    }
    if (!readInt(takeoff, "timeout_s", cfg.takeoff_timeout_s, error) || !readDouble(takeoff, "height_tolerance_m", cfg.height_tolerance_m, error) || !readDouble(takeoff, "height_stable_s", cfg.height_stable_s, error)) {
        return false;
    }

    const YAML::Node offboard = root["offboard"];
    if (!checkKeys(offboard, {"rate_hz", "prestream_s", "hover_altitude_tolerance_m"}, "offboard", error)) {
        return false;
    }
    if (!readInt(offboard, "rate_hz", cfg.offboard_rate_hz, error) || !readDouble(offboard, "prestream_s", cfg.offboard_prestream_s, error) || !readDouble(offboard, "hover_altitude_tolerance_m", cfg.hover_altitude_tolerance_m, error)) {
        return false;
    }

    const YAML::Node land = root["land"];
    if (!checkKeys(land, {"timeout_s"}, "land", error)) {
        return false;
    }
    if (!readInt(land, "timeout_s", cfg.land_timeout_s, error)) {
        return false;
    }

    return validateConfig(cfg, error);
}

// ============================================================================
//  日志
// ============================================================================

class Logger {
  public:
    bool open(std::string& error) {
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
        std::strftime(name, sizeof(name), "Log/test_drone_%Y%m%d_%H%M%S.log", &tm);

        file_.open(name, std::ios::out | std::ios::trunc);
        if (!file_.is_open()) {
            error = std::string("打开日志文件失败: ") + name;
            return false;
        }
        return true;
    }

    void info(Phase phase, const std::string& message) {
        write("INFO", phase, message);
    }

    void warn(Phase phase, const std::string& message) {
        write("WARN", phase, message);
    }

    void error(Phase phase, const std::string& message) {
        write("ERROR", phase, message);
    }

  private:
    void write(const char* level, Phase phase, const std::string& message) {
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

    std::ofstream file_;
};

// ============================================================================
//  通用工具
// ============================================================================

struct TestContext {
    TestDroneConfig cfg;
    Communicator comm;
    Logger log;
    std::uint64_t reconnect_count = 0;
    double ground_z = 0.0;
    bool airborne = false;
};

std::string fixed(double value, int precision = 2) {
    std::ostringstream os;
    os << std::fixed << std::setprecision(precision) << value;
    return os.str();
}

std::string modeText(const Communicator& comm) {
    std::ostringstream os;
    os << comm.getFlightModeNameCn() << " (0x" << std::hex << std::uppercase << std::setw(8) << std::setfill('0') << comm.getCustomMode() << ")";
    return os.str();
}

bool sleepInterruptible(int milliseconds) {
    const auto deadline = Clock::now() + std::chrono::milliseconds(milliseconds);
    while (Clock::now() < deadline) {
        if (interrupted()) {
            return false;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
    return !interrupted();
}

template <typename Predicate>
bool waitUntil(const std::chrono::milliseconds timeout, Predicate predicate) {
    const auto deadline = Clock::now() + timeout;
    while (Clock::now() < deadline) {
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

bool linkAlive(TestContext& ctx, std::string& error) {
    if (!ctx.comm.isConnected()) {
        error = "飞控链路断开";
        return false;
    }
    const std::int64_t heartbeat_age = ctx.comm.lastHeartbeatAgeMs();
    if (heartbeat_age < 0 || heartbeat_age > ctx.cfg.heartbeat_max_age_ms) {
        error = "心跳超时: lastHeartbeatAgeMs=" + std::to_string(heartbeat_age);
        return false;
    }
    return true;
}

bool readAltitude(TestContext& ctx, double& altitude_m, double& climb_mps, std::string& error) {
    LocalPosition position;
    if (!ctx.comm.getLocalPosition(position, ctx.cfg.local_position_max_age_ms)) {
        error = "位置数据不新鲜: localPositionAgeMs=" + std::to_string(ctx.comm.localPositionAgeMs());
        return false;
    }
    if (!std::isfinite(position.x) || !std::isfinite(position.y) || !std::isfinite(position.z) || !std::isfinite(position.vx) || !std::isfinite(position.vy) || !std::isfinite(position.vz)) {
        error = "位置或速度出现 NaN/Inf";
        return false;
    }
    altitude_m = -(static_cast<double>(position.z) - ctx.ground_z);
    climb_mps = -static_cast<double>(position.vz);
    return true;
}

// ============================================================================
//  wait_link
// ============================================================================

bool waitLink(TestContext& ctx) {
    const std::string path = findConfigFile("config/driver.yaml");
    CommConfig comm_cfg;
    if (!loadCommConfig(path, comm_cfg)) {
        ctx.log.error(Phase::kWaitLink, "通信配置加载失败: " + path);
        return false;
    }

    ctx.comm.setSystemId(2);
    ctx.comm.setComponentId(1);
    if (!ctx.comm.start(comm_cfg)) {
        ctx.log.error(Phase::kWaitLink, "串口打开失败: device=" + comm_cfg.device + " baud=" + std::to_string(comm_cfg.baudrate));
        return false;
    }

    ctx.log.info(Phase::kWaitLink, "串口已打开: device=" + comm_cfg.device + " baud=" + std::to_string(comm_cfg.baudrate));
    const auto start = Clock::now();
    auto next_log = start;
    bool requested_streams = false;

    while (Clock::now() - start < std::chrono::seconds(ctx.cfg.link_timeout_s)) {
        if (interrupted()) {
            ctx.log.warn(Phase::kWaitLink, "等待链路时收到中断");
            return false;
        }

        const bool connected = ctx.comm.isConnected();
        const bool locked = ctx.comm.targetLocked();
        const bool px4 = ctx.comm.getAutopilot() == MAV_AUTOPILOT_PX4;
        const std::int64_t heartbeat_age = ctx.comm.lastHeartbeatAgeMs();
        const std::int64_t attitude_age = ctx.comm.attitudeAgeMs();
        const std::int64_t local_age = ctx.comm.localPositionAgeMs();
        const std::int64_t system_age = ctx.comm.systemTimeAgeMs();
        const bool battery_seen = ctx.comm.getCountBattery() > 0;

        const bool ready = connected && locked && px4 && heartbeat_age >= 0 && heartbeat_age <= ctx.cfg.heartbeat_max_age_ms && attitude_age >= 0 && attitude_age <= ctx.cfg.attitude_max_age_ms && local_age >= 0 && local_age <= ctx.cfg.local_position_max_age_ms && system_age >= 0 &&
                           system_age <= ctx.cfg.system_time_max_age_ms && battery_seen;
        if (ready) {
            ctx.reconnect_count = ctx.comm.reconnectCount();
            ctx.log.info(Phase::kWaitLink, "链路就绪: autopilot=" + std::string(ctx.comm.getAutopilotNameCn()) + " mode=" + modeText(ctx.comm));
            return true;
        }

        if (locked && !px4 && ctx.comm.getAutopilot() != 0) {
            ctx.log.error(Phase::kWaitLink, "固件不是 PX4: autopilot=" + std::string(ctx.comm.getAutopilotNameCn()));
            return false;
        }

        if (!requested_streams && Clock::now() - start > std::chrono::seconds(2) && (attitude_age < 0 || local_age < 0 || system_age < 0)) {
            ctx.comm.requestDefaultStreams();
            requested_streams = true;
            ctx.log.warn(Phase::kWaitLink, "遥测流未齐，已重新请求默认消息流");
        }

        if (Clock::now() >= next_log) {
            ctx.log.info(Phase::kWaitLink, "等待中: connected=" + std::to_string(connected) + " locked=" + std::to_string(locked) + " autopilot=" + std::string(ctx.comm.getAutopilotNameCn()) + " heartbeatAgeMs=" + std::to_string(heartbeat_age) + " attitudeAgeMs=" + std::to_string(attitude_age) +
                                               " localAgeMs=" + std::to_string(local_age) + " systemAgeMs=" + std::to_string(system_age) + " batterySeen=" + std::to_string(battery_seen));
            next_log += std::chrono::milliseconds(kStatusPeriodMs);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(kMainLoopPeriodMs));
    }

    ctx.log.error(Phase::kWaitLink, "链路超时: connected=" + std::to_string(ctx.comm.isConnected()) + " locked=" + std::to_string(ctx.comm.targetLocked()) + " autopilot=" + std::string(ctx.comm.getAutopilotNameCn()) + " heartbeatAgeMs=" + std::to_string(ctx.comm.lastHeartbeatAgeMs()) +
                                        " attitudeAgeMs=" + std::to_string(ctx.comm.attitudeAgeMs()) + " localAgeMs=" + std::to_string(ctx.comm.localPositionAgeMs()) + " systemAgeMs=" + std::to_string(ctx.comm.systemTimeAgeMs()) + " batteryCount=" + std::to_string(ctx.comm.getCountBattery()));
    return false;
}

// ============================================================================
//  preflight
// ============================================================================

constexpr std::size_t kPreflightCheckCount = 17;

struct CheckItem {
    const char* name = "";
    int ok = 0;  // 0 = 失败，1 = 成功
    std::string reason;
};

using PreflightChecks = std::array<CheckItem, kPreflightCheckCount>;

void setCheck(PreflightChecks& checks, std::size_t index, const char* name, bool ok, std::string reason) {
    checks[index] = CheckItem{name, ok ? 1 : 0, ok ? std::string{} : std::move(reason)};
}

bool runPreflight(TestContext& ctx) {
    Attitude attitude;
    LocalPosition position;
    Battery battery;
    SystemTime system_time;

    const bool attitude_ok = ctx.comm.getAttitude(attitude, ctx.cfg.attitude_max_age_ms);
    const bool position_ok = ctx.comm.getLocalPosition(position, ctx.cfg.local_position_max_age_ms);
    const bool system_time_ok = ctx.comm.getSystemTime(system_time, ctx.cfg.system_time_max_age_ms);
    const bool battery_ok = ctx.comm.getCountBattery() > 0;
    ctx.comm.getBattery(battery);

    const bool quat_finite = std::isfinite(attitude.q[0]) && std::isfinite(attitude.q[1]) && std::isfinite(attitude.q[2]) && std::isfinite(attitude.q[3]);
    const double quat_norm = quat_finite ? std::sqrt(attitude.q[0] * attitude.q[0] + attitude.q[1] * attitude.q[1] + attitude.q[2] * attitude.q[2] + attitude.q[3] * attitude.q[3]) : 0.0;
    const bool attitude_finite = attitude_ok && std::isfinite(attitude.roll) && std::isfinite(attitude.pitch) && std::isfinite(attitude.yaw) && quat_finite && std::fabs(quat_norm - 1.0) <= kQuatNormTolerance;
    const bool attitude_level = attitude_ok && std::fabs(static_cast<double>(attitude.roll)) * kRadToDeg <= ctx.cfg.max_tilt_deg && std::fabs(static_cast<double>(attitude.pitch)) * kRadToDeg <= ctx.cfg.max_tilt_deg;

    const bool position_finite = position_ok && std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z) && std::isfinite(position.vx) && std::isfinite(position.vy) && std::isfinite(position.vz);
    const bool vertical_speed_ok = position_ok && std::fabs(static_cast<double>(position.vz)) <= ctx.cfg.max_vertical_speed_mps;

    const std::uint32_t mode = ctx.comm.getCustomMode();
    const bool mode_safe = mode != 0 && mode != px4_mode::OFFBOARD && mode != px4_mode::AUTO_TAKEOFF && mode != px4_mode::AUTO_LAND && mode != px4_mode::AUTO_MISSION && mode != px4_mode::AUTO_RTL;

    PreflightChecks checks{};
    setCheck(checks, 0, "link_connected", ctx.comm.isConnected(), "isConnected=0");
    setCheck(checks, 1, "target_locked", ctx.comm.targetLocked(), "targetLocked=0");
    setCheck(checks, 2, "autopilot_px4", ctx.comm.getAutopilot() == MAV_AUTOPILOT_PX4, "autopilot=" + std::string(ctx.comm.getAutopilotNameCn()));
    setCheck(checks, 3, "heartbeat_fresh", ctx.comm.lastHeartbeatAgeMs() >= 0 && ctx.comm.lastHeartbeatAgeMs() <= ctx.cfg.heartbeat_max_age_ms, "lastHeartbeatAgeMs=" + std::to_string(ctx.comm.lastHeartbeatAgeMs()));
    setCheck(checks, 4, "attitude_fresh", attitude_ok, "attitudeAgeMs=" + std::to_string(ctx.comm.attitudeAgeMs()));
    setCheck(checks, 5, "local_position_fresh", position_ok, "localPositionAgeMs=" + std::to_string(ctx.comm.localPositionAgeMs()));
    setCheck(checks, 6, "system_time_fresh", system_time_ok, "systemTimeAgeMs=" + std::to_string(ctx.comm.systemTimeAgeMs()));
    setCheck(checks, 7, "not_armed", !ctx.comm.isArmed(), "isArmed=1");
    setCheck(checks, 8, "mode_safe", mode_safe, "mode=" + modeText(ctx.comm));
    setCheck(checks, 9, "battery_received", battery_ok, "batteryCount=" + std::to_string(ctx.comm.getCountBattery()));
    setCheck(checks, 10, "battery_voltage_valid", battery_ok && std::isfinite(battery.voltage) && battery.voltage > 0.0F, "voltage=" + fixed(battery.voltage));
    setCheck(checks, 11, "battery_level_ok", battery_ok && battery.remaining >= ctx.cfg.min_battery_percent, "remaining=" + std::to_string(static_cast<int>(battery.remaining)) + "% < " + std::to_string(ctx.cfg.min_battery_percent) + "%");
    setCheck(checks, 12, "attitude_finite", attitude_finite, "attitude 含 NaN/Inf 或四元数模长异常");
    setCheck(checks, 13, "attitude_level", attitude_level, "roll=" + fixed(static_cast<double>(attitude.roll) * kRadToDeg) + "deg pitch=" + fixed(static_cast<double>(attitude.pitch) * kRadToDeg) + "deg");
    setCheck(checks, 14, "local_position_finite", position_finite, "位置或速度含 NaN/Inf");
    setCheck(checks, 15, "vertical_speed_small", vertical_speed_ok, "vz=" + fixed(static_cast<double>(position.vz)));
    setCheck(checks, 16, "reconnect_stable", ctx.reconnect_count == ctx.comm.reconnectCount(), "reconnectCount=" + std::to_string(ctx.comm.reconnectCount()));

    const bool pass = std::all_of(checks.begin(), checks.end(), [](const CheckItem& item) { return item.ok == 1; });
    for (const auto& item : checks) {
        if (item.ok == 0) {
            ctx.log.error(Phase::kPreflight, std::string(item.name) + ": " + item.reason);
        }
    }

    if (pass) {
        ctx.log.info(Phase::kPreflight, "自检通过 17/17");
    } else {
        ctx.log.error(Phase::kPreflight, "自检失败，禁止解锁");
    }
    return pass;
}

// ============================================================================
//  地面参考与高度
// ============================================================================

bool captureGroundReference(TestContext& ctx) {
    double sum = 0.0;
    double sum_sq = 0.0;
    int count = 0;

    for (int i = 0; i < kGroundReferenceSamples; ++i) {
        if (interrupted()) {
            ctx.log.warn(Phase::kGroundReference, "采集地面参考时收到中断");
            return false;
        }
        LocalPosition position;
        if (!ctx.comm.getLocalPosition(position, ctx.cfg.local_position_max_age_ms) || !std::isfinite(position.z)) {
            ctx.log.error(Phase::kGroundReference, "地面参考读取失败: localPositionAgeMs=" + std::to_string(ctx.comm.localPositionAgeMs()));
            return false;
        }
        sum += static_cast<double>(position.z);
        sum_sq += static_cast<double>(position.z) * static_cast<double>(position.z);
        ++count;
        if (!sleepInterruptible(kGroundReferenceIntervalMs)) {
            ctx.log.warn(Phase::kGroundReference, "采集地面参考时收到中断");
            return false;
        }
    }

    if (count != kGroundReferenceSamples) {
        ctx.log.error(Phase::kGroundReference, "地面参考样本不足: " + std::to_string(count));
        return false;
    }
    const double mean = sum / static_cast<double>(count);
    const double variance = std::max(0.0, sum_sq / static_cast<double>(count) - mean * mean);
    const double stddev = std::sqrt(variance);
    if (stddev > kMaxGroundReferenceStddevM) {
        ctx.log.error(Phase::kGroundReference, "地面参考抖动过大: stddev=" + fixed(stddev, 3) + "m");
        return false;
    }

    ctx.ground_z = mean;
    ctx.log.info(Phase::kGroundReference, "地面参考已锁定: z=" + fixed(ctx.ground_z, 3) + "m stddev=" + fixed(stddev, 3) + "m");
    return true;
}

bool waitAltitude(TestContext& ctx, double target_altitude_m, int timeout_s) {
    const auto deadline = Clock::now() + std::chrono::seconds(timeout_s);
    auto next_log = Clock::now();
    std::optional<Clock::time_point> stable_since;

    while (Clock::now() < deadline) {
        if (interrupted()) {
            ctx.log.warn(Phase::kClimb, "爬升时收到中断");
            return false;
        }
        std::string link_error;
        if (!linkAlive(ctx, link_error)) {
            ctx.log.error(Phase::kClimb, link_error);
            return false;
        }
        if (!ctx.comm.isArmed()) {
            ctx.log.error(Phase::kClimb, "爬升过程中检测到已解除武装");
            return false;
        }

        double altitude_m = 0.0;
        double climb_mps = 0.0;
        std::string height_error;
        const bool height_ok = readAltitude(ctx, altitude_m, climb_mps, height_error);
        const std::uint32_t mode = ctx.comm.getCustomMode();

        if (height_ok && mode != px4_mode::AUTO_TAKEOFF && mode != px4_mode::AUTO_LOITER) {
            ctx.log.error(Phase::kClimb, "爬升模式异常: mode=" + modeText(ctx.comm));
            return false;
        }

        const bool near_target = height_ok && altitude_m >= target_altitude_m - ctx.cfg.height_tolerance_m && std::fabs(climb_mps) <= ctx.cfg.max_vertical_speed_mps;
        if (near_target) {
            if (!stable_since) {
                stable_since = Clock::now();
            }
            if (Clock::now() - *stable_since >= std::chrono::duration<double>(ctx.cfg.height_stable_s)) {
                ctx.log.info(Phase::kClimb, "到达目标高度: alt=" + fixed(altitude_m) + "m vz=" + fixed(climb_mps) + "m/s mode=" + modeText(ctx.comm));
                return true;
            }
        } else {
            stable_since.reset();
        }

        if (Clock::now() >= next_log) {
            if (height_ok) {
                ctx.log.info(Phase::kClimb, "alt=" + fixed(altitude_m) + "m vz=" + fixed(climb_mps) + "m/s mode=" + modeText(ctx.comm));
            } else {
                ctx.log.warn(Phase::kClimb, height_error);
            }
            next_log += std::chrono::milliseconds(kStatusPeriodMs);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(kMainLoopPeriodMs));
    }

    ctx.log.error(Phase::kClimb, "爬升超时: target=" + fixed(target_altitude_m) + "m timeout=" + std::to_string(timeout_s) + "s");
    return false;
}

// ============================================================================
//  arm / mode / takeoff
// ============================================================================

bool arm(TestContext& ctx) {
    std::uint8_t ack = 0;
    if (!ctx.comm.sendArm(true, ctx.cfg.ack_timeout_ms, &ack)) {
        ctx.log.error(Phase::kArm, "解锁未收到 ACK: mode=" + modeText(ctx.comm));
        return false;
    }
    if (ack != MAV_RESULT_ACCEPTED) {
        ctx.log.error(Phase::kArm, "解锁被拒绝: result=" + std::to_string(static_cast<int>(ack)) + " text=" + std::string(ctx.comm.getLastAck()) + " mode=" + modeText(ctx.comm));
        return false;
    }
    if (!waitUntil(std::chrono::seconds(ctx.cfg.arm_timeout_s), [&ctx] { return ctx.comm.isArmed(); })) {
        ctx.log.error(Phase::kArm, "ACK 已接受但未进入解锁状态: armed=" + std::to_string(ctx.comm.isArmed()) + " mode=" + modeText(ctx.comm));
        return false;
    }

    ctx.log.info(Phase::kArm, "已解锁: mode=" + modeText(ctx.comm));
    return true;
}

bool switchMode(TestContext& ctx, std::uint32_t custom_mode, const char* name, Phase phase) {
    std::uint8_t ack = 0;
    ctx.log.info(phase, std::string("请求切换模式: ") + name);
    if (!ctx.comm.sendSetMode(custom_mode, ctx.cfg.ack_timeout_ms, &ack)) {
        ctx.log.error(phase, std::string("切换 ") + name + " 未收到 ACK: mode=" + modeText(ctx.comm));
        return false;
    }
    if (ack != MAV_RESULT_ACCEPTED) {
        ctx.log.error(phase, std::string("切换 ") + name + " 被拒绝: result=" + std::to_string(static_cast<int>(ack)) + " text=" + std::string(ctx.comm.getLastAck()) + " mode=" + modeText(ctx.comm));
        return false;
    }
    if (!waitUntil(std::chrono::seconds(ctx.cfg.mode_timeout_s), [&ctx, custom_mode] { return ctx.comm.getCustomMode() == custom_mode; })) {
        ctx.log.error(phase, std::string("模式未生效: 期望=") + name + " 当前=" + modeText(ctx.comm));
        return false;
    }

    ctx.log.info(phase, std::string("模式已切换: ") + name);
    return true;
}

bool takeoff(TestContext& ctx) {
    if (!switchMode(ctx, px4_mode::AUTO_TAKEOFF, "AUTO_TAKEOFF", Phase::kTakeoffMode)) {
        return false;
    }

    std::uint8_t ack = 0;
    if (!ctx.comm.sendTakeoff(ctx.cfg.takeoff_altitude_m, ctx.cfg.ack_timeout_ms, &ack)) {
        ctx.log.error(Phase::kTakeoffCommand, "NAV_TAKEOFF 未收到 ACK: mode=" + modeText(ctx.comm));
        return false;
    }
    if (ack != MAV_RESULT_ACCEPTED) {
        ctx.log.error(Phase::kTakeoffCommand, "NAV_TAKEOFF 被拒绝: result=" + std::to_string(static_cast<int>(ack)) + " text=" + std::string(ctx.comm.getLastAck()) + " mode=" + modeText(ctx.comm));
        return false;
    }

    ctx.airborne = true;
    ctx.log.info(Phase::kTakeoffCommand, "起飞命令已接受: target=" + fixed(ctx.cfg.takeoff_altitude_m) + "m");
    return true;
}

// ============================================================================
//  OFFBOARD 零速度推流
// ============================================================================

class OffboardStreamer {
  public:
    OffboardStreamer(Communicator& comm, int rate_hz) : comm_(comm), period_us_(1000000 / rate_hz) {
    }

    ~OffboardStreamer() {
        stop();
    }

    OffboardStreamer(const OffboardStreamer&) = delete;
    OffboardStreamer& operator=(const OffboardStreamer&) = delete;

    bool start(std::string& error) {
        if (running_.load()) {
            return true;
        }
        if (period_us_.count() <= 0) {
            error = "OFFBOARD 频率非法";
            return false;
        }
        running_.store(true);
        thread_ = std::thread(&OffboardStreamer::run, this);
        return true;
    }

    void stop() {
        running_.store(false);
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    std::uint64_t sent() const {
        return sent_.load();
    }

    std::uint64_t failed() const {
        return failed_.load();
    }

    bool running() const {
        return running_.load();
    }

  private:
    void run() {
        auto next = Clock::now();
        while (running_.load()) {
            GuidanceCmd cmd;
            cmd.mode = GuidanceMode::VELOCITY;
            cmd.vx = 0.0F;
            cmd.vy = 0.0F;
            cmd.vz = 0.0F;
            if (comm_.pushGuidanceCmd(cmd)) {
                sent_.fetch_add(1);
            } else {
                failed_.fetch_add(1);
            }
            next += period_us_;
            std::this_thread::sleep_until(next);
        }
    }

    Communicator& comm_;
    std::chrono::microseconds period_us_;
    std::atomic<bool> running_{false};
    std::thread thread_;
    std::atomic<std::uint64_t> sent_{0};
    std::atomic<std::uint64_t> failed_{0};
};

bool offboardHover(TestContext& ctx, OffboardStreamer& streamer) {
    std::string start_error;
    if (!streamer.start(start_error)) {
        ctx.log.error(Phase::kOffboard, start_error);
        return false;
    }
    if (!sleepInterruptible(static_cast<int>(ctx.cfg.offboard_prestream_s * 1000.0))) {
        ctx.log.warn(Phase::kOffboard, "OFFBOARD 预推流时收到中断");
        return false;
    }
    if (streamer.failed() > 0 || streamer.sent() == 0) {
        ctx.log.error(Phase::kOffboard, "OFFBOARD 预推流失败: sent=" + std::to_string(streamer.sent()) + " failed=" + std::to_string(streamer.failed()));
        return false;
    }

    if (!switchMode(ctx, px4_mode::OFFBOARD, "OFFBOARD", Phase::kOffboard)) {
        return false;
    }

    const auto deadline = Clock::now() + std::chrono::seconds(ctx.cfg.hover_s);
    auto next_log = Clock::now();
    while (Clock::now() < deadline) {
        if (interrupted()) {
            ctx.log.warn(Phase::kHover, "悬停时收到中断");
            return false;
        }
        std::string link_error;
        if (!linkAlive(ctx, link_error)) {
            ctx.log.error(Phase::kHover, link_error);
            return false;
        }
        if (!ctx.comm.isArmed()) {
            ctx.log.error(Phase::kHover, "悬停过程中检测到已解除武装");
            return false;
        }
        if (ctx.comm.getCustomMode() != px4_mode::OFFBOARD) {
            ctx.log.error(Phase::kHover, "OFFBOARD 模式丢失: mode=" + modeText(ctx.comm));
            return false;
        }
        if (streamer.failed() > 0) {
            ctx.log.error(Phase::kHover, "OFFBOARD setpoint 发送失败: failed=" + std::to_string(streamer.failed()));
            return false;
        }

        double altitude_m = 0.0;
        double climb_mps = 0.0;
        std::string height_error;
        if (!readAltitude(ctx, altitude_m, climb_mps, height_error)) {
            ctx.log.error(Phase::kHover, height_error);
            return false;
        }
        if (std::fabs(altitude_m - static_cast<double>(ctx.cfg.takeoff_altitude_m)) > ctx.cfg.hover_altitude_tolerance_m) {
            ctx.log.error(Phase::kHover, "悬停高度偏差过大: alt=" + fixed(altitude_m) + "m target=" + fixed(ctx.cfg.takeoff_altitude_m) + "m");
            return false;
        }

        Battery battery;
        ctx.comm.getBattery(battery);
        if (battery.remaining >= 0 && battery.remaining < kBatteryAlarmPercent) {
            ctx.log.error(Phase::kHover, "电量过低: remaining=" + std::to_string(static_cast<int>(battery.remaining)) + "%");
            return false;
        }

        if (Clock::now() >= next_log) {
            ctx.log.info(Phase::kHover, "alt=" + fixed(altitude_m) + "m vz=" + fixed(climb_mps) + "m/s mode=" + modeText(ctx.comm) + " sent=" + std::to_string(streamer.sent()));
            next_log += std::chrono::milliseconds(kStatusPeriodMs);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(kMainLoopPeriodMs));
    }

    ctx.log.info(Phase::kHover, "OFFBOARD 悬停完成: " + std::to_string(ctx.cfg.hover_s) + "s");
    return true;
}

// ============================================================================
//  降落 / 解除武装 / 安全退出
// ============================================================================

bool waitLanded(TestContext& ctx, int timeout_s) {
    const auto deadline = Clock::now() + std::chrono::seconds(timeout_s);
    std::optional<Clock::time_point> stable_since;

    while (Clock::now() < deadline) {
        if (interrupted()) {
            ctx.log.warn(Phase::kLand, "等待落地时收到中断");
            return false;
        }
        if (!ctx.comm.isArmed()) {
            ctx.log.info(Phase::kLand, "飞控已自动解除武装");
            return true;
        }

        double altitude_m = 0.0;
        double climb_mps = 0.0;
        std::string height_error;
        if (readAltitude(ctx, altitude_m, climb_mps, height_error)) {
            if (altitude_m <= kLandAltitudeM && std::fabs(climb_mps) <= kLandClimbMps) {
                if (!stable_since) {
                    stable_since = Clock::now();
                }
                if (Clock::now() - *stable_since >= std::chrono::seconds(1)) {
                    ctx.log.info(Phase::kLand, "已落地: alt=" + fixed(altitude_m) + "m");
                    return true;
                }
            } else {
                stable_since.reset();
            }
        } else {
            stable_since.reset();
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(kMainLoopPeriodMs));
    }

    ctx.log.error(Phase::kLand, "降落超时: timeout=" + std::to_string(timeout_s) + "s armed=" + std::to_string(ctx.comm.isArmed()));
    return false;
}

bool disarm(TestContext& ctx) {
    if (!ctx.comm.isArmed()) {
        ctx.log.info(Phase::kDisarm, "已处于解除武装状态");
        return true;
    }

    std::uint8_t ack = 0;
    if (!ctx.comm.sendArm(false, ctx.cfg.ack_timeout_ms, &ack)) {
        ctx.log.error(Phase::kDisarm, "解除武装未收到 ACK: mode=" + modeText(ctx.comm));
        return false;
    }
    if (ack != MAV_RESULT_ACCEPTED) {
        ctx.log.error(Phase::kDisarm, "解除武装被拒绝: result=" + std::to_string(static_cast<int>(ack)) + " text=" + std::string(ctx.comm.getLastAck()));
        return false;
    }
    if (!waitUntil(std::chrono::seconds(ctx.cfg.arm_timeout_s), [&ctx] { return !ctx.comm.isArmed(); })) {
        ctx.log.error(Phase::kDisarm, "解除武装超时: armed=" + std::to_string(ctx.comm.isArmed()));
        return false;
    }

    ctx.log.info(Phase::kDisarm, "已解除武装");
    return true;
}

bool landAndDisarm(TestContext& ctx, OffboardStreamer& streamer) {
    std::uint8_t ack = 0;
    bool land_sent = ctx.comm.sendLand(ctx.cfg.ack_timeout_ms, &ack);
    if (!land_sent || ack != MAV_RESULT_ACCEPTED) {
        ctx.log.warn(Phase::kLand, "NAV_LAND 未被接受，尝试 AUTO_LAND 模式");
        if (!switchMode(ctx, px4_mode::AUTO_LAND, "AUTO_LAND", Phase::kLand)) {
            ctx.log.error(Phase::kLand, "降落命令与 AUTO_LAND 模式均失败: mode=" + modeText(ctx.comm));
            streamer.stop();
            return false;
        }
    } else if (!waitUntil(std::chrono::seconds(ctx.cfg.mode_timeout_s), [&ctx] { return ctx.comm.getCustomMode() == px4_mode::AUTO_LAND; })) {
        ctx.log.warn(Phase::kLand, "NAV_LAND 后模式未变为 AUTO_LAND，尝试再次设置");
        if (!switchMode(ctx, px4_mode::AUTO_LAND, "AUTO_LAND", Phase::kLand)) {
            ctx.log.error(Phase::kLand, "无法进入 AUTO_LAND: mode=" + modeText(ctx.comm));
            streamer.stop();
            return false;
        }
    }

    streamer.stop();
    ctx.log.info(Phase::kLand, "已进入自动降落，OFFBOARD 推流停止");

    if (!waitLanded(ctx, ctx.cfg.land_timeout_s)) {
        return false;
    }
    if (!disarm(ctx)) {
        return false;
    }

    ctx.airborne = false;
    ctx.log.info(Phase::kLand, "降落并解除武装完成");
    return true;
}

void safeAbort(TestContext& ctx, OffboardStreamer* streamer) {
    ctx.log.warn(Phase::kAbort, "进入安全退出");

    if (!ctx.comm.isConnected()) {
        ctx.log.error(Phase::kAbort, "链路已断，停止 OFFBOARD 推流，交给飞控 failsafe");
        if (streamer != nullptr) {
            streamer->stop();
        }
        return;
    }

    bool landed = !ctx.airborne;
    if (ctx.comm.isArmed() && ctx.airborne) {
        std::uint8_t ack = 0;
        if (!ctx.comm.sendLand(ctx.cfg.ack_timeout_ms, &ack) || ack != MAV_RESULT_ACCEPTED) {
            ctx.comm.sendSetMode(px4_mode::AUTO_LAND, ctx.cfg.ack_timeout_ms, nullptr);
        }
        waitUntil(std::chrono::seconds(ctx.cfg.mode_timeout_s), [&ctx] { return ctx.comm.getCustomMode() == px4_mode::AUTO_LAND; });
        if (streamer != nullptr) {
            streamer->stop();
        }
        landed = waitLanded(ctx, 10);
    }

    if (streamer != nullptr) {
        streamer->stop();
    }
    if (ctx.comm.isArmed() && landed) {
        disarm(ctx);
    } else if (ctx.comm.isArmed()) {
        ctx.log.error(Phase::kAbort, "未确认落地，保持解锁状态，交给飞控/操作员处理");
    }
}

// ============================================================================
//  命令行
// ============================================================================

struct Args {
    std::string config = "config/test_drone.yaml";
    bool help = false;
};

void usage(const char* program) {
    std::printf(
        "用法: %s [--config FILE]\n"
        "  --config FILE   测试配置，默认 config/test_drone.yaml\n"
        "  -h, --help      查看帮助\n",
        program);
}

bool parseArgs(int argc, char** argv, Args& args, std::string& error) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            args.help = true;
            continue;
        }
        if (arg == "--config") {
            if (++i >= argc) {
                error = "--config 缺少参数";
                return false;
            }
            args.config = argv[i];
            continue;
        }
        error = "未知参数: " + arg;
        return false;
    }
    return true;
}

}  // namespace

int main(int argc, char** argv) {
    Args args;
    std::string parse_error;
    if (!parseArgs(argc, argv, args, parse_error)) {
        std::fprintf(stderr, "%s\n", parse_error.c_str());
        usage(argv[0]);
        return 2;
    }
    if (args.help) {
        usage(argv[0]);
        return 0;
    }

    TestContext ctx;
    std::string log_error;
    if (!ctx.log.open(log_error)) {
        std::fprintf(stderr, "日志文件打开失败，继续使用控制台日志: %s\n", log_error.c_str());
    }

    std::string config_error;
    const std::string config_path = findConfigFile(args.config);
    if (!loadConfig(config_path, ctx.cfg, config_error)) {
        ctx.log.error(Phase::kConfig, config_error);
        return 2;
    }
    ctx.log.info(Phase::kConfig, "配置加载成功: " + config_path);

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    if (!waitLink(ctx)) {
        return 1;
    }
    if (!runPreflight(ctx)) {
        safeAbort(ctx, nullptr);
        return 1;
    }
    if (!captureGroundReference(ctx)) {
        safeAbort(ctx, nullptr);
        return 1;
    }

    OffboardStreamer streamer(ctx.comm, ctx.cfg.offboard_rate_hz);
    if (!arm(ctx)) {
        safeAbort(ctx, &streamer);
        return 1;
    }
    if (!takeoff(ctx)) {
        safeAbort(ctx, &streamer);
        return 1;
    }
    if (!waitAltitude(ctx, static_cast<double>(ctx.cfg.takeoff_altitude_m), ctx.cfg.takeoff_timeout_s)) {
        safeAbort(ctx, &streamer);
        return 1;
    }
    if (!offboardHover(ctx, streamer)) {
        safeAbort(ctx, &streamer);
        return 1;
    }
    if (!landAndDisarm(ctx, streamer)) {
        safeAbort(ctx, &streamer);
        return 1;
    }

    ctx.comm.stop();
    ctx.log.info(Phase::kDone, "test_drone 完成");
    return 0;
}
