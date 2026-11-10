// 机载 main：OFFBOARD 之前的飞行流程。
//
// 本版本只做：
//   连接 → 自检 → guidance 配置/相机标定初始化 → 地面参考
//   → arm → AUTO_TAKEOFF → 爬升 → AUTO_LOITER
//   → 零速度预推流 → 切 OFFBOARD
//
// 本版本不启动 guidance 线程、不接检测目标、不启动 VIO/相机/检测，
// 也不实现 OFFBOARD 后的输出仲裁和正式状态机。
// 切进 OFFBOARD 后只保留一个零速度保持循环，保证 setpoint 不断流；
// Ctrl-C 或链路/模式异常时先切 AUTO_LAND，再停止 setpoint。
//
// 配置见 config/main.yaml、config/onboard-guidance.yaml、
// config/guidance-camera.yaml；参数校验失败返回 2，运行失败返回 1。

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
#include <initializer_list>
#include <iomanip>
#include <memory>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <utility>

#include "common/mavlink.h"
#include "communicator.hpp"
#include "guidance_loop.hpp"
#include "runtime/guidance_ports.hpp"
#include "startup/guidance_application.hpp"

namespace {

using namespace pure_uav;
namespace flight = guidance::flight;
using Clock = std::chrono::steady_clock;

// 代码中的绝对安全上限；YAML 只能在这个范围内调参。
constexpr double kHardMaxTakeoffAltitudeM = 3.0;
constexpr int kHardMaxTimeoutSeconds = 120;
constexpr int kMainLoopPeriodMs = 50;
constexpr int kStatusPeriodMs = 1000;
constexpr int kGroundReferenceSamples = 20;
constexpr int kGroundReferenceIntervalMs = 50;
constexpr double kMaxGroundReferenceStddevM = 0.05;
constexpr double kQuatNormTolerance = 1e-3;
constexpr double kRadToDeg = 57.29577951308232;
constexpr int kLandTimeoutSeconds = 30;
constexpr int kBatteryAlarmPercent = 20;
constexpr double kLandAltitudeM = 0.10;
constexpr double kLandClimbMps = 0.10;

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

struct LinkConfig {
    int timeout_s = 10;
    int heartbeat_max_age_ms = 3000;
    int attitude_max_age_ms = 200;
    int local_position_max_age_ms = 200;
    int system_time_max_age_ms = 2000;
};

struct CommandConfig {
    int ack_timeout_ms = 2000;
    int arm_timeout_s = 5;
    int mode_timeout_s = 5;
};

struct PreflightConfig {
    int min_battery_percent = 30;
    double max_tilt_deg = 10.0;
    double max_vertical_speed_mps = 0.2;
};

struct TakeoffConfig {
    double altitude_m = 1.0;
    int timeout_s = 30;
    double height_tolerance_m = 0.15;
    double height_stable_s = 1.0;
};

struct OffboardConfig {
    int rate_hz = 20;
    double prestream_s = 1.0;
};

struct MainConfig {
    std::string driver_config = "config/driver.yaml";
    std::string guidance_config = "config/onboard-guidance.yaml";
    std::string camera_calibration = "config/guidance-camera.yaml";
    double status_period_s = 1.0;

    LinkConfig link;
    CommandConfig command;
    PreflightConfig preflight;
    TakeoffConfig takeoff;
    OffboardConfig offboard;
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

template <typename T>
bool readValue(const YAML::Node& node, const char* key, T& out, std::string& error, const char* type) {
    if (!node || !node[key]) {
        return true;
    }
    try {
        out = node[key].as<T>();
    } catch (const YAML::Exception&) {
        error = std::string("键 ") + key + " 不是" + type;
        return false;
    }
    return true;
}

bool readString(const YAML::Node& node, const char* key, std::string& out, std::string& error) {
    return readValue<std::string>(node, key, out, error, "字符串");
}

bool readInt(const YAML::Node& node, const char* key, int& out, std::string& error) {
    return readValue<int>(node, key, out, error, "整数");
}

bool readDouble(const YAML::Node& node, const char* key, double& out, std::string& error) {
    return readValue<double>(node, key, out, error, "浮点数");
}

bool readBool(const YAML::Node& node, const char* key, bool& out, std::string& error) {
    return readValue<bool>(node, key, out, error, "布尔值");
}

bool rangeError(const char* field, double value, const char* range, std::string& error) {
    std::ostringstream os;
    os << field << "=" << value << " 超出允许范围 " << range;
    error = os.str();
    return false;
}

bool validateMainConfig(const MainConfig& c, std::string& error) {
    if (c.driver_config.empty()) {
        error = "main.driver_config 不能为空";
        return false;
    }
    if (c.guidance_config.empty()) {
        error = "main.guidance_config 不能为空";
        return false;
    }
    if (c.camera_calibration.empty()) {
        error = "main.camera_calibration 不能为空";
        return false;
    }
    if (c.status_period_s < 0.1 || c.status_period_s > 60.0) {
        return rangeError("main.status_period_s", c.status_period_s, "[0.1, 60]", error);
    }
    if (c.link.timeout_s < 1 || c.link.timeout_s > kHardMaxTimeoutSeconds) {
        return rangeError("link.timeout_s", c.link.timeout_s, "[1, 120]", error);
    }
    if (c.link.heartbeat_max_age_ms < 100 || c.link.heartbeat_max_age_ms > 30000) {
        return rangeError("link.heartbeat_max_age_ms", c.link.heartbeat_max_age_ms, "[100, 30000]", error);
    }
    if (c.link.attitude_max_age_ms < 50 || c.link.attitude_max_age_ms > 5000) {
        return rangeError("link.attitude_max_age_ms", c.link.attitude_max_age_ms, "[50, 5000]", error);
    }
    if (c.link.local_position_max_age_ms < 50 || c.link.local_position_max_age_ms > 5000) {
        return rangeError("link.local_position_max_age_ms", c.link.local_position_max_age_ms, "[50, 5000]", error);
    }
    if (c.link.system_time_max_age_ms < 100 || c.link.system_time_max_age_ms > 30000) {
        return rangeError("link.system_time_max_age_ms", c.link.system_time_max_age_ms, "[100, 30000]", error);
    }
    if (c.command.ack_timeout_ms < 100 || c.command.ack_timeout_ms > 10000) {
        return rangeError("command.ack_timeout_ms", c.command.ack_timeout_ms, "[100, 10000]", error);
    }
    if (c.command.arm_timeout_s < 1 || c.command.arm_timeout_s > 30) {
        return rangeError("command.arm_timeout_s", c.command.arm_timeout_s, "[1, 30]", error);
    }
    if (c.command.mode_timeout_s < 1 || c.command.mode_timeout_s > 30) {
        return rangeError("command.mode_timeout_s", c.command.mode_timeout_s, "[1, 30]", error);
    }
    if (c.preflight.min_battery_percent < 10 || c.preflight.min_battery_percent > 100) {
        return rangeError("preflight.min_battery_percent", c.preflight.min_battery_percent, "[10, 100]", error);
    }
    if (c.preflight.max_tilt_deg < 1.0 || c.preflight.max_tilt_deg > 30.0) {
        return rangeError("preflight.max_tilt_deg", c.preflight.max_tilt_deg, "[1, 30]", error);
    }
    if (c.preflight.max_vertical_speed_mps < 0.0 || c.preflight.max_vertical_speed_mps > 1.0) {
        return rangeError("preflight.max_vertical_speed_mps", c.preflight.max_vertical_speed_mps, "[0, 1]", error);
    }
    if (c.takeoff.altitude_m < 0.3 || c.takeoff.altitude_m > kHardMaxTakeoffAltitudeM) {
        return rangeError("takeoff.altitude_m", c.takeoff.altitude_m, "[0.3, 3.0]", error);
    }
    if (c.takeoff.timeout_s < 5 || c.takeoff.timeout_s > kHardMaxTimeoutSeconds) {
        return rangeError("takeoff.timeout_s", c.takeoff.timeout_s, "[5, 120]", error);
    }
    if (c.takeoff.height_tolerance_m < 0.05 || c.takeoff.height_tolerance_m > 0.5) {
        return rangeError("takeoff.height_tolerance_m", c.takeoff.height_tolerance_m, "[0.05, 0.5]", error);
    }
    if (c.takeoff.height_stable_s < 0.2 || c.takeoff.height_stable_s > 5.0) {
        return rangeError("takeoff.height_stable_s", c.takeoff.height_stable_s, "[0.2, 5]", error);
    }
    if (c.offboard.rate_hz < 10 || c.offboard.rate_hz > 50) {
        return rangeError("offboard.rate_hz", c.offboard.rate_hz, "[10, 50]", error);
    }
    if (c.offboard.prestream_s < 0.2 || c.offboard.prestream_s > 5.0) {
        return rangeError("offboard.prestream_s", c.offboard.prestream_s, "[0.2, 5]", error);
    }
    return true;
}

bool loadMainConfig(const std::string& path, MainConfig& cfg, std::string& error) {
    try {
        const YAML::Node root = YAML::LoadFile(path);
        if (!root || !root.IsMap()) {
            error = "配置文件不是映射: " + path;
            return false;
        }
        if (!checkKeys(root, {"main", "link", "command", "preflight", "takeoff", "offboard"}, "root", error)) {
            return false;
        }

        const YAML::Node main = root["main"];
        if (!main) {
            error = "缺少 main 段";
            return false;
        }
        if (!checkKeys(main, {"driver_config", "guidance_config", "camera_calibration", "status_period_s"}, "main", error)) {
            return false;
        }
        if (!readString(main, "driver_config", cfg.driver_config, error) || !readString(main, "guidance_config", cfg.guidance_config, error) || !readString(main, "camera_calibration", cfg.camera_calibration, error) || !readDouble(main, "status_period_s", cfg.status_period_s, error)) {
            return false;
        }

        const YAML::Node link = root["link"];
        if (!checkKeys(link, {"timeout_s", "heartbeat_max_age_ms", "attitude_max_age_ms", "local_position_max_age_ms", "system_time_max_age_ms"}, "link", error)) {
            return false;
        }
        if (!readInt(link, "timeout_s", cfg.link.timeout_s, error) || !readInt(link, "heartbeat_max_age_ms", cfg.link.heartbeat_max_age_ms, error) || !readInt(link, "attitude_max_age_ms", cfg.link.attitude_max_age_ms, error) ||
            !readInt(link, "local_position_max_age_ms", cfg.link.local_position_max_age_ms, error) || !readInt(link, "system_time_max_age_ms", cfg.link.system_time_max_age_ms, error)) {
            return false;
        }

        const YAML::Node command = root["command"];
        if (!checkKeys(command, {"ack_timeout_ms", "arm_timeout_s", "mode_timeout_s"}, "command", error)) {
            return false;
        }
        if (!readInt(command, "ack_timeout_ms", cfg.command.ack_timeout_ms, error) || !readInt(command, "arm_timeout_s", cfg.command.arm_timeout_s, error) || !readInt(command, "mode_timeout_s", cfg.command.mode_timeout_s, error)) {
            return false;
        }

        const YAML::Node preflight = root["preflight"];
        if (!checkKeys(preflight, {"min_battery_percent", "max_tilt_deg", "max_vertical_speed_mps"}, "preflight", error)) {
            return false;
        }
        if (!readInt(preflight, "min_battery_percent", cfg.preflight.min_battery_percent, error) || !readDouble(preflight, "max_tilt_deg", cfg.preflight.max_tilt_deg, error) || !readDouble(preflight, "max_vertical_speed_mps", cfg.preflight.max_vertical_speed_mps, error)) {
            return false;
        }

        const YAML::Node takeoff = root["takeoff"];
        if (!checkKeys(takeoff, {"altitude_m", "timeout_s", "height_tolerance_m", "height_stable_s"}, "takeoff", error)) {
            return false;
        }
        if (!readDouble(takeoff, "altitude_m", cfg.takeoff.altitude_m, error) || !readInt(takeoff, "timeout_s", cfg.takeoff.timeout_s, error) || !readDouble(takeoff, "height_tolerance_m", cfg.takeoff.height_tolerance_m, error) ||
            !readDouble(takeoff, "height_stable_s", cfg.takeoff.height_stable_s, error)) {
            return false;
        }

        const YAML::Node offboard = root["offboard"];
        if (!checkKeys(offboard, {"rate_hz", "prestream_s"}, "offboard", error)) {
            return false;
        }
        if (!readInt(offboard, "rate_hz", cfg.offboard.rate_hz, error) || !readDouble(offboard, "prestream_s", cfg.offboard.prestream_s, error)) {
            return false;
        }

        return validateMainConfig(cfg, error);
    } catch (const YAML::Exception& e) {
        error = std::string("YAML 解析失败: ") + e.what();
        return false;
    } catch (const std::exception& e) {
        error = std::string("配置加载失败: ") + e.what();
        return false;
    }
}

// ============================================================================
//  日志
// ============================================================================

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
        std::strftime(name, sizeof(name), "Log/main_%Y%m%d_%H%M%S.log", &tm);

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

// ============================================================================
//  guidance：早做参数/标定，不启动线程
// ============================================================================

class CommInsSource final : public flight::GuidanceInsSource {
  public:
    CommInsSource(Communicator& comm, double max_skew_s) : comm_(comm), max_skew_s_(max_skew_s) {
    }

    bool poll(flight::GuidanceInsSample& out) override {
        if (!comm_.isConnected()) {
            return false;
        }
        Attitude attitude;
        LocalPosition position;
        if (!comm_.getAttitude(attitude) || !comm_.getLocalPosition(position)) {
            return false;
        }
        flight::GuidanceInsSample sample;
        if (!app::makeGuidanceIns(attitude, position, max_skew_s_, sample) || sample.boot_us <= last_) {
            return false;
        }
        last_ = sample.boot_us;
        out = sample;
        return true;
    }

    bool alive() const override {
        return comm_.isConnected();
    }

  private:
    Communicator& comm_;
    double max_skew_s_ = 0.2;
    std::uint64_t last_ = 0;
};

// 本阶段没有检测目标；占位源只保证 GuidanceLoop 能构造和 init。
class NullTargetSource final : public flight::GuidanceTargetSource {
  public:
    bool poll(flight::GuidanceTargetSample& /*out*/) override {
        return false;
    }
    bool alive() const override {
        return true;
    }
};

struct CameraCalibration {
    int width = 0;
    int height = 0;
    double fx = 0.0;
    double fy = 0.0;
    double cx = 0.0;
    double cy = 0.0;
    double distortion[5] = {0.0, 0.0, 0.0, 0.0, 0.0};
    double R_body_cam[9] = {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::string source;
};

enum class CameraLoadResult {
    kOk,
    kNotCalibrated,
    kError,
};

bool rotationValid(const double R[9], std::string& error) {
    for (int i = 0; i < 9; ++i) {
        if (!std::isfinite(R[i])) {
            error = "R_body_cam 含非有限数";
            return false;
        }
    }
    for (int row = 0; row < 3; ++row) {
        for (int col = 0; col < 3; ++col) {
            double dot = 0.0;
            for (int k = 0; k < 3; ++k) {
                dot += R[row * 3 + k] * R[col * 3 + k];
            }
            const double expected = (row == col) ? 1.0 : 0.0;
            if (std::fabs(dot - expected) > 1e-3) {
                error = "R_body_cam 不是正交矩阵";
                return false;
            }
        }
    }
    const double det = R[0] * (R[4] * R[8] - R[5] * R[7]) - R[1] * (R[3] * R[8] - R[5] * R[6]) + R[2] * (R[3] * R[7] - R[4] * R[6]);
    if (std::fabs(det - 1.0) > 1e-3) {
        error = "R_body_cam 行列式不是 +1";
        return false;
    }
    return true;
}

class GuidanceBootstrap {
  public:
    bool loadParams(const std::string& path, std::string& error) {
        loop_.reset();
        ins_.reset();
        target_.reset();
        sink_.reset();
        ready_ = false;
        params_ok_ = false;

        params_ = app::guidance_default_params();
        const app::GuidanceParamResult loaded = app::guidance_load_yaml_file(path, &params_);
        if (!loaded.ok) {
            error = loaded.error;
            return false;
        }
        const app::GuidanceParamResult valid = app::guidance_validate_params(params_);
        if (!valid.ok) {
            error = valid.error;
            return false;
        }
        if (params_.guidance.law != GuidanceLaw::PPN || params_.guidance.los_rate_source != LosRateSource::FROM_LOS_DIFFERENCE || params_.guidance.t_go_freeze != 0.0) {
            error = "当前无测距机载路径只支持 PPN + FROM_LOS_DIFFERENCE + t_go_freeze=0";
            return false;
        }
        params_ok_ = true;
        return true;
    }

    CameraLoadResult loadCamera(const std::string& path, std::string& error) {
        camera_ok_ = false;
        camera_fatal_ = false;
        camera_reason_.clear();
        camera_ = CameraCalibration{};
        params_.loop.cam = flight::GuidanceCameraModel{};

        std::error_code ec;
        if (!std::filesystem::is_regular_file(path, ec)) {
            camera_reason_ = "找不到相机标定文件: " + path;
            camera_fatal_ = true;
            error = camera_reason_;
            return CameraLoadResult::kError;
        }

        try {
            const YAML::Node root = YAML::LoadFile(path);
            if (!root || !root.IsMap()) {
                camera_reason_ = "相机标定文件不是映射: " + path;
                camera_fatal_ = true;
                error = camera_reason_;
                return CameraLoadResult::kError;
            }
            if (!checkKeys(root, {"calibrated", "source", "width", "height", "camera_model", "distortion_model", "fx", "fy", "cx", "cy", "distortion", "R_body_cam"}, "camera", error)) {
                camera_reason_ = error;
                camera_fatal_ = true;
                return CameraLoadResult::kError;
            }

            bool calibrated = false;
            if (!root["calibrated"]) {
                camera_reason_ = "相机标定缺少 calibrated";
                camera_fatal_ = true;
                error = camera_reason_;
                return CameraLoadResult::kError;
            }
            if (!readBool(root, "calibrated", calibrated, error)) {
                camera_reason_ = error;
                camera_fatal_ = true;
                return CameraLoadResult::kError;
            }
            if (!readString(root, "source", camera_.source, error)) {
                camera_reason_ = error;
                camera_fatal_ = true;
                return CameraLoadResult::kError;
            }

            if (!calibrated) {
                camera_reason_ = "calibrated=false";
                if (!camera_.source.empty()) {
                    camera_reason_ += " (" + camera_.source + ")";
                }
                return CameraLoadResult::kNotCalibrated;
            }

            if (!readInt(root, "width", camera_.width, error) || !readInt(root, "height", camera_.height, error) || !readDouble(root, "fx", camera_.fx, error) || !readDouble(root, "fy", camera_.fy, error) || !readDouble(root, "cx", camera_.cx, error) || !readDouble(root, "cy", camera_.cy, error)) {
                camera_reason_ = error;
                camera_fatal_ = true;
                return CameraLoadResult::kError;
            }

            std::string camera_model;
            std::string distortion_model;
            if (!readString(root, "camera_model", camera_model, error) || !readString(root, "distortion_model", distortion_model, error)) {
                camera_reason_ = error;
                camera_fatal_ = true;
                return CameraLoadResult::kError;
            }
            if (camera_model != "pinhole" || distortion_model != "radtan") {
                camera_reason_ = "当前只支持 pinhole + radtan 相机模型";
                camera_fatal_ = true;
                error = camera_reason_;
                return CameraLoadResult::kError;
            }

            const YAML::Node distortion = root["distortion"];
            if (!distortion || !distortion.IsSequence() || (distortion.size() != 4 && distortion.size() != 5)) {
                camera_reason_ = "distortion 必须是 4 或 5 个系数";
                camera_fatal_ = true;
                error = camera_reason_;
                return CameraLoadResult::kError;
            }
            for (std::size_t i = 0; i < distortion.size(); ++i) {
                try {
                    camera_.distortion[i] = distortion[i].as<double>();
                } catch (const YAML::Exception&) {
                    camera_reason_ = "distortion 含非数字";
                    camera_fatal_ = true;
                    error = camera_reason_;
                    return CameraLoadResult::kError;
                }
                if (!std::isfinite(camera_.distortion[i])) {
                    camera_reason_ = "distortion 含非有限数";
                    camera_fatal_ = true;
                    error = camera_reason_;
                    return CameraLoadResult::kError;
                }
            }

            const YAML::Node rotation = root["R_body_cam"];
            if (!rotation || !rotation.IsSequence() || rotation.size() != 9) {
                camera_reason_ = "R_body_cam 必须是 9 个数";
                camera_fatal_ = true;
                error = camera_reason_;
                return CameraLoadResult::kError;
            }
            for (std::size_t i = 0; i < rotation.size(); ++i) {
                try {
                    camera_.R_body_cam[i] = rotation[i].as<double>();
                } catch (const YAML::Exception&) {
                    camera_reason_ = "R_body_cam 含非数字";
                    camera_fatal_ = true;
                    error = camera_reason_;
                    return CameraLoadResult::kError;
                }
            }

            if (camera_.width <= 0 || camera_.height <= 0 || !(camera_.fx > 0.0) || !(camera_.fy > 0.0) || !std::isfinite(camera_.cx) || !std::isfinite(camera_.cy)) {
                camera_reason_ = "相机内参/分辨率非法";
                camera_fatal_ = true;
                error = camera_reason_;
                return CameraLoadResult::kError;
            }
            if (!rotationValid(camera_.R_body_cam, error)) {
                camera_reason_ = error;
                camera_fatal_ = true;
                return CameraLoadResult::kError;
            }

            params_.loop.cam.fx = camera_.fx;
            params_.loop.cam.fy = camera_.fy;
            params_.loop.cam.cx = camera_.cx;
            params_.loop.cam.cy = camera_.cy;
            for (int i = 0; i < 9; ++i) {
                params_.loop.cam.R_body_cam[i] = camera_.R_body_cam[i];
            }
            camera_ok_ = true;
            camera_reason_ = camera_.source.empty() ? "calibrated=true" : ("calibrated=true (" + camera_.source + ")");
            return CameraLoadResult::kOk;
        } catch (const YAML::Exception& e) {
            camera_reason_ = std::string("相机 YAML 解析失败: ") + e.what();
            camera_fatal_ = true;
            error = camera_reason_;
            return CameraLoadResult::kError;
        } catch (const std::exception& e) {
            camera_reason_ = std::string("相机标定加载失败: ") + e.what();
            camera_fatal_ = true;
            error = camera_reason_;
            return CameraLoadResult::kError;
        }
    }

    // 参数和相机都就绪时，才构造 GuidanceLoop；本阶段只 init，不 run。
    bool attach(Communicator& comm, std::string& error) {
        loop_.reset();
        sink_.reset();
        target_.reset();
        ins_.reset();
        ready_ = false;
        if (!params_ok_) {
            error = "guidance 参数尚未初始化";
            return false;
        }
        if (!camera_ok_) {
            error = camera_reason_.empty() ? "相机标定未就绪" : camera_reason_;
            return false;
        }

        ins_ = std::make_unique<CommInsSource>(comm, params_.loop.max_sample_age_s);
        target_ = std::make_unique<NullTargetSource>();
        sink_ = std::make_unique<flight::GuidanceNullSink>();
        loop_ = std::make_unique<flight::GuidanceLoop>(params_.guidance, params_.loop, *ins_, *target_, *sink_);
        if (!loop_->init()) {
            error = "GuidanceLoop::init 失败（参数或相机模型无效）";
            camera_fatal_ = true;
            loop_.reset();
            return false;
        }
        ready_ = true;
        return true;
    }

    [[nodiscard]] bool paramsOk() const {
        return params_ok_;
    }
    [[nodiscard]] bool cameraOk() const {
        return camera_ok_;
    }
    [[nodiscard]] bool cameraFatal() const {
        return camera_fatal_;
    }
    [[nodiscard]] bool readyForGuidance() const {
        return ready_;
    }
    [[nodiscard]] const std::string& cameraReason() const {
        return camera_reason_;
    }

  private:
    app::GuidanceAppParams params_;
    std::unique_ptr<CommInsSource> ins_;
    std::unique_ptr<NullTargetSource> target_;
    std::unique_ptr<flight::GuidanceNullSink> sink_;
    std::unique_ptr<flight::GuidanceLoop> loop_;
    bool params_ok_ = false;
    bool camera_ok_ = false;
    bool camera_fatal_ = false;
    bool ready_ = false;
    std::string camera_reason_;
    CameraCalibration camera_;
};

// ============================================================================
//  飞行流程
// ============================================================================

struct AppContext {
    MainConfig cfg;
    Communicator comm;
    Logger log;
    GuidanceBootstrap guidance;
    std::uint64_t reconnect_count = 0;
    double ground_z = 0.0;
    bool airborne = false;
};

bool linkAlive(const AppContext& ctx, std::string& error) {
    if (!ctx.comm.isConnected()) {
        error = "飞控链路断开";
        return false;
    }
    const std::int64_t heartbeat_age = ctx.comm.lastHeartbeatAgeMs();
    if (heartbeat_age < 0 || heartbeat_age > ctx.cfg.link.heartbeat_max_age_ms) {
        error = "心跳超时: lastHeartbeatAgeMs=" + std::to_string(heartbeat_age);
        return false;
    }
    if (ctx.comm.attitudeAgeMs() < 0 || ctx.comm.attitudeAgeMs() > ctx.cfg.link.attitude_max_age_ms) {
        error = "姿态数据不新鲜: attitudeAgeMs=" + std::to_string(ctx.comm.attitudeAgeMs());
        return false;
    }
    if (ctx.comm.localPositionAgeMs() < 0 || ctx.comm.localPositionAgeMs() > ctx.cfg.link.local_position_max_age_ms) {
        error = "位置数据不新鲜: localPositionAgeMs=" + std::to_string(ctx.comm.localPositionAgeMs());
        return false;
    }
    return true;
}

bool readAltitude(const AppContext& ctx, double& altitude_m, double& climb_mps, std::string& error) {
    LocalPosition position;
    if (!ctx.comm.getLocalPosition(position, ctx.cfg.link.local_position_max_age_ms)) {
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

bool waitLink(AppContext& ctx) {
    const std::string path = findConfigFile(ctx.cfg.driver_config);
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

    while (Clock::now() - start < std::chrono::seconds(ctx.cfg.link.timeout_s)) {
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

        const bool ready = connected && locked && px4 && heartbeat_age >= 0 && heartbeat_age <= ctx.cfg.link.heartbeat_max_age_ms && attitude_age >= 0 && attitude_age <= ctx.cfg.link.attitude_max_age_ms && local_age >= 0 && local_age <= ctx.cfg.link.local_position_max_age_ms && system_age >= 0 &&
                           system_age <= ctx.cfg.link.system_time_max_age_ms && battery_seen;
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

constexpr std::size_t kPreflightCheckCount = 17;

struct CheckItem {
    const char* name = "";
    int ok = 0;
    std::string reason;
};

using PreflightChecks = std::array<CheckItem, kPreflightCheckCount>;

void setCheck(PreflightChecks& checks, std::size_t index, const char* name, bool ok, std::string reason) {
    checks[index] = CheckItem{name, ok ? 1 : 0, ok ? std::string{} : std::move(reason)};
}

bool runPreflight(AppContext& ctx) {
    Attitude attitude;
    LocalPosition position;
    Battery battery;
    SystemTime system_time;

    const bool attitude_ok = ctx.comm.getAttitude(attitude, ctx.cfg.link.attitude_max_age_ms);
    const bool position_ok = ctx.comm.getLocalPosition(position, ctx.cfg.link.local_position_max_age_ms);
    const bool system_time_ok = ctx.comm.getSystemTime(system_time, ctx.cfg.link.system_time_max_age_ms);
    const bool battery_ok = ctx.comm.getCountBattery() > 0;
    ctx.comm.getBattery(battery);

    const bool quat_finite = std::isfinite(attitude.q[0]) && std::isfinite(attitude.q[1]) && std::isfinite(attitude.q[2]) && std::isfinite(attitude.q[3]);
    const double quat_norm = quat_finite ? std::sqrt(attitude.q[0] * attitude.q[0] + attitude.q[1] * attitude.q[1] + attitude.q[2] * attitude.q[2] + attitude.q[3] * attitude.q[3]) : 0.0;
    const bool attitude_finite = attitude_ok && std::isfinite(attitude.roll) && std::isfinite(attitude.pitch) && std::isfinite(attitude.yaw) && quat_finite && std::fabs(quat_norm - 1.0) <= kQuatNormTolerance;
    const bool attitude_level = attitude_ok && std::fabs(static_cast<double>(attitude.roll)) * kRadToDeg <= ctx.cfg.preflight.max_tilt_deg && std::fabs(static_cast<double>(attitude.pitch)) * kRadToDeg <= ctx.cfg.preflight.max_tilt_deg;

    const bool position_finite = position_ok && std::isfinite(position.x) && std::isfinite(position.y) && std::isfinite(position.z) && std::isfinite(position.vx) && std::isfinite(position.vy) && std::isfinite(position.vz);
    const bool vertical_speed_ok = position_ok && std::fabs(static_cast<double>(position.vz)) <= ctx.cfg.preflight.max_vertical_speed_mps;

    const std::uint32_t mode = ctx.comm.getCustomMode();
    const bool mode_safe = mode != 0 && mode != px4_mode::OFFBOARD && mode != px4_mode::AUTO_TAKEOFF && mode != px4_mode::AUTO_LAND && mode != px4_mode::AUTO_MISSION && mode != px4_mode::AUTO_RTL;

    PreflightChecks checks{};
    setCheck(checks, 0, "link_connected", ctx.comm.isConnected(), "isConnected=0");
    setCheck(checks, 1, "target_locked", ctx.comm.targetLocked(), "targetLocked=0");
    setCheck(checks, 2, "autopilot_px4", ctx.comm.getAutopilot() == MAV_AUTOPILOT_PX4, "autopilot=" + std::string(ctx.comm.getAutopilotNameCn()));
    setCheck(checks, 3, "heartbeat_fresh", ctx.comm.lastHeartbeatAgeMs() >= 0 && ctx.comm.lastHeartbeatAgeMs() <= ctx.cfg.link.heartbeat_max_age_ms, "lastHeartbeatAgeMs=" + std::to_string(ctx.comm.lastHeartbeatAgeMs()));
    setCheck(checks, 4, "attitude_fresh", attitude_ok, "attitudeAgeMs=" + std::to_string(ctx.comm.attitudeAgeMs()));
    setCheck(checks, 5, "local_position_fresh", position_ok, "localPositionAgeMs=" + std::to_string(ctx.comm.localPositionAgeMs()));
    setCheck(checks, 6, "system_time_fresh", system_time_ok, "systemTimeAgeMs=" + std::to_string(ctx.comm.systemTimeAgeMs()));
    setCheck(checks, 7, "not_armed", !ctx.comm.isArmed(), "isArmed=1");
    setCheck(checks, 8, "mode_safe", mode_safe, "mode=" + modeText(ctx.comm));
    setCheck(checks, 9, "battery_received", battery_ok, "batteryCount=" + std::to_string(ctx.comm.getCountBattery()));
    setCheck(checks, 10, "battery_voltage_valid", battery_ok && std::isfinite(battery.voltage) && battery.voltage > 0.0F, "voltage=" + fixed(battery.voltage));
    setCheck(checks, 11, "battery_level_ok", battery_ok && battery.remaining >= ctx.cfg.preflight.min_battery_percent, "remaining=" + std::to_string(static_cast<int>(battery.remaining)) + "% < " + std::to_string(ctx.cfg.preflight.min_battery_percent) + "%");
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

bool captureGroundReference(AppContext& ctx) {
    double sum = 0.0;
    double sum_sq = 0.0;
    int count = 0;

    for (int i = 0; i < kGroundReferenceSamples; ++i) {
        if (interrupted()) {
            ctx.log.warn(Phase::kGroundReference, "采集地面参考时收到中断");
            return false;
        }
        LocalPosition position;
        if (!ctx.comm.getLocalPosition(position, ctx.cfg.link.local_position_max_age_ms) || !std::isfinite(position.z)) {
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

bool arm(AppContext& ctx) {
    std::uint8_t ack = 0;
    if (!ctx.comm.sendArm(true, ctx.cfg.command.ack_timeout_ms, &ack)) {
        ctx.log.error(Phase::kArm, "解锁未收到 ACK: mode=" + modeText(ctx.comm));
        return false;
    }
    if (ack != MAV_RESULT_ACCEPTED) {
        ctx.log.error(Phase::kArm, "解锁被拒绝: result=" + std::to_string(static_cast<int>(ack)) + " text=" + std::string(ctx.comm.getLastAck()) + " mode=" + modeText(ctx.comm));
        return false;
    }
    if (!waitUntil(std::chrono::seconds(ctx.cfg.command.arm_timeout_s), [&ctx] { return ctx.comm.isArmed(); })) {
        ctx.log.error(Phase::kArm, "ACK 已接受但未进入解锁状态: armed=" + std::to_string(ctx.comm.isArmed()) + " mode=" + modeText(ctx.comm));
        return false;
    }

    ctx.log.info(Phase::kArm, "已解锁: mode=" + modeText(ctx.comm));
    return true;
}

bool switchMode(AppContext& ctx, std::uint32_t custom_mode, const char* name, Phase phase) {
    std::uint8_t ack = 0;
    ctx.log.info(phase, std::string("请求切换模式: ") + name);
    if (!ctx.comm.sendSetMode(custom_mode, ctx.cfg.command.ack_timeout_ms, &ack)) {
        ctx.log.error(phase, std::string("切换 ") + name + " 未收到 ACK: mode=" + modeText(ctx.comm));
        return false;
    }
    if (ack != MAV_RESULT_ACCEPTED) {
        ctx.log.error(phase, std::string("切换 ") + name + " 被拒绝: result=" + std::to_string(static_cast<int>(ack)) + " text=" + std::string(ctx.comm.getLastAck()) + " mode=" + modeText(ctx.comm));
        return false;
    }
    if (!waitUntil(std::chrono::seconds(ctx.cfg.command.mode_timeout_s), [&ctx, custom_mode] { return ctx.comm.getCustomMode() == custom_mode; })) {
        ctx.log.error(phase, std::string("模式未生效: 期望=") + name + " 当前=" + modeText(ctx.comm));
        return false;
    }

    ctx.log.info(phase, std::string("模式已切换: ") + name);
    return true;
}

bool takeoff(AppContext& ctx) {
    if (!switchMode(ctx, px4_mode::AUTO_TAKEOFF, "AUTO_TAKEOFF", Phase::kTakeoffMode)) {
        return false;
    }

    std::uint8_t ack = 0;
    if (!ctx.comm.sendTakeoff(static_cast<float>(ctx.cfg.takeoff.altitude_m), ctx.cfg.command.ack_timeout_ms, &ack)) {
        ctx.log.error(Phase::kTakeoffCommand, "NAV_TAKEOFF 未收到 ACK: mode=" + modeText(ctx.comm));
        return false;
    }
    if (ack != MAV_RESULT_ACCEPTED) {
        ctx.log.error(Phase::kTakeoffCommand, "NAV_TAKEOFF 被拒绝: result=" + std::to_string(static_cast<int>(ack)) + " text=" + std::string(ctx.comm.getLastAck()) + " mode=" + modeText(ctx.comm));
        return false;
    }

    ctx.airborne = true;
    ctx.log.info(Phase::kTakeoffCommand, "起飞命令已接受: target=" + fixed(ctx.cfg.takeoff.altitude_m) + "m");
    return true;
}

bool waitAltitude(AppContext& ctx) {
    const double target = ctx.cfg.takeoff.altitude_m;
    const auto deadline = Clock::now() + std::chrono::seconds(ctx.cfg.takeoff.timeout_s);
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

        const bool near_target = height_ok && std::fabs(altitude_m - target) <= ctx.cfg.takeoff.height_tolerance_m && std::fabs(climb_mps) <= ctx.cfg.preflight.max_vertical_speed_mps;
        if (near_target) {
            if (!stable_since) {
                stable_since = Clock::now();
            }
            if (Clock::now() - *stable_since >= std::chrono::duration<double>(ctx.cfg.takeoff.height_stable_s)) {
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

    ctx.log.error(Phase::kClimb, "爬升超时: target=" + fixed(target) + "m timeout=" + std::to_string(ctx.cfg.takeoff.timeout_s) + "s");
    return false;
}

bool hover(AppContext& ctx) {
    if (!switchMode(ctx, px4_mode::AUTO_LOITER, "AUTO_LOITER", Phase::kHover)) {
        return false;
    }
    if (!sleepInterruptible(1000)) {
        ctx.log.warn(Phase::kHover, "悬停稳定等待时收到中断");
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

    double altitude_m = 0.0;
    double climb_mps = 0.0;
    std::string height_error;
    if (!readAltitude(ctx, altitude_m, climb_mps, height_error)) {
        ctx.log.error(Phase::kHover, height_error);
        return false;
    }
    if (std::fabs(altitude_m - ctx.cfg.takeoff.altitude_m) > ctx.cfg.takeoff.height_tolerance_m) {
        ctx.log.error(Phase::kHover, "悬停高度偏差过大: alt=" + fixed(altitude_m) + "m target=" + fixed(ctx.cfg.takeoff.altitude_m) + "m");
        return false;
    }

    ctx.log.info(Phase::kHover, "AUTO_LOITER 稳定: alt=" + fixed(altitude_m) + "m vz=" + fixed(climb_mps) + "m/s mode=" + modeText(ctx.comm));
    return true;
}

class OffboardZeroStreamer {
  public:
    OffboardZeroStreamer(Communicator& comm, int rate_hz) : comm_(comm), period_us_(1000000 / rate_hz) {
    }

    ~OffboardZeroStreamer() {
        stop();
    }

    OffboardZeroStreamer(const OffboardZeroStreamer&) = delete;
    OffboardZeroStreamer& operator=(const OffboardZeroStreamer&) = delete;

    bool start(std::string& error) {
        if (running_.load()) {
            return true;
        }
        if (period_us_.count() <= 0) {
            error = "OFFBOARD 频率非法";
            return false;
        }
        running_.store(true);
        try {
            thread_ = std::thread(&OffboardZeroStreamer::run, this);
        } catch (const std::exception& e) {
            running_.store(false);
            error = std::string("OFFBOARD 推流线程创建失败: ") + e.what();
            return false;
        }
        return true;
    }

    void stop() {
        running_.store(false);
        if (thread_.joinable()) {
            thread_.join();
        }
    }

    [[nodiscard]] std::uint64_t sent() const {
        return sent_.load();
    }
    [[nodiscard]] std::uint64_t failed() const {
        return failed_.load();
    }
    [[nodiscard]] bool running() const {
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
            if (next < Clock::now()) {
                next = Clock::now() + period_us_;
            }
        }
    }

    Communicator& comm_;
    std::chrono::microseconds period_us_;
    std::atomic<bool> running_{false};
    std::thread thread_;
    std::atomic<std::uint64_t> sent_{0};
    std::atomic<std::uint64_t> failed_{0};
};

bool enterOffboard(AppContext& ctx, OffboardZeroStreamer& streamer) {
    std::string start_error;
    if (!streamer.start(start_error)) {
        ctx.log.error(Phase::kOffboard, start_error);
        return false;
    }

    const int prestream_ms = static_cast<int>(ctx.cfg.offboard.prestream_s * 1000.0);
    if (!sleepInterruptible(prestream_ms)) {
        ctx.log.warn(Phase::kOffboard, "OFFBOARD 预推流时收到中断");
        streamer.stop();
        return false;
    }
    if (streamer.failed() > 0 || streamer.sent() == 0) {
        ctx.log.error(Phase::kOffboard, "OFFBOARD 预推流失败: sent=" + std::to_string(streamer.sent()) + " failed=" + std::to_string(streamer.failed()));
        streamer.stop();
        return false;
    }

    if (!switchMode(ctx, px4_mode::OFFBOARD, "OFFBOARD", Phase::kOffboard)) {
        ctx.log.warn(Phase::kOffboard, "切 OFFBOARD 失败，尝试退回 AUTO_LOITER");
        (void)switchMode(ctx, px4_mode::AUTO_LOITER, "AUTO_LOITER", Phase::kHover);
        streamer.stop();
        return false;
    }

    ctx.log.info(Phase::kOffboard, "已进入 OFFBOARD：零速度保持，后续 guidance 线程/状态机未启动");
    return true;
}

bool holdOffboard(AppContext& ctx, OffboardZeroStreamer& streamer) {
    auto next_log = Clock::now();
    while (!interrupted()) {
        std::string link_error;
        if (!linkAlive(ctx, link_error)) {
            ctx.log.error(Phase::kOffboard, link_error);
            return false;
        }
        if (!ctx.comm.isArmed()) {
            ctx.log.error(Phase::kOffboard, "OFFBOARD 保持期间检测到已解除武装");
            return false;
        }
        if (ctx.comm.getCustomMode() != px4_mode::OFFBOARD) {
            ctx.log.error(Phase::kOffboard, "OFFBOARD 模式丢失: mode=" + modeText(ctx.comm));
            return false;
        }
        if (streamer.failed() > 0) {
            ctx.log.error(Phase::kOffboard, "OFFBOARD setpoint 发送失败: failed=" + std::to_string(streamer.failed()));
            return false;
        }

        Battery battery;
        ctx.comm.getBattery(battery);
        if (battery.remaining >= 0 && battery.remaining < kBatteryAlarmPercent) {
            ctx.log.error(Phase::kOffboard, "电量过低: remaining=" + std::to_string(static_cast<int>(battery.remaining)) + "%");
            return false;
        }

        if (Clock::now() >= next_log) {
            double altitude_m = 0.0;
            double climb_mps = 0.0;
            std::string height_error;
            if (readAltitude(ctx, altitude_m, climb_mps, height_error)) {
                ctx.log.info(Phase::kOffboard, "alt=" + fixed(altitude_m) + "m vz=" + fixed(climb_mps) + "m/s sent=" + std::to_string(streamer.sent()));
            } else {
                ctx.log.warn(Phase::kOffboard, height_error);
            }
            next_log += std::chrono::milliseconds(kStatusPeriodMs);
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(kMainLoopPeriodMs));
    }
    return true;
}

bool waitLanded(AppContext& ctx, int timeout_s) {
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

bool disarm(AppContext& ctx) {
    if (!ctx.comm.isArmed()) {
        ctx.log.info(Phase::kDisarm, "已处于解除武装状态");
        return true;
    }

    std::uint8_t ack = 0;
    if (!ctx.comm.sendArm(false, ctx.cfg.command.ack_timeout_ms, &ack)) {
        ctx.log.error(Phase::kDisarm, "解除武装未收到 ACK: mode=" + modeText(ctx.comm));
        return false;
    }
    if (ack != MAV_RESULT_ACCEPTED) {
        ctx.log.error(Phase::kDisarm, "解除武装被拒绝: result=" + std::to_string(static_cast<int>(ack)) + " text=" + std::string(ctx.comm.getLastAck()));
        return false;
    }
    if (!waitUntil(std::chrono::seconds(ctx.cfg.command.arm_timeout_s), [&ctx] { return !ctx.comm.isArmed(); })) {
        ctx.log.error(Phase::kDisarm, "解除武装超时: armed=" + std::to_string(ctx.comm.isArmed()));
        return false;
    }

    ctx.log.info(Phase::kDisarm, "已解除武装");
    return true;
}

bool landAndDisarm(AppContext& ctx, OffboardZeroStreamer& streamer) {
    if (!ctx.comm.isArmed()) {
        ctx.log.info(Phase::kLand, "已解除武装，无需降落");
        streamer.stop();
        ctx.airborne = false;
        return true;
    }

    std::uint8_t ack = 0;
    bool land_sent = ctx.comm.sendLand(ctx.cfg.command.ack_timeout_ms, &ack);
    if (!land_sent || ack != MAV_RESULT_ACCEPTED) {
        ctx.log.warn(Phase::kLand, "NAV_LAND 未被接受，尝试 AUTO_LAND 模式");
        if (!switchMode(ctx, px4_mode::AUTO_LAND, "AUTO_LAND", Phase::kLand)) {
            ctx.log.error(Phase::kLand, "降落命令与 AUTO_LAND 模式均失败: mode=" + modeText(ctx.comm));
            return false;
        }
    } else if (!waitUntil(std::chrono::seconds(ctx.cfg.command.mode_timeout_s), [&ctx] { return ctx.comm.getCustomMode() == px4_mode::AUTO_LAND; })) {
        ctx.log.warn(Phase::kLand, "NAV_LAND 后模式未变为 AUTO_LAND，尝试再次设置");
        if (!switchMode(ctx, px4_mode::AUTO_LAND, "AUTO_LAND", Phase::kLand)) {
            ctx.log.error(Phase::kLand, "无法进入 AUTO_LAND: mode=" + modeText(ctx.comm));
            return false;
        }
    }

    streamer.stop();
    ctx.log.info(Phase::kLand, "已进入自动降落，OFFBOARD 推流停止");

    if (!waitLanded(ctx, kLandTimeoutSeconds)) {
        return false;
    }
    if (!disarm(ctx)) {
        return false;
    }

    ctx.airborne = false;
    ctx.log.info(Phase::kLand, "降落并解除武装完成");
    return true;
}

void safeAbort(AppContext& ctx, OffboardZeroStreamer* streamer) {
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
        if (!ctx.comm.sendLand(ctx.cfg.command.ack_timeout_ms, &ack) || ack != MAV_RESULT_ACCEPTED) {
            (void)ctx.comm.sendSetMode(px4_mode::AUTO_LAND, ctx.cfg.command.ack_timeout_ms, nullptr);
        }
        (void)waitUntil(std::chrono::seconds(ctx.cfg.command.mode_timeout_s), [&ctx] { return ctx.comm.getCustomMode() == px4_mode::AUTO_LAND; });
        if (streamer != nullptr) {
            streamer->stop();
        }
        landed = waitLanded(ctx, 10);
    }

    if (streamer != nullptr) {
        streamer->stop();
    }
    if (ctx.comm.isArmed() && landed) {
        (void)disarm(ctx);
    } else if (ctx.comm.isArmed()) {
        ctx.log.error(Phase::kAbort, "未确认落地，保持解锁状态，交给飞控/操作员处理");
    }
}

struct Args {
    std::string config = "config/main.yaml";
    bool check_config = false;
    bool help = false;
};

void usage(const char* program) {
    std::printf(
        "用法: %s [选项]\n"
        "  --config FILE      主配置，默认 config/main.yaml\n"
        "  --check-config     只校验配置和相机标定，不打开串口/设备\n"
        "  -h, --help         查看帮助\n",
        program);
}

bool parseArgs(int argc, char** argv, Args& args, std::string& error) {
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            args.help = true;
            continue;
        }
        if (arg == "--check-config") {
            args.check_config = true;
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

bool checkConfigOnly(const MainConfig& cfg) {
    const std::string driver_path = findConfigFile(cfg.driver_config);
    CommConfig comm_cfg;
    if (!loadCommConfig(driver_path, comm_cfg)) {
        std::fprintf(stderr, "通信配置加载失败: %s\n", driver_path.c_str());
        return false;
    }

    // 声明顺序保证 guidance 先于 unused_comm 析构；attach 只构造引用，不打开串口。
    Communicator unused_comm;
    GuidanceBootstrap guidance;
    std::string error;
    const std::string guidance_path = findConfigFile(cfg.guidance_config);
    if (!guidance.loadParams(guidance_path, error)) {
        std::fprintf(stderr, "guidance 参数加载失败: %s\n", error.c_str());
        return false;
    }

    const std::string camera_path = findConfigFile(cfg.camera_calibration);
    const CameraLoadResult camera = guidance.loadCamera(camera_path, error);
    if (camera == CameraLoadResult::kError) {
        std::fprintf(stderr, "相机标定加载失败: %s\n", error.c_str());
        return false;
    }
    if (camera == CameraLoadResult::kOk) {
        // GuidanceLoop::init() 不依赖真实串口；这里用未启动的 Communicator
        // 只验证参数和相机模型，不打开设备。
        if (!guidance.attach(unused_comm, error)) {
            std::fprintf(stderr, "GuidanceLoop 初始化失败: %s\n", error.c_str());
            return false;
        }
    }

    std::printf("配置校验通过\n");
    std::printf("  guidance loop: %s\n", guidance.readyForGuidance() ? "initialized" : "not initialized（相机未标定）");
    std::printf("  driver: %s (%s @ %d)\n", driver_path.c_str(), comm_cfg.device.c_str(), comm_cfg.baudrate);
    std::printf("  guidance: %s\n", guidance_path.c_str());
    std::printf("  camera: %s (%s)\n", camera_path.c_str(), guidance.cameraReason().c_str());
    std::printf("  takeoff: %.2f m, timeout=%d s\n", cfg.takeoff.altitude_m, cfg.takeoff.timeout_s);
    std::printf("  offboard: %d Hz, prestream=%.2f s\n", cfg.offboard.rate_hz, cfg.offboard.prestream_s);
    return camera != CameraLoadResult::kError;
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

    const std::string config_path = findConfigFile(args.config);
    MainConfig cfg;
    std::string config_error;
    if (!loadMainConfig(config_path, cfg, config_error)) {
        std::fprintf(stderr, "配置加载失败: %s\n", config_error.c_str());
        return 2;
    }

    if (args.check_config) {
        return checkConfigOnly(cfg) ? 0 : 2;
    }

    AppContext ctx;
    ctx.cfg = std::move(cfg);

    std::string log_error;
    if (!ctx.log.open(log_error)) {
        std::fprintf(stderr, "日志文件打开失败，继续使用控制台日志: %s\n", log_error.c_str());
    }
    ctx.log.info(Phase::kConfig, "配置加载成功: " + config_path);

    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);

    if (!waitLink(ctx)) {
        ctx.comm.stop();
        return interrupted() ? 0 : 1;
    }

    // 早做 guidance 的参数/标定初始化；本阶段不启动 guidance 线程。
    std::string guidance_error;
    const bool params_ok = ctx.guidance.loadParams(findConfigFile(ctx.cfg.guidance_config), guidance_error);
    if (!params_ok) {
        ctx.log.error(Phase::kGuidance, "参数初始化失败: " + guidance_error);
    }

    const CameraLoadResult camera_result = ctx.guidance.loadCamera(findConfigFile(ctx.cfg.camera_calibration), guidance_error);
    if (camera_result == CameraLoadResult::kError) {
        ctx.log.error(Phase::kGuidance, "相机标定初始化失败: " + guidance_error);
    } else if (camera_result == CameraLoadResult::kNotCalibrated) {
        ctx.log.warn(Phase::kGuidance, "相机尚未标定: " + ctx.guidance.cameraReason());
    }

    if (params_ok && camera_result == CameraLoadResult::kOk) {
        if (!ctx.guidance.attach(ctx.comm, guidance_error)) {
            ctx.log.error(Phase::kGuidance, "GuidanceLoop 初始化失败: " + guidance_error);
        } else {
            ctx.log.info(Phase::kGuidance, "GuidanceLoop 已初始化（未启动线程）: " + ctx.guidance.cameraReason());
        }
    }
    ctx.log.info(Phase::kGuidance,
                 "状态: params_ok=" + std::to_string(static_cast<int>(ctx.guidance.paramsOk())) + " camera_ok=" + std::to_string(static_cast<int>(ctx.guidance.cameraOk())) + " ready=" + std::to_string(static_cast<int>(ctx.guidance.readyForGuidance())) + " reason=" + ctx.guidance.cameraReason());

    if (!runPreflight(ctx)) {
        safeAbort(ctx, nullptr);
        ctx.comm.stop();
        return interrupted() ? 0 : 1;
    }

    if (!params_ok || ctx.guidance.cameraFatal()) {
        ctx.log.error(Phase::kGuidance, "guidance 配置/标定存在致命错误，禁止解锁");
        safeAbort(ctx, nullptr);
        ctx.comm.stop();
        return 1;
    }
    if (!ctx.guidance.readyForGuidance()) {
        ctx.log.warn(Phase::kGuidance, "guidance 未就绪：本阶段只飞 OFFBOARD 零速度，不启动 guidance");
    }

    if (!captureGroundReference(ctx)) {
        safeAbort(ctx, nullptr);
        ctx.comm.stop();
        return interrupted() ? 0 : 1;
    }

    OffboardZeroStreamer streamer(ctx.comm, ctx.cfg.offboard.rate_hz);

    if (!arm(ctx)) {
        safeAbort(ctx, &streamer);
        ctx.comm.stop();
        return 1;
    }
    if (!takeoff(ctx)) {
        safeAbort(ctx, &streamer);
        ctx.comm.stop();
        return 1;
    }
    if (!waitAltitude(ctx)) {
        safeAbort(ctx, &streamer);
        ctx.comm.stop();
        return 1;
    }
    if (!hover(ctx)) {
        safeAbort(ctx, &streamer);
        ctx.comm.stop();
        return 1;
    }
    if (!enterOffboard(ctx, streamer)) {
        safeAbort(ctx, &streamer);
        ctx.comm.stop();
        return 1;
    }

    const bool hold_ok = holdOffboard(ctx, streamer);
    if (!hold_ok) {
        ctx.log.warn(Phase::kOffboard, "OFFBOARD 保持结束，进入降落");
    }

    if (!landAndDisarm(ctx, streamer)) {
        ctx.log.error(Phase::kLand, "自动降落失败，进入安全退出");
        safeAbort(ctx, &streamer);
        ctx.comm.stop();
        return 1;
    }

    ctx.comm.stop();
    ctx.log.info(Phase::kDone, "main OFFBOARD 前流程完成");
    return 0;
}
