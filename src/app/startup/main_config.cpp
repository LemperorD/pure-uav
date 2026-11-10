#include "startup/main_config.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <exception>
#include <initializer_list>
#include <sstream>
#include <string>

namespace pure_uav::app {
namespace {

constexpr double kHardMaxTakeoffAltitudeM = 3.0;
constexpr int kHardMaxTimeoutSeconds = 120;

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

}  // namespace

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

}  // namespace pure_uav::app
