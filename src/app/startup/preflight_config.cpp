#include "startup/preflight_config.hpp"

#include <yaml-cpp/yaml.h>

#include <cmath>
#include <initializer_list>

namespace pure_uav::app {
namespace {
bool checkKeys(const YAML::Node& node, std::initializer_list<const char*> allowed, std::string& error) {
    if (!node || !node.IsMap()) {
        error = "preflight 必须是映射";
        return false;
    }
    for (const auto& item : node) {
        if (!item.first.IsScalar()) {
            error = "preflight 存在非字符串键";
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
            error = "preflight 存在未知键: " + key;
            return false;
        }
    }
    return true;
}

template <typename T>
bool readValue(const YAML::Node& node, const char* key, T& out, std::string& error, const char* type) {
    if (!node[key]) {
        return true;
    }
    try {
        out = node[key].as<T>();
    } catch (const YAML::Exception&) {
        error = std::string("preflight.") + key + " 不是" + type;
        return false;
    }
    return true;
}

bool validRange(int value, int low, int high) {
    return value >= low && value <= high;
}

bool validRange(double value, double low, double high) {
    return std::isfinite(value) && value >= low && value <= high;
}
}  // namespace

bool loadPreflightConfig(const std::string& path, PreflightConfig& out, std::string& error) {
    try {
        const YAML::Node root = YAML::LoadFile(path);
        if (!root || !root.IsMap() || root.size() != 1 || !root["preflight"]) {
            error = path + ": 缺少唯一的 preflight 映射";
            return false;
        }
        const YAML::Node preflight = root["preflight"];
        if (!checkKeys(preflight, {"min_battery_percent", "max_tilt_deg", "max_vertical_speed_mps", "heartbeat_max_age_ms", "attitude_max_age_ms", "local_position_max_age_ms", "system_time_max_age_ms"}, error)) {
            error = path + ": " + error;
            return false;
        }

        PreflightConfig loaded;
        if (!readValue(preflight, "min_battery_percent", loaded.min_battery_percent, error, "整数") || !readValue(preflight, "max_tilt_deg", loaded.max_tilt_deg, error, "浮点数") || !readValue(preflight, "max_vertical_speed_mps", loaded.max_vertical_speed_mps, error, "浮点数") ||
            !readValue(preflight, "heartbeat_max_age_ms", loaded.heartbeat_max_age_ms, error, "整数") || !readValue(preflight, "attitude_max_age_ms", loaded.attitude_max_age_ms, error, "整数") || !readValue(preflight, "local_position_max_age_ms", loaded.local_position_max_age_ms, error, "整数") ||
            !readValue(preflight, "system_time_max_age_ms", loaded.system_time_max_age_ms, error, "整数")) {
            error = path + ": " + error;
            return false;
        }

        if (!validRange(loaded.min_battery_percent, 0, 100) || !validRange(loaded.max_tilt_deg, 0.0, 90.0) || !validRange(loaded.max_vertical_speed_mps, 0.0, 10.0) || !validRange(loaded.heartbeat_max_age_ms, 1, 60000) || !validRange(loaded.attitude_max_age_ms, 1, 60000) ||
            !validRange(loaded.local_position_max_age_ms, 1, 60000) || !validRange(loaded.system_time_max_age_ms, 1, 60000)) {
            error = path + ": preflight 阈值越界";
            return false;
        }
        out = loaded;
        error.clear();
        return true;
    } catch (const std::exception& e) {
        error = path + ": " + e.what();
        return false;
    }
}

}  // namespace pure_uav::app
