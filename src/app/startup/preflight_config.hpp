#pragma once

#include <string>

namespace pure_uav::app {

/// 任务起飞前自检阈值；只影响 MissionInput.preflight_ready。
struct PreflightConfig {
    int min_battery_percent = 30;
    double max_tilt_deg = 10.0;
    double max_vertical_speed_mps = 0.2;
    int heartbeat_max_age_ms = 3000;
    int attitude_max_age_ms = 200;
    int local_position_max_age_ms = 200;
    int system_time_max_age_ms = 2000;
};

/// 从 config/preflight.yaml 读取；未知键、类型错误、越界都会返回 false。
bool loadPreflightConfig(const std::string& path, PreflightConfig& out, std::string& error);

}  // namespace pure_uav::app
