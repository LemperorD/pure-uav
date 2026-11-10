#pragma once

#include <string>

namespace pure_uav::app {

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

/// 加载并校验 config/main.yaml；未知键、越界值、缺失段都返回 false。
bool loadMainConfig(const std::string& path, MainConfig& cfg, std::string& error);

}  // namespace pure_uav::app
