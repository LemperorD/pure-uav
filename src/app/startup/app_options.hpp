#pragma once

#include <string>
#include <vector>

namespace pure_uav::app {

struct AppOptions {
    bool help = false;
    bool no_send = false;
    bool check_config = false;
    bool no_detection = false;
    bool mission = false;
    std::string mission_config = "config/mission.yaml";
    std::string detect_bin, detection_config;
    std::vector<std::string> detect_args;
    std::string config = "config/app.yaml";
    std::string driver_config_path, vio_config_path, shm_vio, vio_calibration_dir;
    int shm_timeout_ms = -1;
    std::string guidance = "config/onboard-guidance.yaml";
    std::string camera_calibration = "config/guidance-camera.yaml";
    std::string blackbox;  // 预留：黑匣子尚未接入
};

// 仅解析参数，不打开设备；非法参数抛出 std::invalid_argument。
AppOptions parseAppOptions(int argc, char** argv);
void printAppUsage(const char* name);

}  // namespace pure_uav::app
