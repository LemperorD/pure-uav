#pragma once
#include <string>

#include "communicator.hpp"
#include "orbbec_camera.hpp"
#include "pure_uav/detection/detection_runtime.hpp"
#include "pure_uav/vio_runtime.hpp"

namespace pure_uav::app {
struct AppConfig {
    std::string repository_root, app_path, driver_path;
    int main_loop_hz = 50;
    double heartbeat_wait_s = 5, status_period_s = 1;
    int max_age_ms = 150, imu_hz = 200;
    bool guidance_enabled = false, detection_enabled = false;
    CommConfig comm;
    OrbbecCamera::Config camera;
    VioEstimator::Options vio;
    VioPublicationPolicy publication;
    detection::RuntimeConfig detection;
};
bool loadAppConfig(const std::string& app_path, const std::string& driver_path, AppConfig& out, std::string& error);
std::string describeConfig(const AppConfig& config);
}  // namespace pure_uav::app
