#pragma once

#include <string>

#include "communicator.hpp"
#include "pure_uav/vio_runtime.hpp"
#include "shm_camera_source.hpp"

namespace pure_uav::app {
struct ConfigOverrides {
    std::string driver_path, vio_path, config_dir, shm_vio;
    int shm_timeout_ms = -1;
};
struct AppConfig {
    std::string repository_root, app_path, driver_path, vio_path;
    int main_loop_hz = 50;
    double heartbeat_wait_s = 5, status_period_s = 1;
    int max_age_ms = 150, imu_hz = 200;
    bool guidance_enabled = false;
    CommConfig comm;
    ShmCameraSource::Config camera;
    VioEstimator::Options vio;
    VioPublicationPolicy publication;
};
bool loadAppConfig(const std::string& app_path, const ConfigOverrides& overrides, AppConfig& out, std::string& error);
std::string describeConfig(const AppConfig& config);
}  // namespace pure_uav::app
