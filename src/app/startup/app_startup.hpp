#pragma once

#include <atomic>
#include <string>

#include "app_config.hpp"
#include "camera_service.hpp"
#include "glue/comm_imu_source.hpp"
#include "glue/guidance_session.hpp"
#include "mission/mission_runtime.hpp"
#include "startup/app_options.hpp"
#include "startup/guidance_app_params.hpp"
#include "startup/preflight_config.hpp"

namespace pure_uav::app {

struct AppStartupConfig {
    AppConfig app;
    mission::RuntimeConfig mission;
    GuidanceAppParams guidance;
    CameraCalibration camera;
    PreflightConfig preflight;
};

// 只做加载和校验，不连接设备。
bool loadAppStartupConfig(const AppOptions& options, AppStartupConfig& settings, std::string& error);
void printAppStartupConfig(const AppOptions& options, const AppStartupConfig& settings);
enum class FlightConnectionResult { kReady, kStopped, kFailed };
FlightConnectionResult connectFlightController(Communicator& comm, const AppConfig& config, const std::atomic<bool>& interrupted);

}  // namespace pure_uav::app
