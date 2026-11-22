#include "startup/app_startup.hpp"

#include <cstdio>
#include <filesystem>

#include "glue/guidance_session.hpp"
#include "mission/mission_config.hpp"
#include "startup/guidance_application.hpp"

namespace pure_uav::app {
bool loadAppStartupConfig(const AppOptions& options, AppStartupConfig& settings, std::string& error) {
    // 1. 应用只组合模块配置；VIO 参数来自独立 vio.yaml。
    auto& config = settings.app;
    ConfigOverrides overrides;
    overrides.driver_path = options.driver_config_path;
    overrides.vio_path = options.vio_config_path;
    overrides.camera_path = options.camera_config;
    overrides.shm_vio = options.shm_vio;
    overrides.shm_timeout_ms = options.shm_timeout_ms;
    if (!loadAppConfig(options.config, overrides, config, error)) {
        return false;
    }
    auto& mission_config = settings.mission;
    auto& mission_guidance = settings.guidance;
    mission_guidance = guidance_default_params();
    if (options.mission) {
        const auto resolve = [&](const std::string& path) { return (std::filesystem::path(config.repository_root) / path).string(); };
        if (!mission::loadRuntimeConfig(resolve(options.mission_config), mission_config, error)) {
            return false;
        }
        if (!loadPreflightConfig(resolve(options.preflight_config), settings.preflight, error)) {
            return false;
        }
        mission_config.send_enabled = !options.no_send;
        GuidanceSession guidance;
        if (!guidance.init(resolve(options.guidance), resolve(options.camera_calibration), config.color, error)) {
            return false;
        }
        if (!guidance.cameraOk()) {
            std::fprintf(stderr, "[MAIN] 彩色相机未标定：%s；任务只搜索，不进入撞板\n", guidance.cameraReason().c_str());
        }
        mission_guidance = guidance.params();
        // 日志文件相对路径按仓库根目录解析，和 config/*.yaml 同一规则。
        if (!mission_guidance.log.file.empty() && std::filesystem::path(mission_guidance.log.file).is_relative()) {
            mission_guidance.log.file = resolve(mission_guidance.log.file);
        }
        settings.camera = guidance.cameraCalibration();
        if (config.main_loop_hz < 10 || 1.0 / config.main_loop_hz >= mission_config.max_setpoint_gap_s) {
            error = "main_loop_hz is too low for mission setpoint continuity";
            return false;
        }
        mission_guidance.loop.loop_hz = config.main_loop_hz;
    }
    return true;
}

void printAppStartupConfig(const AppOptions& options, const AppStartupConfig& settings) {
    const auto& config = settings.app;
    const auto& mission_guidance = settings.guidance;
    std::printf("Configuration OK\nflight_stack: PX4\n%sodometry_send_enabled: %s\ndetection_enabled: %s\n", describeConfig(config).c_str(), options.no_send ? "false" : "true", options.no_detection ? "false" : "true");
    std::printf("camera_mode: %s\n", options.external_camera ? "external" : "managed");
    std::printf("mission_enabled: %s\nmission_send_enabled: %s\nmission_guidance_ready: %s\n", options.mission ? "true" : "false", options.mission && !options.no_send ? "true" : "false", options.mission && mission_guidance.loop.cam.valid() ? "true" : "false");
    std::printf("guidance_log: %s %.1f Hz %s\n", mission_guidance.log.enabled ? "enabled" : "disabled", mission_guidance.log.rate_hz, mission_guidance.log.file.empty() ? "stdout" : mission_guidance.log.file.c_str());
}

FlightConnectionResult connectFlightController(Communicator& comm, const AppConfig& config, const std::atomic<bool>& interrupted) {
    if (!comm.start(config.comm)) {
        std::fprintf(stderr, "[MAIN] 串口启动失败：%s\n", config.comm.device.c_str());
        return FlightConnectionResult::kFailed;
    }
    std::printf("[MAIN] 等待飞控心跳（最多 %.1f s）...\n", config.heartbeat_wait_s);
    const auto heartbeat = waitForHeartbeat(comm, interrupted, config.heartbeat_wait_s);
    if (heartbeat == HeartbeatWaitResult::stopped) {
        comm.stop();
        return FlightConnectionResult::kStopped;
    }
    if (heartbeat == HeartbeatWaitResult::timeout) {
        std::fprintf(stderr, "[MAIN] %.1f s 内未收到飞控心跳，VIO 未启动。\n", config.heartbeat_wait_s);
        comm.stop();
        return FlightConnectionResult::kFailed;
    }
    if (comm.getAutopilot() != autopilot::PX4) {
        std::fprintf(stderr, "[MAIN] 当前入口只接受 PX4；检测到：%s。\n", comm.getAutopilotNameCn());
        comm.stop();
        return FlightConnectionResult::kFailed;
    }
    std::printf("[MAIN] 已连接，固件：%s，模式：%s，会话=%llu\n", comm.getAutopilotNameCn(), comm.getFlightModeNameCn(), static_cast<unsigned long long>(comm.reconnectCount()));

    // 读一次姿态示例（通信模块连通性检查）
    Attitude a;
    if (comm.getAttitude(a)) {
        std::printf("[MAIN] 姿态示例 roll=%.3f pitch=%.3f yaw=%.3f\n", a.roll, a.pitch, a.yaw);
    }

    return FlightConnectionResult::kReady;
}

}  // namespace pure_uav::app
