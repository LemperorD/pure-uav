#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <exception>
#include <memory>
#include <thread>

#include "startup/app_options.hpp"

#ifdef PURE_APP_HAS_VIO
#include "runtime/guidance_camera.hpp"
#include "runtime/landing_failsafe.hpp"
#include "runtime/vio_pose_publisher.hpp"
#include "startup/app_startup.hpp"
#include "startup/guidance_application.hpp"
#endif

namespace {
using namespace pure_uav;
using namespace pure_uav::app;
using Clock = std::chrono::steady_clock;

std::atomic<bool> interrupted{false};
std::atomic<bool> return_requested{false};
static_assert(std::atomic<bool>::is_always_lock_free, "Signal flag must be lock-free");

void onSignal(int /*unused*/) {
    interrupted.store(true, std::memory_order_relaxed);
}

void onReturnSignal(int /*unused*/) {
    return_requested.store(true, std::memory_order_relaxed);
}

}  // namespace

int main(int argc, char** argv) {
    AppOptions options;
    try {
        options = parseAppOptions(argc, argv);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 2;
    }
    if (options.help) {
        printAppUsage(argv[0]);
        return 0;
    }
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    std::signal(SIGUSR1, onReturnSignal);
    std::signal(SIGPIPE, SIG_IGN);

#ifdef PURE_APP_HAS_VIO
    try {
        AppStartupConfig settings;
        std::string error;
        if (!loadAppStartupConfig(options, settings, error)) {
            std::fprintf(stderr, "Configuration error: %s\n", error.c_str());
            return 2;
        }
        if (options.check_config) {
            printAppStartupConfig(options, settings);
            return 0;
        }
        const auto& config = settings.app;

        // 启动：唯一飞控连接 → 相机输入 → VIO → 任务 → 检测。
        Communicator comm;
        const auto connected = connectFlightController(comm, config, interrupted);
        if (connected != FlightConnectionResult::kReady) {
            return connected == FlightConnectionResult::kStopped ? 0 : 1;
        }
        CommImuSource imu(comm);
        CameraService camera;
        if (!camera.start(config.camera, &error)) {
            std::fprintf(stderr, "[CAM] 连接 camera_server 失败：%s\n", error.c_str());
            comm.stop();
            return 1;
        }
        if (options.mission) {
            const auto range_error = rangeMeasurementError(camera.streamMeta());
            if (!range_error.empty()) {
                if (!options.no_send) {
                    std::fprintf(stderr, "[CAM] 目标测距不可用：%s\n[CAM] 任务无法进入撞板；只做像素/状态机联调请加 --no-send。\n", range_error.c_str());
                    camera.stop();
                    comm.stop();
                    return 1;
                }
                std::fprintf(stderr, "[CAM] 目标测距不可用：%s（--no-send：继续运行，但不会撞板）\n", range_error.c_str());
            }
            if (!guidanceCameraMatchesStream(settings.guidance.loop.cam, camera.streamMeta(), error)) {
                std::fprintf(stderr, "[CAM] %s\n", error.c_str());
                camera.stop();
                comm.stop();
                return 1;
            }
            // 测距不可用时把 TPN/APN 自动降级为 PPN；有测距时保留用户配置。
            std::string guidance_note;
            if (guidance_apply_range_capability(settings.guidance, range_error.empty(), guidance_note)) {
                std::fprintf(stderr, "[GUIDANCE] %s\n", guidance_note.c_str());
            }
        }
        VioRuntime vio(config.vio, imu, camera, config.publication);
        if (interrupted.load()) {
            camera.stop();
            comm.stop();
            return 0;
        }
        if (!vio.start(&error)) {
            std::fprintf(stderr, "[VIO] 启动失败：%s\n", error.c_str());
            camera.stop();
            comm.stop();
            return 1;
        }
        std::printf("[VIO] 已启动：IMU 来自通信模块逐帧队列，图像来自 camera_server 共享内存。\n");
        auto mission_service = createMissionService(options, settings, vio, comm);
        std::unique_ptr<DetectionSupervisor> detection;
        if (!startDetectionProcess(options, mission_service.get(), detection)) {
            vio.stop();
            camera.stop();
            comm.stop();
            return 1;
        }

        VioPosePublishState publish_state;
        // 故障兜底：停止任务/检测后，由飞控执行降落；不在这里直接退出。
        const auto failsafeLand = [&]() -> bool {
            if (mission_service) {
                mission_service->stop();
            }
            if (detection) {
                detection->stop();
            }
            if (!comm.isArmed()) {
                return true;
            }
            const auto keepalive = [&] { return publishVioPose(vio, comm, !options.no_send, config.max_age_ms, publish_state); };
            return runLandingFailsafe(comm, settings.mission.mission.landing_timeout_s, settings.mission.mission.request_retry_s, interrupted, keepalive);
        };

        // 公共循环：监督相机/检测进程，发布 VIO 并调度任务。
        uint64_t overruns = 0;
        const auto period = std::chrono::nanoseconds(1000000000LL / config.main_loop_hz);
        auto next_tick = Clock::now();
        auto last_status = next_tick;
        int result = 0;
        while (!interrupted.load()) {
            SuperviseResult detection_result;
            if (detection && detection->finished(detection_result)) {
                std::fprintf(stderr, "[MAIN] 检测监督结束：restarts=%llu error=%s\n", static_cast<unsigned long long>(detection_result.restarts), detection_result.error.empty() ? "-" : detection_result.error.c_str());
                detection.reset();
                if (mission_service) {
                    return_requested.store(true, std::memory_order_relaxed);
                    std::fprintf(stderr, "[FAILSAFE] 检测不可用，请求任务返航\n");
                }
            }
            // 共享相机监督：相机停止 = 图像断供，不再把旧位姿当新结果发布。
            if (!camera.running()) {
                std::fprintf(stderr, "[MAIN] 共享相机已停止：%s\n", camera.lastError().empty() ? "原因未报告" : camera.lastError().c_str());
                failsafeLand();
                result = 1;
                break;
            }
            // VIO 一拍：门控 + 发布。喂数在 VioRuntime 内部线程完成。
            if (!publishVioPose(vio, comm, !options.no_send, config.max_age_ms, publish_state)) {
                const auto stats = vio.stats();
                std::fprintf(stderr, "[MAIN] VIO 工作线程已停止：%s\n", stats.last_error.empty() ? "原因未报告" : stats.last_error.c_str());
                failsafeLand();
                result = 1;
                break;
            }

            if (mission_service) {
                const auto& status = mission_service->tick(return_requested.load(std::memory_order_relaxed));
                if (status.mission.state_changed) {
                    std::printf("[MISSION] state=%s reason=%s strike=%s\n", mission::toString(status.mission.state), mission::toString(status.mission.reason), status.mission.strike_confirmed ? "confirmed" : "unconfirmed");
                }
                if (status.mission.state == mission::MissionState::kCompleted) {
                    result = 0;
                    break;
                }
                if (status.mission.state == mission::MissionState::kAborted) {
                    std::fprintf(stderr, "[FAILSAFE] 任务中止：%s\n", mission::toString(status.mission.reason));
                    failsafeLand();
                    result = 1;
                    break;
                }
            }

            // 状态行限频输出（1 Hz），不逐帧打印。
            const auto now = Clock::now();
            if (now - last_status >= std::chrono::duration<double>(config.status_period_s)) {
                last_status = now;
                logAppStatus(comm, vio, camera, publish_state, overruns);
            }
            // 固定节拍：错过就跳过，不补发突发流。
            next_tick += period;
            const auto after_work = Clock::now();
            if (next_tick <= after_work) {
                ++overruns;
                next_tick = after_work + period;
            }
            std::this_thread::sleep_until(next_tick);
        }

        if (mission_service) {
            mission_service->stop();
        }

        // 停止：回收检测子进程，再停止 VIO 与通信；相机服务继续由外部托管。
        if (detection) {
            detection->stop();
        }
        vio.stop();
        camera.stop();
        comm.stop();
        logAppStopped(vio, publish_state);
        return result;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "[MAIN] 异常退出：%s\n", error.what());
        return 1;
    }
#else
    std::fprintf(stderr, "[MAIN] 当前构建未启用 VIO：缺少实时 VIO 依赖（pure_vio_runtime 目标）。\n");
    return 1;
#endif
}
