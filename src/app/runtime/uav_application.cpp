#include "runtime/uav_application.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <stdexcept>
#include <thread>
#include <utility>

#include "camera_server_process.hpp"
#include "pure_uav_factory.hpp"
#include "runtime/guidance_camera.hpp"
#include "runtime/landing_failsafe.hpp"
#include "runtime/vio_pose_publisher.hpp"
#include "startup/app_startup.hpp"
#include "startup/guidance_application.hpp"

namespace pure_uav::app {
namespace {
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

struct UavApplication::Impl {
    explicit Impl(AppOptions options) : options(std::move(options)), imu(comm) {
        interrupted.store(false);
        return_requested.store(false);
        previous_int = std::signal(SIGINT, onSignal);
        previous_term = std::signal(SIGTERM, onSignal);
        previous_return = std::signal(SIGUSR1, onReturnSignal);
        previous_pipe = std::signal(SIGPIPE, SIG_IGN);
    }
    ~Impl() {
        stop();
        std::signal(SIGINT, previous_int);
        std::signal(SIGTERM, previous_term);
        std::signal(SIGUSR1, previous_return);
        std::signal(SIGPIPE, previous_pipe);
    }
    void stop() {
        if (guidance) {
            guidance->stop();
        }
        if (detection) {
            detection->stop();
        }
        if (vio) {
            vio->stop();
        }
        camera.stop();
        camera_server.stop();
        comm.stop();
    }
    std::unique_ptr<IPureUavFactory> prepare() {
        if (prepared) {
            throw std::logic_error("application is already prepared");
        }
        std::string error;
        if (!loadAppStartupConfig(options, settings, error)) {
            std::fprintf(stderr, "Configuration error: %s\n", error.c_str());
            result = 2;
            return {};
        }
        if (options.check_config) {
            printAppStartupConfig(options, settings);
            return {};
        }
        const auto& config = settings.app;

        // 启动：唯一飞控连接 → 相机输入 → VIO → 任务 → 检测。
        const auto connected = connectFlightController(comm, config, interrupted);
        if (connected != FlightConnectionResult::kReady) {
            result = connected == FlightConnectionResult::kStopped ? 0 : 1;
            return {};
        }
        if (interrupted.load()) {
            return {};
        }
        auto camera_input = config.camera;
        if (!options.external_camera) {
            if (!camera_server.start(config.camera_path, camera_input.channel, camera_input.timeout_ms, interrupted, error)) {
                if (!interrupted.load()) {
                    std::fprintf(stderr, "[CAM] 启动 camera_server 失败：%s\n", error.c_str());
                    result = 1;
                }
                return {};
            }
            camera_input.timeout_ms = 0;
        }
        if (interrupted.load()) {
            return {};
        }
        if (!camera.start(camera_input, &error)) {
            std::fprintf(stderr, "[CAM] 连接 camera_server 失败：%s\n", error.c_str());
            result = 1;
            return {};
        }
        if (options.mission) {
            const auto range_error = rangeMeasurementError(camera.streamMeta());
            if (!range_error.empty()) {
                if (!options.no_send) {
                    std::fprintf(stderr, "[CAM] 目标测距不可用：%s\n[CAM] 任务无法进入撞板；只做像素/状态机联调请加 --no-send。\n", range_error.c_str());
                    result = 1;
                    return {};
                }
                std::fprintf(stderr, "[CAM] 目标测距不可用：%s（--no-send：继续运行，但不会撞板）\n", range_error.c_str());
            }
            if (!guidanceCameraMatchesStream(settings.guidance.loop.cam, camera.streamMeta(), error)) {
                std::fprintf(stderr, "[CAM] %s\n", error.c_str());
                result = 1;
                return {};
            }
            // 测距不可用时把 TPN/APN 自动降级为 PPN；有测距时保留用户配置。
            std::string guidance_note;
            if (guidance_apply_range_capability(settings.guidance, range_error.empty(), guidance_note)) {
                std::fprintf(stderr, "[GUIDANCE] %s\n", guidance_note.c_str());
            }
        }
        if (interrupted.load()) {
            return {};
        }
        prepared = true;
        DetectionProductConfig detection_config{options};
        detection_config.options.camera_config = config.camera_path;
        VioProductConfig vio_config{config.vio, imu, camera, config.publication};
        GuidanceProductConfig guidance_config{options, settings, comm};
        return std::make_unique<PureUavFactory>(std::move(detection_config), std::move(vio_config), std::move(guidance_config));
    }
    void init(std::shared_ptr<IVIO> localization, std::shared_ptr<IGuidance> controller, std::shared_ptr<IDetector> detector) {
        if (!prepared || vio) {
            throw std::logic_error("prepare application once before initializing products");
        }
        if (!localization || localization->initialized() || (controller && controller->initialized()) || (detector && detector->initialized())) {
            throw std::invalid_argument("application requires uninitialized products");
        }
        vio = std::move(localization);
        guidance = std::move(controller);
        detection = std::move(detector);
        if (interrupted.load()) {
            return;
        }
        if (guidance) {
            guidance->setVIO(vio);
        }
        if (detection) {
            detection->setGuidance(guidance);
        }
        vio->init();
        if (guidance) {
            guidance->init();
        }
        if (detection) {
            detection->init();
        }
    }
    int run() {
        if (!vio) {
            throw std::logic_error("initialize application products before run");
        }
        const auto& config = settings.app;
        // 故障兜底：停止任务/检测后，由飞控执行降落；不在这里直接退出。
        const auto failsafeLand = [&]() -> bool {
            if (guidance) {
                guidance->stop();
            }
            if (detection) {
                detection->stop();
            }
            if (!comm.isArmed()) {
                return true;
            }
            const auto keepalive = [&] { return publishVioPose(*vio, comm, !options.no_send, config.max_age_ms, publish_state); };
            return runLandingFailsafe(comm, settings.mission.mission.landing_timeout_s, settings.mission.mission.request_retry_s, interrupted, keepalive);
        };

        // 公共循环：监督相机/检测进程，发布 VIO 并调度任务。
        uint64_t overruns = 0;
        const auto period = std::chrono::nanoseconds(1000000000LL / config.main_loop_hz);
        auto next_tick = Clock::now();
        auto last_status = next_tick;
        while (!interrupted.load()) {
            SuperviseResult detection_result;
            if (detection && detection->detect(detection_result)) {
                std::fprintf(stderr, "[MAIN] 检测监督结束：restarts=%llu error=%s\n", static_cast<unsigned long long>(detection_result.restarts), detection_result.error.empty() ? "-" : detection_result.error.c_str());
                detection.reset();
                if (guidance) {
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
            if (!publishVioPose(*vio, comm, !options.no_send, config.max_age_ms, publish_state)) {
                const auto stats = vio->stats();
                std::fprintf(stderr, "[MAIN] VIO 工作线程已停止：%s\n", stats.last_error.empty() ? "原因未报告" : stats.last_error.c_str());
                failsafeLand();
                result = 1;
                break;
            }

            if (guidance) {
                const auto& status = guidance->guide(return_requested.load(std::memory_order_relaxed));
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
                logAppStatus(comm, *vio, camera, publish_state, overruns);
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

        stop();
        logAppStopped(*vio, publish_state);
        return result;
    }

    AppOptions options;
    AppStartupConfig settings;
    Communicator comm;
    CommImuSource imu;
    pure_uav::camera::CameraServerProcess camera_server;
    CameraService camera;
    std::shared_ptr<IVIO> vio;
    std::shared_ptr<IGuidance> guidance;
    std::shared_ptr<IDetector> detection;
    VioPosePublishState publish_state;
    int result = 0;
    bool prepared = false;
    using SignalHandler = void (*)(int);
    SignalHandler previous_int, previous_term, previous_return, previous_pipe;
};

UavApplication::UavApplication(AppOptions options) : impl_(std::make_unique<Impl>(std::move(options))) {
}
UavApplication::~UavApplication() = default;
std::unique_ptr<IPureUavFactory> UavApplication::prepare() {
    return impl_->prepare();
}
void UavApplication::init(std::shared_ptr<IVIO> vio, std::shared_ptr<IGuidance> guidance, std::shared_ptr<IDetector> detector) {
    impl_->init(std::move(vio), std::move(guidance), std::move(detector));
}
int UavApplication::run() {
    return impl_->run();
}
int UavApplication::result() const {
    return impl_->result;
}

}  // namespace pure_uav::app
