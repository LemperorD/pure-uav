// 机载组合根：单串口、单相机，通信 / 检测 / VIO / 制导并发运行。
// 生命周期和配置见 assets/docs/app/usage.md。
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <exception>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "communicator.hpp"
#include "detection_supervisor.hpp"
#include "guidance_loop.hpp"
#include "runtime/bounded_queue.hpp"
#include "runtime/guidance_ports.hpp"
#include "startup/guidance_application.hpp"

#ifdef PURE_APP_HAS_VIO
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "app_config.hpp"
#include "camera_service.hpp"
#include "fcu_clock.hpp"
#include "glue/comm_imu_source.hpp"
#include "live_timing.hpp"
#include "pose_gate.hpp"
#include "pure_uav/vio_runtime.hpp"
#endif

namespace {
using namespace pure_uav;
namespace flight = guidance::flight;
using Clock = std::chrono::steady_clock;

// 信号处理只置位；心跳等待和主循环都只读这一个退出标志，其他模块也复用。
std::atomic<bool> interrupted{false};
static_assert(std::atomic<bool>::is_always_lock_free, "Signal flag must be lock-free");

void onSignal(int /*unused*/) {
    interrupted.store(true, std::memory_order_relaxed);
}

int64_t nowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
}

struct Options {
    bool help = false;
    bool no_send = false;
    bool check_config = false;
    bool no_detection = false;
    std::string detect_bin, detection_config;
    std::vector<std::string> detect_args;
    std::string config = "config/app.yaml";
    std::string driver, vio_config, shm_vio, vio;
    int shm_timeout_ms = -1;
    std::string guidance = "config/onboard-guidance.yaml";  // 预留：制导尚未接入
    std::string blackbox;                                   // 预留：黑匣子尚未接入
};

void usage(const char* name) {
    std::printf(
        "用法: %s [选项]\n"
        "启动通信、连接 camera_server 和运行 VIO，按固定节拍发布位姿；Ctrl-C 停止。\n"
        "飞控心跳就绪后启动检测子进程；制导与应用状态机尚未接入。\n"
        "  --config FILE         应用调度配置（默认 config/app.yaml）\n"
        "  --vio-config FILE     VIO 运行配置（默认由 app.yaml 指向 config/vio.yaml）\n"
        "  --check-config        校验并打印实际配置后退出，不打开设备\n"
        "  --driver-config FILE  通信配置（默认 config/driver.yaml）\n"
        "  --config-dir DIR      覆盖 VIO 标定目录（默认由 vio.yaml 指定）\n"
        "  --shm-vio NAME        camera_server 的 VIO 通道（默认 /pure_uav_camera_vio）\n"
        "  --shm-timeout-ms N    等待相机服务的超时（默认 5000 ms）\n"
        "  --no-send             运行 VIO 和发布检查，但不发送位姿\n"
        "  --detect-bin FILE     检测子程序路径（默认与 main 同目录的 detect_rt）\n"
        "  --detection-config FILE  传给检测子进程的 --config 路径\n"
        "  --detect-arg ARG      可重复，原样传给检测子进程的运行参数\n"
        "  --no-detection        仅运行通信和 VIO，不拉起检测子进程\n"
        "  -h, --help            查看帮助\n"
        "  --guidance-config/--blackbox 为后续制导预留，当前不生效。\n",
        name);
}

Options parseOptions(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "-h" || arg == "--help") {
            options.help = true;
            return options;
        }
        if (arg == "--check-config") {
            options.check_config = true;
            continue;
        }
        if (arg == "--no-detection") {
            options.no_detection = true;
            continue;
        }
        if (arg == "--detect-arg") {
            if (++i == argc) {
                throw std::invalid_argument(arg + " 缺少参数值");
            }
            options.detect_args.emplace_back(argv[i]);
            continue;
        }
        if (arg == "--no-send") {
            options.no_send = true;
            continue;
        }
        if (arg == "--shm-timeout-ms") {
            if (++i == argc) {
                throw std::invalid_argument(arg + " 缺少参数值");
            }
            size_t used = 0;
            options.shm_timeout_ms = std::stoi(argv[i], &used);
            if (used != std::strlen(argv[i]) || options.shm_timeout_ms < 0 || options.shm_timeout_ms > 60000) {
                throw std::invalid_argument("--shm-timeout-ms 只接受 0..60000");
            }
            continue;
        }
        std::string* value = nullptr;
        if (arg == "--config") {
            value = &options.config;
        } else if (arg == "--vio-config") {
            value = &options.vio_config;
        } else if (arg == "--driver-config") {
            value = &options.driver;
        } else if (arg == "--shm-vio") {
            value = &options.shm_vio;
        } else if (arg == "--config-dir") {
            value = &options.vio;
        } else if (arg == "--guidance-config") {
            value = &options.guidance;
        } else if (arg == "--detect-bin") {
            value = &options.detect_bin;
        } else if (arg == "--detection-config") {
            value = &options.detection_config;
        } else if (arg == "--blackbox") {
            value = &options.blackbox;
        } else {
            throw std::invalid_argument("未知参数：" + arg);
        }
        if (++i == argc) {
            throw std::invalid_argument(arg + " 缺少参数值");
        }
        *value = argv[i];
    }
    return options;
}

std::string locateDetectBinary(const std::string& configured) {
    if (!configured.empty()) {
        return configured;
    }
    const auto self = std::filesystem::read_symlink("/proc/self/exe");
    return (self.parent_path() / "detect_rt").string();
}

#ifdef PURE_APP_HAS_VIO
struct Frame {
    cv::Mat gray, depth;
    int64_t host_ns = 0;
    uint64_t session = 0;
};

// 线程异常统一回到监督线程；析构时先通知退出，再 join，禁止后台悬空访问资源。
// 后续检测/制导若要自己的线程，可复用这个壳；VIO 不用它（VioRuntime 自带内部线程）。
class Workers {
  public:
    std::atomic<bool> stop{false};
    std::atomic<bool> failed{false};
    template <typename F>
    void launch(const char* name, F work) {
        threads_.emplace_back([this, name, work] {
            try {
                work();
            } catch (const std::exception& error) {
                std::fprintf(stderr, "[%s] %s\n", name, error.what());
                failed = true;
                stop = true;
            } catch (...) {
                std::fprintf(stderr, "[%s] 未知异常\n", name);
                failed = true;
                stop = true;
            }
        });
    }
    void join() {
        stop = true;
        for (auto& thread : threads_) {
            if (thread.joinable()) {
                thread.join();
            }
        }
    }
    std::vector<std::thread> threads_;
};  // class Workers

using namespace pure_uav::app;

constexpr uint8_t kArduPilotAutopilot = 3;  // MAV_AUTOPILOT_ARDUPILOTMEGA

// 位姿发布去重状态：重连或时间基变化时整体归零后重新起算。
struct PosePublisher {
    uint64_t last_sent_us = 0;
    uint64_t session = 0;
    uint64_t generation = 0;
    uint64_t sent = 0, send_errors = 0, would_send = 0;
    std::array<uint64_t, static_cast<size_t>(PoseGateReason::count)> blocked{};
};

// 公共主循环里 VIO 的一拍：取受保护快照 -> 门控 -> 填质量/重置计数 -> 经同一个
// Communicator 发送。IMU/图像的喂数由 VioRuntime 内部线程按数据到达处理，这里不 feed。
// 返回 false 表示 VIO 工作线程已停止，上层应结束本拍并退出。
bool tickVio(VioRuntime& vio, Communicator& comm, bool send, int max_age_ms, PosePublisher& publisher) {
    if (!vio.running()) {
        return false;
    }
    // publication guard 只覆盖最终检查与发送，不包含休眠或日志。
    auto publication = vio.publication();
    const auto& snapshot = publication.snapshot();
    const auto session = comm.reconnectCount();
    const auto generation = publication.generation();
    if (session != publisher.session || generation != publisher.generation) {
        publisher.session = session;
        publisher.generation = generation;
        publisher.last_sent_us = 0;  // 重连或时间基变化后同步重置去重状态
    }
    PoseGateContext gate;
    gate.last_sent_us = publisher.last_sent_us;
    gate.publishable = publication.publishable();
    gate.clock_valid = publication.clockValid();
    gate.timebase_generation = publisher.generation;
    gate.connected = comm.isConnected();
    gate.session = publisher.session;
    gate.now_ns = nowNs();
    gate.max_age_ms = max_age_ms;
    const auto reason = poseGateReason(snapshot, gate);
    if (reason != PoseGateReason::allow) {
        ++publisher.blocked[static_cast<size_t>(reason)];
        return true;
    }
    if (!send) {
        ++publisher.would_send;  // --no-send：门控照跑，只跳过串口发送
        return true;
    }
    // snapshot.pose.timestamp_us 已是主机 Unix 微秒（VioRuntime 经 FcuClock 映射），
    // 发布前不再用 SYSTEM_TIME 做一次时间转换。
    VisionPose pose = snapshot.pose;
    pose.quality = static_cast<int8_t>(snapshot.quality);
    pose.reset_counter = odometryResetCounter(static_cast<uint64_t>(snapshot.rebuilds));
    pose.pos_sigma_m = snapshot.pos_sigma_m;
    pose.vel_sigma_mps = snapshot.vel_sigma_mps;
    // Communicator 在实际写入前会再复核会话，会话变化不会越过这次发送。
    if (comm.pushVisionPoseForSession(pose, publisher.session)) {
        publisher.last_sent_us = pose.timestamp_us;  // 只有发送成功才推进去重
        ++publisher.sent;
    } else {
        ++publisher.send_errors;
    }
    return true;
}

void logStatus(const Communicator& comm, const VioRuntime& vio, const CameraService& camera, const PosePublisher& publisher, uint64_t overruns) {
    const auto stats = vio.stats();
    VioPoseSnapshot snapshot;
    vio.latestPose(snapshot);
    const auto& blocked = publisher.blocked;
    const double capture_age_ms = snapshot.sample_host_ns > 0 ? static_cast<double>(nowNs() - snapshot.sample_host_ns) * 1e-6 : -1.0;
    std::printf(
        "[MAIN] connected=%d firmware=%s mode=%s session=%llu generation=%llu "
        "imu=%.1fHz health=%s features=%d zupt=%d "
        "sent=%llu send_errors=%llu would_send=%llu "
        "blocked(stale/duplicate/lost/timebase/session/disconnected)=%llu/%llu/%llu/%llu/%llu/%llu "
        "frames=%llu dropped=%llu imu_dropped=%llu camera_dropped=%llu poses=%llu resets=%llu "
        "capture_age_ms=%.1f overruns=%llu error=%s\n",
        static_cast<int>(comm.isConnected()), comm.getAutopilotNameCn(), comm.getFlightModeNameCn(), static_cast<unsigned long long>(publisher.session), static_cast<unsigned long long>(publisher.generation), stats.imu_hz, toString(static_cast<VioHealthState>(snapshot.health)), snapshot.features,
        stats.zupt_enabled ? 1 : 0, static_cast<unsigned long long>(publisher.sent), static_cast<unsigned long long>(publisher.send_errors), static_cast<unsigned long long>(publisher.would_send), static_cast<unsigned long long>(blocked[static_cast<size_t>(PoseGateReason::stale)]),
        static_cast<unsigned long long>(blocked[static_cast<size_t>(PoseGateReason::duplicate)]), static_cast<unsigned long long>(blocked[static_cast<size_t>(PoseGateReason::lost)]), static_cast<unsigned long long>(blocked[static_cast<size_t>(PoseGateReason::timebase)]),
        static_cast<unsigned long long>(blocked[static_cast<size_t>(PoseGateReason::session)]), static_cast<unsigned long long>(blocked[static_cast<size_t>(PoseGateReason::disconnected)]), static_cast<unsigned long long>(stats.frames_fed), static_cast<unsigned long long>(stats.frames_dropped),
        static_cast<unsigned long long>(stats.imu_dropped), static_cast<unsigned long long>(camera.dropped()), static_cast<unsigned long long>(stats.poses), static_cast<unsigned long long>(stats.timebase_resets), capture_age_ms, static_cast<unsigned long long>(overruns),
        stats.last_error.empty() ? "-" : stats.last_error.c_str());
}

int runVio(const Options& options) {
    // 1. 应用只组合模块配置；VIO 参数来自独立 vio.yaml。
    AppConfig config;
    ConfigOverrides overrides;
    overrides.driver_path = options.driver;
    overrides.vio_path = options.vio_config;
    overrides.config_dir = options.vio;
    overrides.shm_vio = options.shm_vio;
    overrides.shm_timeout_ms = options.shm_timeout_ms;
    std::string config_error;
    if (!loadAppConfig(options.config, overrides, config, config_error)) {
        std::fprintf(stderr, "Configuration error: %s\n", config_error.c_str());
        return 2;
    }
    if (options.check_config) {
        std::printf("Configuration OK\n%sodometry_send_enabled: %s\ndetection_enabled: %s\n", describeConfig(config).c_str(), options.no_send ? "false" : "true", options.no_detection ? "false" : "true");
        return 0;
    }
    const auto& vio_options = config.vio;
    const auto& comm_config = config.comm;
    // 2. 整机唯一的 Communicator：VIO 不另开飞控串口。
    Communicator comm;
    if (!comm.start(comm_config)) {
        std::fprintf(stderr, "[MAIN] 串口启动失败：%s\n", comm_config.device.c_str());
        return 1;
    }
    std::printf("[MAIN] 等待飞控心跳（最多 %.1f s）...\n", config.heartbeat_wait_s);
    const auto heartbeat = app::waitForHeartbeat(comm, interrupted, config.heartbeat_wait_s);
    if (heartbeat == app::HeartbeatWaitResult::stopped) {
        comm.stop();
        return 0;
    }
    if (heartbeat == app::HeartbeatWaitResult::timeout) {
        std::fprintf(stderr, "[MAIN] %.1f s 内未收到飞控心跳，VIO 未启动。\n", config.heartbeat_wait_s);
        comm.stop();
        return 1;
    }
    if (comm.getAutopilot() != kArduPilotAutopilot) {
        std::fprintf(stderr, "[MAIN] 当前入口只接受 ArduPilot；检测到：%s。\n", comm.getAutopilotNameCn());
        comm.stop();
        return 1;
    }
    std::printf("[MAIN] 已连接，固件：%s，模式：%s，会话=%llu\n", comm.getAutopilotNameCn(), comm.getFlightModeNameCn(), static_cast<unsigned long long>(comm.reconnectCount()));

    // 读一次姿态示例（通信模块连通性检查）
    Attitude a;
    if (comm.getAttitude(a)) {
        std::printf("[MAIN] 姿态示例 roll=%.3f pitch=%.3f yaw=%.3f\n", a.roll, a.pitch, a.yaw);
    }

    // 检测进程可独立读取 camera_server 的彩色通道，不能再打开相机设备。

    // 3. 输入源：同一个 Communicator 的逐帧 IMU + 共享相机（不另开相机）。
    app::CommImuSource imu(comm);
    app::CameraService camera;
    const auto& camera_config = config.camera;

    // 4. 先连接相机通道，再启动 VIO，使 discardBefore 的会话边界生效。
    std::string error;
    if (!camera.start(camera_config, &error)) {
        std::fprintf(stderr, "[CAM] 连接 camera_server 失败：%s\n", error.c_str());
        comm.stop();
        return 1;
    }
    VioRuntime vio(vio_options, imu, camera, config.publication);
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

    // 检测单独消费彩色通道；在心跳、相机与 VIO 就绪后启动，主循环继续发布位姿。
    std::unique_ptr<app::DetectionSupervisor> detection;
    if (!options.no_detection) {
#ifdef PURE_APP_HAS_DETECTION
        app::DetectionProcessOptions process_options;
        process_options.executable = locateDetectBinary(options.detect_bin);
        if (!options.detection_config.empty()) {
            process_options.arguments.emplace_back("--config");
            process_options.arguments.push_back(options.detection_config);
        }
        for (const auto& argument : options.detect_args) {
            process_options.arguments.push_back(argument);
        }
        detection = std::make_unique<app::DetectionSupervisor>(std::move(process_options));
        if (!detection->start(&error)) {
            std::fprintf(stderr, "[MAIN] 检测启动失败：%s\n", error.c_str());
            vio.stop();
            camera.stop();
            comm.stop();
            return 1;
        }
#else
        std::fprintf(stderr, "[MAIN] 当前构建缺少 detect_rt；可用 --no-detection 只运行 VIO\n");
        vio.stop();
        camera.stop();
        comm.stop();
        return 1;
#endif
    }

    // 5. 公共主循环：监督相机/检测进程，同时持续执行 VIO 发布门控。
    PosePublisher publisher;
    uint64_t overruns = 0;
    const auto period = std::chrono::nanoseconds(1000000000LL / config.main_loop_hz);
    auto next_tick = Clock::now();
    auto last_status = next_tick;
    int result = 0;
    while (!interrupted.load()) {
        app::SuperviseResult detection_result;
        if (detection && detection->finished(detection_result)) {
            result = detection_result.status == app::SuperviseStatus::exited || detection_result.status == app::SuperviseStatus::stopped ? 0 : 1;
            std::fprintf(stderr, "[MAIN] 检测监督结束：restarts=%llu error=%s\n", static_cast<unsigned long long>(detection_result.restarts), detection_result.error.empty() ? "-" : detection_result.error.c_str());
            break;
        }
        // 5.1 共享相机监督：相机停止 = 图像断供，不再把旧位姿当新结果发布。
        if (!camera.running()) {
            std::fprintf(stderr, "[MAIN] 共享相机已停止：%s\n", camera.lastError().empty() ? "原因未报告" : camera.lastError().c_str());
            result = 1;
            break;
        }
        // 5.2 VIO 一拍：门控 + 发布。喂数在 VioRuntime 内部线程完成。
        if (!tickVio(vio, comm, !options.no_send, config.max_age_ms, publisher)) {
            const auto stats = vio.stats();
            std::fprintf(stderr, "[MAIN] VIO 工作线程已停止：%s\n", stats.last_error.empty() ? "原因未报告" : stats.last_error.c_str());
            result = 1;
            break;
        }

        // ==================== 后续模块待接位置（本次不实现）====================
        // 检测结果尚未接入制导；推理在独立 detect_rt 进程执行。
        // 制导：      50 Hz 消费飞控状态与目标，产出期望指令，经同一个
        //             Communicator 发送。
        // 任务框架见 mission/mission_controller.hpp；在此构造 MissionInput 并调 tick。
        // 按 MissionOutput 选择任务 setpoint 或制导输出，异步执行飞控命令请求。
        // 检测 IPC、落地反馈和命令执行器接齐后，再启用整机任务。
        // =====================================================================

        // 5.3 状态行限频输出（1 Hz），不逐帧打印。
        const auto now = Clock::now();
        if (now - last_status >= std::chrono::duration<double>(config.status_period_s)) {
            last_status = now;
            logStatus(comm, vio, camera, publisher, overruns);
        }
        // 5.4 固定节拍：错过就跳过，不补发突发流。
        next_tick += period;
        const auto after_work = Clock::now();
        if (next_tick <= after_work) {
            ++overruns;
            next_tick = after_work + period;
        }
        std::this_thread::sleep_until(next_tick);
    }

    // 6. 停止：回收检测子进程，再停止 VIO 与通信；相机服务继续由外部托管。
    if (detection) {
        detection->stop();
    }
    vio.stop();
    camera.stop();
    comm.stop();
    const auto stats = vio.stats();
    std::printf("[MAIN] 已停止：sent=%llu send_errors=%llu would_send=%llu frames=%llu poses=%llu resets=%llu error=%s\n", static_cast<unsigned long long>(publisher.sent), static_cast<unsigned long long>(publisher.send_errors), static_cast<unsigned long long>(publisher.would_send),
                static_cast<unsigned long long>(stats.frames_fed), static_cast<unsigned long long>(stats.poses), static_cast<unsigned long long>(stats.timebase_resets), stats.last_error.empty() ? "-" : stats.last_error.c_str());
    return result;
}
#endif  // PURE_APP_HAS_VIO
}  // namespace

int main(int argc, char** argv) {
    Options options;
    try {
        options = parseOptions(argc, argv);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 2;
    }
    if (options.help) {
        usage(argv[0]);
        return 0;
    }
    std::signal(SIGINT, onSignal);
    std::signal(SIGTERM, onSignal);
    std::signal(SIGPIPE, SIG_IGN);

#ifdef PURE_APP_HAS_VIO
    try {
        return runVio(options);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "[MAIN] 异常退出：%s\n", error.what());
        return 1;
    }
#else
    std::fprintf(stderr, "[MAIN] 当前构建未启用 VIO：缺少实时 VIO 依赖（pure_vio_runtime 目标）。\n");
    return 1;
#endif
}
