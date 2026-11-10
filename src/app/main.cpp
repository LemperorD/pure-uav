// 机载组合根：单串口、单相机，通信 / 检测 / VIO / 制导并发运行。
// 生命周期和配置见 assets/docs/app/usage.md。
#include <unistd.h>

#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <exception>
#include <filesystem>
#include <functional>
#include <memory>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include "communicator.hpp"
#include "guidance_loop.hpp"
#include "runtime/bounded_queue.hpp"
#include "runtime/guidance_ports.hpp"
#include "startup/guidance_application.hpp"

#ifdef PURE_APP_HAS_VIO
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include "camera_service.hpp"
#include "fcu_clock.hpp"
#include "glue/comm_imu_source.hpp"
#include "live_timing.hpp"
#include "orbbec_camera.hpp"
#include "pose_gate.hpp"
#include "pure_uav/vio_runtime.hpp"
#endif
#ifdef PURE_APP_HAS_DETECTION
#include "detector.hpp"
#include "target_bridge.hpp"
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
    std::string driver = "config/driver.yaml";
    std::string vio = "src/vio/estimator/config/gemini_1920x1080";
    std::string guidance = "config/onboard-guidance.yaml";  // 预留：制导尚未接入
    std::string model;                                      // 预留：检测尚未接入
    std::string blackbox;                                   // 预留：黑匣子尚未接入
};

void usage(const char* name) {
    std::printf(
        "用法: %s [选项]\n"
        "启动通信、共享相机和 VIO，按固定节拍发布位姿；Ctrl-C 停止。\n"
        "检测、制导、自检/解锁/起飞/Offboard 状态机尚未接入。\n"
        "  --driver-config FILE  通信配置（默认 config/driver.yaml）\n"
        "  --config-dir DIR      VIO 配置目录（默认 gemini_1920x1080）\n"
        "  --no-send             运行 VIO 和发布检查，但不发送位姿\n"
        "  -h, --help            查看帮助\n"
        "  --guidance-config/--model/--blackbox 为后续制导/检测预留，当前不生效。\n",
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
        if (arg == "--no-send") {
            options.no_send = true;
            continue;
        }
        std::string* value = nullptr;
        if (arg == "--driver-config") {
            value = &options.driver;
        } else if (arg == "--config-dir") {
            value = &options.vio;
        } else if (arg == "--guidance-config") {
            value = &options.guidance;
        } else if (arg == "--model") {
            value = &options.model;
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

constexpr int kMainLoopHz = 50;
constexpr int kPoseMaxAgeMs = 150;      // 与 PoseGateContext 默认值一致
constexpr double kStatusPeriodS = 1.0;  // 状态行限频，避免逐帧打印
constexpr double kHeartbeatWaitS = 5.0;
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
        comm.isConnected(), comm.getAutopilotNameCn(), comm.getFlightModeNameCn(), static_cast<unsigned long long>(publisher.session), static_cast<unsigned long long>(publisher.generation), stats.imu_hz,
        toString(static_cast<VioHealthState>(snapshot.health)), snapshot.features, stats.zupt_enabled ? 1 : 0, static_cast<unsigned long long>(publisher.sent), static_cast<unsigned long long>(publisher.send_errors),
        static_cast<unsigned long long>(publisher.would_send), static_cast<unsigned long long>(blocked[static_cast<size_t>(PoseGateReason::stale)]), static_cast<unsigned long long>(blocked[static_cast<size_t>(PoseGateReason::duplicate)]),
        static_cast<unsigned long long>(blocked[static_cast<size_t>(PoseGateReason::lost)]), static_cast<unsigned long long>(blocked[static_cast<size_t>(PoseGateReason::timebase)]), static_cast<unsigned long long>(blocked[static_cast<size_t>(PoseGateReason::session)]),
        static_cast<unsigned long long>(blocked[static_cast<size_t>(PoseGateReason::disconnected)]), static_cast<unsigned long long>(stats.frames_fed), static_cast<unsigned long long>(stats.frames_dropped), static_cast<unsigned long long>(stats.imu_dropped),
        static_cast<unsigned long long>(camera.dropped()), static_cast<unsigned long long>(stats.poses), static_cast<unsigned long long>(stats.timebase_resets), capture_age_ms, static_cast<unsigned long long>(overruns),
        stats.last_error.empty() ? "-" : stats.last_error.c_str());
}

int runVio(const Options& options) {
    // 1. VIO 配置：沿用现有 --config-dir 和配置文件存在性检查。
    VioEstimator::Options vio_options;
    vio_options.config_dir = findConfigFile(options.vio);
    vio_options.verbose = false;       // 嵌入运行：状态只走限频状态行
    vio_options.quiet_runtime = true;  // 关闭 OpenVINS 控制台热路径与定时文件写入
    const auto vio_config_path = std::filesystem::path(vio_options.config_dir) / vio_options.estimator_config;
    if (!std::filesystem::is_regular_file(vio_config_path)) {
        std::fprintf(stderr, "[VIO] 配置文件不存在或不是普通文件：%s\n", vio_config_path.c_str());
        return 1;
    }

    // 2. 整机唯一的 Communicator：VIO 的 IMU 只从它的逐帧队列取，不另开串口。
    CommConfig comm_config;
    const auto driver_path = findConfigFile(options.driver);
    if (!loadCommConfig(driver_path, comm_config)) {
        std::fprintf(stderr, "[MAIN] 通信配置加载失败：%s\n", driver_path.c_str());
        return 1;
    }
    Communicator comm;
    if (!comm.start(comm_config)) {
        std::fprintf(stderr, "[MAIN] 串口启动失败：%s\n", comm_config.device.c_str());
        return 1;
    }
    std::printf("[MAIN] 等待飞控心跳（最多 %.1f s）...\n", kHeartbeatWaitS);
    const auto heartbeat = app::waitForHeartbeat(comm, interrupted, kHeartbeatWaitS);
    if (heartbeat == app::HeartbeatWaitResult::stopped) {
        comm.stop();
        return 0;
    }
    if (heartbeat == app::HeartbeatWaitResult::timeout) {
        std::fprintf(stderr, "[MAIN] %.1f s 内未收到飞控心跳，VIO 未启动。\n", kHeartbeatWaitS);
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

    // 检测待接：旧的一次性检测示例已删除。接入时检测必须共享 CameraService 的彩色帧
    //（CameraService::start 的 color_callback），不能像旧示例那样自己再打开一台相机；
    // 每拍的处理位置见下方“后续模块待接位置”。本次不实现。

    // 3. 输入源：同一个 Communicator 的逐帧 IMU + 共享相机（不另开相机）。
    app::CommImuSource imu(comm);
    app::CameraService camera;
    OrbbecCamera::Config camera_config;  // 默认 1920x1080@30，与默认 VIO 标定一致
    // 本次不启用深度初始化；若以后打开 use_depth_init，这里会同步打开深度流，
    // CameraService 启动时校验深度标定，VioRuntime 逐帧校验深度尺寸与设备时间。
    camera_config.use_depth = vio_options.use_depth_init;

    // 4. VioRuntime 内部线程是唯一调用 VioEstimator 的地方；主线程不 feed。
    VioRuntime vio(vio_options, imu, camera);
    std::string error;
    if (!vio.start(&error)) {
        std::fprintf(stderr, "[VIO] 启动失败：%s\n", error.c_str());
        comm.stop();
        return 1;
    }
    if (interrupted.load()) {
        vio.stop();
        comm.stop();
        return 0;
    }
    if (!camera.start(camera_config, &error)) {
        std::fprintf(stderr, "[CAM] 共享相机启动失败：%s\n", error.c_str());
        camera.stop();
        vio.stop();
        comm.stop();
        return 1;
    }
    std::printf("[VIO] 已启动：IMU 来自通信模块逐帧队列，图像来自共享相机。\n");

    // 5. 公共主循环：各模块按固定顺序执行自己的一拍；本次只启用 VIO。
    PosePublisher publisher;
    uint64_t overruns = 0;
    const auto period = std::chrono::nanoseconds(1000000000LL / kMainLoopHz);
    auto next_tick = Clock::now();
    auto last_status = next_tick;
    int result = 0;
    while (!interrupted.load()) {
        // 5.1 共享相机监督：相机停止 = 图像断供，不再把旧位姿当新结果发布。
        if (!camera.running()) {
            std::fprintf(stderr, "[MAIN] 共享相机已停止：%s\n", camera.lastError().empty() ? "原因未报告" : camera.lastError().c_str());
            result = 1;
            break;
        }
        // 5.2 VIO 一拍：门控 + 发布。喂数在 VioRuntime 内部线程完成。
        if (!tickVio(vio, comm, !options.no_send, kPoseMaxAgeMs, publisher)) {
            const auto stats = vio.stats();
            std::fprintf(stderr, "[MAIN] VIO 工作线程已停止：%s\n", stats.last_error.empty() ? "原因未报告" : stats.last_error.c_str());
            result = 1;
            break;
        }

        // ==================== 后续模块待接位置（本次不实现）====================
        // 检测主循环：消费共享相机的彩色帧（CameraService::ColorCallback），
        //             不能自己再打开相机；结果按时间/会话与 VIO 位姿对齐。
        // 制导：      50 Hz 消费飞控状态与目标，产出期望指令，经同一个
        //             Communicator 发送。
        // 应用状态机：自检、解锁、起飞、Offboard 切换在此调度。
        // =====================================================================

        // 5.3 状态行限频输出（1 Hz），不逐帧打印。
        const auto now = Clock::now();
        if (now - last_status >= std::chrono::duration<double>(kStatusPeriodS)) {
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

    // 6. 停止：先停 VIO 并 join 内部线程，再停相机/通信；输入源对象最后析构。
    vio.stop();
    camera.stop();
    comm.stop();
    const auto stats = vio.stats();
    std::printf("[MAIN] 已停止：sent=%llu send_errors=%llu would_send=%llu frames=%llu poses=%llu resets=%llu error=%s\n", static_cast<unsigned long long>(publisher.sent),
                static_cast<unsigned long long>(publisher.send_errors), static_cast<unsigned long long>(publisher.would_send), static_cast<unsigned long long>(stats.frames_fed), static_cast<unsigned long long>(stats.poses),
                static_cast<unsigned long long>(stats.timebase_resets), stats.last_error.empty() ? "-" : stats.last_error.c_str());
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
