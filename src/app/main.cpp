#include <array>
#include <atomic>
#include <chrono>
#include <csignal>
#include <cstdio>
#include <exception>
#include <memory>
#include <string>
#include <thread>

#include "app_config.hpp"
#include "camera_service.hpp"
#include "glue/comm_imu_source.hpp"
#include "live_timing.hpp"
#include "pose_gate.hpp"

namespace {
std::atomic<bool> g_stop{false};
static_assert(std::atomic<bool>::is_always_lock_free, "Signal flag must be lock-free");
void stopSignal(int) {
    g_stop.store(true, std::memory_order_relaxed);
}
int64_t nowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}
void help(const char* program) {
    std::printf(
        "Usage: %s [--config PATH] [--driver-config PATH] [--check-config] [--no-send] [--help]\n"
        "  --config PATH         default: config/app.yaml (relative to repository root)\n"
        "  --driver-config PATH  default: config/driver.yaml\n"
        "  --check-config        validate/print configuration; never open hardware\n"
        "  --no-send             read sensors and run VIO/detection, but never send ODOMETRY\n"
        "  --help, -h            show help without opening hardware\n"
        "Defaults: main 50 Hz, heartbeat wait 5s, pose age <=150ms, IMU request 200 Hz,\n"
        "          camera 1920x1080@30 + depth, ArduPilot ODOMETRY; no flight commands.\n"
        "Optional detection: set app.detection_enabled=true in a build with NPU support.\n"
        "Detection shares the VIO camera, defaults to 10 Hz; output is local target boxes only.\n"
        "SIGINT/SIGTERM stop capture, VIO worker, then communication.\n",
        program);
}
int run(int argc, char** argv) {
    using namespace pure_uav;
    using namespace pure_uav::app;
    using Clock = std::chrono::steady_clock;
    std::string app_path = "config/app.yaml", driver_path = "config/driver.yaml";
    bool check = false, send_odometry = true;
    for (int i = 1; i < argc; ++i) {
        const std::string arg(argv[i]);
        if (arg == "--help" || arg == "-h") {
            help(argv[0]);
            return 0;
        }
        if (arg == "--check-config") {
            check = true;
            continue;
        }
        if (arg == "--no-send") {
            send_odometry = false;
            continue;
        }
        if (arg == "--config" || arg == "--driver-config") {
            if (i + 1 >= argc || std::string(argv[i + 1]).rfind("--", 0) == 0) {
                std::fprintf(stderr, "%s requires a path\n", arg.c_str());
                return 2;
            }
            (arg == "--config" ? app_path : driver_path) = argv[++i];
        } else {
            std::fprintf(stderr, "Unknown argument: %s\n", arg.c_str());
            return 2;
        }
    }
    AppConfig config;
    std::string error;
    if (!loadAppConfig(app_path, driver_path, config, error)) {
        std::fprintf(stderr, "Configuration error: %s\n", error.c_str());
        return 2;
    }
    std::printf("%s", describeConfig(config).c_str());
    std::printf("odometry_send_enabled: %s\n", send_odometry ? "true" : "false");
    if (check) {
        std::puts("Configuration OK; hardware was not opened.");
        return 0;
    }
    std::signal(SIGINT, stopSignal);
    std::signal(SIGTERM, stopSignal);
    Communicator comm;
    comm.setSystemId(2);
    if (!comm.start(config.comm)) return 1;
    std::puts("Waiting for ArduPilot heartbeat...");
    const auto waited = waitForHeartbeat(comm, g_stop, config.heartbeat_wait_s);
    if (waited == HeartbeatWaitResult::stopped) {
        comm.stop();
        return 0;
    }
    if (waited == HeartbeatWaitResult::timeout) {
        std::fprintf(stderr, "No FCU heartbeat within %.2fs\n", config.heartbeat_wait_s);
        comm.stop();
        return 1;
    }
    if (comm.getAutopilot() != 3) {
        std::fprintf(stderr, "Expected ArduPilot, got %s; no odometry sent\n", comm.getAutopilotNameCn());
        comm.stop();
        return 1;
    }
    CommImuSource imu(comm);
    std::unique_ptr<detection::DetectionRuntime> detector;
    if (config.detection_enabled) {
        detector = std::make_unique<detection::DetectionRuntime>(config.detection, detection::makeNpuBackend());
        if (!detector->start(&error)) {
            std::fprintf(stderr, "Detection startup failed: %s\n", error.c_str());
            comm.stop();
            return 1;
        }
    }
    CameraService camera;
    VioRuntime vio(config.vio, imu, camera, config.publication);
    if (!vio.start(&error)) {
        std::fprintf(stderr, "VIO startup failed: %s\n", error.c_str());
        comm.stop();
        return 1;
    }
    if (g_stop.load()) {
        vio.stop();
        comm.stop();
        return 0;
    }
    OrbbecCamera::ColorCallback color_sink;
    if (detector) color_sink = [&](const EncodedColorFrame& frame) { detector->submit(frame); };
    if (!camera.start(config.camera, &error, std::move(color_sink))) {
        std::fprintf(stderr, "Camera startup failed: %s\n", error.c_str());
        camera.stop();
        vio.stop();
        comm.stop();
        return 1;
    }
    uint64_t session = comm.reconnectCount(), generation = vio.timebaseGeneration();
    uint64_t last_sent_us = 0, sent = 0, send_errors = 0, overruns = 0;
    uint64_t last_zupt_change = 0;
    std::array<uint64_t, static_cast<size_t>(PoseGateReason::count)> blocked{};
    const auto period = std::chrono::nanoseconds(1000000000LL / config.main_loop_hz);
    auto next_tick = Clock::now(), last_status = next_tick;
    const auto camera_started = next_tick;
    int result = 0;
    bool detection_failed = false;
    while (!g_stop.load()) {
        if (!vio.running() || !camera.running()) {
            std::fprintf(stderr, "Module stopped: VIO=%s camera=%s\n", vio.stats().last_error.c_str(), camera.lastError().c_str());
            result = 1;
            break;
        }
        if (detector) detector->setEpoch(comm.reconnectCount(), vio.timebaseGeneration());
        const int64_t frame_host_ns = camera.lastFrameHostNs();
        if ((frame_host_ns > 0 && nowNs() - frame_host_ns > 2000000000LL) || (frame_host_ns == 0 && Clock::now() - camera_started > std::chrono::seconds(5))) {
            std::fprintf(stderr, "Camera frame stream stalled\n");
            result = 1;
            break;
        }
        {
            auto publication = vio.publication();
            const auto& snapshot = publication.snapshot();
            const auto new_session = comm.reconnectCount();
            const auto new_generation = publication.generation();
            if (new_session != session || new_generation != generation) {
                session = new_session;
                generation = new_generation;
                last_sent_us = 0;
            }
            PoseGateContext gate{last_sent_us, publication.publishable(), publication.clockValid(), generation, comm.isConnected(), session, nowNs(), config.max_age_ms};
            const auto reason = poseGateReason(snapshot, gate);
            if (reason == PoseGateReason::allow && send_odometry) {
                VisionPose pose = snapshot.pose;
                pose.reset_counter = odometryResetCounter(snapshot.rebuilds);
                pose.quality = static_cast<int8_t>(snapshot.quality);
                pose.pos_sigma_m = snapshot.pos_sigma_m;
                pose.vel_sigma_mps = snapshot.vel_sigma_mps;
                // Session is checked again under Communicator's send mutex.
                if (comm.pushVisionPoseForSession(pose, session)) {
                    last_sent_us = pose.timestamp_us;
                    ++sent;
                } else
                    ++send_errors;
            } else if (reason != PoseGateReason::allow)
                ++blocked[static_cast<size_t>(reason)];
        }
        // Future application/control work is injected here. VIO publication stays independent.
        const auto now = Clock::now();
        if (detector && !detector->running() && !detection_failed) {
            detection_failed = true;
            std::fprintf(stderr, "[DETECTION] disabled after failure: %s; VIO continues\n", detector->stats().last_error.c_str());
        }
        if (now - last_status >= std::chrono::duration<double>(config.status_period_s)) {
            last_status = now;
            const auto stats = vio.stats();
            VioPoseSnapshot s;
            vio.latestPose(s);
            if (stats.zupt_changes != last_zupt_change) {
                last_zupt_change = stats.zupt_changes;
                std::printf("[ZUPT] %s (armed=%d)\n", stats.zupt_enabled ? "enabled" : "disabled", comm.isArmed());
            }
            std::printf(
                "[MAIN] connected=%d firmware=%s mode=%s session=%llu generation=%llu "
                "imu=%.1fHz health=%s features=%d sent=%llu send_errors=%llu "
                "blocked(stale/duplicate/lost/timebase/session/disconnected)=%llu/%llu/%llu/%llu/%llu/%llu "
                "resets=%llu camera_dropped=%llu imu_dropped=%llu overruns=%llu images=%llu poses=%llu "
                "depth_used/rejected=%llu/%llu capture_age_ms=%.1f error=%s\n",
                comm.isConnected(), comm.getAutopilotNameCn(), comm.getFlightModeNameCn(), static_cast<unsigned long long>(session), static_cast<unsigned long long>(generation), stats.imu_hz, pure_uav::toString(static_cast<VioHealthState>(s.health)), s.features,
                static_cast<unsigned long long>(sent), static_cast<unsigned long long>(send_errors), static_cast<unsigned long long>(blocked[static_cast<size_t>(PoseGateReason::stale)]), static_cast<unsigned long long>(blocked[static_cast<size_t>(PoseGateReason::duplicate)]),
                static_cast<unsigned long long>(blocked[static_cast<size_t>(PoseGateReason::lost)]), static_cast<unsigned long long>(blocked[static_cast<size_t>(PoseGateReason::timebase)]), static_cast<unsigned long long>(blocked[static_cast<size_t>(PoseGateReason::session)]),
                static_cast<unsigned long long>(blocked[static_cast<size_t>(PoseGateReason::disconnected)]), static_cast<unsigned long long>(stats.timebase_resets), static_cast<unsigned long long>(camera.dropped()), static_cast<unsigned long long>(stats.imu_dropped),
                static_cast<unsigned long long>(overruns), static_cast<unsigned long long>(stats.frames_fed), static_cast<unsigned long long>(stats.poses), static_cast<unsigned long long>(stats.depth_frames_used), static_cast<unsigned long long>(stats.depth_frames_rejected),
                s.sample_host_ns > 0 ? static_cast<double>(nowNs() - s.sample_host_ns) * 1e-6 : -1.0, stats.last_error.empty() ? "-" : stats.last_error.c_str());
            if (detector) {
                detection::TargetSnapshot target;
                const bool target_valid = detector->latest(target, nowNs()) && comm.isConnected() && target.session == comm.reconnectCount() && target.generation == vio.timebaseGeneration();
                const auto detection_stats = detector->stats();
                std::printf(
                    "[DETECTION] running=%d valid=%d present=%d class=%d conf=%.3f "
                    "center=%.1f,%.1f size=%.1f,%.1f boxes=%d frames=%llu replaced=%llu "
                    "stale=%llu limited=%llu infer_ms=%.1f age_ms=%.1f error=%s\n",
                    detector->running(), target_valid, target_valid && target.result.present, target_valid ? target.result.class_id : -1, target_valid ? target.result.conf : 0.0, target_valid ? target.result.u_px : 0.0, target_valid ? target.result.v_px : 0.0,
                    target_valid ? target.result.w_px : 0.0, target_valid ? target.result.h_px : 0.0, target_valid ? target.result.count : 0, static_cast<unsigned long long>(detection_stats.processed), static_cast<unsigned long long>(detection_stats.replaced),
                    static_cast<unsigned long long>(detection_stats.stale), static_cast<unsigned long long>(detection_stats.rate_limited), detection_stats.last_infer_ms, target_valid ? static_cast<double>(nowNs() - target.captured_host_ns) * 1e-6 : -1.0,
                    detection_stats.last_error.empty() ? "-" : detection_stats.last_error.c_str());
            }
        }
        next_tick += period;
        const auto after_work = Clock::now();
        if (next_tick <= after_work) {
            ++overruns;
            next_tick = after_work + period;  // skip missed ticks, never burst-send to catch up
        }
        std::this_thread::sleep_until(next_tick);
    }
    camera.stop();
    vio.stop();
    comm.stop();
    if (detector) detector->stop();
    std::printf("Stopped: sent=%llu send_errors=%llu frames=%llu resets=%llu\n", static_cast<unsigned long long>(sent), static_cast<unsigned long long>(send_errors), static_cast<unsigned long long>(vio.stats().frames_fed), static_cast<unsigned long long>(vio.stats().timebase_resets));
    return result;
}
}  // namespace
int main(int argc, char** argv) {
    try {
        return run(argc, argv);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Fatal error: %s\n", e.what());
        return 1;
    } catch (...) {
        std::fprintf(stderr, "Fatal unknown error\n");
        return 1;
    }
}
