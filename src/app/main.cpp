// 机载组合根：单串口、单相机，通信 / 检测 / VIO / 制导并发运行。
// 生命周期和配置见 assets/docs/app/usage.md。
#include "communicator.hpp"
#include "startup/guidance_application.hpp"
#include "guidance_loop.hpp"
#include "runtime/bounded_queue.hpp"
#include "runtime/guidance_ports.hpp"

#include <atomic>
#include <chrono>
#include <csignal>
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

#ifdef PURE_APP_HAS_VIO
#include "orbbec_camera.hpp"
#include "fcu_clock.hpp"
#include "live_timing.hpp"
#include "pure_uav/vio_estimator.hpp"
#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#endif
#ifdef PURE_APP_HAS_DETECTION
#include "pure_uav/detection/detector.hpp"
#include "pure_uav/detection/target_bridge.hpp"
#endif

namespace {
using namespace pure_uav;
namespace flight = guidance::flight;
using Clock = std::chrono::steady_clock;
volatile std::sig_atomic_t interrupted = 0;
void onSignal(int) { interrupted = 1; }
int64_t nowNs() {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(Clock::now().time_since_epoch()).count();
}

struct Options {
    std::string driver = "config/driver.yaml";
    std::string vio = "src/vio/estimator/config/gemini_1920x1080";
    std::string guidance = "config/onboard-guidance.yaml";
    std::string model;
    std::string blackbox;
    bool no_send = false;
};

void usage(const char* name) {
    std::printf("用法: %s [选项]\n"
                "同时运行通信、共享相机、检测、VIO、制导线程；Ctrl-C 停止。\n"
                "  --driver-config FILE  通信配置（默认 config/driver.yaml）\n"
                "  --config-dir DIR      VIO 配置目录（默认 gemini_1920x1080）\n"
                "  --guidance-config FILE 制导参数覆盖（相机默认读取 VIO 标定）\n"
                "  --model FILE          检测模型（默认 yolo26n，按编译后端选择）\n"
                "  --blackbox FILE       退出时保存制导黑匣子\n"
                "  --no-send             运行计算但不发送位姿和制导指令\n"
                "  -h, --help            查看帮助\n", name);
}

Options parseOptions(int argc, char** argv) {
    Options options;
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--no-send") { options.no_send = true; continue; }
        std::string* value = nullptr;
        if (arg == "--driver-config") value = &options.driver;
        else if (arg == "--config-dir") value = &options.vio;
        else if (arg == "--guidance-config") value = &options.guidance;
        else if (arg == "--model") value = &options.model;
        else if (arg == "--blackbox") value = &options.blackbox;
        else throw std::invalid_argument("未知参数：" + arg);
        if (++i == argc) throw std::invalid_argument(arg + " 缺少参数值");
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
class Workers {
public:
    std::atomic<bool> stop{false};
    std::atomic<bool> failed{false};
    template <typename F> void launch(const char* name, F work) {
        threads_.emplace_back([this, name, work] {
            try { work(); }
            catch (const std::exception& error) {
                std::fprintf(stderr, "[%s] %s\n", name, error.what());
                failed = true;
                stop = true;
            }
            catch (...) {
                std::fprintf(stderr, "[%s] 未知异常\n", name);
                failed = true;
                stop = true;
            }
        });
    }
    void join() {
        stop = true;
        for (auto& thread : threads_) if (thread.joinable()) thread.join();
    }
    ~Workers() { join(); }
private:
    std::vector<std::thread> threads_;
};

struct Calibration {
    app::GuidanceAppParams params = app::guidance_default_params();
    OrbbecCamera::Config camera;
    cv::Mat K, distortion;
};

Calibration loadCalibration(const Options& options) {
    Calibration result;
    // 检测桥只有像素；PPN + 视线差分不依赖距离，禁用基于距离的末端冻结。
    result.params.guidance.los_rate_source = LosRateSource::FROM_LOS_DIFFERENCE;
    result.params.guidance.t_go_freeze = 0.0;
    cv::FileStorage estimator(options.vio + "/estimator_config.yaml", cv::FileStorage::READ);
    if (!estimator.isOpened()) throw std::runtime_error("无法读取 VIO 配置：" + options.vio);
    std::string chain;
    estimator["relative_config_imucam"] >> chain;
    cv::FileStorage file((std::filesystem::path(options.vio) / chain).string(), cv::FileStorage::READ);
    if (!file.isOpened()) throw std::runtime_error("无法读取 VIO 相机标定");
    const auto camera = file["cam0"];
    std::vector<double> intrinsics, distortion;
    std::vector<int> resolution;
    camera["intrinsics"] >> intrinsics;
    camera["distortion_coeffs"] >> distortion;
    camera["resolution"] >> resolution;
    std::string model, camera_model;
    camera["camera_model"] >> camera_model;
    camera["distortion_model"] >> model;
    if (intrinsics.size() != 4 || resolution.size() != 2 || camera_model != "pinhole" || model != "radtan" || distortion.size() != 4)
        throw std::runtime_error("主流程需要 pinhole/radtan 单目标定");
    result.camera.width = resolution[0];
    result.camera.height = resolution[1];
    // 主流程只需要彩色，避免深度流限制彩色吞吐；VIO 沿用其单目初始化。
    result.camera.use_depth = false;
    auto& cam = result.params.loop.cam;
    cam.fx = intrinsics[0]; cam.fy = intrinsics[1];
    cam.cx = intrinsics[2]; cam.cy = intrinsics[3];
    const auto transform = camera["T_imu_cam"];
    if (transform.size() != 4) throw std::runtime_error("缺少 T_imu_cam 外参");
    for (int row = 0; row < 3; ++row) {
        if (transform[row].size() != 4) throw std::runtime_error("T_imu_cam 格式错误");
        for (int col = 0; col < 3; ++col) cam.R_body_cam[row * 3 + col] = static_cast<double>(transform[row][col]);
    }
    if (!options.guidance.empty()) {
        const auto loaded = app::guidance_load_yaml_file(findConfigFile(options.guidance), &result.params);
        if (!loaded.ok) throw std::runtime_error(loaded.error);
    }
    if (result.params.guidance.law != GuidanceLaw::PPN ||
        result.params.guidance.los_rate_source != LosRateSource::FROM_LOS_DIFFERENCE ||
        result.params.guidance.freeze_criterion != FreezeCriterion::BY_TGO ||
        result.params.guidance.t_go_freeze != 0.0)
        throw std::runtime_error("当前像素检测无测距：主流程需要 PPN、FROM_LOS_DIFFERENCE、BY_TGO、t_go_freeze=0");
    const auto valid = app::guidance_validate_params(result.params);
    if (!valid.ok) throw std::runtime_error(valid.error);
    result.K = (cv::Mat_<double>(3, 3) << cam.fx, 0, cam.cx, 0, cam.fy, cam.cy, 0, 0, 1);
    result.distortion = cv::Mat(distortion).clone();
    return result;
}

class InsSource final : public flight::GuidanceInsSource {
public:
    InsSource(Communicator& comm, double skew) : comm_(comm), skew_(skew) {}
    bool poll(flight::GuidanceInsSample& out) override {
        if (!comm_.isConnected()) return false;
        Attitude attitude;
        LocalPosition position;
        comm_.getAttitude(attitude);
        comm_.getLocalPosition(position);
        flight::GuidanceInsSample sample;
        if (!app::makeGuidanceIns(attitude, position, skew_, sample) || sample.boot_us <= last_) return false;
        last_ = sample.boot_us;
        out = sample;
        return true;
    }
    bool alive() const override { return comm_.isConnected(); }
private:
    Communicator& comm_;
    double skew_;
    uint64_t last_ = 0;
};

class CommandSink final : public flight::GuidanceCommandSink {
public:
    CommandSink(Communicator& comm, bool no_send, std::function<bool()> current)
        : comm_(comm), no_send_(no_send), current_(std::move(current)) {}
    bool send(const flight::GuidanceOutput& out) override {
        return no_send_ || !current_() || comm_.pushGuidanceCmd(app::makeGuidanceCmd(out));
    }
    bool alive() const override { return no_send_ || comm_.isConnected(); }
private:
    Communicator& comm_;
    bool no_send_;
    std::function<bool()> current_;
};
#endif

#if defined(PURE_APP_HAS_VIO) && defined(PURE_APP_HAS_DETECTION)
class TargetSource final : public flight::GuidanceTargetSource {
public:
    TargetSource(detection::Detector& detector, FcuClock& clock, const Calibration& calibration,
                 std::atomic<int64_t>& epoch) : detector_(detector), clock_(clock), calibration_(calibration), epoch_(epoch) {}
    bool poll(flight::GuidanceTargetSample& out) override {
        detection::FrameResult result;
        uint64_t boot_us = 0;
        if (!detector_.poll(result) || result.host_monotonic_ns < epoch_.load() ||
            !clock_.hostToFcuUs(result.host_monotonic_ns, boot_us)) return false;
        detection::to_target_sample(result, boot_us, out);
        if (out.present) {
            std::vector<cv::Point2d> pixels{{out.u_px, out.v_px}}, corrected;
            cv::undistortPoints(pixels, corrected, calibration_.K, calibration_.distortion,
                                cv::noArray(), calibration_.K);
            out.u_px = corrected[0].x; out.v_px = corrected[0].y;
        }
        return true;
    }
    bool alive() const override { return detector_.alive(); }
private:
    detection::Detector& detector_;
    FcuClock& clock_;
    const Calibration& calibration_;
    std::atomic<int64_t>& epoch_;
};

int run(const Options& options) {
    const Calibration calibration = loadCalibration(options);
    CommConfig comm_config;
    if (!loadCommConfig(findConfigFile(options.driver), comm_config))
        throw std::runtime_error("无法读取通信配置：" + options.driver);
    // VIO 使用逐帧 IMU，避免默认的低频示例配置。
    comm_config.highres_imu_interval_us = 5000;
    Communicator comm;
    comm.setSystemId(2);
    comm.setImuQueueCapacity(2000);
    detection::Detector detector;
    detection::DetectorConfig detector_config;
    detector_config.external_camera = true;
    detector_config.model_path = options.model;
    if (detector_config.model_path.empty()) {
#ifdef PURE_DETECTION_BACKEND_CPU
        detector_config.model_path = findConfigFile("assets/models/yolo26n.onnx");
#else
        detector_config.model_path = findConfigFile("assets/models/yolo26n.nb");
#endif
    }
    VioEstimator::Options vio_options;
    vio_options.config_dir = options.vio;
    VioEstimator estimator(vio_options);
    if (!estimator.ok()) throw std::runtime_error(estimator.error());
    std::string error;
    if (!detector.start(detector_config, error)) throw std::runtime_error("检测启动失败：" + error);
    if (!comm.start(comm_config)) throw std::runtime_error("打开串口失败");

    app::BoundedQueue<Frame> frames(4);
    FcuClock clock;
    std::atomic<int64_t> epoch_ns{nowNs()}, last_frame_ns{nowNs()}, last_imu_ns{nowNs()};
    OrbbecCamera camera;
    uint64_t frame_seq = 0;
    if (!camera.start(calibration.camera,
            [&](const cv::Mat& gray, const cv::Mat& depth, int64_t host_ns, int64_t, int64_t) {
                if (gray.cols != calibration.camera.width || gray.rows != calibration.camera.height)
                    throw std::runtime_error("相机分辨率与标定不一致");
                frames.push(Frame{gray, depth, host_ns, comm.reconnectCount()});
                last_frame_ns = host_ns;
            }, &error,
            [&](const cv::Mat& bgr, int64_t host_ns) {
                const auto unix_us = std::chrono::duration_cast<std::chrono::microseconds>(
                    std::chrono::system_clock::now().time_since_epoch()).count();
                detector.submitFrame(bgr, unix_us, 0, ++frame_seq, host_ns);
            })) throw std::runtime_error("相机启动失败：" + error);

    // Workers 最后构造，异常展开时先 join，再析构采集器、队列和数据源。
    Workers workers;
    std::atomic<uint64_t> vio_images{0}, vio_poses{0}, guidance_ticks{0};
    flight::GuidanceRingLog blackbox;
    workers.launch("VIO", [&] {
        std::optional<Frame> pending;
        uint64_t last_imu = 0, last_image = 0, last_pose = 0, last_raw_imu = 0, session = comm.reconnectCount();
        uint64_t dropped = comm.droppedImu();
        int warmup = 150;
        ImuTimestampFilter filter;
        auto reset = [&] {
            epoch_ns = nowNs();
            clock.reset(); filter.reset(); frames.clear(); pending.reset();
            last_imu = last_image = last_pose = last_raw_imu = 0; warmup = 150;
            estimator.resetForNewTimebase("机载共享链路时间基变化");
        };
        while (!workers.stop.load()) {
            if (session != comm.reconnectCount() || dropped != comm.droppedImu()) {
                session = comm.reconnectCount(); dropped = comm.droppedImu(); reset();
            }
            estimator.setZuptEnabled(!comm.isArmed());
            Communicator::TimedImu item;
            // 每轮有界消费，让退出、图像处理不被高频 IMU 饿死。
            for (int n = 0; n < 400 && comm.popImu(item); ++n) {
                if (item.session != session || !comm.isConnected()) continue;
                last_imu_ns = item.host_ns;
                if (warmup > 0) { --warmup; continue; }
                if (last_raw_imu && item.imu.timestamp_us > last_raw_imu + 100000) {
                    reset(); break;
                }
                const auto state = clock.addSample(item.host_ns, item.imu.timestamp_us);
                if (state == FcuClock::AddResult::kTimebaseReset) { reset(); break; }
                if (state == FcuClock::AddResult::kOutOfOrder) continue;
                last_raw_imu = item.imu.timestamp_us;
                ImuSample imu;
                if (!filter.filter(item.imu.timestamp_us, 5000.0, imu.timestamp_us)) continue;
                imu.gx=item.imu.gx; imu.gy=item.imu.gy; imu.gz=item.imu.gz;
                imu.ax=item.imu.ax; imu.ay=item.imu.ay; imu.az=item.imu.az;
                estimator.feedImu(imu);
                last_imu = imu.timestamp_us;
            }
            if (!pending) { Frame frame; if (frames.pop(frame)) pending = std::move(frame); }
            if (pending) {
                uint64_t timestamp = 0;
                if (pending->session != session || pending->host_ns < epoch_ns.load() ||
                    nowNs() - pending->host_ns > 500000000LL) { pending.reset(); continue; }
                if (clock.hostToFcuUs(pending->host_ns, timestamp) &&
                    imuCoversImage(last_imu, timestamp, estimator.cameraTimeOffsetSec())) {
                    if (timestamp > last_image) {
                        estimator.feedImage(pending->gray, timestamp);
                        last_image = timestamp; ++vio_images;
                        VisionPose pose;
                        if (estimator.publishable() && estimator.getLatestPose(pose) &&
                            comm.isConnected() && comm.reconnectCount() == session && pose.timestamp_us > last_pose) {
                            pose.quality = estimator.healthState() == VioHealthState::OK ? 90 : 50;
                            pose.reset_counter = odometryResetCounter(estimator.rebuildCount());
                            pose.pos_sigma_m = pose.quality == 90 ? 0.05f : 0.2f;
                            pose.vel_sigma_mps = pose.quality == 90 ? 0.10f : 0.5f;
                            if (options.no_send || comm.pushVisionPose(pose)) { last_pose = pose.timestamp_us; ++vio_poses; }
                        }
                    }
                    pending.reset();
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    });
    workers.launch("制导", [&] {
        uint64_t session = comm.reconnectCount();
        int64_t epoch = epoch_ns.load();
        auto next = Clock::now();
        std::unique_ptr<InsSource> ins;
        TargetSource targets(detector, clock, calibration, epoch_ns);
        CommandSink sink(comm, options.no_send, [&] {
            return comm.reconnectCount() == session && epoch_ns.load() == epoch && clock.valid();
        });
        std::unique_ptr<flight::GuidanceLoop> loop;
        struct SaveBlackbox {
            std::unique_ptr<flight::GuidanceLoop>& loop;
            flight::GuidanceRingLog& log;
            ~SaveBlackbox() { if (loop) log = loop->blackbox(); }
        } save{loop, blackbox};
        while (!workers.stop.load()) {
            if (!loop || session != comm.reconnectCount() || epoch != epoch_ns.load()) {
                session = comm.reconnectCount(); epoch = epoch_ns.load();
                loop.reset();
                ins = std::make_unique<InsSource>(comm, calibration.params.loop.max_sample_age_s);
                loop = std::make_unique<flight::GuidanceLoop>(calibration.params.guidance,
                    calibration.params.loop, *ins, targets, sink);
                if (!loop->init()) throw std::runtime_error("制导初始化失败");
            }
            if (comm.isConnected() && clock.valid()) {
                const auto status = loop->run_once();
                ++guidance_ticks;
                if (status == flight::GuidanceLoopStatus::kSensorTimeout || status == flight::GuidanceLoopStatus::kSinkError)
                    throw std::runtime_error("制导数据超时或指令发送失败");
            }
            next += std::chrono::microseconds(1000000 / calibration.params.loop.loop_hz);
            std::this_thread::sleep_until(next);
            if (next < Clock::now()) next = Clock::now();
        }
    });

    std::printf("机载线程已启动：通信 + 相机采集 + 检测推理 + VIO + 制导%s\n",
                options.no_send ? "（不发送位姿/制导指令）" : "");
    const auto started = Clock::now();
    auto report = started;
    auto last_detection = started;
    uint64_t processed = 0;
    while (!interrupted && !workers.stop.load()) {
        const auto now = Clock::now();
        const auto count = detector.stats().process_frames;
        if (count != processed) { processed = count; last_detection = now; }
        if (!camera.running() || !detector.alive() || now - last_detection > std::chrono::seconds(10) ||
            (now - started > std::chrono::seconds(10) && !comm.isConnected()) ||
            nowNs() - last_frame_ns.load() > 3000000000LL ||
            (now - started > std::chrono::seconds(10) && nowNs() - last_imu_ns.load() > 3000000000LL)) {
            std::fprintf(stderr, "机载数据链路停止或超时，正在停止各线程\n");
            workers.failed = true;
            break;
        }
        if (now - report >= std::chrono::seconds(1)) {
            std::printf("[状态] 飞控=%s VIO图像=%llu 位姿=%llu 检测=%llu 制导拍=%llu 丢图像=%zu 丢IMU=%llu\n",
                comm.isConnected() ? "在线" : "等待心跳",
                static_cast<unsigned long long>(vio_images.load()), static_cast<unsigned long long>(vio_poses.load()),
                static_cast<unsigned long long>(detector.stats().process_frames),
                static_cast<unsigned long long>(guidance_ticks.load()), frames.dropped(),
                static_cast<unsigned long long>(comm.droppedImu()));
            report = now;
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    workers.join();
    camera.stop();       // 先停止投递，再释放推理后端和串口。
    detector.stop();
    comm.stop();
    if (!options.blackbox.empty() && blackbox.dump_csv(options.blackbox.c_str(), 0) < 0)
        throw std::runtime_error("黑匣子写入失败：" + options.blackbox);
    return workers.failed ? 1 : 0;
}
#endif
}  // namespace

int main(int argc, char** argv) {
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--help" || std::string(argv[i]) == "-h") { usage(argv[0]); return 0; }
    }
    try {
        const auto options = parseOptions(argc, argv);
#if defined(PURE_APP_HAS_VIO) && defined(PURE_APP_HAS_DETECTION)
        std::signal(SIGINT, onSignal);
        std::signal(SIGTERM, onSignal);
        return run(options);
#else
        std::fprintf(stderr, "机载主流程需要同时构建 VIO 实时模块和检测模块；请检查 CMake 依赖。\n");
        return 1;
#endif
    } catch (const std::invalid_argument& error) {
        std::fprintf(stderr, "%s\n", error.what()); return 2;
    } catch (const std::exception& error) {
        std::fprintf(stderr, "机载启动/运行失败：%s\n", error.what()); return 1;
    }
}
