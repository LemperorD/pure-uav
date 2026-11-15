// 实时 VIO：camera_server 共享内存图像 + 飞控 MAVLink IMU → VisionPose → 飞控 ODOMETRY。
//
// 线程模型：
//   - MAVLink 接收线程（MavlinkLink 内部）：收 HIGHRES_IMU / HEARTBEAT
//   - 共享内存读取线程：消费 camera_server 的 VIO 通道
//   - VIO 线程（本文件）：按时间顺序喂 IMU/图像，输出 VisionPose 并发送
//
// 时间戳：
//   - IMU：飞控 HIGHRES_IMU.time_usec 去抖后，由 FcuClock 换算为主机 Unix 微秒
//   - 相机：保留服务端采集时间，再经飞控映射换算到同一 Unix 时间基
//   - 输出：t_imu = t_cam + timeshift_cam_imu（由 OpenVINS 配置给出）
//
// 参数与帮助见 live_options.hpp；历史调试记录见 ../LIVE_DEBUG_NOTES.md。
// 保持本文件的处理顺序：采集入队 → 喂 IMU → 等待图像覆盖 → 估计/记录/发送 → 诊断/显示。

#include <Eigen/Geometry>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstdlib>
#include <deque>
#include <exception>
#include <filesystem>
#include <mutex>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgcodecs.hpp>
#include <optional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include "fcu_clock.hpp"
#include "live_options.hpp"
#include "live_timing.hpp"
#include "live_view.hpp"
#include "mavlink_link.hpp"
#include "pure_uav/pose_logger.hpp"
#include "pure_uav/vio_estimator.hpp"
#include "pure_uav/vision_pose.hpp"
#include "shm_camera_source.hpp"

namespace {

std::atomic<bool> g_stop{false};

void onSigint(int /*unused*/) {
    g_stop.store(true);
}

int64_t steadyNowNs() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

struct FrameItem {
    cv::Mat gray;
    int64_t host_ns = 0;
    uint64_t mapped_fcu_us = 0;  // 等待 IMU 时固定本帧时间，不重复加入相机平滑窗口
    int64_t system_us = 0;       // SDK systemTimeStampUs，保持既有 --cam-ts device 平滑时基
    int64_t device_us = 0;       // 彩色设备时间戳，用于保留深度与彩色的时间差。
    int64_t depth_dev_us = 0;    // 深度设备时间戳，与彩色使用同一设备时钟。
    cv::Mat depth;               // 原生深度相机坐标下的毫米深度（CV_16UC1），未启用时为空。
};

// 灰度、深度和估计器实际使用的 Unix 时间戳一起入队，保持录制配对。
struct RecordedFrame {
    cv::Mat gray;
    cv::Mat depth;
    uint64_t timestamp_us = 0;
};

Eigen::Matrix3d rpyDegToMatrix(double roll_deg, double pitch_deg, double yaw_deg) {
    // ZYX 欧拉角：R = Rz(yaw) * Ry(pitch) * Rx(roll)
    const double r = roll_deg * M_PI / 180.0;
    const double p = pitch_deg * M_PI / 180.0;
    const double y = yaw_deg * M_PI / 180.0;
    return (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) * Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX())).toRotationMatrix();
}

void dumpFcuParams(pure_uav::MavlinkLink& link) {
    // 关心的是"固件对 IMU 做了什么"：低通截止、陷波、采样率、以及日志带宽
    const char* names[] = {
        "INS_ACCEL_FILTER", "INS_GYRO_FILTER", "INS_FAST_SAMPLE", "INS_LOG_BAT_MASK", "INS_LOG_BAT_OPT", "INS_HNTCH_ENABLE", "INS_HNTCH_FREQ", "INS_HNTCH_BW",    "INS_HNTC2_ENABLE", "SCHED_LOOP_RATE",
        "LOG_BITMASK",      "LOG_DARM_MASK",   "EK3_ENABLE",      "AHRS_EKF_TYPE",    "INS_USE",         "LOG_FILE_DSRMROT", "LOG_DISARMED",   "LOG_MAV_BUFFERS", "LOG_REPLAY",       "INS_GYRO_RATE",
    };
    const size_t want = sizeof(names) / sizeof(names[0]);
    for (int round = 0; round < 3; ++round) {
        for (size_t i = 0; i < want; ++i) {
            link.requestParamByName(names[i]);
            std::this_thread::sleep_for(std::chrono::milliseconds(15));
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(400));
    }
    const auto got = link.receivedParams();
    std::printf("\n===== 飞控参数（只读，共收到 %zu 个应答）=====\n", got.size());
    for (const auto& kv : got) {
        std::printf("  %-20s = %.6g\n", kv.first.c_str(), static_cast<double>(kv.second));
    }
    std::printf("\n没列出来的说明飞控没这个参数名（固件版本不同）。\n");
}

}  // namespace

int main(int argc, char** argv) {
    pure_uav::live::Options opt;
    const auto parsed = pure_uav::live::parseOptions(argc, argv, opt);
    if (parsed != pure_uav::live::ParseResult::kRun) {
        return parsed == pure_uav::live::ParseResult::kHelp ? 0 : 2;
    }
    std::signal(SIGINT, onSigint);
    std::signal(SIGTERM, onSigint);

    // 启动顺序：估计器 → 飞控链路 → 相机输入 → 输出文件 → 工作线程。
    // 1. VIO 估计器
    pure_uav::VioEstimator::Options vio_opt;
    // 失效保护：判定为 LOST 就停发位姿，恢复时（可选）重建并重锚定
    vio_opt.health.min_features = opt.health_min_features;
    vio_opt.health.lost_after_s = opt.health_lost_after;
    vio_opt.health.reinit_on_recover = opt.reinit_on_recover;
    vio_opt.config_dir = opt.config_dir;
    vio_opt.verbose = !opt.quiet;
    vio_opt.estimate_camera_timeoffset = opt.online_timeoffset;
    vio_opt.init_threads = opt.init_threads;
    vio_opt.use_depth_init = opt.use_depth_init;
    vio_opt.init_vel_zero = opt.init_vel_zero;
    vio_opt.static_init = opt.static_init;
    vio_opt.R_imu_to_body = rpyDegToMatrix(opt.itb_rpy[0], opt.itb_rpy[1], opt.itb_rpy[2]);
    vio_opt.t_imu_body = Eigen::Vector3d(opt.itb_t[0], opt.itb_t[1], opt.itb_t[2]);
    pure_uav::VioEstimator estimator(vio_opt);
    if (!estimator.ok()) {
        std::fprintf(stderr, "VioEstimator 初始化失败: %s\n", estimator.error().c_str());
        return 1;
    }
    if (opt.static_init) {
        // OpenVINS 的 StaticInitializer 判据（wait_for_jerk=true 时）：
        //   旧窗 [-2.0,-1.0]s 的加速度方差必须 < init_imu_thresh(1.5)  ← 那一段要真静止；
        //   新窗 [-1.0, 0]s   的加速度方差必须 > init_imu_thresh(1.5)  ← 那一段要真加速。
        // 姿态（重力方向）取自【旧窗】的加速度均值，所以旧窗干净与否直接决定倾角误差。
        std::printf(
            "[VIO] 静态初始化（先静后猛动）：\n"
            "      1) 相机完全静止 ≥2 秒（别用手托着，放桌上更稳）；\n"
            "      2) 然后【果断】动起来 —— 前 1 秒内加速度要有明显变化（方差 >1.5 m/s²），\n"
            "         慢慢晃不算，那样会一直不触发；\n"
            "      3) 触发后正常走。触发前不输出位姿，窗口显示 NOT INITIALIZED。\n");
        std::fflush(stdout);
    }

    // 2. 飞控 MAVLink
    pure_uav::MavlinkLink::Config link_cfg;
    link_cfg.device = opt.serial;
    link_cfg.baudrate = opt.baud;
    link_cfg.highres_imu_interval_us = static_cast<int>((1e6 / opt.imu_req_hz) + 0.5);
    pure_uav::MavlinkLink link;
    std::string err;
    if (!link.start(link_cfg, &err)) {
        std::fprintf(stderr, "MAVLink 链路启动失败: %s\n", err.c_str());
        return 1;
    }
    std::printf("等待飞控心跳（%s @ %d）…\n", opt.serial.c_str(), opt.baud);
    for (int i = 0; i < 50 && !link.connected() && !g_stop.load(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!link.connected()) {
        std::fprintf(stderr, "[WARN] 5 秒内没收到飞控心跳，继续等待…\n");
        if (opt.dump_params) {
            link.stop();
            return 1;
        }
    } else {
        std::printf("飞控已连接 sysid=%u\n", static_cast<unsigned>(link.fcuSysId()));
    }

    // --dump-params：只读查询，然后退出（不打开相机）
    if (opt.dump_params) {
        dumpFcuParams(link);
        link.stop();
        return 0;
    }

    // 3. camera_server VIO 通道、相机诊断和有界帧队列
    std::mutex frame_mtx;
    std::deque<FrameItem> frame_q;
    constexpr size_t kMaxFrameQueue = 5;
    uint64_t dropped_frames = 0;

    pure_uav::ShmCameraSource camera;
    pure_uav::ShmCameraSource::Config cam_cfg;
    cam_cfg.channel = opt.shm_vio;
    cam_cfg.timeout_ms = opt.shm_timeout_ms;
    cam_cfg.config_dir = opt.config_dir;
    cam_cfg.width = opt.width;
    cam_cfg.height = opt.height;
    cam_cfg.fps = opt.fps;
    cam_cfg.use_depth = opt.enable_depth;

    // 图像诊断：每秒打一行亮度和 FAST 特征数。
    // VIO 出问题时，"场景太暗/没纹理" 和 "滤波器不更新" 从日志上分不出来，这两个数能分开。
    //   mean < 25 或 FAST < 150 → 先解决光照/增益，别调 VIO 参数。
    struct CamDiag {
        int64_t frames = 0;
        double mean_sum = 0.0;
        double fast_sum = 0.0;
        int64_t fast_samples = 0;
        int64_t last_log_ns = 0;
    };
    CamDiag diag;
    // 给查看器读的最新亮度/特征数（相机线程写、VIO 线程读，用 atomic 免加锁）
    std::atomic<float> live_mean{-1.0F};
    std::atomic<float> live_fast{-1.0F};

    std::deque<double> lat_host_ms;  // 服务端采集帧间隔（毫秒），相机线程维护。
    int depth_w = 0;
    int depth_h = 0;
    double depth_valid_ratio = 0.0;
    double depth_median_mm = 0.0;
    // 深度年龄：彩色与深度的设备时间差，用于诊断深度是否过期。
    std::deque<double> depth_age_ms;
    long long n_frame_with_depth = 0;
    long long n_frame_total = 0;
    int64_t prev_host_ns = 0;
    int64_t prev_device_us = 0;
    auto onCameraFrame = [&](const cv::Mat& gray, const cv::Mat& depth_mm, int64_t host_ns, int64_t device_us, int64_t depth_dev_us, int64_t system_us) {
        // --- 图像诊断（只在相机线程调用，不需要加锁）---
        {
            static cv::Ptr<cv::FastFeatureDetector> fast = cv::FastFeatureDetector::create(20, true);
            diag.frames++;
            diag.mean_sum += cv::mean(gray)[0];
            if (!depth_mm.empty()) {
                ++n_frame_with_depth;
                // 深度年龄 = 彩色设备时刻 − 深度设备时刻（同一台设备的时钟，可直接相减）
                if (depth_dev_us > 0 && device_us > 0) {
                    depth_age_ms.push_back(std::abs(static_cast<double>(device_us - depth_dev_us)) * 1e-3);
                    if (depth_age_ms.size() > 400) {
                        depth_age_ms.pop_front();
                    }
                }
            }
            ++n_frame_total;
            // 深度尺寸、有效像素比例和中位深度，用于检查深度流是否正常。
            if (!depth_mm.empty()) {
                depth_w = depth_mm.cols;
                depth_h = depth_mm.rows;
                if (diag.frames % 10 == 1) {
                    std::vector<unsigned short> vals;
                    vals.reserve(4096);
                    for (int r = 0; r < depth_mm.rows; r += 8) {
                        const auto* row = depth_mm.ptr<unsigned short>(r);
                        for (int c = 0; c < depth_mm.cols; c += 8) {
                            const unsigned short v = row[c];
                            if (v != 0 && v != 0xFFFF) {
                                vals.push_back(v);
                            }
                        }
                    }
                    const auto total = static_cast<size_t>(((depth_mm.rows / 8) + 1) * ((depth_mm.cols / 8) + 1));
                    depth_valid_ratio = total ? static_cast<double>(vals.size()) / total : 0.0;
                    if (!vals.empty()) {
                        std::nth_element(vals.begin(), vals.begin() + vals.size() / 2, vals.end());
                        depth_median_mm = vals[vals.size() / 2];
                    }
                }
            } else {
                depth_w = depth_h = 0;
            }
            // 保持原有采样条件：设备时间戳有效时才统计主机帧间隔。
            if (device_us > 0) {
                if (prev_device_us > 0) {
                    const double d_host_us = static_cast<double>(host_ns - prev_host_ns) * 1e-3;
                    lat_host_ms.push_back(d_host_us * 1e-3);
                    if (lat_host_ms.size() > 600) {
                        lat_host_ms.pop_front();
                    }
                }
                prev_host_ns = host_ns;
                prev_device_us = device_us;
            }
            if (diag.frames % 5 == 1) {  // 每 5 帧算一次特征数，省 CPU
                std::vector<cv::KeyPoint> kp;
                fast->detect(gray, kp);
                diag.fast_sum += static_cast<double>(kp.size());
                diag.fast_samples++;
            }
            if (diag.last_log_ns == 0) {
                diag.last_log_ns = host_ns;
            } else if (host_ns - diag.last_log_ns >= 1000000000LL) {
                const double mean = diag.mean_sum / (diag.frames > 0 ? diag.frames : 1);
                const double fast_avg = diag.fast_sum / (diag.fast_samples > 0 ? diag.fast_samples : 1);
                // 裁掉帧间隔离群点，避免停顿掩盖正常采集抖动。
                double host_p50 = 0.0;
                double host_p95 = 0.0;
                double host_robust_std = 0.0;
                if (lat_host_ms.size() > 20) {
                    std::vector<double> v(lat_host_ms.begin(), lat_host_ms.end());
                    std::sort(v.begin(), v.end());
                    // 只取 0.5% ~ 99.5% 分位之间的样本（剔除停顿/时钟跳变）
                    const size_t lo = v.size() * 5 / 1000;
                    const size_t hi = v.size() * 995 / 1000;
                    std::vector<double> t(v.begin() + lo, v.begin() + std::max(hi, lo + 1));
                    host_p50 = t[t.size() / 2];
                    host_p95 = t[t.size() * 95 / 100];
                    // 稳健 σ ≈ (p84 - p16)/2
                    host_robust_std = 0.5 * (t[t.size() * 84 / 100] - t[t.size() * 16 / 100]);
                }
                std::printf(
                    "[cam] %lld 帧/秒  mean=%.1f  FAST≈%.0f  深度 %dx%d 有效%.0f%% 中位%.2fm  "
                    "\u5e27\u95f4\u9694 p50=%.2f p95=%.2f ms  \u65f6\u95f4\u6233\u6296\u52a8(\u7a33\u5065\u03c3)=%.2f ms\n",
                    (long long)diag.frames, mean, fast_avg, depth_w, depth_h, 100.0 * depth_valid_ratio, depth_median_mm * 1e-3, host_p50, host_p95, host_robust_std);
                if (!depth_age_ms.empty()) {
                    std::vector<double> v(depth_age_ms.begin(), depth_age_ms.end());
                    std::sort(v.begin(), v.end());
                    std::printf("[cam]   深度年龄 p50=%.1f p95=%.1f ms｜带深度的帧 %lld/%lld (%.0f%%)\n", v[v.size() / 2], v[v.size() * 95 / 100], n_frame_with_depth, n_frame_total, n_frame_total ? 100.0 * n_frame_with_depth / n_frame_total : 0.0);
                    depth_age_ms.clear();
                    n_frame_with_depth = n_frame_total = 0;
                }
                std::fflush(stdout);
                live_mean.store(static_cast<float>(mean));
                live_fast.store(static_cast<float>(fast_avg));
                const int64_t keep = host_ns;
                diag = CamDiag{};
                diag.last_log_ns = keep;
            }
        }

        FrameItem item;
        item.gray = gray.clone();  // 相机线程的 Mat 生命周期不可控，克隆一份
        if (!depth_mm.empty()) {
            item.depth = depth_mm.clone();
        }
        item.host_ns = host_ns;
        item.device_us = device_us;
        item.system_us = system_us;
        item.depth_dev_us = depth_dev_us;
        {
            std::scoped_lock lk(frame_mtx);
            if (frame_q.size() >= kMaxFrameQueue) {
                frame_q.pop_front();
                dropped_frames++;
            }
            frame_q.push_back(std::move(item));
        }
    };

    if (!camera.start(cam_cfg, &err)) {
        std::fprintf(stderr, "连接 camera_server 失败: %s\n", err.c_str());
        link.stop();
        return 1;
    }
    const auto cam_depth = camera.depthInfo();
    const double depth_t_m[3] = {cam_depth.t_c2d_m[0], cam_depth.t_c2d_m[1], cam_depth.t_c2d_m[2]};
    if (opt.use_depth_init && !cam_depth.valid()) {
        std::fprintf(stderr, "[WARN] 请求了深度初始化，但深度内参不可用，本轮不会注入深度\n");
    }

    // 4. 位姿输出与可选的原始 IMU 日志
    pure_uav::PoseLogger logger;
    pure_uav::PoseLogger safe_logger;
    if (!opt.safe_tum_out.empty() && !safe_logger.open("", opt.safe_tum_out)) {
        std::fprintf(stderr, "[WARN] 打不开发布轨迹 %s，跳过\n", opt.safe_tum_out.c_str());
    }
    if (!logger.open(opt.csv_out, opt.tum_out)) {
        std::fprintf(stderr, "无法打开输出文件\n");
        camera.stop();
        link.stop();
        return 1;
    }

    // 原始 IMU 日志仅用于事后核对，不参与估计。
    std::FILE* imu_log = nullptr;
    if (!opt.imu_log_path.empty()) {
        imu_log = std::fopen(opt.imu_log_path.c_str(), "w");
        if (imu_log != nullptr) {
            std::fprintf(imu_log, "# t_us,ax,ay,az,gx,gy,gz\n");
        } else {
            std::fprintf(stderr, "[WARN] 打不开 IMU 日志 %s，跳过\n", opt.imu_log_path.c_str());
        }
    }

    // 线程共享的运行计数：VIO 线程更新，主线程 join 后输出汇总。
    std::atomic<int64_t> stats_poses{0};
    std::atomic<int64_t> stats_blocked{0};
    std::atomic<int64_t> stats_timebase_resets{0};     // 飞控重启/串口重连导致的重新初始化次数
    std::atomic<long long> stats_imu_out_of_order{0};  // 重复/乱序被丢弃的 IMU 样本数
    std::atomic<double> cur_speed_mps{0.0};
    // 以下统计仅由 VIO 线程维护，不参与健康门控。
    std::deque<double> pose_age_ms;     // 服务端采集到位姿产出的延迟（毫秒）。
    std::deque<double> vio_vs_ekf3_ms;  // VIO 与飞控 EKF3 的速率差（mm/s）。
    // 深度一致性近 2 s 均值（见状态行注释）
    long long last_deperr_n = 0;
    double last_deperr_sum = 0.0;
    double last_deperr_recent = -1.0;
    bool warned_deperr = false;
    bool was_blocked = false;
    std::atomic<int64_t> stats_sent{0};
    std::atomic<int64_t> stats_drop_no_clock{0};

    // 5. 可选离线数据集录制；图像由独立写盘线程消费，IMU 在喂入时记录。
    std::mutex rec_mtx;
    std::deque<RecordedFrame> rec_q;
    std::atomic<bool> rec_stop{false};
    std::atomic<long long> rec_frames{0};
    std::FILE* rec_images_csv = nullptr;
    std::FILE* rec_imu_csv = nullptr;
    std::thread rec_thread;
    if (!opt.rec_dir.empty()) {
        // 已存在 images.csv 就拒绝录制，保护历史数据。
        if (std::filesystem::exists(opt.rec_dir + "/images.csv")) {
            std::fprintf(stderr,
                         "[rec] 目标目录里已经有录制：%s\n"
                         "      不覆盖。换一个 --record 名字再跑（例如 %s_2）。\n",
                         opt.rec_dir.c_str(), opt.rec_dir.c_str());
            return 2;
        }
        std::error_code ec;
        std::filesystem::create_directories(opt.rec_dir + "/cam0", ec);
        std::filesystem::create_directories(opt.rec_dir + "/depth", ec);
        rec_images_csv = std::fopen((opt.rec_dir + "/images.csv").c_str(), "w");
        rec_imu_csv = std::fopen((opt.rec_dir + "/imu.csv").c_str(), "w");
        if ((rec_images_csv == nullptr) || (rec_imu_csv == nullptr)) {
            std::fprintf(stderr, "[WARN] 录制文件打不开（%s），录制关闭\n", opt.rec_dir.c_str());
            if (rec_images_csv != nullptr) {
                std::fclose(rec_images_csv);
                rec_images_csv = nullptr;
            }
            if (rec_imu_csv != nullptr) {
                std::fclose(rec_imu_csv);
                rec_imu_csv = nullptr;
            }
            opt.rec_dir.clear();
        } else {
            std::fprintf(rec_images_csv, "timestamp_us,filename\n");
            std::fprintf(rec_imu_csv, "timestamp_us,gx,gy,gz,ax,ay,az\n");
            std::fflush(rec_images_csv);
            std::printf("[rec] 录制到 %s（cam0/*.png + depth/*.png + images.csv + imu.csv）\n", opt.rec_dir.c_str());
            // 深度元数据：回放时要用它把彩色像素投到深度图上取深度
            {
                const auto di = camera.depthInfo();
                std::FILE* dm = std::fopen((opt.rec_dir + "/depth_meta.txt").c_str(), "w");
                if (dm != nullptr) {
                    if (di.valid()) {
                        std::fprintf(dm, "valid=1\n");
                        std::fprintf(dm, "fx=%.6f\nfy=%.6f\ncx=%.6f\ncy=%.6f\n", di.fx, di.fy, di.cx, di.cy);
                        std::fprintf(dm, "width=%d\nheight=%d\n", di.width, di.height);
                        // 同时保存彩色到深度的旋转和平移，供离线投影使用。
                        std::fprintf(dm, "tx_mm=%.4f\nty_mm=%.4f\ntz_mm=%.4f\n", di.t_c2d_m[0] * 1e3, di.t_c2d_m[1] * 1e3, di.t_c2d_m[2] * 1e3);
                        std::fprintf(dm, "R_c2d=");
                        for (int k = 0; k < 9; ++k) {
                            std::fprintf(dm, "%.9f%s", di.R_c2d[k], k == 8 ? "\n" : ",");
                        }
                        std::printf("[rec] 深度: %dx%d fx=%.2f cx=%.2f cy=%.2f（写入 depth_meta.txt）\n", di.width, di.height, di.fx, di.cx, di.cy);
                    } else {
                        std::fprintf(dm, "valid=0\n");
                        std::printf("[rec] 深度不可用（本轮只录了彩色）\n");
                    }
                    std::fclose(dm);
                }
            }
            std::fflush(stdout);
            rec_thread = std::thread([&] {
                const std::vector<int> png_params{cv::IMWRITE_PNG_COMPRESSION, 1};
                while (true) {
                    RecordedFrame item_frame;
                    bool has = false;
                    {
                        std::scoped_lock lk(rec_mtx);
                        if (!rec_q.empty()) {
                            item_frame = std::move(rec_q.front());
                            rec_q.pop_front();
                            has = true;
                        }
                    }
                    if (has) {
                        const cv::Mat& gray_item = item_frame.gray;
                        const cv::Mat& depth_item = item_frame.depth;
                        const uint64_t ts = item_frame.timestamp_us;
                        char name[64];
                        std::snprintf(name, sizeof(name), "%llu.png", static_cast<unsigned long long>(ts) * 1000ULL);
                        const std::string rel = std::string("cam0/") + name;
                        cv::imwrite(opt.rec_dir + "/" + rel, gray_item, png_params);
                        std::fprintf(rec_images_csv, "%llu,%s\n", static_cast<unsigned long long>(ts), rel.c_str());
                        std::fflush(rec_images_csv);
                        // 保存毫米单位的 16 位深度，供离线验证深度初始化。
                        if (!depth_item.empty()) {
                            const std::string drel = std::string("depth/") + name;
                            cv::imwrite(opt.rec_dir + "/" + drel, depth_item, png_params);
                        }
                        rec_frames.fetch_add(1);
                    } else {
                        if (rec_stop.load()) {
                            break;
                        }
                        std::this_thread::sleep_for(std::chrono::milliseconds(2));
                    }
                }
            });
        }
    }

    // 6. 工作线程：相机只诊断并入队；VIO 线程独占估计器与查看器。
    std::thread capture_thread([&] {
        try {
            while (!g_stop.load()) {
                pure_uav::VioFrame frame;
                if (camera.pop(frame, 20)) {
                    onCameraFrame(frame.gray, frame.depth_mm, frame.host_ns, frame.device_us, frame.depth_device_us, frame.system_us);
                } else if (!camera.running()) {
                    break;
                }
            }
        } catch (const std::exception& e) {
            std::fprintf(stderr, "[CAM] 共享内存读取线程异常: %s\n", e.what());
            camera.stop();
        }
    });

    std::thread vio_thread([&] {
        pure_uav::FcuClock clock;
        std::optional<FrameItem> pending;
        pure_uav::ImuTimestampFilter imu_timestamp_filter;
        uint64_t last_raw_imu_us = 0;
        uint64_t last_fed_imu_us = 0;
        bool imu_stream_ok = false;      // IMU 时间戳流是否已稳定（预热闸门）
        int imu_warmup_left = 150;       // 预热计数初值，实际跳过数由下方预递减判据决定。
        long long imu_warmup_total = 0;  // 预热累计（用于日志）
        int64_t pending_since_ns = 0;
        uint64_t last_sent_ts = 0;
        // 相机时间戳：SDK 系统时间 + 平滑偏移（见下方 D 步骤）。
        std::deque<int64_t> cam_off_us;
        uint64_t prev_fed_us = 0;
        std::deque<double> fed_dt_ms;
        int64_t last_status_ns = steadyNowNs();
        uint64_t last_received_imu = link.receivedImu();

        // 查看器（只在 VIO 线程里调用，highgui 要求同一线程）
        pure_uav::LiveView viewer;
        if (opt.view) {
            if (!viewer.open()) {
                std::fprintf(stderr, "[WARN] 实时窗口打不开，继续跑无界面模式\n");
            }
            if (!opt.view_shot_prefix.empty()) {
                viewer.setShotPrefix(opt.view_shot_prefix);
            }
        }
        cv::Mat last_gray;
        int64_t last_render_ns = 0;
        // fps / 更新率统计（都按滑窗算，避免把"刚启动"算成异常）
        std::deque<int64_t> frame_times;
        long long last_updates = 0;
        int64_t stats_win_start_ns = steadyNowNs();
        double update_rate = 0.0;
        auto imu_count_prev = static_cast<long long>(link.receivedImu());
        int64_t imu_hz_time_ns = steadyNowNs();
        double imu_hz = 0.0;
        bool imu_rate_warned = false;

        // 时间基变化必须整体恢复：暂停发送、清队列/映射/平滑状态，再重建估计器。
        // 只重置时钟会让旧滤波状态接到新时间基，造成错误传播。
        uint64_t last_reconnect_count = link.reconnectCount();
        bool timebase_recovering = false;
        auto onFcuTimebaseReset = [&](const char* why) {
            timebase_recovering = true;
            clock.reset();         // 主机↔飞控映射作废，重新累积样本
            link.clearImuQueue();  // 旧时间基的 IMU 不再喂进估计器
            {
                std::scoped_lock lk(frame_mtx);
                frame_q.clear();  // 旧时间基的相机帧同理
            }
            pending.reset();
            pending_since_ns = 0;
            prev_fed_us = 0;  // 图像时间戳平滑状态
            fed_dt_ms.clear();
            cam_off_us.clear();
            last_sent_ts = 0;  // 恢复后第一帧必须发出去
            pose_age_ms.clear();
            vio_vs_ekf3_ms.clear();
            imu_timestamp_filter.reset();
            last_raw_imu_us = last_fed_imu_us = 0;
            was_blocked = true;  // 让恢复时打印"恢复发送"
            estimator.resetForNewTimebase(why);
            stats_timebase_resets.fetch_add(1);
            std::printf(
                "[VIO] ⏱ 飞控时间基变化（%s）：暂停发送，已清空 IMU/图像队列、"
                "时钟映射和时间戳平滑状态，按新时间基重新初始化\n",
                why);
            std::fflush(stdout);
        };

        while (!g_stop.load()) {
            bool did_work = false;

            // A. 按解锁状态切换 ZUPT：地面启用，空中关闭，避免悬停被误判为静止。
            // 尚无心跳时 armed=false，沿用地面行为。
            {
                static int zupt_state = -1;  // -1 未定 / 0 关 / 1 开
                const int want = link.armed() ? 0 : 1;
                if (want != zupt_state) {
                    zupt_state = want;
                    estimator.setZuptEnabled(want == 1);
                    std::printf("[ZUPT] %s → %s零速更新\n", want == 1 ? "地面待机（未解锁）" : "已解锁（空中）", want == 1 ? "启用" : "关闭");
                    std::fflush(stdout);
                }
            }

            // B. 排空 IMU 队列：检查会话 → 预热 → 更新时间映射 → 去抖 → 喂入/记录。
            pure_uav::MavlinkLink::TimedImu imu_item;
            while (link.popImu(imu_item)) {
                // 超时/重连可能发生在出队同时，旧会话样本不再进入估计器。
                if (imu_item.session != link.reconnectCount()) {
                    continue;
                }
                // 启动缓冲可能含旧时间基数据，预热期间不送入时钟映射或估计器。
                if (!imu_stream_ok) {
                    // 保留既有预递减判据：计数从 150 减到 0 时，当前样本开始参与处理。
                    // 不用连续 dt 判稳：实测 2.5/7.5 ms 交替，严格间隔判据会一直拒收。
                    if (--imu_warmup_left > 0) {
                        ++imu_warmup_total;
                        continue;
                    }
                    imu_stream_ok = true;
                    std::printf("[imu] 预热完成（丢弃 %lld 个样本），开始喂 IMU\n", imu_warmup_total);
                    std::fflush(stdout);
                }
                // 串口重连也算一次时间基变化（新会话的 boot 时间很可能重新开始）
                if (imu_item.session != last_reconnect_count) {
                    last_reconnect_count = imu_item.session;
                    onFcuTimebaseReset("连接会话变化");
                }
                const bool imu_gap = last_raw_imu_us != 0 && imu_item.imu.timestamp_us > last_raw_imu_us && imu_item.imu.timestamp_us - last_raw_imu_us > 100000;
                const auto clock_res = clock.addSample(imu_item.host_ns, imu_item.imu.timestamp_us);
                if (clock_res == pure_uav::FcuClock::AddResult::kTimebaseReset || imu_gap) {
                    // FcuClock 已清空旧样本；本帧作为新时间基的第一帧继续往下喂。
                    onFcuTimebaseReset(imu_gap ? "IMU 中断超过 100ms" : "飞控时间戳跳变");
                    clock.addSample(imu_item.host_ns, imu_item.imu.timestamp_us);
                } else if (clock_res == pure_uav::FcuClock::AddResult::kOutOfOrder) {
                    stats_imu_out_of_order.fetch_add(1);  // 乱序帧不喂 OpenVINS（会破坏时间递增）
                    continue;
                }
                // 用均匀网格与慢速 PLL 去除调度抖动；raw 模式保留原始时间戳。
                // 时间基检查仍基于原始 FCU 时间，送入估计器前再转为 Unix 微秒。
                pure_uav::ImuSample imu_feed = imu_item.imu;
                if (opt.imu_ts_uniform) {
                    // 有实测速率时按实测值重建网格，否则使用请求值；保留原有更新时机。
                    auto eff_hz = static_cast<double>(opt.imu_req_hz);
                    if (imu_hz > 20.0) {
                        eff_hz = imu_hz;
                        if (!imu_rate_warned && std::abs(imu_hz - static_cast<double>(opt.imu_req_hz)) > 0.15 * opt.imu_req_hz) {
                            imu_rate_warned = true;
                            std::printf(
                                "[WARN] 请求 IMU %d Hz，实测只有 %.0f Hz —— 已自动按实测值"
                                "重建时间戳网格。要用别的速率请改 --imu-hz %.0f\n",
                                opt.imu_req_hz, imu_hz, imu_hz);
                            std::fflush(stdout);
                        }
                    }
                    if (!imu_timestamp_filter.filter(imu_item.imu.timestamp_us, 1e6 / eff_hz, imu_feed.timestamp_us)) {
                        continue;
                    }
                }
                last_raw_imu_us = imu_item.imu.timestamp_us;
                const uint64_t fed_fcu_us = imu_feed.timestamp_us;
                uint64_t raw_unix_us = 0;
                if (!clock.fcuToUnixUs(fed_fcu_us, imu_feed.timestamp_us) || !clock.fcuToUnixUs(imu_item.imu.timestamp_us, raw_unix_us)) {
                    continue;
                }
                estimator.feedImu(imu_feed);
                last_fed_imu_us = fed_fcu_us;  // 图像覆盖判定仍使用原始 FCU 时钟域
                if (rec_imu_csv) {
                    // 录"实际喂给估计器的时间戳"（= 去抖后的），这样回放能 1:1 复现实时行为。
                    std::fprintf(rec_imu_csv, "%llu,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n", static_cast<unsigned long long>(imu_feed.timestamp_us), imu_item.imu.gx, imu_item.imu.gy, imu_item.imu.gz, imu_item.imu.ax, imu_item.imu.ay, imu_item.imu.az);
                }
                if (imu_log) {
                    std::fprintf(imu_log, "%llu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", static_cast<unsigned long long>(raw_unix_us), imu_item.imu.ax, imu_item.imu.ay, imu_item.imu.az, imu_item.imu.gx, imu_item.imu.gy, imu_item.imu.gz);
                }
                did_work = true;
            }

            // C. 只取一帧图像；pending 保留到 IMU 覆盖或等待超时。
            if (!pending) {
                std::scoped_lock lk(frame_mtx);
                if (!frame_q.empty()) {
                    pending = std::move(frame_q.front());
                    frame_q.pop_front();
                    pending_since_ns = steadyNowNs();
                }
            }

            // D. 固定图像时间 → 等待 IMU 覆盖 → 注入深度 → 估计/录制/发布。
            if (pending) {
                uint64_t fcu_us = 0;
                if (!clock.valid() || !clock.hostToFcuUs(pending->host_ns, fcu_us)) {
                    if (steadyNowNs() - pending_since_ns > 2000000000LL) {
                        pending.reset();
                        stats_drop_no_clock.fetch_add(1);
                    }
                } else {
                    // host 模式使用服务端采集时刻；device 模式沿用 SDK system_us 平滑。
                    // 取最近 90 个 FCU−SDK 偏移的中位数；等待 IMU 时复用本帧映射，避免重复采样。
                    if (pending->mapped_fcu_us != 0) {
                        fcu_us = pending->mapped_fcu_us;
                    } else if (opt.cam_ts_device && pending->system_us > 0) {
                        const int64_t off_i = static_cast<int64_t>(fcu_us) - pending->system_us;
                        cam_off_us.push_back(off_i);
                        while (cam_off_us.size() > 90) {
                            cam_off_us.pop_front();
                        }
                        std::vector<int64_t> tmp(cam_off_us.begin(), cam_off_us.end());
                        std::nth_element(tmp.begin(), tmp.begin() + tmp.size() / 2, tmp.end());
                        const int64_t off_med = tmp[tmp.size() / 2];
                        const auto t_dev = static_cast<uint64_t>(pending->system_us + off_med);
                        // 合理性护栏：与主机换算值相差超过 200 ms 说明偏移估计还没收敛或时钟跳变
                        const int64_t dev_minus_host = static_cast<int64_t>(t_dev) - static_cast<int64_t>(fcu_us);
                        if (std::llabs(dev_minus_host) < 200000) {
                            fcu_us = t_dev;
                        }
                        // 记录喂给 VIO 的时间戳间隔，用于和主机侧抖动对比
                        if (prev_fed_us > 0) {
                            const double d = static_cast<double>(fcu_us) - static_cast<double>(prev_fed_us);
                            if (d > 0 && d < 200000) {
                                fed_dt_ms.push_back(d * 1e-3);
                                if (fed_dt_ms.size() > 400) {
                                    fed_dt_ms.pop_front();
                                }
                            }
                        }
                        prev_fed_us = fcu_us;
                    }
                    pending->mapped_fcu_us = fcu_us;
                    const int64_t waited_ns = steadyNowNs() - pending_since_ns;
                    // 以实际喂给估计器的 IMU 和标定时间偏移判断覆盖。超时丢图，不能强行传播。
                    const bool covered = pure_uav::imuCoversImage(last_fed_imu_us, fcu_us, estimator.cameraTimeOffsetSec());
                    if (!covered && waited_ns > 100000000LL) {
                        pending.reset();
                        stats_drop_no_clock.fetch_add(1);
                    } else if (covered) {
                        // 深度时间 = 映射后的彩色时间 + 两个同源设备时间戳的差值。
                        // 两个设备时间戳同源，所以(color_fcu - color_dev) 的偏移对深度同样成立。
                        uint64_t depth_fcu_us = fcu_us;
                        bool depth_time_valid = false;
                        if (pending->depth_dev_us > 0 && pending->device_us > 0) {
                            const int64_t off = static_cast<int64_t>(fcu_us) - pending->device_us;
                            const int64_t d = pending->depth_dev_us + off;
                            if (d > 0 && std::llabs(d - static_cast<int64_t>(fcu_us)) < 200000) {
                                depth_fcu_us = static_cast<uint64_t>(d);
                                depth_time_valid = true;
                            }
                        }
                        uint64_t image_unix_us = 0;
                        uint64_t depth_unix_us = 0;
                        if (!clock.fcuToUnixUs(fcu_us, image_unix_us) || !clock.fcuToUnixUs(depth_fcu_us, depth_unix_us)) {
                            pending.reset();
                            stats_drop_no_clock.fetch_add(1);
                            continue;
                        }
                        if (opt.use_depth_init && depth_time_valid && !pending->depth.empty() && cam_depth.valid()) {
                            estimator.setDepthImage(pending->depth, depth_unix_us, cam_depth.fx, cam_depth.fy, cam_depth.cx, cam_depth.cy, cam_depth.R_c2d, depth_t_m);
                        }
                        estimator.feedImage(pending->gray, image_unix_us);
                        if (rec_images_csv) {
                            std::scoped_lock lk(rec_mtx);
                            if (rec_q.size() < 300) {
                                rec_q.push_back({pending->gray, pending->depth, image_unix_us});
                            }
                        }
                        const int64_t frame_host_ns = pending->host_ns;  // 位姿年龄统计用
                        last_gray = pending->gray;                       // 浅拷贝（引用计数），给查看器画
                        pending.reset();
                        frame_times.push_back(steadyNowNs());

                        pure_uav::VisionPose pose;
                        if (estimator.initialized() && estimator.getLatestPose(pose) && pose.timestamp_us != last_sent_ts) {
                            last_sent_ts = pose.timestamp_us;
                            // 纯诊断：记录处理延迟及 VIO/飞控速率差，不参与发送门控。
                            pose_age_ms.push_back(static_cast<double>(steadyNowNs() - frame_host_ns) * 1e-6);
                            if (pose_age_ms.size() > 600) {
                                pose_age_ms.pop_front();
                            }
                            const auto ekf3 = link.ekf3Velocity();
                            if (ekf3.valid) {
                                const double vio_sp = std::sqrt((static_cast<double>(pose.vx) * pose.vx) + (static_cast<double>(pose.vy) * pose.vy) + (static_cast<double>(pose.vz) * pose.vz));
                                const double fcu_sp = std::sqrt((ekf3.vx * ekf3.vx) + (ekf3.vy * ekf3.vy) + (ekf3.vz * ekf3.vz));
                                vio_vs_ekf3_ms.push_back((vio_sp - fcu_sp) * 1e3);  // 单位 mm/s
                                if (vio_vs_ekf3_ms.size() > 600) {
                                    vio_vs_ekf3_ms.pop_front();
                                }
                            }
                            logger.log(pose);
                            stats_poses.fetch_add(1);
                            // 当前速率供状态行与经验模糊上限对照。
                            {
                                const double sp = std::sqrt((static_cast<double>(pose.vx) * pose.vx) + (static_cast<double>(pose.vy) * pose.vy) + (static_cast<double>(pose.vz) * pose.vz));
                                cur_speed_mps.store(sp);
                            }
                            // 健康门控始终生效；实际发送还要求链路、会话和时间基均就绪。
                            // no-send 只记录结果，沿用原来的链路门控豁免。
                            const bool gate_ok = !opt.send || (link.connected() && !timebase_recovering && link.reconnectCount() == last_reconnect_count);
                            if (!estimator.publishable() || !gate_ok) {
                                stats_blocked.fetch_add(1);
                                if (!was_blocked) {
                                    was_blocked = true;
                                    if (!estimator.publishable()) {
                                        const auto h = estimator.healthReport();
                                        std::printf(
                                            "[VIO] ⛔ 视觉失效（%s）：特征 %d，距上次更新 %.2fs"
                                            " → 停止发送位姿\n",
                                            pure_uav::toString(h.state), h.features, h.since_update_s);
                                    } else {
                                        std::printf("[VIO] ⏸ 暂停发送位姿：%s\n", !link.connected() ? "飞控链路未连接" : "飞控时间基重建中");
                                    }
                                    std::fflush(stdout);
                                }
                            } else {
                                if (was_blocked) {
                                    was_blocked = false;
                                    const auto h = estimator.healthReport();
                                    std::printf("[VIO] ✅ 恢复发送位姿（健康 %s，特征 %d，重建 %d 次）\n", pure_uav::toString(h.state), h.features, h.recoveries);
                                    std::fflush(stdout);
                                }
                                // 发送质量、重建计数与经验协方差：
                                //   OK -> 质量 90 / 位置 σ 0.15 m / 速度 σ 0.25 m/s
                                //   DEGRADED -> 质量 50 / 位置 σ 0.5 m / 速度 σ 0.8 m/s
                                //   reset_counter 用估计器重建次数，飞控据此处理重锚定跳变
                                const auto hs = estimator.healthState();
                                const int q = (hs == pure_uav::VioHealthState::OK) ? 90 : 50;
                                const float ps = (hs == pure_uav::VioHealthState::OK) ? 0.15F : 0.5F;
                                const float vs = (hs == pure_uav::VioHealthState::OK) ? 0.25F : 0.8F;
                                const uint8_t rc = pure_uav::odometryResetCounter(estimator.rebuildCount());
                                if (opt.send && link.sendVisionPose(pose, q, rc, ps, vs)) {
                                    safe_logger.log(pose);
                                    stats_sent.fetch_add(1);
                                }
                            }
                        }
                    }
                }
            }

            // 新时间基下估计器初始化且时钟就绪后，解除恢复标志。
            if (timebase_recovering && estimator.initialized() && clock.valid()) {
                timebase_recovering = false;
                std::printf("[VIO] ✅ 已按新时间基重新初始化（时钟样本 %zu）→ 恢复发送位姿\n", clock.sampleCount());
                std::fflush(stdout);
            }

            // E. 控制台诊断（每 2 秒）；保持各项统计与提示的原有算法。
            const int64_t now = steadyNowNs();
            if (!opt.quiet && now - last_status_ns > 2000000000LL) {
                last_status_ns = now;
                const uint64_t rx = link.receivedImu();
                const auto tc = estimator.tiltCheck();
                char tiltBuf[32];
                if (tc.valid) {
                    std::snprintf(tiltBuf, sizeof(tiltBuf), "%.1fdeg", tc.deg);
                } else {
                    std::snprintf(tiltBuf, sizeof(tiltBuf), "-");
                }
                const std::string tiltStr(tiltBuf);
                const double off_ms = clock.valid() ? clock.offsetNs() / 1e6 : 0.0;
                // 端到端位姿年龄与 VIO/EKF3 速率差。
                double age_p50 = 0.0;
                double age_p95 = 0.0;
                double dvel_med = 0.0;
                if (pose_age_ms.size() > 10) {
                    std::vector<double> v(pose_age_ms.begin(), pose_age_ms.end());
                    std::sort(v.begin(), v.end());
                    age_p50 = v[v.size() / 2];
                    age_p95 = v[v.size() * 95 / 100];
                }
                if (vio_vs_ekf3_ms.size() > 10) {
                    std::vector<double> v(vio_vs_ekf3_ms.begin(), vio_vs_ekf3_ms.end());
                    std::sort(v.begin(), v.end());
                    dvel_med = v[v.size() / 2];
                }
                // 两次状态输出之间的深度重投影误差均值；样本不足时沿用上次结果。
                // 此处仅打印诊断，估计器自己的健康判定使用另一套稳健统计。
                long long dn = 0;
                double dmean = 0.0;
                long long dhist[6] = {0, 0, 0, 0, 0, 0};
                estimator.depthErrStats(dn, dmean, dhist);
                const double dsum = dmean * static_cast<double>(dn);
                double dep_recent = -1.0;
                long long dep_dn = dn - last_deperr_n;
                if (dn > last_deperr_n && dep_dn > 5) {
                    dep_recent = std::max(0.0, dsum - last_deperr_sum) / static_cast<double>(dep_dn);
                } else if (dn > 0 && dep_dn <= 5) {
                    dep_recent = last_deperr_recent;  // 没有新样本就沿用上一次
                }
                if (dep_recent >= 0.0) {
                    last_deperr_recent = dep_recent;
                }
                last_deperr_n = dn;
                last_deperr_sum = dsum;
                if (dep_recent > 20.0 && !warned_deperr) {
                    warned_deperr = true;
                    std::printf(
                        "[VIO] ⛔ 深度一致性崩塌（%.0f px，正常 2-4 px）→ 特征跟踪已错配，"
                        "这趟数据基本废了，尽快停下重录\n",
                        dep_recent);
                    std::fflush(stdout);
                }
                // 经验速率参考 0.21×Z，仅供诊断，沿用调试时的系数。
                char spdBuf[40];
                {
                    const double v = cur_speed_mps.load();
                    const double z = depth_median_mm * 1e-3;  // 最近一次采样的中位深度。
                    if (z > 0.2) {
                        const double vlim = 0.21 * z;
                        std::snprintf(spdBuf, sizeof(spdBuf), "%.2f/%.2f", v, vlim);
                    } else {
                        std::snprintf(spdBuf, sizeof(spdBuf), "%.2f/-", v);
                    }
                }
                // device 模式下的相机时间戳间隔，用稳健 σ 与 [cam] 帧间隔对照。
                char tsjBuf[24];
                {
                    if (fed_dt_ms.size() > 20) {
                        std::vector<double> v(fed_dt_ms.begin(), fed_dt_ms.end());
                        std::sort(v.begin(), v.end());
                        const double p84 = v[v.size() * 84 / 100];
                        const double p16 = v[v.size() * 16 / 100];
                        std::snprintf(tsjBuf, sizeof(tsjBuf), "%.2f", 0.5 * (p84 - p16));
                    } else {
                        std::snprintf(tsjBuf, sizeof(tsjBuf), "-");
                    }
                }
                char depBuf[24];
                if (dep_recent >= 0.0) {
                    std::snprintf(depBuf, sizeof(depBuf), "%.1fpx", dep_recent);
                } else {
                    std::snprintf(depBuf, sizeof(depBuf), "-");
                }
                // 沿用状态行的质量与重建计数显示规则，便于比对历史日志。
                const int qual_now = (estimator.healthState() == pure_uav::VioHealthState::OK) ? 90 : 50;
                const int rc_now = static_cast<int>(std::min<long long>(estimator.rebuildCount(), 255));
                std::printf(
                    "[VIO] init=%d img=%d pose=%lld sent=%lld blocked=%lld health=%s "
                    "qual=%d rc=%d feat=%d tilt=%s lin=%.2f tri=%.3f depthused=%lld deperr=%s v=%sm/s tsjit=%sms imu=%llu(+%llu/2s) q=%zu clock=%.1fms "
                    "age=%.0f/%.0fms dvel=%+.0fmm/s\n",
                    estimator.initialized() ? 1 : 0, estimator.imageCount(), static_cast<long long>(stats_poses.load()), static_cast<long long>(stats_sent.load()), static_cast<long long>(stats_blocked.load()), pure_uav::toString(estimator.healthState()), qual_now, rc_now,
                    estimator.healthReport().features, tiltStr.c_str(), estimator.linAccelDev(), estimator.triangulationRate(), estimator.depthUsed(), depBuf, spdBuf, tsjBuf, static_cast<unsigned long long>(rx), static_cast<unsigned long long>(rx - last_received_imu), link.imuQueueSize(), off_ms,
                    age_p50, age_p95, dvel_med);
                std::fflush(stdout);
                last_received_imu = rx;
            }

            // F. 实时查看器（限速约 33 Hz），仍在 VIO 线程内同步渲染。
            if (opt.view && now - last_render_ns > 30000000LL) {
                last_render_ns = now;

                // 相机帧率（最近 2 秒）
                while (!frame_times.empty() && now - frame_times.front() > 2000000000LL) {
                    frame_times.pop_front();
                }
                double display_fps = 0.0;
                if (frame_times.size() >= 2) {
                    display_fps = (frame_times.size() - 1) * 1e9 / static_cast<double>(frame_times.back() - frame_times.front());
                }

                // 保留原有行为：实测速率在查看器分支每 2 秒更新，也供上面的 IMU 去抖使用。
                if (now - imu_hz_time_ns > 2000000000LL) {
                    const auto cnt = static_cast<long long>(link.receivedImu());
                    imu_hz = (cnt - imu_count_prev) * 1e9 / static_cast<double>(now - imu_hz_time_ns);
                    imu_count_prev = cnt;
                    imu_hz_time_ns = now;
                }

                // MSCKF 更新率（累计计数差分 / 2 秒）
                {
                    long long c = 0;
                    long long f = 0;
                    long long r = 0;
                    long long u = 0;
                    long long rows = 0;
                    long long cd = 0;
                    long long td = 0;
                    estimator.msckfStats(c, f, r, u, rows, cd, td);
                    last_updates = std::min(u, last_updates);  // 重建估计器后计数器归零
                    if (now - stats_win_start_ns > 2000000000LL) {
                        update_rate = static_cast<double>(u - last_updates) / 2.0;
                        last_updates = u;
                        stats_win_start_ns = now;
                    }
                }

                pure_uav::LiveView::Status st;
                st.initialized = estimator.initialized();
                st.health = estimator.healthState();
                st.publishable = estimator.publishable();
                const auto hr = estimator.healthReport();
                st.features = hr.features;
                st.since_update_s = hr.since_update_s;
                st.recoveries = hr.recoveries;
                st.update_rate = update_rate;
                st.images = estimator.imageCount();
                st.poses = stats_poses.load();
                st.sent = stats_sent.load();
                st.blocked = stats_blocked.load();
                st.fps = display_fps;
                st.imu_hz = imu_hz;
                st.lin_acc = static_cast<float>(estimator.linAccelDev());
                st.tri_rate = static_cast<float>(estimator.triangulationRate());
                st.recording = opt.send;
                const auto tcc = estimator.tiltCheck();
                st.tilt_valid = tcc.valid;
                st.tilt_deg = tcc.deg;

                pure_uav::VisionPose pose;
                if (estimator.getLatestPose(pose)) {
                    st.has_pose = true;
                    st.x = pose.x;
                    st.y = pose.y;
                    st.z = pose.z;
                    st.vx = pose.vx;
                    st.vy = pose.vy;
                    st.vz = pose.vz;
                    st.qw = pose.q[0];
                    st.qx = pose.q[1];
                    st.qy = pose.q[2];
                    st.qz = pose.q[3];
                }
                pure_uav::VioEstimator::RawState raw;
                if (estimator.getLatestRawState(raw)) {
                    st.has_raw = true;
                    st.bias_a = raw.bias_a;
                    st.bias_g = raw.bias_g;
                }
                const float mean_now = live_mean.load();
                const float fast_now = live_fast.load();
                if (mean_now >= 0.0F) {
                    char nb[128];
                    std::snprintf(nb, sizeof(nb), "mean %.0f  fast %.0f", static_cast<double>(mean_now), static_cast<double>(fast_now));
                    st.note = nb;
                }

                viewer.render(last_gray, estimator.trackedPoints(), st);
                if (viewer.poll()) {
                    std::printf("[VIO] 查看器被关闭（q / Esc / 点 ×）→ 停止采集\n");
                    std::fflush(stdout);
                    g_stop.store(true);
                }
            }

            if (!did_work) {
                std::this_thread::sleep_for(std::chrono::milliseconds(1));
            }
        }
    });

    // 7. 主线程监督退出；先停采集和估计，再排空录制队列、关闭输出。
    const auto t0 = std::chrono::steady_clock::now();
    int result = 0;
    while (!g_stop.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (!camera.running()) {
            std::fprintf(stderr, "[WARN] camera_server 帧通道已停止: %s\n", camera.lastError().c_str());
            result = 1;
            break;
        }
        if (opt.duration > 0.0) {
            const double elapsed = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            if (elapsed >= opt.duration) {
                break;
            }
        }
    }
    g_stop.store(true);

    capture_thread.join();
    vio_thread.join();
    camera.stop();
    link.stop();
    logger.close();
    safe_logger.close();
    if (imu_log != nullptr) {
        std::fclose(imu_log);
        std::printf("  IMU 日志      : %s\n", opt.imu_log_path.c_str());
    }
    if (!opt.rec_dir.empty()) {
        rec_stop.store(true);
        if (rec_thread.joinable()) {
            rec_thread.join();
        }
        if (rec_images_csv != nullptr) {
            std::fclose(rec_images_csv);
        }
        if (rec_imu_csv != nullptr) {
            std::fclose(rec_imu_csv);
        }
        std::printf("  录制数据集    : %s（%lld 帧）\n", opt.rec_dir.c_str(), static_cast<long long>(rec_frames.load()));
    }

    std::printf("\n结束:\n");
    std::printf("  图像帧数     : %d\n", estimator.imageCount());
    std::printf("  VisionPose   : %lld\n", static_cast<long long>(stats_poses.load()));
    std::printf("  已发送        : %lld\n", static_cast<long long>(stats_sent.load()));
    std::printf("  丢帧(无时钟)  : %lld\n", static_cast<long long>(stats_drop_no_clock.load()));
    std::printf("  时间基重置    : %lld（飞控重启/串口重连后重新初始化）\n", static_cast<long long>(stats_timebase_resets.load()));
    std::printf("  IMU 乱序丢弃  : %lld\n", static_cast<long long>(stats_imu_out_of_order.load()));
    std::printf("  相机队列丢帧  : %llu\n", static_cast<unsigned long long>(dropped_frames));
    std::printf("  IMU 丢帧      : %llu\n", static_cast<unsigned long long>(link.droppedImu()));
    std::printf("  CSV           : %s\n", opt.csv_out.c_str());
    std::printf("  TUM           : %s\n", opt.tum_out.c_str());
    if (!opt.safe_tum_out.empty()) {
        std::printf("  发布轨迹 TUM  : %s\n", opt.safe_tum_out.c_str());
    }
    return result;
}
