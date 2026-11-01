// 实时 VIO：Orbbec 彩色相机 + 飞控 MAVLink IMU → VisionPose → 飞控 ODOMETRY。
//
// 线程模型：
//   - MAVLink 接收线程（MavlinkLink 内部）：收 HIGHRES_IMU / HEARTBEAT
//   - Orbbec 采集线程（OrbbecCamera 内部）：收彩色帧
//   - VIO 线程（本文件）：按时间顺序喂 IMU/图像，输出 VisionPose 并发送
//
// 时间戳：
//   - IMU：飞控 HIGHRES_IMU.time_usec（boot 时间，µs）
//   - 相机：帧到达主机时的 CLOCK_MONOTONIC，用 FcuClock 映射到 IMU 时钟
//   - 输出：t_imu = t_cam + timeshift_cam_imu（由 OpenVINS 配置给出）
//
// 用法示例：
//   ./vio_live --serial /dev/ttyACM0 --baud 921600 \
//              --config-dir config/gemini_1920x1080 \
//              --width 1920 --height 1080 --fps 30 \
//              --csv-out out/live_pose.csv --tum-out out/live_traj.tum

#include <algorithm>
#include <atomic>
#include <chrono>
#include <csignal>
#include <filesystem>
#include <thread>

#include <opencv2/imgcodecs.hpp>
#include <cstdio>
#include <cstring>
#include <deque>
#include <mutex>
#include <optional>
#include <string>
#include <thread>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include "fcu_clock.hpp"
#include "live_view.hpp"
#include "mavlink_link.hpp"
#include "orbbec_camera.hpp"
#include "pure_uav/pose_logger.hpp"
#include "pure_uav/vio_estimator.hpp"
#include "pure_uav/vision_pose.hpp"

namespace {

std::atomic<bool> g_stop{false};

void onSigint(int) { g_stop.store(true); }

int64_t steadyNowNs() {
    using namespace std::chrono;
    return duration_cast<nanoseconds>(steady_clock::now().time_since_epoch()).count();
}

struct FrameItem {
    cv::Mat gray;
    int64_t host_ns = 0;
    int64_t device_us = 0;   // 【2026-10-23】相机自己的时间戳（稳定，不含主机调度抖动）
    int64_t depth_dev_us = 0; // 【2026-10-25】深度帧自己的设备时间戳（和彩色同源时钟）
    cv::Mat depth;   // 对齐到彩色的深度（毫米，CV_16UC1）；没开深度时空
};

void printUsage(const char* argv0) {
    std::printf(
        "用法: %s [选项]\n"
        "\n"
        "  --serial DEV        飞控串口（默认 /dev/ttyACM0）\n"
        "  --baud N            波特率（默认 921600）\n"
        "  --config-dir DIR    OpenVINS 配置目录（默认 config/gemini_1920x1080）\n"
        "  --width N           彩色图宽（默认 1920）\n"
        "  --height N          彩色图高（默认 1080）\n"
        "  --fps N             彩色帧率（默认 30）\n"
        "  --csv-out PATH      VisionPose CSV 输出（默认 out/live_pose.csv）\n"
        "  --tum-out PATH      TUM 轨迹输出（默认 out/live_traj.tum，含所有估计位姿）\n"
        "  --safe-tum-out PATH 只写「实际发出」的位姿（健康门限通过的），给 RViz 看用\n"
        "  --imu-log PATH      把原始 IMU 存成 CSV（t_us,ax,ay,az,gx,gy,gz）\n"
        "  --imu-ts MODE       IMU 时间戳处理：uniform(默认，去抖) | raw(原样喂)\n"
        "  --max-exposure-ms M  曝光硬上限（默认 15）。模糊预算更大时可以抬高换亮度；\n"
        "                      暗场地想看清就抬到 20-25，代价是运动上限按比例下降\n"
        "  --strict-blur        严守模糊预算：光不够也不把曝光放长（画面偏暗但更锐）\n"
        "  --cam-ts MODE        相机时间戳来源：host(默认，主机到达时刻) | device(设备时钟)\n"
        "                      device 在 --aggregate color 下把抖动从 13 ms 降到 0.8 ms；\n"
        "                      但 full 模式下尚未收敛(实测会变差)，别混用\n"
        "  --aggregate MODE     帧集聚合：full(默认) | color(彩色驱动，深度可选) | any\n"
        "                      彩色被深度的帧率拖住时用 color；见 orbbec_camera.hpp 注释\n"
        "  --no-depth          不开深度流（只跑彩色）。排查帧率上限时用：\n"
        "                      实测 640x480@60 + 深度双流只交付 31 fps，需要它来二分。\n"
        "  --imu-hz N          向飞控请求的 HIGHRES_IMU 速率（默认 200；上限受\n"
        "                      SCHED_LOOP_RATE 和串口带宽约束，921600 下 400 是稳妥值）\n"
        "  --init-threads N    动态初始化 MLE 线程数（默认用配置文件）\n"
        "  --use-depth-init    用深度直接初始化特征位置（纯旋转也能出测量；A/B 用）\n"
        "  --init-vel-zero     静止起步时把初始速度置 0（配 IMU 静止判定，防初始速度抽签）\n"
        "  --record DIR        录制成离线数据集（DIR/cam0/*.png + images.csv + imu.csv），\n"
        "                      之后可用 vio_offline --dataset DIR 无限次回放调试\n"
        "                      ——用来事后核对「相机到底动没动」，不参与估计\n"
        "  --duration S        运行 S 秒后自动退出（默认一直运行）\n"
        "  --no-send           不向飞控发送 ODOMETRY（只记录/调试）\n"
        "  --online-timeoffset 打开 OpenVINS 在线 cam-IMU 时移估计（默认已开）\n"
        "  --static-init       用静态初始化（先静止 2 秒再动），默认是动态初始化。\n"
        "                      做 A/B 对照用：静态初始化定出的重力方向更准，倾角误差更小\n"
        "  --no-online-timeoffset 关掉它，退回到只用标定的常数时移（做对照用）\n"
        "  --exposure N        锁定曝光，单位 100us（默认 100 = 10 ms，与标定一致）\n"
        "  --gain N            锁定增益（默认 10）\n"
        "  --no-tune           跳过进场自动测光，直接用 --exposure/--gain 的锁定值\n"
        "                      （复现录制时的配置用；不测光就没有「这个场地够不够亮」的判定）\n"
        "  --auto-exposure     不锁曝光，交给相机自动（会引入几 ms 的时移漂移，仅调试用）\n"
        "  --tune-expected-rate D  预期最大角速度（度/秒，默认 100），用来从模糊预算反推曝光上限\n"
        "  --tune-blur-budget P    允许的运动模糊上限（像素，默认 3.0）\n"
        "  --tune-max-gain N       增益上限（默认 40；100 的噪声会把 KLT 拖死）\n"
        "  --allow-blur        光不够也硬飞（只用于排查）\n"
        "  --allow-dark        测光连灰度 45 都到不了也硬开（只用于排查/演示）\n"
        "  --view              打开实时窗口：图像+跟踪点+状态+轨迹（按 q 退出）\n"
        "  --view-shot-prefix P  每 2 秒把窗口存成 P_000.png…（预览用）\n"
        "  --imu-to-body-rpy R P Y  R_ItoB：把 IMU 系向量转到机体系（ZYX 欧拉角，度）\n"
        "  --imu-to-body-t X Y Z    机体系原点在 IMU 系下的坐标（米）\n"
        "  --dump-params       只读飞控参数后退出（查 IMU 低通/陷波设置，不碰相机）\n"
        "  --quiet             少打印\n"
        "\n失效保护（默认开）：\n"
        "  --health-min-features N   跟踪特征数低于 N 算不足（默认 60）\n"
        "  --health-lost-after S     断供超过 S 秒判 LOST（默认 1.5）\n"
        "  --reinit-on-recover       LOST 后重建估计器并重锚定（默认关；开了才可能自己恢复）\n"
        "\n  LOST 期间不发送 ODOMETRY；飞控按 EKF2_EV_TIMEOUT / EK3 源超时自己退回姿态模式。\n"
        "  -h, --help\n",
        argv0);
}


Eigen::Matrix3d rpyDegToMatrix(double roll_deg, double pitch_deg, double yaw_deg) {
    // ZYX 欧拉角：R = Rz(yaw) * Ry(pitch) * Rx(roll)
    const double r = roll_deg * M_PI / 180.0;
    const double p = pitch_deg * M_PI / 180.0;
    const double y = yaw_deg * M_PI / 180.0;
    return (Eigen::AngleAxisd(y, Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(p, Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(r, Eigen::Vector3d::UnitX()))
        .toRotationMatrix();
}

}  // namespace

int main(int argc, char** argv) {
    std::string serial = "/dev/ttyACM0";
    int baud = 921600;
    std::string config_dir = "config/gemini_1920x1080";
    int width = 1920, height = 1080, fps = 30;
    std::string csv_out = "out/live_pose.csv";
    std::string tum_out = "out/live_traj.tum";
    std::string safe_tum_out;   // 只记"实际发布出去"的位姿（健康门限通过的那些）
    std::string imu_log_path;
    std::string rec_dir;
    // IMU 时间戳去抖：默认打开（见 --imu-ts 说明）。--imu-ts raw 可关掉做对照。
    bool imu_ts_uniform = true;
    int imu_req_hz = 200;   // HIGHRES_IMU 请求速率；见 --imu-hz
    bool enable_depth = true;  // --no-depth 关掉深度流
    int aggregate_mode = 0;    // --aggregate
    bool cam_ts_device = false;   // --cam-ts device
    bool strict_blur = false;     // --strict-blur
    double max_exposure_ms = -1.0;  // --max-exposure-ms
    double duration = 0.0;
    bool send = true;
    bool quiet = false;
    bool online_timeoffset = true;
    int init_threads = 0;   // 0 = 用配置文件；1 = 可复现
    bool use_depth_init = false;  // 用深度初始化特征（A/B 用）
    bool init_vel_zero = false;   // 静止起步时把初始速度置 0（需配合静止判定）
    bool static_init = false;
    bool dump_params = false;
    int exposure = 100;      // 100us 单位：100 = 10 ms
    int gain = 30;           // 对齐 capture_config.py（唯一真源：100/30）
    bool auto_exposure = false;   // 默认开：live 相机时间戳是"USB 到包时间"，标定值不一定完全适用
    double tune_expected_rate = 100.0;  // 度/秒，用于从模糊预算反推曝光上限
    double tune_blur_budget = 3.0;      // 像素
    int tune_max_gain = 40;
    bool tune_allow_blur = false;
    bool tune_allow_dark = false;
    bool no_tune = false;
    bool view = false;
    std::string view_shot_prefix;
    // 失效保护参数
    int health_min_features = 60;
    double health_lost_after = 1.5;
    bool reinit_on_recover = true;   // 见 vio_health.hpp 的说明
    double itb_rpy[3] = {0.0, 0.0, 0.0};
    double itb_t[3] = {0.0, 0.0, 0.0};

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto need = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "参数 %s 缺少值\n", name);
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "--serial") {
            serial = need("--serial");
        } else if (a == "--baud") {
            baud = std::stoi(need("--baud"));
        } else if (a == "--config-dir") {
            config_dir = need("--config-dir");
        } else if (a == "--width") {
            width = std::stoi(need("--width"));
        } else if (a == "--height") {
            height = std::stoi(need("--height"));
        } else if (a == "--fps") {
            fps = std::stoi(need("--fps"));
        } else if (a == "--csv-out") {
            csv_out = need("--csv-out");
        } else if (a == "--tum-out") {
            tum_out = need("--tum-out");
        } else if (a == "--safe-tum-out") {
            safe_tum_out = need("--safe-tum-out");
        } else if (a == "--imu-log") {
            imu_log_path = need("--imu-log");
        } else if (a == "--record") {
            rec_dir = need("--record");
        } else if (a == "--max-exposure-ms") {
            max_exposure_ms = std::stod(need("--max-exposure-ms"));
            if (max_exposure_ms < 1.0 || max_exposure_ms > 60.0) {
                std::fprintf(stderr, "--max-exposure-ms 只接受 1..60\n"); return 2;
            }
        } else if (a == "--strict-blur") {
            strict_blur = true;
        } else if (a == "--cam-ts") {
            const std::string v = need("--cam-ts");
            if (v == "host") cam_ts_device = false;
            else if (v == "device") cam_ts_device = true;
            else { std::fprintf(stderr, "--cam-ts 只接受 host|device\n"); return 2; }
        } else if (a == "--aggregate") {
            const std::string v = need("--aggregate");
            if (v == "full") aggregate_mode = 0;
            else if (v == "color") aggregate_mode = 1;
            else if (v == "any") aggregate_mode = 2;
            else { std::fprintf(stderr, "--aggregate 只接受 full|color|any\n"); return 2; }
        } else if (a == "--no-depth") {
            enable_depth = false;
        } else if (a == "--imu-hz") {
            imu_req_hz = std::stoi(need("--imu-hz"));
            if (imu_req_hz < 50 || imu_req_hz > 1000) {
                std::fprintf(stderr, "--imu-hz 只接受 50..1000\n");
                return 2;
            }
        } else if (a == "--imu-ts") {
            const std::string v = need("--imu-ts");
            if (v == "raw") imu_ts_uniform = false;
            else if (v == "uniform") imu_ts_uniform = true;
            else { std::fprintf(stderr, "--imu-ts 只接受 raw 或 uniform\n"); return 2; }
        } else if (a == "--duration") {
            duration = std::stod(need("--duration"));
        } else if (a == "--no-send") {
            send = false;
        } else if (a == "--online-timeoffset") {
            online_timeoffset = true;
        } else if (a == "--init-vel-zero") {
            init_vel_zero = true;
        } else if (a == "--use-depth-init") {
            use_depth_init = true;
        } else if (a == "--init-threads") {
            init_threads = std::stoi(need("--init-threads"));
        } else if (a == "--no-online-timeoffset") {
            online_timeoffset = false;
        } else if (a == "--static-init") {
            static_init = true;
        } else if (a == "--dump-params") {
            dump_params = true;
        } else if (a == "--exposure") {
            exposure = std::stoi(need("--exposure"));
        } else if (a == "--gain") {
            gain = std::stoi(need("--gain"));
        } else if (a == "--health-min-features") {
            health_min_features = std::stoi(need("--health-min-features"));
        } else if (a == "--health-lost-after") {
            health_lost_after = std::stod(need("--health-lost-after"));
        } else if (a == "--reinit-on-recover") {
            reinit_on_recover = true;
        } else if (a == "--no-reinit-on-recover") {
            reinit_on_recover = false;
        } else if (a == "--tune-expected-rate") {
            tune_expected_rate = std::stod(need("--tune-expected-rate"));
        } else if (a == "--tune-blur-budget") {
            tune_blur_budget = std::stod(need("--tune-blur-budget"));
        } else if (a == "--tune-max-gain") {
            tune_max_gain = std::stoi(need("--tune-max-gain"));
        } else if (a == "--allow-blur") {
            tune_allow_blur = true;
        } else if (a == "--allow-dark") {
            tune_allow_dark = true;
        } else if (a == "--no-tune") {
            no_tune = true;
        } else if (a == "--view") {
            view = true;
        } else if (a == "--view-shot-prefix") {
            view_shot_prefix = need("--view-shot-prefix");
        } else if (a == "--auto-exposure") {
            auto_exposure = true;
        } else if (a == "--imu-to-body-rpy") {
            itb_rpy[0] = std::stod(need("--imu-to-body-rpy"));
            itb_rpy[1] = std::stod(need("--imu-to-body-rpy"));
            itb_rpy[2] = std::stod(need("--imu-to-body-rpy"));
        } else if (a == "--imu-to-body-t") {
            itb_t[0] = std::stod(need("--imu-to-body-t"));
            itb_t[1] = std::stod(need("--imu-to-body-t"));
            itb_t[2] = std::stod(need("--imu-to-body-t"));
        } else if (a == "--quiet") {
            quiet = true;
        } else if (a == "-h" || a == "--help") {
            printUsage(argv[0]);
            return 0;
        } else {
            std::fprintf(stderr, "未知参数: %s\n", a.c_str());
            printUsage(argv[0]);
            return 2;
        }
    }

    std::signal(SIGINT, onSigint);
    std::signal(SIGTERM, onSigint);

    // 1) VIO 估计器
    pure_uav::VioEstimator::Options vio_opt;
    // 失效保护：判定为 LOST 就停发位姿，恢复时（可选）重建并重锚定
    vio_opt.health.min_features = health_min_features;
    vio_opt.health.lost_after_s = health_lost_after;
    vio_opt.health.reinit_on_recover = reinit_on_recover;
    vio_opt.config_dir = config_dir;
    vio_opt.verbose = !quiet;
    vio_opt.estimate_camera_timeoffset = online_timeoffset;
    vio_opt.init_threads = init_threads;
    vio_opt.use_depth_init = use_depth_init;
    vio_opt.init_vel_zero = init_vel_zero;
    vio_opt.static_init = static_init;
    vio_opt.R_imu_to_body = rpyDegToMatrix(itb_rpy[0], itb_rpy[1], itb_rpy[2]);
    vio_opt.t_imu_body = Eigen::Vector3d(itb_t[0], itb_t[1], itb_t[2]);
    pure_uav::VioEstimator estimator(vio_opt);
    if (!estimator.ok()) {
        std::fprintf(stderr, "VioEstimator 初始化失败: %s\n", estimator.error().c_str());
        return 1;
    }
    if (static_init) {
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

    // 2) 飞控 MAVLink
    pure_uav::MavlinkLink::Config link_cfg;
    link_cfg.device = serial;
    link_cfg.baudrate = baud;
    link_cfg.highres_imu_interval_us = static_cast<int>(1e6 / imu_req_hz + 0.5);
    pure_uav::MavlinkLink link;
    std::string err;
    if (!link.start(link_cfg, &err)) {
        std::fprintf(stderr, "MAVLink 链路启动失败: %s\n", err.c_str());
        return 1;
    }
    std::printf("等待飞控心跳（%s @ %d）…\n", serial.c_str(), baud);
    for (int i = 0; i < 50 && !link.connected() && !g_stop.load(); ++i) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    if (!link.connected()) {
        std::fprintf(stderr, "[WARN] 5 秒内没收到飞控心跳，继续等待…\n");
        if (dump_params) {
            link.stop();
            return 1;
        }
    } else {
        std::printf("飞控已连接 sysid=%u\n", static_cast<unsigned>(link.fcuSysId()));
    }

    // --dump-params：只读查询，然后退出（不打开相机）
    if (dump_params) {
        // 关心的是"固件对 IMU 做了什么"：低通截止、陷波、采样率、以及日志带宽
        const char* names[] = {
            "INS_ACCEL_FILTER", "INS_GYRO_FILTER", "INS_FAST_SAMPLE",
            "INS_LOG_BAT_MASK", "INS_LOG_BAT_OPT", "INS_HNTCH_ENABLE",
            "INS_HNTCH_FREQ", "INS_HNTCH_BW", "INS_HNTC2_ENABLE",
            "SCHED_LOOP_RATE", "LOG_BITMASK", "LOG_DARM_MASK",
            "EK3_ENABLE", "AHRS_EKF_TYPE", "INS_USE", "LOG_FILE_DSRMROT",
            "LOG_DISARMED", "LOG_MAV_BUFFERS", "LOG_REPLAY", "INS_GYRO_RATE",
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
        link.stop();
        return 0;
    }

    // 3) 相机 + 帧队列
    std::mutex frame_mtx;
    std::deque<FrameItem> frame_q;
    constexpr size_t kMaxFrameQueue = 5;
    uint64_t dropped_frames = 0;

    pure_uav::OrbbecCamera camera;
    pure_uav::OrbbecCamera::Config cam_cfg;
    cam_cfg.width = width;
    cam_cfg.height = height;
    cam_cfg.fps = fps;
    cam_cfg.use_depth = enable_depth;
    cam_cfg.aggregate_mode = aggregate_mode;
    cam_cfg.exposure = exposure;
    cam_cfg.gain = gain;
    cam_cfg.auto_exposure = auto_exposure;
    cam_cfg.tune_expected_rate_dps = tune_expected_rate;
    cam_cfg.tune_blur_budget_px = tune_blur_budget;
    cam_cfg.tune_max_gain = tune_max_gain;
    cam_cfg.tune_allow_blur = tune_allow_blur;
    cam_cfg.tune_strict_blur = strict_blur;
    if (max_exposure_ms > 0) cam_cfg.tune_max_exposure_ms = max_exposure_ms;
    cam_cfg.tune_allow_dark = tune_allow_dark;
    if (no_tune) cam_cfg.auto_tune_exposure = false;

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
    std::atomic<float> live_mean{-1.0f};
    std::atomic<float> live_fast{-1.0f};

    std::deque<double> lat_dev_ms;    // 相机设备时间戳的帧间隔（毫秒）
    std::deque<double> lat_host_ms;   // 主机侧帧间隔（毫秒）—— 这就是喂给 VIO 的时间戳抖动
    std::deque<double> pose_age_ms;   // 帧到达→位姿发布的端到端延迟（毫秒）
    std::deque<double> vio_vs_ekf3_ms; // VIO 速度 − 飞控 EKF3 速度（mm/s）
    int depth_w = 0, depth_h = 0;
    double depth_valid_ratio = 0.0;
    double depth_median_mm = 0.0;
    // 【2026-10-23】深度年龄：彩色帧与深度帧的**设备时间戳**之差。
    // 外审指出 VioManager 接受 50 ms 以内的深度，而 33 ms 在 0.9 m/s、f=452、Z=2 m 时
    // 就是 15 px —— 必须实测到底有多少帧带着过期的深度。
    std::deque<double> depth_age_ms;
    long long n_frame_with_depth = 0, n_frame_total = 0;
    int64_t prev_host_ns = 0, prev_device_us = 0;
    auto frame_cb = [&](const cv::Mat& gray, const cv::Mat& depth_mm, int64_t host_ns, int64_t device_us,
                        int64_t depth_dev_us) {
        // --- 图像诊断（只在相机线程调用，不需要加锁）---
        {
            static cv::Ptr<cv::FastFeatureDetector> fast =
                cv::FastFeatureDetector::create(20, true);
            diag.frames++;
            diag.mean_sum += cv::mean(gray)[0];
            if (!depth_mm.empty()) {
                ++n_frame_with_depth;
                // 深度年龄 = 彩色设备时刻 − 深度设备时刻（同一台设备的时钟，可直接相减）
                if (depth_dev_us > 0 && device_us > 0) {
                    depth_age_ms.push_back(std::abs(static_cast<double>(device_us - depth_dev_us)) * 1e-3);
                    if (depth_age_ms.size() > 400) depth_age_ms.pop_front();
                }
            }
            ++n_frame_total;
            // 【2026-10-21】相机链路延迟统计：主机到达时刻 − 相机设备时间戳
            // = 曝光之后到我们能处理之间的全部延迟（MJPEG 编码 + USB 传输 + 解码 + 排队）。
            // VIO 把"主机到达时刻"当曝光时刻用，所以这条链的**抖动**直接就是时间戳噪声。
            // 相机时间戳和主机时钟**不在同一时间基准**（实测直接相减是 -13.8 万秒），
            // 所以不比绝对值，比**帧间增量**：如果链路只是固定延迟，
            // d(host) 应该恒等于 d(device)；两者之差就是链路引入的抖动。
            // 【2026-10-21】深度有效性统计：对齐后的深度图尺寸、有效像素占比、中位深度。
            // 如果"有效"接近 0 或者尺寸和彩色不一致，说明对齐/深度流没真正工作。
            if (!depth_mm.empty()) {
                depth_w = depth_mm.cols;
                depth_h = depth_mm.rows;
                if (diag.frames % 10 == 1) {
                    std::vector<unsigned short> vals;
                    vals.reserve(4096);
                    for (int r = 0; r < depth_mm.rows; r += 8) {
                        const unsigned short *row = depth_mm.ptr<unsigned short>(r);
                        for (int c = 0; c < depth_mm.cols; c += 8) {
                            const unsigned short v = row[c];
                            if (v != 0 && v != 0xFFFF) vals.push_back(v);
                        }
                    }
                    const size_t total = static_cast<size_t>((depth_mm.rows / 8 + 1) * (depth_mm.cols / 8 + 1));
                    depth_valid_ratio = total ? static_cast<double>(vals.size()) / total : 0.0;
                    if (!vals.empty()) {
                        std::nth_element(vals.begin(), vals.begin() + vals.size() / 2, vals.end());
                        depth_median_mm = vals[vals.size() / 2];
                    }
                }
            } else {
                depth_w = depth_h = 0;
            }
            if (device_us > 0) {
                if (prev_device_us > 0) {
                    const double d_host_us = static_cast<double>(host_ns - prev_host_ns) * 1e-3;
                    const double d_dev_us = static_cast<double>(device_us - prev_device_us);
                    lat_dev_ms.push_back(d_dev_us * 1e-3);
                    lat_host_ms.push_back(d_host_us * 1e-3);
                    if (lat_host_ms.size() > 600) {
                        lat_dev_ms.pop_front();
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
                // 稳健统计：CLOCK_REALTIME 会被 NTP 跳，秒级异常值必须剔除，
                // 否则 std 被一两个离群点主导，看不出链路真实抖动。
                double host_p50 = 0.0, host_p95 = 0.0, host_robust_std = 0.0;
                if (lat_host_ms.size() > 20) {
                    std::vector<double> v(lat_host_ms.begin(), lat_host_ms.end());
                    std::sort(v.begin(), v.end());
                    // 只取 0.5% ~ 99.5% 分位之间的样本（剔除停顿/时钟跳变）
                    const size_t lo = v.size() * 5 / 1000, hi = v.size() * 995 / 1000;
                    std::vector<double> t(v.begin() + lo, v.begin() + std::max(hi, lo + 1));
                    host_p50 = t[t.size() / 2];
                    host_p95 = t[t.size() * 95 / 100];
                    // 稳健 σ ≈ (p84 - p16)/2
                    host_robust_std = 0.5 * (t[t.size() * 84 / 100] - t[t.size() * 16 / 100]);
                }
                std::printf("[cam] %lld 帧/秒  mean=%.1f  FAST≈%.0f  深度 %dx%d 有效%.0f%% 中位%.2fm  "
                            "\u5e27\u95f4\u9694 p50=%.2f p95=%.2f ms  \u65f6\u95f4\u6233\u6296\u52a8(\u7a33\u5065\u03c3)=%.2f ms\n",
                            (long long)diag.frames, mean, fast_avg, depth_w, depth_h,
                            100.0 * depth_valid_ratio, depth_median_mm * 1e-3,
                            host_p50, host_p95, host_robust_std);
                if (!depth_age_ms.empty()) {
                    std::vector<double> v(depth_age_ms.begin(), depth_age_ms.end());
                    std::sort(v.begin(), v.end());
                    std::printf("[cam]   深度年龄 p50=%.1f p95=%.1f ms｜带深度的帧 %lld/%lld (%.0f%%)\n",
                                v[v.size() / 2], v[v.size() * 95 / 100],
                                n_frame_with_depth, n_frame_total,
                                n_frame_total ? 100.0 * n_frame_with_depth / n_frame_total : 0.0);
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
        if (!depth_mm.empty()) item.depth = depth_mm.clone();
        item.host_ns = host_ns;
        item.device_us = device_us;
        item.depth_dev_us = depth_dev_us;
        {
            std::lock_guard<std::mutex> lk(frame_mtx);
            if (frame_q.size() >= kMaxFrameQueue) {
                frame_q.pop_front();
                dropped_frames++;
            }
            frame_q.push_back(std::move(item));
        }
    };

    if (!camera.start(cam_cfg, frame_cb, &err)) {
        std::fprintf(stderr, "相机启动失败: %s\n", err.c_str());
        link.stop();
        return 1;
    }
    const auto cam_depth = camera.depthInfo();
    const double depth_t_m[3] = {cam_depth.t_mm[0] * 1e-3, cam_depth.t_mm[1] * 1e-3,
                                 cam_depth.t_mm[2] * 1e-3};
    if (use_depth_init && !cam_depth.valid()) {
        std::fprintf(stderr, "[WARN] 请求了深度初始化，但深度内参不可用，本轮不会注入深度\n");
    }

    // 4) 输出
    pure_uav::PoseLogger logger;
    pure_uav::PoseLogger safe_logger;
    if (!safe_tum_out.empty() && !safe_logger.open("", safe_tum_out)) {
        std::fprintf(stderr, "[WARN] 打不开发布轨迹 %s，跳过\n", safe_tum_out.c_str());
    }
    if (!logger.open(csv_out, tum_out)) {
        std::fprintf(stderr, "无法打开输出文件\n");
        camera.stop();
        link.stop();
        return 1;
    }

    // 4.1) 原始 IMU 落盘（可选）。只用于事后核对静止/运动，不参与估计。
    std::FILE* imu_log = nullptr;
    if (!imu_log_path.empty()) {
        imu_log = std::fopen(imu_log_path.c_str(), "w");
        if (imu_log) {
            std::fprintf(imu_log, "# t_us,ax,ay,az,gx,gy,gz\n");
        } else {
            std::fprintf(stderr, "[WARN] 打不开 IMU 日志 %s，跳过\n", imu_log_path.c_str());
        }
    }

    // 5) VIO 线程
    std::atomic<int64_t> stats_poses{0};
    std::atomic<int64_t> stats_blocked{0};
    std::atomic<int64_t> stats_timebase_resets{0};  // 飞控重启/串口重连导致的重新初始化次数
    std::atomic<long long> stats_imu_out_of_order{0};  // 重复/乱序被丢弃的 IMU 样本数
    std::atomic<double> cur_speed_mps{0.0};
    // 深度一致性近 2 s 均值（见状态行注释）
    long long last_deperr_n = 0;
    double last_deperr_sum = 0.0;
    double last_deperr_recent = -1.0;
    bool warned_deperr = false;
    bool was_blocked = false;
    std::atomic<int64_t> stats_sent{0};
    std::atomic<int64_t> stats_drop_no_clock{0};

    //===================================================================================
    // 【2026-10-21】数据集录制（--record DIR）
    // 目的：把实时过程存成和 datasets/sessionXX_vio 一样的格式，之后用 vio_offline 反复回放，
    //       调三角化/前端时不用再让人举着机器晃。写盘在独立线程，不阻塞估计。
    //===================================================================================
    std::mutex rec_mtx;
    // (gray, depth(mm), 飞控时间戳us) —— 深度随帧一起入队，避免用第二个队列产生竞态
    std::deque<std::tuple<cv::Mat, cv::Mat, uint64_t>> rec_q;
    std::atomic<long long> rec_depth_frames{0};
    std::atomic<bool> rec_stop{false};
    std::atomic<long long> rec_frames{0};
    std::FILE *rec_images_csv = nullptr;
    std::FILE *rec_imu_csv = nullptr;
    std::thread rec_thread;
    if (!rec_dir.empty()) {
        // 【2026-10-21】绝不覆盖已有录制：一次只录一份，录砸了也留着证据。
        // （之前一次误操作 rm -rf 把用户录好的 1337 帧删了，加这道闸防止再犯。）
        if (std::filesystem::exists(rec_dir + "/images.csv")) {
            std::fprintf(stderr,
                         "[rec] 目标目录里已经有录制：%s\n"
                         "      不覆盖。换一个 --record 名字再跑（例如 %s_2）。\n",
                         rec_dir.c_str(), rec_dir.c_str());
            return 2;
        }
        std::error_code ec;
        std::filesystem::create_directories(rec_dir + "/cam0", ec);
        std::filesystem::create_directories(rec_dir + "/depth", ec);
        rec_images_csv = std::fopen((rec_dir + "/images.csv").c_str(), "w");
        rec_imu_csv = std::fopen((rec_dir + "/imu.csv").c_str(), "w");
        if (!rec_images_csv || !rec_imu_csv) {
            std::fprintf(stderr, "[WARN] 录制文件打不开（%s），录制关闭\n", rec_dir.c_str());
            if (rec_images_csv) { std::fclose(rec_images_csv); rec_images_csv = nullptr; }
            if (rec_imu_csv) { std::fclose(rec_imu_csv); rec_imu_csv = nullptr; }
            rec_dir.clear();
        } else {
            std::fprintf(rec_images_csv, "timestamp_us,filename\n");
            std::fprintf(rec_imu_csv, "timestamp_us,gx,gy,gz,ax,ay,az\n");
            std::fflush(rec_images_csv);
            std::printf("[rec] 录制到 %s（cam0/*.png + depth/*.png + images.csv + imu.csv）\n",
                        rec_dir.c_str());
            // 深度元数据：回放时要用它把彩色像素投到深度图上取深度
            {
                const auto di = camera.depthInfo();
                std::FILE *dm = std::fopen((rec_dir + "/depth_meta.txt").c_str(), "w");
                if (dm) {
                    if (di.valid()) {
                        std::fprintf(dm, "valid=1\n");
                        std::fprintf(dm, "fx=%.6f\nfy=%.6f\ncx=%.6f\ncy=%.6f\n",
                                     di.fx, di.fy, di.cx, di.cy);
                        std::fprintf(dm, "width=%d\nheight=%d\n", di.width, di.height);
                        // 彩色→深度 的平移与旋转都存（旋转之前没人存过，回放只能按单位阵用）
                        std::fprintf(dm, "tx_mm=%.4f\nty_mm=%.4f\ntz_mm=%.4f\n",
                                     di.t_mm[0], di.t_mm[1], di.t_mm[2]);
                        std::fprintf(dm, "R_c2d=");
                        for (int k = 0; k < 9; ++k)
                            std::fprintf(dm, "%.9f%s", di.R_c2d[k], k == 8 ? "\n" : ",");
                        std::printf("[rec] 深度: %dx%d fx=%.2f cx=%.2f cy=%.2f（写入 depth_meta.txt）\n",
                                    di.width, di.height, di.fx, di.cx, di.cy);
                    } else {
                        std::fprintf(dm, "valid=0\n");
                        std::printf("[rec] 深度不可用（本轮只录了彩色）\n");
                    }
                    std::fclose(dm);
                }
            }
            std::fflush(stdout);
            rec_thread = std::thread([&]() {
                const std::vector<int> png_params{cv::IMWRITE_PNG_COMPRESSION, 1};
                while (true) {
                    std::tuple<cv::Mat, cv::Mat, uint64_t> item_frame;
                    bool has = false;
                    {
                        std::lock_guard<std::mutex> lk(rec_mtx);
                        if (!rec_q.empty()) {
                            item_frame = std::move(rec_q.front());
                            rec_q.pop_front();
                            has = true;
                        }
                    }
                    if (has) {
                        const cv::Mat &gray_item = std::get<0>(item_frame);
                        const cv::Mat &depth_item = std::get<1>(item_frame);
                        const uint64_t ts = std::get<2>(item_frame);
                        char name[64];
                        std::snprintf(name, sizeof(name), "%llu.png",
                                      static_cast<unsigned long long>(ts) * 1000ULL);
                        const std::string rel = std::string("cam0/") + name;
                        cv::imwrite(rec_dir + "/" + rel, gray_item, png_params);
                        std::fprintf(rec_images_csv, "%llu,%s\n",
                                     static_cast<unsigned long long>(ts), rel.c_str());
                        std::fflush(rec_images_csv);
                        // 【2026-10-21】深度也存下来（16 位 PNG，单位毫米），
                        // 否则没法离线验证"用深度初始化特征"这条改动。
                        if (!depth_item.empty()) {
                            const std::string drel = std::string("depth/") + name;
                            cv::imwrite(rec_dir + "/" + drel, depth_item, png_params);
                            rec_depth_frames.fetch_add(1);
                        }
                        rec_frames.fetch_add(1);
                    } else {
                        if (rec_stop.load()) break;
                        std::this_thread::sleep_for(std::chrono::milliseconds(2));
                    }
                }
            });
        }
    }

    std::thread vio_thread([&]() {
        pure_uav::FcuClock clock;
        std::optional<FrameItem> pending;
        double imu_ts_uni_us = -1.0;   // IMU 时间戳去抖状态（见 --imu-ts）
        uint64_t imu_prev_raw_us = 0;  // 上一个原始 IMU 时间戳，用来检测时间基跳变
        int64_t imu_prev_host_ns = 0;  // 上一个 IMU 的主机到达时刻（单调，不会跳）
        bool imu_stream_ok = false;    // IMU 时间戳流是否已稳定（预热闸门）
        int imu_warmup_left = 150;     // 预热窗口：丢掉启动后前 150 个 IMU 样本
        long long imu_skipped = 0;     // 被跳过的跳变/丢包样本数
        long long imu_warmup_total = 0; // 预热累计（用于日志）
        int64_t pending_since_ns = 0;
        uint64_t last_sent_ts = 0;
        // 相机时间戳：设备时钟 + 平滑偏移（见 5.3 的说明）
        std::deque<int64_t> cam_off_us;
        uint64_t prev_fed_us = 0;
        std::deque<double> fed_dt_ms;
        long long n_img_ts_from_device = 0, n_img_ts_fallback = 0;
        int64_t last_status_ns = steadyNowNs();
        uint64_t last_received_imu = link.receivedImu();

        // 查看器（只在 VIO 线程里调用，highgui 要求同一线程）
        pure_uav::LiveView viewer;
        if (view) {
            if (!viewer.open()) {
                std::fprintf(stderr, "[WARN] 实时窗口打不开，继续跑无界面模式\n");
            }
            if (!view_shot_prefix.empty()) viewer.setShotPrefix(view_shot_prefix);
        }
        cv::Mat last_gray;
        int64_t last_render_ns = 0;
        // fps / 更新率统计（都按滑窗算，避免把"刚启动"算成异常）
        std::deque<int64_t> frame_times;
        long long last_updates = 0;
        int64_t stats_win_start_ns = steadyNowNs();
        double update_rate = 0.0;
        long long imu_count_prev = static_cast<long long>(link.receivedImu());
        int64_t imu_hz_time_ns = steadyNowNs();
        double imu_hz = 0.0;
        bool imu_rate_warned = false;

        //===================================================================================
        // 【2026-10-29】飞控时间基变化的统一处理（飞控重启 / 串口重连 / 时间戳大幅回退）
        //
        // 验收要求：暂停发布 → 清旧队列、时钟映射、时间戳平滑状态 → 按新时间基重建估计器。
        // 只重置 FcuClock 是不够的：估计器内部还停在旧时间基上，下一帧相机时间会从
        // 303 s 掉到 1 s，OpenVINS 会按 302 秒的间隔做传播 → 直接发散。
        //===================================================================================
        uint64_t last_reconnect_count = link.reconnectCount();
        bool timebase_recovering = false;
        auto onFcuTimebaseReset = [&](const char* why) {
            timebase_recovering = true;
            clock.reset();                 // 主机↔飞控映射作废，重新累积样本
            link.clearImuQueue();          // 旧时间基的 IMU 不再喂进估计器
            {
                std::lock_guard<std::mutex> lk(frame_mtx);
                frame_q.clear();           // 旧时间基的相机帧同理
            }
            pending.reset();
            pending_since_ns = 0;
            prev_fed_us = 0;               // 图像时间戳平滑状态
            fed_dt_ms.clear();
            cam_off_us.clear();
            last_sent_ts = 0;              // 恢复后第一帧必须发出去
            pose_age_ms.clear();
            vio_vs_ekf3_ms.clear();
            imu_prev_raw_us = 0;           // IMU 去抖 + PLL 状态
            imu_prev_host_ns = 0;
            imu_ts_uni_us = -1.0;
            was_blocked = true;            // 让恢复时打印"恢复发送"
            estimator.resetForNewTimebase(why);
            stats_timebase_resets.fetch_add(1);
            std::printf("[VIO] ⏱ 飞控时间基变化（%s）：暂停发送，已清空 IMU/图像队列、"
                        "时钟映射和时间戳平滑状态，按新时间基重新初始化\n", why);
            std::fflush(stdout);
        };

        while (!g_stop.load()) {
            bool did_work = false;

            //===================================================================================
            // 【2026-10-25】零速更新按"飞控是否解锁"自动开关 —— 飞行中不需要任何人操作
            //
            // 为什么这么切：
            //   · 地面（未解锁）：静止时视觉塌了也不会加速度乱涨，零速更新直接把 v 钉 0
            //     （实测 turn4：路径 32.7 → 10.7 m、|v|max 5.92 → 2.7）。
            //   · 空中（已解锁）：悬停时画面几乎不动，容易被判成静止 → v 被硬钉 0、位置估计
            //     被冻住，飞控还以为没漂。这条路上宁可"停发位姿"（LOST 门限已经在做），
            //     也不要"发一个不动但其实在漂的位姿"。
            // 没收到心跳（纯手持、或 --no-send 演示）→ armed=false → 当作地面，行为不变。
            //===================================================================================
            {
                static int zupt_state = -1;  // -1 未定 / 0 关 / 1 开
                const int want = link.armed() ? 0 : 1;
                if (want != zupt_state) {
                    zupt_state = want;
                    estimator.setZuptEnabled(want == 1);
                    std::printf("[ZUPT] %s → %s零速更新\n",
                                want == 1 ? "地面待机（未解锁）" : "已解锁（空中）",
                                want == 1 ? "启用" : "关闭");
                    std::fflush(stdout);
                }
            }

            // 5.1 把队列里的 IMU 全部喂进去，同时更新主机↔飞控时钟
            pure_uav::MavlinkLink::TimedImu imu_item;
            while (link.popImu(imu_item)) {
                //=======================================================================
                // 【2026-10-21】IMU 流预热：等时间戳稳定了再开始喂
                // 实测启动后前 53 个样本带着上一轮的时间基，然后跳到真实 FCU 时钟（跳 249.6 秒）。
                // 这些样本必须整条丢掉：既不能进 FcuClock（会污染 host↔FCU 回归），
                // 也不能进估计器（OpenVINS 会试图在两帧之间传播 249 秒 → 直接发散）。
                // 判据：连续 40 个样本的原始间隔都在 3–8 ms 内，才算流稳定。
                //=======================================================================
                if (!imu_stream_ok) {
                    // 固定预热窗口：丢掉启动后前 0.75 秒（150 个样本）。
                    // 实测旧时间基块最多到第 53 个样本，丢 150 个确保跨过去；
                    // 不能用"连续 40 个 dt 都在 3–8 ms"做判据——真实流是 2.5/7.5 ms 交替，
                    // 那样永远凑不满（上一版就是这么把 IMU 全丢光的）。
                    imu_prev_raw_us = imu_item.imu.timestamp_us;
                    imu_prev_host_ns = imu_item.host_ns;
                    if (--imu_warmup_left > 0) { ++imu_warmup_total; continue; }
                    imu_stream_ok = true;
                    imu_ts_uni_us = static_cast<double>(imu_item.imu.timestamp_us);
                    std::printf("[imu] 预热完成（丢弃 %lld 个样本），开始喂 IMU\n", imu_warmup_total);
                    std::fflush(stdout);
                }
                // 串口重连也算一次时间基变化（新会话的 boot 时间很可能重新开始）
                if (link.reconnectCount() != last_reconnect_count) {
                    last_reconnect_count = link.reconnectCount();
                    onFcuTimebaseReset("串口重连");
                }
                const auto clock_res =
                    clock.addSample(imu_item.host_ns, imu_item.imu.timestamp_us);
                if (clock_res == pure_uav::FcuClock::AddResult::kTimebaseReset) {
                    // FcuClock 已清空旧样本；本帧作为新时间基的第一帧继续往下喂。
                    onFcuTimebaseReset("飞控重启/时间戳回退");
                } else if (clock_res == pure_uav::FcuClock::AddResult::kOutOfOrder) {
                    stats_imu_out_of_order.fetch_add(1);   // 乱序帧不喂 OpenVINS（会破坏时间递增）
                    continue;
                }
                //=======================================================================
                // 【2026-10-21】IMU 时间戳去抖（重要）
                // HIGHRES_IMU.time_usec 是 FCU 400 Hz 调度器的**发送时刻**，不是采样时刻，
                // 实测相邻间隔在 7.5 / 2.5 ms 之间交替。直接喂给 OpenVINS 等于每 5 ms 塞一个
                // ±2.5 ms 的时序错位。在 s29 上只改这一项（图像和数值完全相同）：
                //     原始 48.3 m / 更新 257 / |ba| 0.13
                //     加抖动 785.6 m / 更新  42 / |ba| 1.84   <- 和实时完全一致
                // 用"均匀网格 + 慢速 PLL 拉回"重建采样时刻：去掉快抖，长期速率和与相机的时间
                // 关系（经 FcuClock）保持不变。--imu-ts raw 可关掉做对照。
                //=======================================================================
                pure_uav::ImuSample imu_feed = imu_item.imu;
                if (imu_ts_uniform) {
                    // 【2026-10-23 实测 CUAV-7-Nano】请求 400 Hz 实际只给 ~200 Hz，而且
                    // 时间戳间隔变得更不整齐（2.68/7.30 交替）。所以标称步长必须跟着
                    // **实测**速率走：前 2 s 还没测出来时用请求值，之后用实测值。
                    // 请求值和实测差太多就明确报警，别让它默默把网格跑成 2 倍速。
                    double eff_hz = static_cast<double>(imu_req_hz);
                    if (imu_hz > 20.0) {
                        eff_hz = imu_hz;
                        if (!imu_rate_warned &&
                            std::abs(imu_hz - static_cast<double>(imu_req_hz)) > 0.15 * imu_req_hz) {
                            imu_rate_warned = true;
                            std::printf("[WARN] 请求 IMU %d Hz，实测只有 %.0f Hz —— 已自动按实测值"
                                        "重建时间戳网格。要用别的速率请改 --imu-hz %.0f\n",
                                        imu_req_hz, imu_hz, imu_hz);
                            std::fflush(stdout);
                        }
                    }
                    const double dt_nom_us = 1e6 / eff_hz;
                    const double raw = static_cast<double>(imu_item.imu.timestamp_us);
                    if (imu_ts_uni_us < 0.0 || imu_prev_raw_us == 0) {
                        // 首个样本：直接锚定
                        imu_ts_uni_us = raw;
                        imu_prev_raw_us = imu_item.imu.timestamp_us;
                        imu_prev_host_ns = imu_item.host_ns;
                    } else {
                        const double d_raw = raw - static_cast<double>(imu_prev_raw_us);
                        // 用**主机单调时钟**做跳变判据：host_ns 不会跳，FCU 的 time_usec 会跳
                        // （实测启动后第 9–53 个样本会跳 126–250 秒，而且那段旧时间基内部间隔
                        //   也是正常的 5 ms，只看 dt 抓不到）。两者不一致超过 50 ms 就是跳变。
                        const double d_host_us =
                            static_cast<double>(imu_item.host_ns - imu_prev_host_ns) * 1e-3;
                        const bool clock_jump = std::abs(d_host_us - d_raw) > 50000.0;
                        if (clock_jump || d_raw < 0.3 * dt_nom_us || d_raw > 3.0 * dt_nom_us) {
                            // 时间基跳变或丢包：**丢掉这一帧，但输出网格保持连续**（不跟着跳），
                            // 否则 OpenVINS 会试图在两帧之间传播几百秒，直接发散。
                            imu_prev_raw_us = imu_item.imu.timestamp_us;
                            imu_prev_host_ns = imu_item.host_ns;
                            ++imu_skipped;
                            continue;
                        }
                        imu_prev_raw_us = imu_item.imu.timestamp_us;
                        imu_prev_host_ns = imu_item.host_ns;
                        imu_ts_uni_us += dt_nom_us;                       // 均匀网格
                        imu_ts_uni_us += 0.03 * (raw - imu_ts_uni_us);    // 慢速 PLL 拉回，跟踪长期速率
                    }
                    imu_feed.timestamp_us = static_cast<uint64_t>(std::llround(imu_ts_uni_us));
                } else {
                    imu_prev_raw_us = imu_item.imu.timestamp_us;
                }
                estimator.feedImu(imu_feed);
                if (rec_imu_csv) {
                    // 录"实际喂给估计器的时间戳"（= 去抖后的），这样回放能 1:1 复现实时行为。
                    std::fprintf(rec_imu_csv, "%llu,%.9f,%.9f,%.9f,%.9f,%.9f,%.9f\n",
                                 static_cast<unsigned long long>(imu_feed.timestamp_us),
                                 imu_item.imu.gx, imu_item.imu.gy, imu_item.imu.gz,
                                 imu_item.imu.ax, imu_item.imu.ay, imu_item.imu.az);
                }
                if (imu_log) {
                    std::fprintf(imu_log, "%llu,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n",
                                 static_cast<unsigned long long>(imu_item.imu.timestamp_us),
                                 imu_item.imu.ax, imu_item.imu.ay, imu_item.imu.az,
                                 imu_item.imu.gx, imu_item.imu.gy, imu_item.imu.gz);
                }
                did_work = true;
            }

            // 5.2 取一帧相机图（只取一帧，保持与 IMU 的顺序关系）
            if (!pending) {
                std::lock_guard<std::mutex> lk(frame_mtx);
                if (!frame_q.empty()) {
                    pending = std::move(frame_q.front());
                    frame_q.pop_front();
                    pending_since_ns = steadyNowNs();
                }
            }

            // 5.3 处理图像
            if (pending) {
                uint64_t fcu_us = 0;
                if (!clock.valid() || !clock.hostToFcuUs(pending->host_ns, fcu_us)) {
                    if (steadyNowNs() - pending_since_ns > 2000000000LL) {
                        pending.reset();
                        stats_drop_no_clock.fetch_add(1);
                    }
                } else {
                    //=================================================================
                    // 【2026-10-23】相机时间戳改用**设备时钟**。
                    //
                    // 以前 fcu_us 来自 hostToFcuUs(pending->host_ns)——即"主机收到这帧的时刻"。
                    // 主机调度/解码排队会带来抖动：30 fps 下 2.0 ms；640x480@60 + 深度
                    // （COLOR_FRAME_REQUIRE）实测 14.2 ms，而帧周期只有 16 ms —— 等于时间戳废掉。
                    //
                    // 现在用：t_img = device_us + 平滑偏移，其中
                    //   offset_i = fcu_at_arrival_i − device_us_i
                    // 偏移里只剩"链路+排队延迟"（慢变），取近 2 s 中位数压掉抖动。
                    // 设备时钟与 FCU 时钟的晶振漂移由这个滑动中位数跟踪。
                    //=================================================================
                    if (cam_ts_device && pending->device_us > 0) {
                        const int64_t off_i = static_cast<int64_t>(fcu_us) - pending->device_us;
                        cam_off_us.push_back(off_i);
                        while (cam_off_us.size() > 90) cam_off_us.pop_front();
                        std::vector<int64_t> tmp(cam_off_us.begin(), cam_off_us.end());
                        std::nth_element(tmp.begin(), tmp.begin() + tmp.size() / 2, tmp.end());
                        const int64_t off_med = tmp[tmp.size() / 2];
                        const uint64_t t_dev = static_cast<uint64_t>(pending->device_us + off_med);
                        // 合理性护栏：与主机换算值相差超过 200 ms 说明偏移估计还没收敛或时钟跳变
                        const int64_t dev_minus_host = static_cast<int64_t>(t_dev) - static_cast<int64_t>(fcu_us);
                        if (std::llabs(dev_minus_host) < 200000) {
                            fcu_us = t_dev;
                            ++n_img_ts_from_device;
                        } else {
                            ++n_img_ts_fallback;
                        }
                        // 记录喂给 VIO 的时间戳间隔，用于和主机侧抖动对比
                        if (prev_fed_us > 0) {
                            const double d = static_cast<double>(fcu_us - prev_fed_us);
                            if (d > 0 && d < 200000) {
                                fed_dt_ms.push_back(d * 1e-3);
                                if (fed_dt_ms.size() > 400) fed_dt_ms.pop_front();
                            }
                        }
                        prev_fed_us = fcu_us;
                    }
                    const uint64_t last_imu_us = link.lastImuTimestampUs();
                    const int64_t waited_ns = steadyNowNs() - pending_since_ns;
                    // 等一个时间戳晚于图像的 IMU（保证传播能覆盖到图像时刻），最多等 100ms
                    if (last_imu_us > fcu_us || waited_ns > 100000000LL) {
                        // 【2026-10-25】深度用它自己的时间戳，不再冒充彩色时间戳。
                        // 两个设备时间戳同源，所以(color_fcu - color_dev) 的偏移对深度同样成立。
                        uint64_t depth_fcu_us = fcu_us;
                        if (pending->depth_dev_us > 0 && pending->device_us > 0) {
                            const int64_t off = static_cast<int64_t>(fcu_us) - pending->device_us;
                            const int64_t d = static_cast<int64_t>(pending->depth_dev_us) + off;
                            if (d > 0 && std::llabs(d - static_cast<int64_t>(fcu_us)) < 200000)
                                depth_fcu_us = static_cast<uint64_t>(d);
                        }
                        if (use_depth_init && !pending->depth.empty() && cam_depth.valid())
                            estimator.setDepthImage(pending->depth, depth_fcu_us, cam_depth.fx,
                                                    cam_depth.fy, cam_depth.cx, cam_depth.cy,
                                                    cam_depth.R_c2d, depth_t_m);
                        estimator.feedImage(pending->gray, fcu_us);
                        if (rec_images_csv) {
                            std::lock_guard<std::mutex> lk(rec_mtx);
                            if (rec_q.size() < 300) rec_q.emplace_back(pending->gray, pending->depth, fcu_us);
                        }
                        const int64_t frame_host_ns = pending->host_ns;  // 位姿年龄统计用
                        last_gray = pending->gray;  // 浅拷贝（引用计数），给查看器画
                        pending.reset();
                        frame_times.push_back(steadyNowNs());

                        pure_uav::VisionPose pose;
                        if (estimator.initialized() && estimator.getLatestPose(pose) &&
                            pose.timestamp_us != last_sent_ts) {
                            last_sent_ts = pose.timestamp_us;
                            //=======================================================
                            // 【2026-10-23】量两件事（纯诊断，不改变任何行为）：
                            //
                            // ① 位姿年龄：从"相机帧到达主机"到"我们现在才发布位姿"
                            //    的端到端延迟。时间同步再准，如果发布的是 150 ms 前的
                            //    位置，控制照样差（1 m/s 就是 15 cm，60°/s 就是 9°）。
                            // ② EKF3 第二意见：飞控自己也在估速度，和我们的速度持续
                            //    背离就说明 VIO 发散了 —— 不需要真值就能抓"不报警地错"。
                            //=======================================================
                            pose_age_ms.push_back(static_cast<double>(steadyNowNs() - frame_host_ns) * 1e-6);
                            if (pose_age_ms.size() > 600) pose_age_ms.pop_front();
                            const auto ekf3 = link.ekf3Velocity();
                            if (ekf3.valid) {
                                const double vio_sp = std::sqrt(static_cast<double>(pose.vx) * pose.vx +
                                                                static_cast<double>(pose.vy) * pose.vy +
                                                                static_cast<double>(pose.vz) * pose.vz);
                                const double fcu_sp = std::sqrt(ekf3.vx * ekf3.vx + ekf3.vy * ekf3.vy +
                                                                ekf3.vz * ekf3.vz);
                                vio_vs_ekf3_ms.push_back((vio_sp - fcu_sp) * 1e3); // 单位 mm/s
                                if (vio_vs_ekf3_ms.size() > 600) vio_vs_ekf3_ms.pop_front();
                            }
                            logger.log(pose);
                            stats_poses.fetch_add(1);
                            // 【2026-10-23】操作员要能**当场**看见自己走多快：
                            // 平移上限 = 0.21×Z m/s（Z 是场景中位深度），超了就是糊。
                            // 以前只能事后分析，ver2 就是这么白跑的。
                            {
                                const double sp = std::sqrt(static_cast<double>(pose.vx) * pose.vx +
                                                            static_cast<double>(pose.vy) * pose.vy +
                                                            static_cast<double>(pose.vz) * pose.vz);
                                cur_speed_mps.store(sp);
                            }
                            // 【失效保护】LOST 期间 / 重建估计器期间一律不发。
                            // 飞控侧（PX4 EKF2_EV_TIMEOUT / ArduPilot EK3 源超时）会自己
                            // 退回姿态+气压，这就是"切姿态模式"的落地方式。
                            // 链路断开 / 时间基重建期间一律不发（sendVisionPose 也挡，
                            // 但在这里挡住才能统计和打日志）
                            // --no-send 模式下不拿链路状态做门（没有飞控只是演示），
                            // 但健康门限照旧生效。
                            const bool gate_ok =
                                !send || (link.connected() && !timebase_recovering);
                            if (!estimator.publishable() || !gate_ok) {
                                stats_blocked.fetch_add(1);
                                if (!was_blocked) {
                                    was_blocked = true;
                                    if (!estimator.publishable()) {
                                        const auto h = estimator.healthReport();
                                        std::printf("[VIO] ⛔ 视觉失效（%s）：特征 %d，距上次更新 %.2fs"
                                                    " → 停止发送位姿\n",
                                                    pure_uav::toString(h.state), h.features,
                                                    h.since_update_s);
                                    } else {
                                        std::printf("[VIO] ⏸ 暂停发送位姿：%s\n",
                                                    !link.connected() ? "飞控链路未连接"
                                                                      : "飞控时间基重建中");
                                    }
                                    std::fflush(stdout);
                                }
                            } else {
                                if (was_blocked) {
                                    was_blocked = false;
                                    const auto h = estimator.healthReport();
                                    std::printf("[VIO] ✅ 恢复发送位姿（健康 %s，特征 %d，重建 %d 次）\n",
                                                pure_uav::toString(h.state), h.features, h.recoveries);
                                    std::fflush(stdout);
                                }
                                safe_logger.log(pose);   // 只记"健康门限通过"的位姿，给 RViz 看实际输出
                                // 【2026-10-22】把可信度、重建次数、协方差一起发出去：
                                //   OK -> 质量 90 / 位置 σ 0.15 m / 速度 σ 0.25 m/s
                                //   DEGRADED -> 质量 50 / 位置 σ 0.5 m / 速度 σ 0.8 m/s
                                //   reset_counter 用估计器重建次数，飞控据此处理重锚定跳变
                                const auto hs = estimator.healthState();
                                const int q = (hs == pure_uav::VioHealthState::OK) ? 90 : 50;
                                const float ps = (hs == pure_uav::VioHealthState::OK) ? 0.15f : 0.5f;
                                const float vs = (hs == pure_uav::VioHealthState::OK) ? 0.25f : 0.8f;
                                const uint8_t rc =
                                    static_cast<uint8_t>(std::min<long long>(estimator.rebuildCount(), 255));
                                if (send && link.sendVisionPose(pose, q, rc, ps, vs)) {
                                    stats_sent.fetch_add(1);
                                }
                            }
                        }
                    }
                }
            }

            // 【2026-10-29】时间基重建完成：估计器重新初始化 + 时钟恢复 → 解除暂停
            if (timebase_recovering && estimator.initialized() && clock.valid()) {
                timebase_recovering = false;
                std::printf("[VIO] ✅ 已按新时间基重新初始化（时钟样本 %zu）→ 恢复发送位姿\n",
                            clock.sampleCount());
                std::fflush(stdout);
            }

            // 5.4 状态打印
            const int64_t now = steadyNowNs();
            if (!quiet && now - last_status_ns > 2000000000LL) {
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
                // 【2026-10-23】端到端位姿年龄 与 VIO/EKF3 速度差（都是纯诊断）
                double age_p50 = 0.0, age_p95 = 0.0, dvel_med = 0.0;
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
                //===================================================================
                // 【2026-10-23】深度一致性（近 2 s）：depth 初始化出来的特征，投影到
                // 别的帧里偏离多少像素。这是**发散前最早出现的信号**：
                //   ver1（成功）全程 2-4 px；ver2（发散）在 6-9 s 就升到 56 px，
                //   而速度斜坡到 11 s 才出现。放进状态行，就能当场看出这一趟已经废了。
                //===================================================================
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
                    dep_recent = last_deperr_recent;   // 没有新样本就沿用上一次
                }
                if (dep_recent >= 0.0) last_deperr_recent = dep_recent;
                last_deperr_n = dn;
                last_deperr_sum = dsum;
                if (dep_recent > 20.0 && !warned_deperr) {
                    warned_deperr = true;
                    std::printf("[VIO] ⛔ 深度一致性崩塌（%.0f px，正常 2-4 px）→ 特征跟踪已错配，"
                                "这趟数据基本废了，尽快停下重录\n", dep_recent);
                    std::fflush(stdout);
                }
                // 速度上限：0.21×Z（模糊预算 3 px、曝光见启动行；Z 用最近一次 [cam] 的中位深度）
                char spdBuf[40];
                {
                    const double v = cur_speed_mps.load();
                    const double z = depth_median_mm * 1e-3;   // 最近一次 [cam] 的滑动中位深度
                    if (z > 0.2) {
                        const double vlim = 0.21 * z;
                        std::snprintf(spdBuf, sizeof(spdBuf), "%.2f/%.2f", v, vlim);
                    } else {
                        std::snprintf(spdBuf, sizeof(spdBuf), "%.2f/-", v);
                    }
                }
                // 喂给 VIO 的相机时间戳抖动（和 [cam] 里那个"主机侧抖动"对比，
                // 后者是改动前喂的东西；前者才是现在真正进估计器的噪声）
                char tsjBuf[24];
                {
                    if (fed_dt_ms.size() > 20) {
                        std::vector<double> v(fed_dt_ms.begin(), fed_dt_ms.end());
                        std::sort(v.begin(), v.end());
                        const double p50 = v[v.size() / 2];
                        const double p84 = v[v.size() * 84 / 100], p16 = v[v.size() * 16 / 100];
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
                // 【2026-10-28】把"实际发出去的 ODOMETRY 元数据"也打出来：
                // 联调时它就是飞控 VISP 日志里应该看到的那两个数（质量、重置计数），
                // 两边对不上就说明链路里有人在改字段。
                const int qual_now =
                    (estimator.healthState() == pure_uav::VioHealthState::OK) ? 90 : 50;
                const int rc_now =
                    static_cast<int>(std::min<long long>(estimator.rebuildCount(), 255));
                std::printf(
                    "[VIO] init=%d img=%d pose=%lld sent=%lld blocked=%lld health=%s "
                    "qual=%d rc=%d feat=%d tilt=%s lin=%.2f tri=%.3f depthused=%lld deperr=%s v=%sm/s tsjit=%sms imu=%llu(+%llu/2s) q=%zu clock=%.1fms "
                    "age=%.0f/%.0fms dvel=%+.0fmm/s\n",
                    estimator.initialized() ? 1 : 0, estimator.imageCount(),
                    static_cast<long long>(stats_poses.load()),
                    static_cast<long long>(stats_sent.load()),
                    static_cast<long long>(stats_blocked.load()),
                    pure_uav::toString(estimator.healthState()),
                    qual_now, rc_now,
                    estimator.healthReport().features,
                    tiltStr.c_str(),
                    estimator.linAccelDev(),
                    estimator.triangulationRate(),
                    static_cast<long long>(estimator.depthUsed()),
                    depBuf, spdBuf, tsjBuf,
                    static_cast<unsigned long long>(rx),
                    static_cast<unsigned long long>(rx - last_received_imu),
                    link.imuQueueSize(), off_ms, age_p50, age_p95, dvel_med);
                std::fflush(stdout);
                last_received_imu = rx;
            }

            // 5.45 实时查看器（限速 ~33 Hz；渲染慢只会掉显示帧，不阻塞估计）
            if (view && now - last_render_ns > 30000000LL) {
                last_render_ns = now;

                // 相机帧率（最近 2 秒）
                while (!frame_times.empty() && now - frame_times.front() > 2000000000LL) {
                    frame_times.pop_front();
                }
                double fps = 0.0;
                if (frame_times.size() >= 2) {
                    fps = (frame_times.size() - 1) * 1e9 /
                          static_cast<double>(frame_times.back() - frame_times.front());
                }

                // IMU 频率（每 2 秒更新一次）
                if (now - imu_hz_time_ns > 2000000000LL) {
                    const long long cnt = static_cast<long long>(link.receivedImu());
                    imu_hz =
                        (cnt - imu_count_prev) * 1e9 / static_cast<double>(now - imu_hz_time_ns);
                    imu_count_prev = cnt;
                    imu_hz_time_ns = now;
                }

                // MSCKF 更新率（累计计数差分 / 2 秒）
                {
                    long long c = 0, f = 0, r = 0, u = 0, rows = 0, cd = 0, td = 0;
                    estimator.msckfStats(c, f, r, u, rows, cd, td);
                    if (u < last_updates) last_updates = u;  // 重建估计器后计数器归零
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
                st.fps = fps;
                st.imu_hz = imu_hz;
                st.lin_acc = static_cast<float>(estimator.linAccelDev());
                st.tri_rate = static_cast<float>(estimator.triangulationRate());
                st.recording = send;
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
                if (mean_now >= 0.0f) {
                    char nb[128];
                    std::snprintf(nb, sizeof(nb), "mean %.0f  fast %.0f",
                                  static_cast<double>(mean_now), static_cast<double>(fast_now));
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

    // 6) 主线程等待退出
    const auto t0 = std::chrono::steady_clock::now();
    while (!g_stop.load()) {
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
        if (!camera.running()) {
            std::fprintf(stderr, "[WARN] 相机线程已停止（掉线/异常），退出\n");
            break;
        }
        if (duration > 0.0) {
            const double elapsed =
                std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
            if (elapsed >= duration) break;
        }
    }
    g_stop.store(true);

    vio_thread.join();
    camera.stop();
    link.stop();
    logger.close();
    safe_logger.close();
    if (imu_log) {
        std::fclose(imu_log);
        std::printf("  IMU 日志      : %s\n", imu_log_path.c_str());
    }
    if (!rec_dir.empty()) {
        rec_stop.store(true);
        if (rec_thread.joinable()) rec_thread.join();
        if (rec_images_csv) std::fclose(rec_images_csv);
        if (rec_imu_csv) std::fclose(rec_imu_csv);
        std::printf("  录制数据集    : %s（%lld 帧）\n", rec_dir.c_str(),
                    static_cast<long long>(rec_frames.load()));
    }

    std::printf("\n结束:\n");
    std::printf("  图像帧数     : %d\n", estimator.imageCount());
    std::printf("  VisionPose   : %lld\n", static_cast<long long>(stats_poses.load()));
    std::printf("  已发送        : %lld\n", static_cast<long long>(stats_sent.load()));
    std::printf("  丢帧(无时钟)  : %lld\n", static_cast<long long>(stats_drop_no_clock.load()));
    std::printf("  时间基重置    : %lld（飞控重启/串口重连后重新初始化）\n",
                static_cast<long long>(stats_timebase_resets.load()));
    std::printf("  IMU 乱序丢弃  : %lld\n", static_cast<long long>(stats_imu_out_of_order.load()));
    std::printf("  相机队列丢帧  : %llu\n", static_cast<unsigned long long>(dropped_frames));
    std::printf("  IMU 丢帧      : %llu\n", static_cast<unsigned long long>(link.droppedImu()));
    std::printf("  CSV           : %s\n", csv_out.c_str());
    std::printf("  TUM           : %s\n", tum_out.c_str());
    if (!safe_tum_out.empty()) std::printf("  发布轨迹 TUM  : %s\n", safe_tum_out.c_str());
    return 0;
}
