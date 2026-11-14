// vio_live 命令行：集中维护默认值、帮助和解析；不启动硬件或估计器。
#pragma once

#include <cstdio>
#include <cstdlib>
#include <exception>
#include <stdexcept>
#include <string>

#include "camera_frame.hpp"

namespace pure_uav::live {

struct Options {
    // 飞控、相机服务和标定。
    std::string serial = "/dev/ttyACM0";
    int baud = 921600;
    std::string config_dir = "config/gemini_1920x1080";
    int width = 1920;
    int height = 1080;
    int fps = 30;
    std::string shm_vio = camera::kVioChannel;
    int shm_timeout_ms = 5000;

    // 位姿日志、原始 IMU 日志和离线数据集。
    std::string csv_out = "out/live_pose.csv";
    std::string tum_out = "out/live_traj.tum";
    std::string safe_tum_out;  // 仅记录串口发送成功的位姿。
    std::string imu_log_path;
    std::string rec_dir;

    // 时间戳处理与深度输入。
    bool imu_ts_uniform = true;
    int imu_req_hz = 200;
    bool enable_depth = true;
    bool cam_ts_device = false;  // device 模式仍平滑 SDK systemTimeStampUs。
    double duration = 0.0;
    bool send = true;
    bool quiet = false;

    // 估计器开关：保留调试时使用的所有对照组合。
    bool online_timeoffset = true;
    int init_threads = 0;  // 0 使用配置文件；1 用于可复现回放。
    bool use_depth_init = false;
    bool init_vel_zero = false;
    bool static_init = false;
    bool dump_params = false;
    bool view = false;
    std::string view_shot_prefix;

    // 健康门限与 IMU 到机体的外参。
    int health_min_features = 60;
    double health_lost_after = 1.5;
    bool reinit_on_recover = true;
    double itb_rpy[3] = {0.0, 0.0, 0.0};
    double itb_t[3] = {0.0, 0.0, 0.0};
};

enum class ParseResult { kRun, kHelp, kError };

inline void printUsage(const char* argv0) {
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
        "  --safe-tum-out PATH 只写串口发送成功的位姿（--no-send 时为空），给 RViz 看用\n"
        "  --imu-log PATH      把原始 IMU 存成 CSV（t_us,ax,ay,az,gx,gy,gz）\n"
        "  --imu-ts MODE       IMU 时间戳处理：uniform(默认，去抖) | raw(原样喂)\n"
        "  --shm-vio NAME      camera_server VIO 通道（默认 /pure_uav_camera_vio）\n"
        "  --shm-timeout-ms N  等待相机服务的超时（默认 5000 ms）\n"
        "  --cam-ts MODE      相机时间：host(默认，服务端采集时刻) | device(SDK 系统时间平滑)\n"
        "  --no-depth         消费端忽略深度图；服务端采集配置由 camera.yaml 管理\n"
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
        "  曝光/增益/聚合/测光参数请在 camera_server 的 config/camera.yaml 中设置。\n"
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

// 保留既有参数顺序、校验、报错和退出行为。
inline ParseResult parseOptions(int argc, char** argv, Options& opt) {
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--exposure" || a == "--gain" || a == "--auto-exposure" || a == "--aggregate" || a == "--no-tune" || a == "--strict-blur" || a == "--max-exposure-ms" || a == "--tune-expected-rate" || a == "--tune-blur-budget" || a == "--tune-max-gain" || a == "--allow-blur" ||
            a == "--allow-dark") {
            std::fprintf(stderr, "%s 由 camera_server 管理，请在相机服务 config/camera.yaml 中设置并重启服务。\n", a.c_str());
            return ParseResult::kError;
        }
        auto need = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "参数 %s 缺少值\n", name);
                std::exit(2);
            }
            return argv[++i];
        };
        if (a == "--serial") {
            opt.serial = need("--serial");
        } else if (a == "--baud") {
            opt.baud = std::stoi(need("--baud"));
        } else if (a == "--config-dir") {
            opt.config_dir = need("--config-dir");
        } else if (a == "--shm-vio") {
            opt.shm_vio = need("--shm-vio");
        } else if (a == "--shm-timeout-ms") {
            const auto value = need("--shm-timeout-ms");
            try {
                size_t used = 0;
                opt.shm_timeout_ms = std::stoi(value, &used);
                if (used != value.size() || opt.shm_timeout_ms < 0 || opt.shm_timeout_ms > 60000) {
                    throw std::invalid_argument("timeout");
                }
            } catch (const std::exception&) {
                std::fprintf(stderr, "--shm-timeout-ms 只接受 0..60000\n");
                return ParseResult::kError;
            }
        } else if (a == "--width") {
            opt.width = std::stoi(need("--width"));
        } else if (a == "--height") {
            opt.height = std::stoi(need("--height"));
        } else if (a == "--fps") {
            opt.fps = std::stoi(need("--fps"));
        } else if (a == "--csv-out") {
            opt.csv_out = need("--csv-out");
        } else if (a == "--tum-out") {
            opt.tum_out = need("--tum-out");
        } else if (a == "--safe-tum-out") {
            opt.safe_tum_out = need("--safe-tum-out");
        } else if (a == "--imu-log") {
            opt.imu_log_path = need("--imu-log");
        } else if (a == "--record") {
            opt.rec_dir = need("--record");
        } else if (a == "--cam-ts") {
            const std::string v = need("--cam-ts");
            if (v == "host") {
                opt.cam_ts_device = false;
            } else if (v == "device") {
                opt.cam_ts_device = true;
            } else {
                std::fprintf(stderr, "--cam-ts 只接受 host|device\n");
                return ParseResult::kError;
            }
        } else if (a == "--no-depth") {
            opt.enable_depth = false;
        } else if (a == "--imu-hz") {
            opt.imu_req_hz = std::stoi(need("--imu-hz"));
            if (opt.imu_req_hz < 50 || opt.imu_req_hz > 1000) {
                std::fprintf(stderr, "--imu-hz 只接受 50..1000\n");
                return ParseResult::kError;
            }
        } else if (a == "--imu-ts") {
            const std::string v = need("--imu-ts");
            if (v == "raw") {
                opt.imu_ts_uniform = false;
            } else if (v == "uniform") {
                opt.imu_ts_uniform = true;
            } else {
                std::fprintf(stderr, "--imu-ts 只接受 raw 或 uniform\n");
                return ParseResult::kError;
            }
        } else if (a == "--duration") {
            opt.duration = std::stod(need("--duration"));
        } else if (a == "--no-send") {
            opt.send = false;
        } else if (a == "--online-timeoffset") {
            opt.online_timeoffset = true;
        } else if (a == "--init-vel-zero") {
            opt.init_vel_zero = true;
        } else if (a == "--use-depth-init") {
            opt.use_depth_init = true;
        } else if (a == "--init-threads") {
            opt.init_threads = std::stoi(need("--init-threads"));
        } else if (a == "--no-online-timeoffset") {
            opt.online_timeoffset = false;
        } else if (a == "--static-init") {
            opt.static_init = true;
        } else if (a == "--dump-params") {
            opt.dump_params = true;
        } else if (a == "--health-min-features") {
            opt.health_min_features = std::stoi(need("--health-min-features"));
        } else if (a == "--health-lost-after") {
            opt.health_lost_after = std::stod(need("--health-lost-after"));
        } else if (a == "--reinit-on-recover") {
            opt.reinit_on_recover = true;
        } else if (a == "--no-reinit-on-recover") {
            opt.reinit_on_recover = false;
        } else if (a == "--view") {
            opt.view = true;
        } else if (a == "--view-shot-prefix") {
            opt.view_shot_prefix = need("--view-shot-prefix");
        } else if (a == "--imu-to-body-rpy") {
            opt.itb_rpy[0] = std::stod(need("--imu-to-body-rpy"));
            opt.itb_rpy[1] = std::stod(need("--imu-to-body-rpy"));
            opt.itb_rpy[2] = std::stod(need("--imu-to-body-rpy"));
        } else if (a == "--imu-to-body-t") {
            opt.itb_t[0] = std::stod(need("--imu-to-body-t"));
            opt.itb_t[1] = std::stod(need("--imu-to-body-t"));
            opt.itb_t[2] = std::stod(need("--imu-to-body-t"));
        } else if (a == "--quiet") {
            opt.quiet = true;
        } else if (a == "-h" || a == "--help") {
            printUsage(argv[0]);
            return ParseResult::kHelp;
        } else {
            std::fprintf(stderr, "未知参数: %s\n", a.c_str());
            printUsage(argv[0]);
            return ParseResult::kError;
        }
    }

    if (opt.use_depth_init && !opt.enable_depth) {
        std::fprintf(stderr, "--use-depth-init 不能与 --no-depth 同时使用\n");
        return ParseResult::kError;
    }
    return ParseResult::kRun;
}

}  // namespace pure_uav::live
