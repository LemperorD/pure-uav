// 离线回放：dataset(images.csv + imu.csv) → OpenVINS → VisionPose (CSV/TUM)
//
// 用法示例：
//   ./vio_offline --dataset ~/calib/datasets/session05 \
//                 --config-dir config/session05_1920x1080 \
//                 --csv-out ~/calib/vio_online/out/session05_pose.csv \
//                 --tum-out ~/calib/vio_online/out/session05_traj.tum
//
// 说明：离线时时间戳沿用数据集（session05 是 Unix 微秒），只验证估计器和
// 坐标系转换；在线运行时同样是这套转换，只是时间戳换成飞控 boot 微秒。

#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iomanip>
#include <string>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/imgcodecs.hpp>

#include "offline_dataset.hpp"
#include "pure_uav/pose_logger.hpp"
#include "pure_uav/vio_estimator.hpp"
#include "pure_uav/vision_pose.hpp"

namespace {

void printUsage(const char* argv0) {
    std::printf(
        "用法: %s --dataset DIR --config-dir DIR [选项]\n"
        "\n"
        "必需:\n"
        "  --dataset DIR        含 images.csv / imu.csv / images/ 的数据集目录\n"
        "  --config-dir DIR     含 estimator_config.yaml 的配置目录\n"
        "\n"
        "可选:\n"
        "  --estimator NAME     estimator 配置文件名（默认 estimator_config.yaml）\n"
        "  --csv-out PATH       VisionPose CSV 输出（默认 <dataset>/vio_pose.csv）\n"
        "  --tum-out PATH       TUM 轨迹输出（默认 <dataset>/vio_traj.tum）\n"
        "  --raw-tum-out PATH   OpenVINS 原始状态 TUM（默认 <dataset>/ov_raw.tum）\n"
        "  --imu-to-body-rpy R P Y  R_ItoB：把 IMU 系向量转到机体系（ZYX 欧拉角，度）\n"
        "  --imu-to-body-t X Y Z    机体系原点在 IMU 系下的坐标（米）\n"
        "  --start-sec S        从第一帧图像后 S 秒开始（默认 0）\n"
        "  --max-images N       最多处理 N 帧图像（默认全部）\n"
        "  --quiet              少打印\n"
        "  --safe-tum-out PATH  只把判定为可发布的位姿写成 TUM（量化失效保护的效果）\n"
        "  --health-min-features N  跟踪特征数低于 N 算不足（默认 60）\n"
        "  --health-lost-after S    断供超过 S 秒判 LOST（默认 1.5）\n"
        "  --reinit-on-recover      LOST 后重建估计器并重锚定（默认开）\n"
        "  --no-reinit-on-recover   关掉重建\n"
        "  --use-depth-init         用深度初始化特征位置（数据集需含 depth/ + depth_meta.txt）\n"
        "  --init-vel-zero          静止起步时把初始速度强制置 0（治本的关键）\n"
        "  --static-init            静态初始化（要求开头静止 2 秒；重力方向比动态准 5-10 倍）\n"
        "  --no-online-timeoffset   关闭在线 cam-IMU 时延估计（默认开，与实时一致）\n"
        "  --init-threads N         动态初始化 MLE 线程数（默认用配置文件 6；\n"
        "                           **做 A/B 对照必须用 1**，否则多线程会让运行不可复现）\n"
        "  -h, --help\n",
        argv0);
}

std::string defaultOut(const std::string& dir, const char* name) {
    if (dir.empty()) return name;
    if (dir.back() == '/') return dir + name;
    return dir + "/" + name;
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
    std::string dataset_dir;
    std::string config_dir = "config/session05_1920x1080";
    std::string estimator = "estimator_config.yaml";
    std::string csv_out;
    std::string tum_out;
    std::string raw_tum_out;
    std::string safe_tum_out;
    double start_sec = 0.0;
    int max_images = 0;
    bool quiet = false;
    // 失效保护参数
    int health_min_features = 60;
    double health_lost_after = 1.5;
    bool reinit_on_recover = true;   // 见 vio_health.hpp 的说明
    bool online_timeoffset = true;   // 与实时保持一致（实时默认开）
    bool use_depth_init = false;      // 用深度初始化特征（数据集里有 depth/ 才有效）
    bool static_init = false;         // 静态初始化（开头站着不动，重力方向更准）
    bool init_vel_zero = false;       // 静止起步：把初始速度强制置 0
    int init_threads = 0;            // 0 = 用配置文件；A/B 对照用 1
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
        if (a == "--dataset") {
            dataset_dir = need("--dataset");
        } else if (a == "--config-dir") {
            config_dir = need("--config-dir");
        } else if (a == "--estimator") {
            estimator = need("--estimator");
        } else if (a == "--csv-out") {
            csv_out = need("--csv-out");
        } else if (a == "--tum-out") {
            tum_out = need("--tum-out");
        } else if (a == "--raw-tum-out") {
            raw_tum_out = need("--raw-tum-out");
        } else if (a == "--start-sec") {
            start_sec = std::stod(need("--start-sec"));
        } else if (a == "--max-images") {
            max_images = std::stoi(need("--max-images"));
        } else if (a == "--imu-to-body-rpy") {
            itb_rpy[0] = std::stod(need("--imu-to-body-rpy"));
            itb_rpy[1] = std::stod(need("--imu-to-body-rpy"));
            itb_rpy[2] = std::stod(need("--imu-to-body-rpy"));
        } else if (a == "--imu-to-body-t") {
            itb_t[0] = std::stod(need("--imu-to-body-t"));
            itb_t[1] = std::stod(need("--imu-to-body-t"));
            itb_t[2] = std::stod(need("--imu-to-body-t"));
        } else if (a == "--safe-tum-out") {
            safe_tum_out = need("--safe-tum-out");
        } else if (a == "--health-min-features") {
            health_min_features = std::stoi(need("--health-min-features"));
        } else if (a == "--health-lost-after") {
            health_lost_after = std::stod(need("--health-lost-after"));
        } else if (a == "--reinit-on-recover") {
            reinit_on_recover = true;
        } else if (a == "--no-reinit-on-recover") {
            reinit_on_recover = false;
        } else if (a == "--init-vel-zero") {
            init_vel_zero = true;
        } else if (a == "--static-init") {
            static_init = true;
        } else if (a == "--use-depth-init") {
            use_depth_init = true;
        } else if (a == "--no-online-timeoffset") {
            online_timeoffset = false;
        } else if (a == "--online-timeoffset") {
            online_timeoffset = true;
        } else if (a == "--init-threads") {
            init_threads = std::stoi(need("--init-threads"));
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

    if (dataset_dir.empty()) {
        std::fprintf(stderr, "缺少 --dataset\n");
        printUsage(argv[0]);
        return 2;
    }
    if (csv_out.empty()) csv_out = defaultOut(dataset_dir, "vio_pose.csv");
    if (tum_out.empty()) tum_out = defaultOut(dataset_dir, "vio_traj.tum");
    if (raw_tum_out.empty()) raw_tum_out = defaultOut(dataset_dir, "ov_raw.tum");

    pure_uav::OfflineDataset dataset;
    std::string err;
    if (!dataset.load(dataset_dir, &err)) {
        std::fprintf(stderr, "读取数据集失败: %s\n", err.c_str());
        return 1;
    }

    const auto& images = dataset.images();
    const auto& imu = dataset.imu();
    std::printf("数据集: %s\n", dataset.dir().c_str());
    std::printf("  图像: %zu 帧 [%.6f, %.6f] s\n", images.size(),
                static_cast<double>(images.front().timestamp_us) * 1e-6,
                static_cast<double>(images.back().timestamp_us) * 1e-6);
    std::printf("  IMU : %zu 帧（%.1f Hz）\n", imu.size(),
                (imu.size() > 1)
                    ? 1e6 * (imu.size() - 1) /
                          static_cast<double>(imu.back().timestamp_us - imu.front().timestamp_us)
                    : 0.0);

    pure_uav::VioEstimator::Options opt;
    // 失效保护：判定为 LOST 就停发位姿，恢复时（可选）重建并重锚定
    opt.health.min_features = health_min_features;
    opt.health.lost_after_s = health_lost_after;
    opt.health.reinit_on_recover = reinit_on_recover;
    opt.estimate_camera_timeoffset = online_timeoffset;
    opt.init_threads = init_threads;
    opt.use_depth_init = use_depth_init;
    opt.static_init = static_init;
    opt.init_vel_zero = init_vel_zero;
    opt.config_dir = config_dir;
    opt.estimator_config = estimator;
    opt.verbose = !quiet;
    opt.R_imu_to_body = rpyDegToMatrix(itb_rpy[0], itb_rpy[1], itb_rpy[2]);
    opt.t_imu_body = Eigen::Vector3d(itb_t[0], itb_t[1], itb_t[2]);

    // 深度元数据（录制时写进数据集的）
    bool depth_meta_valid = false;
    double d_fx = 0, d_fy = 0, d_cx = 0, d_cy = 0, d_tx = 0, d_ty = 0, d_tz = 0;
    double d_R[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};   // 彩色→深度 旋转（行主序）
    bool d_R_valid = false;
    {
        std::ifstream dm(dataset_dir + "/depth_meta.txt");
        std::string line;
        int valid = 0;
        while (std::getline(dm, line)) {
            auto kv = [&](const char *k) -> double {
                const std::string key = std::string(k) + "=";
                if (line.rfind(key, 0) == 0) return std::atof(line.c_str() + key.size());
                return 0.0;
            };
            if (line.rfind("valid=", 0) == 0) valid = std::atoi(line.c_str() + 6);
            if (line.rfind("fx=", 0) == 0) d_fx = kv("fx");
            if (line.rfind("fy=", 0) == 0) d_fy = kv("fy");
            if (line.rfind("cx=", 0) == 0) d_cx = kv("cx");
            if (line.rfind("cy=", 0) == 0) d_cy = kv("cy");
            if (line.rfind("tx_mm=", 0) == 0) d_tx = kv("tx_mm");
            if (line.rfind("ty_mm=", 0) == 0) d_ty = kv("ty_mm");
            if (line.rfind("tz_mm=", 0) == 0) d_tz = kv("tz_mm");
            if (line.rfind("R_c2d=", 0) == 0) {
                const std::string val = line.substr(6);
                size_t pos = 0;
                for (int k = 0; k < 9; ++k) {
                    const size_t comma = val.find(',', pos);
                    const std::string tok = val.substr(pos, comma - pos);
                    d_R[k] = std::atof(tok.c_str());
                    if (comma == std::string::npos) break;
                    pos = comma + 1;
                }
                d_R_valid = true;
            }
        }
        depth_meta_valid = (valid == 1 && d_fx > 0.0);
        if (use_depth_init) {
            std::printf("深度元数据: %s", depth_meta_valid ? "有" : "无");
            if (depth_meta_valid)
                std::printf("（fx=%.2f fy=%.2f cx=%.2f cy=%.2f t=[%.2f %.2f %.2f]mm 旋转=%s）", d_fx, d_fy, d_cx, d_cy, d_tx, d_ty, d_tz, d_R_valid ? "有" : "单位阵");
            std::printf("\n");
        }
    }

    pure_uav::VioEstimator est(opt);
    if (!est.ok()) {
        std::fprintf(stderr, "VioEstimator 初始化失败: %s\n", est.error().c_str());
        return 1;
    }

    pure_uav::PoseLogger logger;
    if (!logger.open(csv_out, tum_out)) {
        std::fprintf(stderr, "无法打开输出文件\n");
        return 1;
    }

    // 原始 OpenVINS 状态（global 系，JPL 四元数 x y z w），用于和 ROS1 参考轨迹对照
    std::ofstream raw_tum(raw_tum_out, std::ios::out | std::ios::trunc);
    if (raw_tum.is_open()) {
        raw_tum << "# timestamp tx ty tz qx qy qz qw bax bay baz bgx bgy bgz\n";
    }

    const uint64_t t_start =
        images.front().timestamp_us +
        static_cast<uint64_t>(std::llround(start_sec * 1e6));

    size_t imu_idx = 0;
    int processed = 0;
    int logged = 0;
    uint64_t last_logged_ts = 0;
    double sum_vio_ms = 0.0;

    struct HealthRow {
        int sec;
        double feat;
        int upd;
        int frames;
        const char* state;
    };
    std::vector<HealthRow> health_log;
    const double t_first_sec = images.empty() ? 0.0
                                              : static_cast<double>(images.front().timestamp_us) * 1e-6;
    int health_sec = -1;
    int health_frames = 0;
    int health_upd = 0;
    long long health_blocked = 0;
    double health_feat_sum = 0.0;
    pure_uav::VioHealthState health_worst = pure_uav::VioHealthState::OK;

    std::ofstream safe_tum;
    uint64_t last_safe_ts = 0;
    int safe_count = 0;
    if (!safe_tum_out.empty()) {
        safe_tum.open(safe_tum_out);
        if (!safe_tum.is_open()) {
            std::fprintf(stderr, "无法打开 safe TUM: %s\n", safe_tum_out.c_str());
            return 1;
        }
    }

    for (size_t k = 0; k < images.size(); ++k) {
        const auto& img = images[k];
        if (img.timestamp_us < t_start) {
            // 还没到开始时刻：只把 IMU 喂进去，让 OpenVINS 有足够初始化窗口
            while (imu_idx < imu.size() && imu[imu_idx].timestamp_us <= img.timestamp_us) {
                est.feedImu(imu[imu_idx]);
                imu_idx++;
            }
            continue;
        }
        if (max_images > 0 && processed >= max_images) break;

        while (imu_idx < imu.size() && imu[imu_idx].timestamp_us <= img.timestamp_us) {
            est.feedImu(imu[imu_idx]);
            imu_idx++;
        }

        cv::Mat gray = cv::imread(img.path, cv::IMREAD_GRAYSCALE);
        if (gray.empty()) {
            std::fprintf(stderr, "[WARN] 读图失败: %s\n", img.path.c_str());
            continue;
        }

        // 【2026-10-21】有深度就一并喂进去：路径把 cam0/ 换成 depth/
        if (use_depth_init && depth_meta_valid) {
            std::string dpath = img.path;
            const std::string key = "/cam0/";
            const auto pos = dpath.rfind(key);
            if (pos != std::string::npos) {
                dpath.replace(pos, key.size(), "/depth/");
                cv::Mat depth_mm = cv::imread(dpath, cv::IMREAD_UNCHANGED);
                if (!depth_mm.empty() && depth_mm.type() == CV_16UC1) {
                    // 老录制只记了平移（旋转按单位阵处理：这台设备两相机基本同向）
                    const double t_m[3] = {d_tx * 1e-3, d_ty * 1e-3, d_tz * 1e-3};
                    // 有记录旋转就用，没有（老录制）按单位阵
                    est.setDepthImage(depth_mm, img.timestamp_us, d_fx, d_fy, d_cx, d_cy,
                                      d_R_valid ? d_R : nullptr, t_m);
                }
            }
        }

        const auto t0 = std::chrono::steady_clock::now();
        const bool fed = est.feedImage(gray, img.timestamp_us);
        const auto t1 = std::chrono::steady_clock::now();
        const double ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
        sum_vio_ms += ms;
        processed++;

        // 只把"会真的发给飞控"的位姿写进 safe TUM，用来量化失效保护的实际效果
        if (safe_tum.is_open() && est.initialized() && est.publishable()) {
            pure_uav::VisionPose sp;
            if (est.getLatestPose(sp) && sp.timestamp_us != last_safe_ts) {
                last_safe_ts = sp.timestamp_us;
                const double tsec = static_cast<double>(sp.timestamp_us) * 1e-6;
                safe_tum << std::fixed << std::setprecision(6) << tsec << ' '
                         << sp.x << ' ' << sp.y << ' ' << sp.z << ' '
                         << sp.q[0] << ' ' << sp.q[1] << ' ' << sp.q[2] << ' ' << sp.q[3] << '\n';
                ++safe_count;
            }
        }

        if (fed && est.initialized()) {
            pure_uav::VisionPose pose;
            if (est.getLatestPose(pose) && pose.timestamp_us != last_logged_ts) {
                last_logged_ts = pose.timestamp_us;
                logger.log(pose);
                logged++;

                pure_uav::VioEstimator::RawState raw;
                if (raw_tum.is_open() && est.getLatestRawState(raw)) {
                    raw_tum << std::fixed << std::setprecision(6)
                            << raw.timestamp_sec << ' '
                            << raw.p_IinG.x() << ' ' << raw.p_IinG.y() << ' ' << raw.p_IinG.z() << ' '
                            << raw.q_GtoI(0) << ' ' << raw.q_GtoI(1) << ' ' << raw.q_GtoI(2) << ' '
                            << raw.q_GtoI(3) << ' '
                            << std::setprecision(9)
                            << raw.bias_a.x() << ' ' << raw.bias_a.y() << ' ' << raw.bias_a.z() << ' '
                            << raw.bias_g.x() << ' ' << raw.bias_g.y() << ' ' << raw.bias_g.z()
                            << '\n';
                }
            }
        }

        // ---- 健康时间线：按秒汇总，用来验证失效判定准不准 ----
        {
            const double t_rel = static_cast<double>(img.timestamp_us) * 1e-6 - t_first_sec;
            const auto h = est.healthReport();
            const int sec = static_cast<int>(std::floor(t_rel));
            if (sec != health_sec) {
                if (health_sec >= 0 && health_frames > 0) {
                    health_log.push_back({health_sec, health_feat_sum / health_frames,
                                          health_upd, health_frames,
                                          pure_uav::toString(health_worst)});
                }
                health_sec = sec;
                health_frames = 0;
                health_feat_sum = 0;
                health_upd = 0;
                health_worst = pure_uav::VioHealthState::OK;
            }
            ++health_frames;
            health_feat_sum += h.features;
            if (h.updated_this_frame) ++health_upd;
            if (static_cast<int>(h.state) > static_cast<int>(health_worst)) health_worst = h.state;
            if (!h.publishable) ++health_blocked;
        }

        if (!quiet && (processed % 30 == 0 || k + 1 == images.size())) {
            std::printf("[%zu/%zu] img_ts=%.6f init=%d pose=%d vio=%.1f ms\n", k + 1,
                        images.size(), static_cast<double>(img.timestamp_us) * 1e-6,
                        est.initialized() ? 1 : 0, logged, ms);
            std::fflush(stdout);
        }
    }

    if (health_sec >= 0 && health_frames > 0) {
        health_log.push_back({health_sec, health_feat_sum / health_frames, health_upd,
                              health_frames, pure_uav::toString(health_worst)});
    }
    logger.close();

    std::printf("\n完成:\n");
    std::printf("  处理图像      : %d\n", processed);
    std::printf("  VisionPose    : %d\n", logged);
    std::printf("  VIO 平均耗时  : %.2f ms/帧\n",
                processed > 0 ? sum_vio_ms / processed : 0.0);
    if (est.initialized()) {
        std::printf("  初始化时刻    : %.6f s\n", est.initializedTimeSec());
    } else {
        std::printf("  初始化时刻    : 未初始化（检查运动激励/配置）\n");
    }
    // MSCKF 更新诊断：区分"没特征可用" vs "有特征但被卡方门限全拒"
    long long m_calls = 0, m_in = 0, m_rej = 0, m_upd = 0, m_rows = 0;
    long long m_clean = 0, m_tri = 0;
    est.msckfStats(m_calls, m_in, m_rej, m_upd, m_rows, m_clean, m_tri);
    if (m_calls > 0) {
        const double rej_pct = 100.0 * static_cast<double>(m_rej) / static_cast<double>(m_in);
        std::printf("  MSCKF 更新    : 调用 %lld 次｜特征进 %lld｜观测<2 丢 %lld（%.1f%%）"
                    "｜三角化失败丢 %lld（%.1f%%）｜卡方拒 %lld（%.1f%%）"
                    "｜实际更新 %lld 次，残差 %lld 行\n",
                    m_calls, m_in, m_clean, 100.0 * m_clean / m_in,
                    m_tri, 100.0 * m_tri / m_in, m_rej, rej_pct, m_upd, m_rows);
        if (rej_pct > 60.0) {
            std::printf("  ⚠️ 超过 60%% 的特征被门限拒掉 —— 视觉约束基本没进滤波器，"
                        "这时轨迹等于纯 IMU 积分\n");
        } else if (m_upd == 0) {
            std::printf("  ⚠️ 一次更新都没执行 —— 检查特征三角化/max_clone\n");
        }
    }
    // ---- 失效保护时间线 ----
    if (!health_log.empty()) {
        int blocked_sec = 0;
        for (const auto& r : health_log) {
            if (std::string(r.state) != "OK" && std::string(r.state) != "DEGRADED") ++blocked_sec;
        }
        std::printf("  失效保护      : %d/%zu 秒判定为 LOST（占比 %.0f%%）｜被拦下的位姿 %lld 个\n",
                    blocked_sec, health_log.size(),
                    health_log.empty() ? 0.0 : 100.0 * blocked_sec / health_log.size(),
                    health_blocked);
        std::printf("  逐秒状态      :\n");
        for (const auto& r : health_log) {
            const int bar = static_cast<int>(std::min(40.0, r.feat / 12.5));
            std::printf("    %3ds  %-8s 特征 %4.0f  更新 %2d/%2d  %s\n", r.sec, r.state, r.feat,
                        r.upd, r.frames, std::string(std::max(0, bar), '#').c_str());
        }
    }
    if (!safe_tum_out.empty()) {
        std::printf("  安全轨迹      : %s（%d 帧，只含判定为可发布的位姿）\n",
                    safe_tum_out.c_str(), safe_count);
    }
    std::printf("  用深度初始化特征: %lld 个%s\n", (long long)est.depthUsed(),
                use_depth_init ? "" : "（开关未开）");
    {
        long long dn = 0;
        double dmean = 0.0;
        long long dh[6] = {0, 0, 0, 0, 0, 0};
        est.depthErrStats(dn, dmean, dh);
        if (dn > 0) {
            std::printf("  深度一致性  : %lld 个特征，中位重投影误差 平均 %.2f px\n", dn, dmean);
            std::printf("                分布 <1px:%lld  <2px:%lld  <5px:%lld  <10px:%lld  <25px:%lld  >=25px:%lld\n",
                        dh[0], dh[1], dh[2], dh[3], dh[4], dh[5]);
        }
    }
    std::printf("  CSV           : %s\n", csv_out.c_str());
    std::printf("  TUM           : %s\n", tum_out.c_str());
    std::printf("  原始状态 TUM  : %s\n", raw_tum_out.c_str());
    return (logged > 0) ? 0 : 1;
}
