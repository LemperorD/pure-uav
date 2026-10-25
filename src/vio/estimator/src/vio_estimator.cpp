#include "pure_uav/vio_estimator.hpp"

#include <algorithm>
#include <cmath>
#include <cstdarg>
#include <cstdio>
#include <deque>
#include <mutex>

#include "core/VioManager.h"
#include "core/VioManagerOptions.h"
#include "state/State.h"
#include "types/IMU.h"
#include "utils/opencv_yaml_parse.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

namespace pure_uav {

namespace {

std::string joinPath(const std::string& dir, const std::string& file) {
    if (file.empty()) return file;
    if (file.front() == '/') return file;
    if (dir.empty()) return file;
    if (dir.back() == '/') return dir + file;
    return dir + "/" + file;
}

}  // namespace

struct VioEstimator::Impl {
    Options opt;
    std::unique_ptr<ov_msckf::VioManager> vio;
    std::unique_ptr<VisionPoseConvert> converter;

    bool launched = false;  // OpenVINS 是否已初始化
    bool latched = false;   // LOCAL_FRD 原点是否已锁存
    bool ok = false;
    std::string error;

    Eigen::Vector3d last_gyro = Eigen::Vector3d::Zero();  // IMU 系，rad/s（未扣零偏）
    Eigen::Vector3d last_accel = Eigen::Vector3d::Zero(); // IMU 系，m/s²（比力，未扣零偏）
    // 【2026-10-21】运动有效性自检：最近 2 s 的比力模长，用来算 |a|-g 的中位。
    // 单目 VIO 靠平移拿视差；实测原地转是 0.015~0.071、走路是 0.156~0.211。
    std::deque<double> acc_norm_hist;
    std::deque<double> gyr_norm_hist;   // 【2026-10-23】初始化静止判定用

    int images = 0;
    long long poses = 0;
    double track_frequency = 20.0;  // 从配置读取，用于复刻 ROS 节点的降频逻辑
    double last_cam_time = -1.0;

    mutable std::mutex pose_mutex;
    VisionPose latest;
    bool has_latest = false;
    RawState raw;
    bool has_raw = false;

    double init_time = -1.0;

    // ---- 失效保护 / 恢复 ----
    ov_msckf::VioManagerOptions params;   // 保存一份，恢复时用它重建 VioManager
    bool has_params = false;
    HealthMonitor health;
    bool recovery_armed = false;          // 进入过 LOST，等待重建
    int segment_id = 0;                  // 当前初始化段编号（每次重建 +1，用于把指标按段分组）
    int inflate_count = 0;                // 本次 LOST 里已经放大过几次协方差
    double last_inflate_time = -1e9;      // 上次放大的时刻
    double overspeed_since = -1.0;        // 速度超物理上限的起始时刻（<0 表示当前正常）
    double lost_since = -1.0;             // LOST 起点
    double last_rebuild_time = -1.0;      // 上次重建时间
    double last_still_remind = -1e9;      // "先停稳再恢复"提示的节流
    int rebuildCount = 0;
    struct Anchor {
        bool set = false;
        double yaw = 0.0;                 // 旧 LOCAL_FRD 相对新 LOCAL_FRD 的偏航
        Eigen::Vector3d t = Eigen::Vector3d::Zero();
    } anchor;
    VisionPose last_published;
    bool has_last_published = false;

    // 可视化：当前帧跟踪点（每次 feedImage 后刷新，只在 VIO 线程读）
    std::vector<VioEstimator::TrackedPoint> tracks;

    void log(const char* fmt, ...) const;
    void logAlways(const char* fmt, ...) const;
    VisionPose applyAnchor(const VisionPose& in) const;
};

namespace {

// 只取绕 z 的偏航角（LOCAL_FRD 是 z 向下，绕 z 的旋转即航向）
double yawOf(const Eigen::Quaterniond& q) {
    const Eigen::Matrix3d R = q.toRotationMatrix();
    return std::atan2(R(1, 0), R(0, 0));
}

}  // namespace

// 恢复后重锚定：新估计器的 LOCAL_FRD 原点/航向是重建那一刻重新锁存的，
// 直接发出去会跳。用「上一次发布过的位置 + 航向」把新轨迹整体搬过去，
// 只保留平移和偏航（横滚/俯仰由重力对齐保证，不需要也没法搬）。
VisionPose VioEstimator::Impl::applyAnchor(const VisionPose& in) const {
    if (!anchor.set) return in;
    VisionPose out = in;
    const Eigen::AngleAxisd rz(anchor.yaw, Eigen::Vector3d::UnitZ());
    const Eigen::Vector3d p_in(in.x, in.y, in.z);
    const Eigen::Vector3d p = rz * p_in + anchor.t;
    const Eigen::Quaterniond q_in(in.q[0], in.q[1], in.q[2], in.q[3]);
    Eigen::Quaterniond q = Eigen::Quaterniond(rz) * q_in;
    q.normalize();
    out.x = static_cast<float>(p.x());
    out.y = static_cast<float>(p.y());
    out.z = static_cast<float>(p.z());
    out.q[0] = static_cast<float>(q.w());
    out.q[1] = static_cast<float>(q.x());
    out.q[2] = static_cast<float>(q.y());
    out.q[3] = static_cast<float>(q.z());
    return out;
}

// 【2026-10-21】关键事件（初始化完成 / 重建 / 自适应恢复）必须无条件打印。
// 原来这些走 log()，被 --quiet 吞掉 —— 排查时看不到"重建了几次、为什么重建"，
// 只能事后从诊断 CSV 的 t_rel 回退去猜（我就是这么误判 clean1 是连续 88 秒的）。
void VioEstimator::Impl::logAlways(const char* fmt, ...) const {
    va_list args;
    va_start(args, fmt);
    std::vprintf(fmt, args);
    va_end(args);
    std::fflush(stdout);
}

void VioEstimator::Impl::log(const char* fmt, ...) const {
    if (!opt.verbose) return;
    va_list args;
    va_start(args, fmt);
    std::vprintf(fmt, args);
    va_end(args);
    std::fflush(stdout);
}

VioEstimator::VioEstimator(const Options& options) : impl_(new Impl()) {
    impl_->opt = options;

    const std::string config_path = joinPath(options.config_dir, options.estimator_config);

    try {
        auto parser = std::make_shared<ov_core::YamlParser>(config_path);

        std::string verbosity = "INFO";
        parser->parse_config("verbosity", verbosity, false);
        ov_core::Printer::setPrintLevel(verbosity);

        ov_msckf::VioManagerOptions params;
        params.print_and_load(parser);
        if (options.init_threads > 0) {
            params.init_options.init_dyn_mle_max_threads = options.init_threads;
            impl_->log("[VIO] 动态初始化线程数覆盖为 %d（可复现性）\n", options.init_threads);
        }
        params.use_multi_threading_subs = false;
        if (options.estimate_camera_timeoffset) {
            params.state_options.do_calib_camera_timeoffset = true;
            impl_->log("[VIO] 在线估计 cam-IMU 时移：开启（初值 %.4f s）\n",
                       params.calib_camimu_dt);
        }
        // 初始化方式：动态（估计器自己找激励窗口）vs 静态（要求先静止 init_window_time）。
        // 静态初始化直接由静止段的加速度计均值定重力方向，倾角误差比动态小一个量级；
        // "--static-init" 就是用它做对照实验，配置本身不动。
        if (options.static_init) {
            params.init_options.init_dyn_use = false;
            impl_->log("[VIO] 初始化方式：静态（要求先静止 %.1f s；倾角来自静止段加速度计均值）\n",
                       params.init_options.init_window_time);
        } else {
            impl_->log("[VIO] 初始化方式：动态（配置 init_dyn_use=%d）\n",
                       static_cast<int>(params.init_options.init_dyn_use));
        }

        if (!parser->successful()) {
            impl_->error = "OpenVINS 配置解析失败（缺少必需参数）: " + config_path;
            return;
        }

        PoseConvertOptions conv_opt;
        conv_opt.R_imu_to_body = options.R_imu_to_body;
        conv_opt.t_imu_body = options.t_imu_body;
        impl_->converter = std::make_unique<VisionPoseConvert>(conv_opt);

        impl_->track_frequency = params.track_frequency;
        impl_->params = params;
        impl_->has_params = true;
        impl_->vio = std::make_unique<ov_msckf::VioManager>(params);
        impl_->vio->set_use_depth_init(options.use_depth_init);
        impl_->vio->set_init_zero_velocity(options.init_vel_zero);
        if (options.use_depth_init)
            impl_->log("[VIO] 使用深度初始化特征（use_depth_init）\n");
        impl_->health.configure(options.health);
        impl_->ok = true;

        impl_->log("[VIO] config: %s\n", config_path.c_str());
        impl_->log("[VIO] OpenVINS 核心已就绪（ROS-free）\n");
    } catch (const std::exception& e) {
        impl_->error = std::string("VIO 初始化异常: ") + e.what();
    } catch (...) {
        impl_->error = "VIO 初始化异常（未知类型）";
    }
}

VioEstimator::~VioEstimator() = default;

bool VioEstimator::ok() const { return impl_ && impl_->ok; }
const std::string& VioEstimator::error() const { return impl_->error; }

void VioEstimator::feedImu(const ImuSample& imu) {
    if (!impl_->ok) return;

    ov_core::ImuData msg;
    msg.timestamp = static_cast<double>(imu.timestamp_us) * 1e-6;
    msg.wm << imu.gx, imu.gy, imu.gz;
    msg.am << imu.ax, imu.ay, imu.az;

    impl_->last_gyro = msg.wm;
    impl_->last_accel = msg.am;
    impl_->acc_norm_hist.push_back(msg.am.norm());
    impl_->gyr_norm_hist.push_back(msg.wm.norm()); // 初始化静止判定用
    while (impl_->acc_norm_hist.size() > 400) impl_->acc_norm_hist.pop_front(); // 2 s @200Hz
    while (impl_->gyr_norm_hist.size() > 400) impl_->gyr_norm_hist.pop_front();
    impl_->vio->feed_measurement_imu(msg);
}

// 倾角自检：把滤波器"自己认为的重力方向"和加速度计实测的重力方向比一比。
//
// 为什么这个数重要（2026-10-21 实时实测）：
//   滤波器算加速度是 a = R·(f − b_a) + g，其中 g 是按它自己估计的姿态扣的。
//   姿态错 θ 度，重力就有 g·sinθ 没被扣掉，被当成真实加速度积分进速度。
//   实测：θ 从 2° 漂到 53°，而速度斜坡从 0.3 m/s² 涨到 6 m/s²，两者对到 5% 以内。
//   所以 θ 是"这条轨迹还能不能信"的直接指标 —— 1° 就已经能造出 0.17 m/s² 的假加速度。
//
// 只在准静止时刻算：此时加速度计量的就是重力。
VioEstimator::TiltCheck VioEstimator::tiltCheck() const {
    TiltCheck out;
    if (!impl_ || !impl_->ok) return out;
    const double g = 9.81;
    if (impl_->last_accel.norm() < 1e-6) return out;

    // 准静止判据：角速度小、比力模长接近 g
    if (impl_->last_gyro.norm() > 1.5 * M_PI / 180.0) return out;
    if (std::abs(impl_->last_accel.norm() - g) > 0.15) return out;

    VisionPose pose;
    {
        std::lock_guard<std::mutex> lk(impl_->pose_mutex);
        if (!impl_->has_latest) return out;
        pose = impl_->latest;
    }
    const Eigen::Quaterniond q(pose.q[0], pose.q[1], pose.q[2], pose.q[3]);  // w x y z
    const Eigen::Matrix3d R = q.toRotationMatrix();          // body -> LOCAL_FRD
    const Eigen::Vector3d down_est = R.transpose() * Eigen::Vector3d(0, 0, 1);
    const Eigen::Vector3d down_meas = -impl_->last_accel.normalized();
    const double c = std::max(-1.0, std::min(1.0, down_est.dot(down_meas)));
    out.deg = std::acos(c) * 180.0 / M_PI;
    out.valid = true;
    return out;
}


void VioEstimator::setDepthImage(const cv::Mat& depth_mm, uint64_t timestamp_us, double fx,
                                 double fy, double cx, double cy, const double R_c2d[9],
                                 const double t_c2d_m[3]) {
    if (!impl_ || !impl_->ok || !impl_->vio) return;
    if (depth_mm.empty()) return;
    impl_->vio->set_depth_image(depth_mm, static_cast<double>(timestamp_us) * 1e-6, fx, fy, cx, cy,
                                R_c2d, t_c2d_m);
}

namespace {
// 【2026-10-23】初始化静止判定：用最近 2 s 的 IMU 统计量。
// 判据来自实测：三次"初始化时 |w|<7°/s"的录制可以通过清零初速度改善，
// 而 rep2 初始化发生在 19.9°/s 转动中，清零会反而伤它。
static double median_of(const std::deque<double>& q) {
    if (q.empty()) return 0.0;
    std::vector<double> v(q.begin(), q.end());
    std::sort(v.begin(), v.end());
    return v[v.size() / 2];
}

bool init_still_likely(const std::deque<double>& acc_hist, const std::deque<double>& gyr_hist,
                       double acc_thresh, double gyro_dps_thresh) {
    if (acc_hist.size() < 100 || gyr_hist.size() < 100) return false;
    std::vector<double> a(acc_hist.begin(), acc_hist.end());
    std::vector<double> w(gyr_hist.begin(), gyr_hist.end());
    std::sort(a.begin(), a.end());
    std::sort(w.begin(), w.end());
    const double acc_dev = std::abs(a[a.size() / 2] - 9.81);
    const double gyr_dps = w[w.size() / 2] * 180.0 / M_PI;
    return acc_dev < acc_thresh && gyr_dps < gyro_dps_thresh;
}
}  // namespace

bool VioEstimator::feedImage(const cv::Mat& gray, uint64_t timestamp_us) {
    if (!impl_->ok) return false;

    // 与 OpenVINS ROS 节点 callback_monocular 一致：按 track_frequency 丢弃过密的图像。
    // 否则 30fps 的相机会让 MSCKF 更新频率与标定/调参时（约 21Hz）不一致。
    const double ts = static_cast<double>(timestamp_us) * 1e-6;
    const double dt_min = 1.0 / std::max(1e-3, impl_->track_frequency);
    if (impl_->last_cam_time >= 0.0 && ts < impl_->last_cam_time + dt_min) {
        return true;  // 丢弃，不视为错误
    }
    impl_->last_cam_time = ts;
    // 把"当前是否接近静止"告诉 VioManager：初始化成功时用它决定要不要清零初速度
    if (impl_->vio) {
        impl_->vio->set_init_still(init_still_likely(impl_->acc_norm_hist, impl_->gyr_norm_hist,
                                                     impl_->opt.init_still_acc_thresh,
                                                     impl_->opt.init_still_gyro_dps));
    }

    cv::Mat img = gray;
    if (img.channels() != 1) {
        cv::cvtColor(img, img, cv::COLOR_BGR2GRAY);
    }
    if (img.depth() != CV_8U) {
        img.convertTo(img, CV_8U);
    }

    ov_core::CameraData cam;
    cam.timestamp = ts;
    cam.sensor_ids = {0};
    cam.images = {img};
    // OpenVINS 要求 masks 与 images 等长，且内部会对 mask 做 resize；
    // 不启用掩膜时放一张全 0（不遮挡）的同尺寸 mask。
    cam.masks = {cv::Mat::zeros(img.rows, img.cols, CV_8UC1)};

    const int before = impl_->images;
    impl_->vio->feed_measurement_camera(cam);
    impl_->images = before + 1;

    // 可视化数据：抄一份当前帧的跟踪点（纯展示，不影响估计）
    {
        const auto &src = impl_->vio->get_tracked_points();
        impl_->tracks.resize(src.size());
        for (size_t i = 0; i < src.size(); ++i) {
            impl_->tracks[i].u = src[i].u;
            impl_->tracks[i].v = src[i].v;
            impl_->tracks[i].length = src[i].len;
        }
    }

    // ---- 运行时失效保护 ----
    // 初始化之前不评估：动态初始化那 1~3 秒本来就没有成熟特征、没有更新，
    // 拿它当"失效"会开门就误报。
    if (impl_->vio->initialized()) {
        const auto fh = impl_->vio->get_frame_health();
        impl_->health.update(ts, fh.features, fh.any_visual_update);

        const VioHealthState st = impl_->health.state();
        if (st == VioHealthState::LOST) {
            if (impl_->lost_since < 0.0) impl_->lost_since = ts;
        } else {
            impl_->lost_since = -1.0;
        }

        // 【2026-10-21】快速自救：比重建轻一个数量级。
        // 先试放大协方差让卡方门限重开；连放 max_inflate_times 次还不行，
        // 再走下面的 force_reinit 重建（那条路要重新初始化，约 8 秒）。
        if (st != VioHealthState::LOST) {
            impl_->inflate_count = 0;
        } else if (impl_->opt.health.inflate_enable && impl_->lost_since >= 0.0 &&
                   impl_->inflate_count < impl_->opt.health.max_inflate_times &&
                   (ts - impl_->last_inflate_time) > impl_->opt.health.inflate_after_s) {
            if (impl_->vio->inflate_covariance(impl_->opt.health.inflate_scale)) {
                impl_->last_inflate_time = ts;
                ++impl_->inflate_count;
                impl_->logAlways("[VIO] 卡方锁死：协方差放大 ×%.0f 自救（第 %d/%d 次）\n",
                           impl_->opt.health.inflate_scale, impl_->inflate_count,
                           impl_->opt.health.max_inflate_times);
            }
        }

        // 触发重建的两种情形：
        //  (a) 已经恢复（特征回来且有更新）但之前挂过 —— 软恢复；
        //  (b) LOST 持续超过 force_reinit_after_s —— 强制重建。
        //      没有 (b) 的话，更新管线卡死时永远等不到 (a)，VIO 就永久死了。
        const bool soft = impl_->recovery_armed && st != VioHealthState::LOST &&
                          impl_->opt.health.reinit_on_soft_recover;
        const bool force = impl_->lost_since >= 0.0 &&
                           (ts - impl_->lost_since) > impl_->opt.health.force_reinit_after_s;
        //=====================================================================
        // 【2026-10-23】LOST 后**禁止在运动中重锚定**（状态机第一版）
        //
        // 实测证据：
        //   · fast1（640x480@60 无深度）：连崩 8 次，每次重初始化都发生在
        //     21–39°/s 的转动中（日志："初始化窗口内非静止，保留初始速度估计"）；
        //   · 每次重锚定都把当时那个不可靠的初速度带进新状态，于是"崩→重建→再崩"。
        //
        // 现在要求：重建前必须**静止**（|ω| < 10°/s 且 ||a||-g < 0.5，用最近 2 s 的
        // IMU 统计量）。不满足就一直保持 LOST、不发位姿——这比"发一条看起来连续
        // 但其实是错的重锚定轨迹"安全得多。操作员停一下就能恢复。
        //=====================================================================
        const bool rebuild_still =
            init_still_likely(impl_->acc_norm_hist, impl_->gyr_norm_hist, 0.5, 10.0);
        const bool can_rebuild = impl_->latched &&
                                 rebuild_still &&
                                 (impl_->last_rebuild_time < 0.0 ||
                                  ts - impl_->last_rebuild_time > impl_->opt.health.min_rebuild_interval_s);
        if ((soft || force) && impl_->latched && !rebuild_still) {
            // 每 2 s 提示一次，别刷屏
            if (ts - impl_->last_still_remind > 2.0) {
                impl_->last_still_remind = ts;
                impl_->logAlways("[VIO] LOST 需要重锚定，但当前在动（|w|=%.1f°/s）→ 先停稳再恢复\n",
                                 median_of(impl_->gyr_norm_hist) * 180.0 / M_PI);
            }
        }

        if (impl_->opt.health.reinit_on_recover && (soft || force) && can_rebuild) {
            impl_->recovery_armed = false;
            impl_->last_rebuild_time = ts;
            impl_->rebuildCount++;
            impl_->anchor.set = impl_->has_last_published;
            if (impl_->has_last_published) {
                const VisionPose& lp = impl_->last_published;
                impl_->anchor.yaw = yawOf(Eigen::Quaterniond(lp.q[0], lp.q[1], lp.q[2], lp.q[3]));
                impl_->anchor.t = Eigen::Vector3d(lp.x, lp.y, lp.z);
            }
            impl_->vio = std::make_unique<ov_msckf::VioManager>(impl_->params);
            impl_->vio->set_use_depth_init(impl_->opt.use_depth_init);
            impl_->vio->set_init_zero_velocity(impl_->opt.init_vel_zero);
            impl_->latched = false;
            impl_->launched = false;
            impl_->init_time = -1.0;
            ++impl_->segment_id;
            impl_->logAlways("[VIO] 视觉%s：重建估计器并重锚定（第 %d 次，segment %d，原因=%s）\n",
                             force ? "超时未恢复" : "恢复", impl_->rebuildCount,
                             impl_->segment_id, force ? "LOST 超时" : "曾经 LOST 后恢复");
        }

        if (impl_->health.state() == VioHealthState::LOST) {
            impl_->recovery_armed = true;
        } else if (impl_->recovery_armed && !impl_->opt.health.reinit_on_recover) {
            impl_->recovery_armed = false;
        }
    }

    if (!impl_->vio->initialized()) {
        return true;
    }

    auto state = impl_->vio->get_state();
    if (!state || !state->_imu) {
        return true;
    }

    const Eigen::Matrix3d R_GtoI = ov_core::quat_2_Rot(state->_imu->quat());
    const Eigen::Vector3d p_IinG = state->_imu->pos();
    const Eigen::Vector3d v_IinG = state->_imu->vel();
    // state->_timestamp 是相机时间；OpenVINS 传播/发布用的是其 IMU 时间：
    //   t_imu = t_cam + calib_camimu_dt
    const double state_time =
        state->_timestamp + state->_calib_dt_CAMtoIMU->value()(0);

    if (!impl_->latched) {
        impl_->launched = true;
        impl_->init_time = impl_->vio->initialized_time();
        impl_->converter->reset(R_GtoI, p_IinG);
        impl_->latched = true;
        impl_->health.notify_recovered();
        // 【2026-10-23】初始化瞬间的静止指标（只记录，不干预）：
        // 这两个数是"静止判定"的原始输入——将来用它决定要不要把初速度置 0。
        {
            double gyr_med = -1.0;
            if (impl_->gyr_norm_hist.size() > 20) {
                std::vector<double> g(impl_->gyr_norm_hist.begin(), impl_->gyr_norm_hist.end());
                std::sort(g.begin(), g.end());
                gyr_med = g[g.size() / 2];
            }
            impl_->logAlways("[VIO] [segment %d] 初始化静止指标: ||a||-g=%.3f m/s^2  |w|中位=%.1f °/s"
                             "（静止判定阈值建议 0.30 / 5.0）\n",
                             impl_->segment_id, [&]() -> double {
                                 if (impl_->acc_norm_hist.size() < 20) return -1.0;
                                 std::vector<double> v(impl_->acc_norm_hist.begin(),
                                                       impl_->acc_norm_hist.end());
                                 std::sort(v.begin(), v.end());
                                 return std::abs(v[v.size() / 2] - 9.81);
                             }(),
                             gyr_med < 0 ? -1.0 : gyr_med * 180.0 / M_PI);
        }
        impl_->logAlways("[VIO] [segment %d] 初始化完成：init_time=%.6f s, 开始输出 LOCAL_FRD 位姿%s\n",
                         impl_->segment_id,
                   impl_->init_time, impl_->anchor.set ? "（已重锚定）" : "");
    }

    const Eigen::Vector3d omega_I = impl_->last_gyro - state->_imu->bias_g();

    VisionPose pose = impl_->applyAnchor(
        impl_->converter->convert(state_time, R_GtoI, p_IinG, v_IinG, omega_I));
    if (impl_->opt.pose_timestamp == VioEstimator::PoseTimestamp::CAMERA) {
        pose.timestamp_us = timestamp_us;
    }

    // 诊断：把当前在线估计的 cam-IMU 时延写进 pose（只进 CSV，不发给飞控）
#ifndef PURE_UAV_VISIONPOSE_NO_CAM_IMU_DT
    pose.cam_imu_dt = static_cast<float>(impl_->vio->get_camera_timeoffset());
#endif

    //=========================================================================
    // 【2026-10-22】物理合理性保护（安全阀）
    //
    // 背景：实测过"健康门限全程 OK、重建 0 次，但估计速度从 0.18 一路爬到
    // 4.9 m/s、闭环误差 94%"的静默发散。这种位姿发给飞控比不发危险得多。
    //
    // 两条硬拦截：
    //   1) 位姿里出现 NaN/Inf  → 立即判 LOST，绝不外发
    //   2) |v| 超过物理上限持续 max_speed_hold_s → 判 LOST
    //
    // 它只做"拦"，不做"修"：不会让好的情况变差，只会让明显错的情况不外发。
    //=========================================================================
    {
        const double ts_check = static_cast<double>(timestamp_us) * 1e-6;
        bool finite = std::isfinite(pose.x) && std::isfinite(pose.y) && std::isfinite(pose.z) &&
                      std::isfinite(pose.vx) && std::isfinite(pose.vy) && std::isfinite(pose.vz) &&
                      std::isfinite(pose.q[0]) && std::isfinite(pose.q[1]) &&
                      std::isfinite(pose.q[2]) && std::isfinite(pose.q[3]);
        const double speed = std::sqrt(static_cast<double>(pose.vx) * pose.vx +
                                       static_cast<double>(pose.vy) * pose.vy +
                                       static_cast<double>(pose.vz) * pose.vz);
        if (!finite) {
            impl_->health.mark_implausible(ts_check);
            impl_->logAlways("[VIO] ⛔ 位姿出现 NaN/Inf → 判失效，停止外发\n");
        } else if (speed > impl_->opt.health.max_speed_mps) {
            if (impl_->overspeed_since < 0.0) impl_->overspeed_since = ts_check;
            if (ts_check - impl_->overspeed_since > impl_->opt.health.max_speed_hold_s) {
                impl_->health.mark_implausible(ts_check);
                impl_->logAlways("[VIO] ⛔ 估计速度 %.2f m/s 超过物理上限 %.1f m/s 持续 %.1f s "
                                 "→ 判失效（疑似发散）\n",
                                 speed, impl_->opt.health.max_speed_mps,
                                 impl_->opt.health.max_speed_hold_s);
                impl_->overspeed_since = -1.0;
            }
        } else {
            impl_->overspeed_since = -1.0;
        }
    }

    {
        std::lock_guard<std::mutex> lk(impl_->pose_mutex);
        impl_->latest = pose;
        impl_->has_latest = true;
        if (impl_->health.publishable()) {
            impl_->last_published = pose;
            impl_->has_last_published = true;
        }

        impl_->raw.timestamp_sec = state_time;
        impl_->raw.p_IinG = p_IinG;
        impl_->raw.v_IinG = v_IinG;
        impl_->raw.q_GtoI = state->_imu->quat();
        impl_->raw.bias_g = state->_imu->bias_g();
        impl_->raw.bias_a = state->_imu->bias_a();
        impl_->raw.camera_time_sec = static_cast<double>(timestamp_us) * 1e-6;
        impl_->has_raw = true;
    }
    impl_->poses++;
    return true;
}

bool VioEstimator::initialized() const {
    return impl_->ok && impl_->latched;
}

bool VioEstimator::getLatestPose(VisionPose& pose) const {
    std::lock_guard<std::mutex> lk(impl_->pose_mutex);
    if (!impl_->has_latest) return false;
    pose = impl_->latest;
    return true;
}

const std::vector<VioEstimator::TrackedPoint>& VioEstimator::trackedPoints() const {
    static const std::vector<TrackedPoint> kEmpty;
    if (!impl_) return kEmpty;
    return impl_->tracks;
}

// 运动有效性：||a||-g 的窗口中位。
// <0.10 基本是原地转（不会有三角化）；>0.15 才算真有平移。
double VioEstimator::linAccelDev() const {
  if (!impl_ || impl_->acc_norm_hist.size() < 20) return -1.0;
  std::vector<double> v(impl_->acc_norm_hist.begin(), impl_->acc_norm_hist.end());
  std::sort(v.begin(), v.end());
  return std::abs(v[v.size() / 2] - 9.81);
}

// 最近一次 MSCKF 的三角化成功率 = tri_ok / feat_in。
// 0 表示根本没有测量进来（视觉等于死的）。
void VioEstimator::depthErrStats(long long& n, double& mean_px, long long hist[6]) const {
    n = 0;
    mean_px = 0.0;
    for (int i = 0; i < 6; ++i) hist[i] = 0;
    if (!impl_ || !impl_->vio) return;
    double sum = 0.0;
    impl_->vio->get_depth_err_hist(n, sum, hist);
    mean_px = (n > 0) ? sum / static_cast<double>(n) : 0.0;
}

long long VioEstimator::rebuildCount() const {
    return impl_ ? impl_->rebuildCount : 0;
}

long long VioEstimator::depthUsed() const {
    if (!impl_ || !impl_->vio) return 0;
    return impl_->vio->get_depth_used();
}

double VioEstimator::triangulationRate() const {
  if (!impl_ || !impl_->vio) return -1.0;
  return impl_->vio->get_msckf_tri_rate();
}

void VioEstimator::msckfStats(long long &calls, long long &feat_in, long long &chi2_rej,
                              long long &updates, long long &rows,
                              long long &clean_drop, long long &tri_drop) const {
  calls = feat_in = chi2_rej = updates = rows = 0;
  clean_drop = tri_drop = 0;
  if (!impl_ || !impl_->vio) return;
  impl_->vio->get_msckf_stats(calls, feat_in, chi2_rej, updates, rows, clean_drop, tri_drop);
}

bool VioEstimator::publishable() const {
    if (!impl_ || !impl_->ok) return false;
    // 重建估计器期间（latched 掉了）也一律不发
    return impl_->health.publishable() && impl_->latched;
}

HealthReport VioEstimator::healthReport() const {
    if (!impl_) return HealthReport();
    const double t = impl_->last_cam_time >= 0.0 ? impl_->last_cam_time : 0.0;
    return impl_->health.report(t);
}

VioHealthState VioEstimator::healthState() const {
    return impl_ ? impl_->health.state() : VioHealthState::UNKNOWN;
}

int VioEstimator::recoveryCount() const {
    return impl_ ? impl_->health.report(0.0).recoveries : 0;
}

int VioEstimator::imageCount() const { return impl_->images; }
int VioEstimator::poseCount() const { return static_cast<int>(impl_->poses); }

uint64_t VioEstimator::lastPoseTimestampUs() const {
    std::lock_guard<std::mutex> lk(impl_->pose_mutex);
    return impl_->has_latest ? impl_->latest.timestamp_us : 0;
}

double VioEstimator::initializedTimeSec() const { return impl_->init_time; }

const VisionPoseConvert& VioEstimator::converter() const { return *impl_->converter; }

bool VioEstimator::getLatestRawState(RawState& state) const {
    std::lock_guard<std::mutex> lk(impl_->pose_mutex);
    if (!impl_->has_raw) return false;
    state = impl_->raw;
    return true;
}

}  // namespace pure_uav
