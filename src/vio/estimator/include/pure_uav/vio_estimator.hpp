// VioEstimator：ROS-free 封装 OpenVINS 的 VioManager。
//
// 输入：IMU 采样（飞控 boot 时间 us）+ 灰度图（同一时钟域 us）
// 输出：pure_uav::VisionPose（LOCAL_FRD / BODY_FRD，见 vision_pose.hpp）
//
// 线程模型：本类不是线程安全的。IMU 数据和图像必须由同一个线程按时间顺序
// 调用 feedImu()/feedImage()（实时程序里用一条 VIO 线程从队列取数据即可）。
// getLatestPose() 只读取 feedImage() 内缓存的最后一帧结果，可以在任意线程调用。

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include <Eigen/Dense>
#include <opencv2/core.hpp>

#include "pure_uav/vio_health.hpp"
#include "pure_uav/vio_types.hpp"
#include "pure_uav/vision_pose.hpp"
#include "pure_uav/vision_pose_convert.hpp"

namespace pure_uav {

class VioEstimator {
public:
    // VisionPose.timestamp_us 用哪个时刻：
    //   IMU   ：状态对应的 IMU 时间 = 相机时间 + timeshift_cam_imu。
    //           与 OpenVINS ROS 节点发布的 poseimu.header.stamp 一致（推荐）。
    //   CAMERA：相机帧原始时间戳（不做 cam-IMU 时间偏移修正）。
    enum class PoseTimestamp { IMU, CAMERA };

    struct Options {
        // 包含 estimator_config.yaml（以及它引用的 imu/camera chain yaml）的目录
        std::string config_dir;
        // estimator 配置文件名，默认 estimator_config.yaml
        std::string estimator_config = "estimator_config.yaml";

        // 机体系相对 IMU 系的安装关系（默认重合）
        Eigen::Matrix3d R_imu_to_body = Eigen::Matrix3d::Identity();
        Eigen::Vector3d t_imu_body = Eigen::Vector3d::Zero();

        // 打印初始化/配置信息
        bool verbose = true;

        // 是否让 OpenVINS 在线估计相机-IMU 时移（初值仍取配置里的 timeshift_cam_imu）。
        // 实时链路相机时间戳是“主机到达时间”，标定时移不一定完全适用，可打开它让滤波器自己修。
        // 【2026-10-21】离线和实时必须用同一个默认值，否则离线 A/B 的结论搬不到机上。
        // 实测差异很大：实时一直开着在线时延估计，离线原来是关的（固定时延）。
        bool estimate_camera_timeoffset = true;

        // 【2026-10-21】动态初始化 MLE 的线程数。<=0 表示沿用配置文件。
        // 为什么要有这个：Ceres 多线程会让**带重建的运行不可复现**（实测同一命令连跑两次，
        // 一次 10 次重建/1.863 m、一次 9 次重建/2.661 m，TUM 的 md5 不同）。
        // 设成 1 后两次运行逐位一致。做 A/B 对照必须用 1；实时默认仍走配置文件（6）。
        int init_threads = 0;

        // 【2026-10-21】用深度初始化特征（默认关，做 A/B 用）。
        // 单目 VIO 没有平移就没有视差、三角化失败就没测量；深度直接给米制位置，
        // 纯旋转也能出测量，且尺度被钉死。需要每帧先调 setDepthImage()。
        bool use_depth_init = false;

        /// 静止起步时把初始速度强制置 0。动态初始化会给 0.2-0.4 m/s 的虚假初速度，
        /// 而录制协议是从静止起步（真实 v=0）——实测 |v_init| 直接决定成败。
        bool init_vel_zero = false;

        /// 静止判定阈值（只用于 init_vel_zero 的门限）
        double init_still_acc_thresh = 0.30;     // ||a||-g 小于它算静
        double init_still_gyro_dps = 10.0;       // |w| 中位小于它算静

        /// 深度查询：算出"每帧最深处理时间"等统计时用（可空）
        void *unused_pad = nullptr;

        // 静态初始化：要求先静止 init_window_time 秒，用静止段的加速度计均值定重力方向。
        // 动态初始化在运动窗口里分离加速度和重力，倾角误差通常 1~3°；
        // 静态初始化能做到 0.1° 量级。做 A/B 对照用，默认沿用配置文件。
        bool static_init = false;

        // 输出时间戳取 IMU 时间（默认，与 OpenVINS ROS 输出一致）还是相机原始时间
        PoseTimestamp pose_timestamp = PoseTimestamp::IMU;

        // 运行时失效保护：视觉断供时判定为 LOST、停发位姿，恢复时重建估计器并重锚定。
        // 它不会让 VIO 变准，只是不让它把跑偏的位姿当真的发出去。
        HealthConfig health;
    };

    explicit VioEstimator(const Options& options);
    ~VioEstimator();

    VioEstimator(const VioEstimator&) = delete;
    VioEstimator& operator=(const VioEstimator&) = delete;

    bool ok() const;
    const std::string& error() const;

    // 喂一帧 IMU。timestamp_us 必须单调递增。
    void feedImu(const ImuSample& imu);

    // 喂一帧灰度图。调用前应已把该时刻之前的 IMU 全部喂入。
    // 返回 true 表示处理成功（不代表已经初始化）。
    bool feedImage(const cv::Mat& gray, uint64_t timestamp_us);

    /// 【2026-10-21】喂一帧深度（毫米，深度相机坐标）。必须紧邻 feedImage 之前调用。
    /// 只在 Options::use_depth_init 打开时生效。
    void setDepthImage(const cv::Mat& depth_mm, uint64_t timestamp_us, double fx, double fy,
                       double cx, double cy, const double R_c2d[9] = nullptr,
                       const double t_c2d_m[3] = nullptr);

    bool initialized() const;

    // 最近一帧 VisionPose（已初始化后才有）。无结果返回 false。
    bool getLatestPose(VisionPose& pose) const;

    // ---- 运行时失效保护 ----
    // 这一帧是否应该把位姿发给飞控。LOST 期间恒为 false（含重建估计器期间）。
    bool publishable() const;
    // 当前健康报告（特征数、距上次更新多久、状态、恢复次数……）
    HealthReport healthReport() const;
    VioHealthState healthState() const;
    // 已经进入过 LOST 并完成重建的次数
    int recoveryCount() const;

    /// 【2026-10-29】飞控时间基变了（飞控重启 / 串口重连 / IMU 时间戳大幅回退）：
    /// 立即丢弃旧时间基的估计，按新时间基从头初始化。
    ///
    /// 与 LOST 重建的区别（这是刻意分开的两条路）：
    ///   · 不要求静止、不看 reinit_on_recover —— 重启是外部事件，不能等"停稳"；
    ///   · 锚点清零：新轨迹从原点开始（飞控那边是 reset_counter 变化，会重新锚定）；
    ///   · rebuildCount 递增，作为 ODOMETRY 的 reset_counter 告诉飞控"我重置了"。
    /// 只能在 feedImu()/feedImage() 的同一线程调用。
    void resetForNewTimebase(const char* reason = nullptr);

    // 统计/调试
    // MSCKF 更新诊断：calls=更新调用次数，feat_in=进入更新的特征数，
    // chi2_rej=被卡方门限拒掉的数，updates=真正执行的更新次数，rows=压缩后残差行数。
    // 用来区分"没特征可用"和"有特征但全被门限拒了"。
    void msckfStats(long long &calls, long long &feat_in, long long &chi2_rej,
                    long long &updates, long long &rows,
                    long long &clean_drop, long long &tri_drop) const;
    int imageCount() const;
    int poseCount() const;
    uint64_t lastPoseTimestampUs() const;
    double initializedTimeSec() const;  // OpenVINS 初始化时刻（输入时钟的秒）
    const VisionPoseConvert& converter() const;

    // 最近一帧 OpenVINS 原始状态（global 系，供回归对照/调试）。
    // q_GtoI 为 OpenVINS 的 JPL 四元数顺序 [x,y,z,w]。
    struct RawState {
        double timestamp_sec = 0.0;
        Eigen::Vector3d p_IinG = Eigen::Vector3d::Zero();
        Eigen::Vector3d v_IinG = Eigen::Vector3d::Zero();
        Eigen::Vector4d q_GtoI = Eigen::Vector4d(0.0, 0.0, 0.0, 1.0);
        Eigen::Vector3d bias_g = Eigen::Vector3d::Zero();
        Eigen::Vector3d bias_a = Eigen::Vector3d::Zero();
        double camera_time_sec = 0.0;  // 对应相机帧的时间（与参考轨迹一致）
    };
    bool getLatestRawState(RawState& state) const;

    // ---- 纯可视化（不参与估计）----
    // 主前端这一帧跟踪到的像素点。长度 = 该点已连续跟踪的帧数。
    struct TrackedPoint {
        float u = 0.f;
        float v = 0.f;
        int length = 0;
    };
    // 只能在调用 feedImage() 的同一线程里用（不做加锁，避免 30 Hz 无谓开销）。
    const std::vector<TrackedPoint>& trackedPoints() const;

    // ---- 倾角自检（判断"这条轨迹还能不能信"的直接指标）----
    // 把滤波器自己认为的重力方向和加速度计实测的重力方向比一比。
    // 只在准静止时刻有效（|ω|<1.5°/s 且 ||f|-g|<0.15），其余时刻 valid=false。
    // 数值含义：姿态错 θ 度 → 未扣掉的重力 g·sinθ 会被当成真实加速度积分进速度。
    //   1° → 0.17 m/s²    5° → 0.86 m/s²    20° → 3.35 m/s²
    // 实时实测（2026-10-21）：θ 从 2° 漂到 53°，速度斜坡从 0.3 涨到 6 m/s²。
    struct TiltCheck {
        bool valid = false;
        double deg = 0.0;
    };
    TiltCheck tiltCheck() const;

    /// 【2026-10-25】运行期开关零速更新（要求 config 里 try_zupt: true）。
    /// 上层按"飞控是否解锁"自动切：地面上开（静止时钉住速度），空中关（防悬停误判）。
    void setZuptEnabled(bool enable);

    // ---- 运动有效性自检（判断"这次测试到底有没有平移"）----
    // 单目 VIO 靠平移拿视差，没有视差就三角化不出来。实测（2026-10-21）：
    //   原地转（手持）      ||a|-g| 中位 0.015~0.071  -> 三角化成功率 0.000
    //   走路（离线数据集）  ||a|-g| 中位 0.156~0.211  -> 三角化成功率 0.121~0.130
    // 返回值 = ||a|| - g 的滑动中位（单位 m/s²），不是瞬时值。
    // 经验判据：< 0.10 基本是原地转，测不出东西；> 0.15 才算真有平移。
    double linAccelDev() const;
    // 最近一次 MSCKF 更新的「三角化成功率」= tri_ok / feat_in（0 表示完全没有测量）
    double triangulationRate() const;

    /// 累计"用深度初始化特征"的个数（诊断：>0 说明深度路径真的在跑）
    long long depthUsed() const;

    /// 估计器重建（重锚定）次数——作为 MAVLink ODOMETRY 的 reset_counter 发给飞控，
    /// 否则飞控会把重建后的位姿跳变当成真实运动。
    long long rebuildCount() const;

    /// 深度一致性：特征数 / 平均中位重投影误差(px) / 6 档直方图(<1 <2 <5 <10 <25 >=25)
    void depthErrStats(long long& n, double& mean_px, long long hist[6]) const;
    /// 【2026-10-25】近 2 s 深度一致性**中位数**（px）。稳健，不会被离群点带偏。
    /// 没有样本时返回 -1。健康门限和状态行都用这个，不要再用均值。
    double recentDepthErrMedianPx(double now_s);


private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace pure_uav
