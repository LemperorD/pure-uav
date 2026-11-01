/*
 * OpenVINS: An Open Platform for Visual-Inertial Research
 * Copyright (C) 2018-2023 Patrick Geneva
 * Copyright (C) 2018-2023 Guoquan Huang
 * Copyright (C) 2018-2023 OpenVINS Contributors
 * Copyright (C) 2018-2019 Kevin Eckenhoff
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OV_MSCKF_VIOMANAGER_H
#define OV_MSCKF_VIOMANAGER_H

#include <Eigen/StdVector>
#include <algorithm>
#include <atomic>
#include <boost/filesystem.hpp>
#include <fstream>
#include <deque>
#include <memory>
#include <mutex>
#include <string>

#include "VioManagerOptions.h"

namespace ov_core {
struct ImuData;
struct CameraData;
class TrackBase;
class FeatureInitializer;
} // namespace ov_core
namespace ov_init {
class InertialInitializer;
} // namespace ov_init

namespace ov_msckf {

class State;
class StateHelper;
class UpdaterMSCKF;
class UpdaterSLAM;
class UpdaterZeroVelocity;
class Propagator;

/**
 * @brief Core class that manages the entire system
 *
 * This class contains the state and other algorithms needed for the MSCKF to work.
 * We feed in measurements into this class and send them to their respective algorithms.
 * If we have measurements to propagate or update with, this class will call on our state to do that.
 */
class VioManager {

public:
  // 【2026-10-23】深度年龄门限拒绝计数（诊断用）
  long long diag_depth_frames = 0;
  long long diag_depth_edge_rej = 0;
  long long diag_depth_no_frame = 0;
  long long diag_depth_too_old = 0;

  /**
   * @brief Default constructor, will load all configuration variables
   * @param params_ Parameters loaded from either ROS or CMDLINE
   */
  VioManager(VioManagerOptions &params_);

  /**
   * @brief Feed function for inertial data
   * @param message Contains our timestamp and inertial information
   */
  void feed_measurement_imu(const ov_core::ImuData &message);

  /**
   * @brief Feed function for camera measurements
   * @param message Contains our timestamp, images, and camera ids
   */
  void feed_measurement_camera(const ov_core::CameraData &message) { track_image_and_update(message); }

  /**
   * @brief Feed function for a synchronized simulated cameras
   * @param timestamp Time that this image was collected
   * @param camids Camera ids that we have simulated measurements for
   * @param feats Raw uv simulated measurements
   */
  void feed_measurement_simulation(double timestamp, const std::vector<int> &camids,
                                   const std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> &feats);

  /**
   * @brief Given a state, this will initialize our IMU state.
   * @param imustate State in the MSCKF ordering: [time(sec),q_GtoI,p_IinG,v_IinG,b_gyro,b_accel]
   */
  void initialize_with_gt(Eigen::Matrix<double, 17, 1> imustate);

  /// If we are initialized or not
  bool initialized() { return is_initialized_vio && timelastupdate != -1; }

  /// Timestamp that the system was initialized at
  double initialized_time() { return startup_time; }

  /// Accessor for current system parameters
  VioManagerOptions get_params() { return params; }

  /// Accessor to get the current state
  std::shared_ptr<State> get_state() { return state; }

  /// Accessor to get the current propagator
  std::shared_ptr<Propagator> get_propagator() { return propagator; }

  /// Get a nice visualization image of what tracks we have
  cv::Mat get_historical_viz_image();

  /// Returns 3d SLAM features in the global frame
  std::vector<Eigen::Vector3d> get_features_SLAM();

  /// Returns 3d ARUCO features in the global frame
  std::vector<Eigen::Vector3d> get_features_ARUCO();

  /// Returns 3d features used in the last update in global frame
  std::vector<Eigen::Vector3d> get_good_features_MSCKF() { return good_features_MSCKF; }

  /// Return the image used when projecting the active tracks
  void get_active_image(double &timestamp, cv::Mat &image) {
    timestamp = active_tracks_time;
    image = active_image;
  }

  /// 【2026-10-21】三角化成功率 = tri_ok / feat_in（累计）。
  /// 0 表示从头到尾没有一次三角化成功 = 滤波器一直在纯惯导积分。
  double get_msckf_tri_rate() const;

  /// 【2026-10-21】累计"用深度初始化、跳过三角化"的特征数（诊断/验证用）。
  long long get_depth_used() const;

  /// 深度一致性诊断：n / 中位重投影误差累积 / 6 档直方图
  void get_depth_err_hist(long long &n, double &sum, long long hist[6]) const;

  /// 【2026-10-21】喂一帧深度（毫米，深度相机坐标系），供特征初始化用。
  /// 必须在对应的 feed_new_camera 之前调用。传空 Mat 或 use_depth_init=false 时不生效。
  void set_depth_image(const cv::Mat &depth_mm, double timestamp, double fx, double fy, double cx,
                       double cy, const double R_c2d[9] = nullptr,
                       const double t_c2d_m[3] = nullptr);

  /// 【2026-10-22】静止起步时把初始速度强制置 0（默认关，做 A/B 用）。
  /// 依据：6 次录制里 |v_init| 判定成败——0.08->2.8%/7%，0.24-0.39->57%~94%。
  /// 录制时人站着不动、真实速度就是 0，动态初始化却给出 0.2-0.4 m/s 的虚假速度。
  void set_init_zero_velocity(bool enable) { init_zero_velocity_ = enable; }

  // 【2026-10-25】运行期开关零速更新（只在 try_zupt=true 时有效）。
  // 为什么要运行期开关：地面上（飞控未解锁）静止时，零速更新能把"停着还在加速"直接钉死；
  // 空中（已解锁）它反而危险 —— 悬停时画面几乎不动，误判会把速度硬钉 0，位置估计被冻住，
  // 飞控还以为没漂。所以由上层按"是否解锁"自动切，不需要任何人在飞行中操作。
  void set_zupt_enabled(bool enable) { zupt_runtime_on_ = enable; }
  bool zupt_enabled() const { return zupt_runtime_on_; }

  /// 【2026-10-23】"初始化窗口内是否接近静止"——由上层用 IMU 统计量判定后每帧传入。
  /// 只在它为 true 时才允许把初速度置 0（rep2 就是初始化发生在 19.9°/s 转动中，
  /// 无条件清零会把它从 7% 变成 62%，门限正好避开这种）。
  void set_init_still(bool still) { init_still_ = still; }

  /// 打开/关闭"用深度初始化特征"（默认关，做 A/B 用）
  void set_use_depth_init(bool enable) { use_depth_init_ = enable; }

  /// 【2026-10-21】当前滤波器内部的 cam-IMU 时延（秒）。只用于诊断记录。
  double get_camera_timeoffset() const;

  /// 【2026-10-21】把当前协方差放大 scale 倍，用来从"卡方门限锁死"里自救。
  /// 只在已经初始化时有效；返回是否真的做了。
  bool inflate_covariance(double scale);

  /// MSCKF 更新诊断（供离线/实时排查"视觉更新是否被门限拒掉"）
  void get_msckf_stats(long long &calls, long long &feat_in, long long &chi2_rej,
                       long long &updates, long long &rows,
                       long long &clean_drop, long long &tri_drop) const;

  /// 每帧刷新的健康指标（供上层做失效保护/恢复）
  struct FrameHealth {
    int features = 0;       ///< 该帧跟踪器里还在跟踪的特征数
    bool msckf_updated = false; ///< 该帧是否真的产生了一次 MSCKF 更新
    /// 【2026-10-21】该帧是否产生了**任意**视觉 EKF 更新（MSCKF 或 SLAM）。
    /// 健康判断应该看这个：max_slam=50 时只走 SLAM 的帧也是有效视觉信息。
    /// 实测 clean1：总共 770 次 EKF 更新，其中 583 次只走 SLAM —— 原来健康门限只看到 24%。
    bool any_visual_update = false;
    /// 【2026-10-25】该帧 MSCKF 实际参与更新的观测行数。
    /// 这是"视觉到底有没有在约束状态"的直接度量：特征数 200 但 rows=2 等于没约束
    /// （实测 run3/run4：rows 掉到 2-3，速度就开始发散）。
    int msckf_rows = 0;
    /// 【2026-10-25】该帧走了零速更新（ZUPT）：没有克隆、没有 MSCKF，但 q/bg/ba/v 都被约束过。
    /// 静止时故意不做 MSCKF 是**正常行为**，上层健康判据必须认这一点，否则会把"静止"误判成"视觉失效"。
    bool zupt = false;
    bool slam_updated = false;  ///< 有 SLAM 更新但 MSCKF 没有（诊断用）
    double time = -1.0;     ///< 该帧时间（秒）
  };
  FrameHealth get_frame_health() const { return frame_health; }

  /// 当前帧跟踪到的像素点（纯可视化用途，不参与估计）
  struct TrackedPoint {
    float u = 0.f;
    float v = 0.f;
    int len = 0;  ///< 该点已连续跟踪多少帧；越大说明它越"结实"
  };
  const std::vector<TrackedPoint> &get_tracked_points() const { return tracked_points; }

  /// Returns active tracked features in the current frame
  void get_active_tracks(double &timestamp, std::unordered_map<size_t, Eigen::Vector3d> &feat_posinG,
                         std::unordered_map<size_t, Eigen::Vector3d> &feat_tracks_uvd) {
    timestamp = active_tracks_time;
    feat_posinG = active_tracks_posinG;
    feat_tracks_uvd = active_tracks_uvd;
  }

protected:
  /**
   * @brief Given a new set of camera images, this will track them.
   *
   * If we are having stereo tracking, we should call stereo tracking functions.
   * Otherwise we will try to track on each of the images passed.
   *
   * @param message Contains our timestamp, images, and camera ids
   */
  void track_image_and_update(const ov_core::CameraData &message);

  /**
   * @brief This will do the propagation and feature updates to the state
   * @param message Contains our timestamp, images, and camera ids
   */
  void do_feature_propagate_update(const ov_core::CameraData &message);

  /**
   * @brief This function will try to initialize the state.
   *
   * This should call on our initializer and try to init the state.
   * In the future we should call the structure-from-motion code from here.
   * This function could also be repurposed to re-initialize the system after failure.
   *
   * @param message Contains our timestamp, images, and camera ids
   * @return True if we have successfully initialized
   */
  bool try_to_initialize(const ov_core::CameraData &message);

  /**
   * @brief This function will will re-triangulate all features in the current frame
   *
   * For all features that are currently being tracked by the system, this will re-triangulate them.
   * This is useful for downstream applications which need the current pointcloud of points (e.g. loop closure).
   * This will try to triangulate *all* points, not just ones that have been used in the update.
   *
   * @param message Contains our timestamp, images, and camera ids
   */
  void retriangulate_active_tracks(const ov_core::CameraData &message);

  /// Manager parameters
  VioManagerOptions params;

  /// Our master state object :D
  std::shared_ptr<State> state;

  /// Propagator of our state
  std::shared_ptr<Propagator> propagator;

  // 【2026-10-21】陀螺预测 KLT 初值：累计[上一帧图像, 当前帧]之间的 IMU 旋转。
  // 在 feed_measurement_imu 里逐步累计，在 track_image_and_update 里取用后清零。
  Eigen::Matrix3d gyro_pred_R_ = Eigen::Matrix3d::Identity();
  double gyro_pred_last_imu_t_ = -1.0;
  double gyro_pred_last_cam_t_ = -1.0;

  // 【2026-10-21】深度历史（用于按锚点时刻查深度）
  struct DepthFrameRec {
    double t;
    cv::Mat depth_mm;
  };
  std::deque<DepthFrameRec> depth_hist_;
  double depth_fx_ = 0, depth_fy_ = 0, depth_cx_ = 0, depth_cy_ = 0;
  // 彩色 → 深度 的旋转/平移（实测平移 10.11 mm，1 米处就是 13.7 像素，不能忽略）
  Eigen::Matrix3d R_c2d_ = Eigen::Matrix3d::Identity();
  Eigen::Vector3d t_c2d_m_ = Eigen::Vector3d::Zero();
  bool use_depth_init_ = false;
  bool init_zero_velocity_ = false;
  /// 零速更新是否**当前**生效（默认跟 try_zupt 一致，可被 set_zupt_enabled 覆盖）
  bool zupt_runtime_on_ = false;
  bool init_still_ = false;

  /// Our sparse feature tracker (klt or descriptor)
  std::shared_ptr<ov_core::TrackBase> trackFEATS;

  /// Our aruoc tracker
  std::shared_ptr<ov_core::TrackBase> trackARUCO;

  /// State initializer
  std::shared_ptr<ov_init::InertialInitializer> initializer;

  /// Boolean if we are initialized or not
  bool is_initialized_vio = false;

  /// Our MSCKF feature updater
  std::shared_ptr<UpdaterMSCKF> updaterMSCKF;

  /// Our SLAM/ARUCO feature updater
  std::shared_ptr<UpdaterSLAM> updaterSLAM;

  /// Our zero velocity tracker
  std::shared_ptr<UpdaterZeroVelocity> updaterZUPT;

  /// 每帧刷新的健康指标（见 get_frame_health）
  FrameHealth frame_health;

  /// 每帧刷新的跟踪点（见 get_tracked_points），只给查看器画图用
  std::vector<TrackedPoint> tracked_points;

  /// This is the queue of measurement times that have come in since we starting doing initialization
  /// After we initialize, we will want to prop & update to the latest timestamp quickly
  std::vector<double> camera_queue_init;
  std::mutex camera_queue_init_mtx;

  // Timing statistic file and variables
  std::ofstream of_statistics;
  boost::posix_time::ptime rT1, rT2, rT3, rT4, rT5, rT6, rT7;

  // Track how much distance we have traveled
  double timelastupdate = -1;
  double distance = 0;

  // Startup time of the filter
  double startup_time = -1;

  // Threads and their atomics
  std::atomic<bool> thread_init_running, thread_init_success;

  // If we did a zero velocity update
  bool did_zupt_update = false;
  bool has_moved_since_zupt = false;

  // Good features that where used in the last update (used in visualization)
  std::vector<Eigen::Vector3d> good_features_MSCKF;

  // Re-triangulated features 3d positions seen from the current frame (used in visualization)
  // For each feature we have a linear system A * p_FinG = b we create and increment their costs
  double active_tracks_time = -1;
  std::unordered_map<size_t, Eigen::Vector3d> active_tracks_posinG;
  std::unordered_map<size_t, Eigen::Vector3d> active_tracks_uvd;
  cv::Mat active_image;
  std::map<size_t, Eigen::Matrix3d> active_feat_linsys_A;
  std::map<size_t, Eigen::Vector3d> active_feat_linsys_b;
  std::map<size_t, int> active_feat_linsys_count;
};

} // namespace ov_msckf

#endif // OV_MSCKF_VIOMANAGER_H
