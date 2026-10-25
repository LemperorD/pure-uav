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

#include "VioManager.h"

#include <cmath>
#include <cstdlib>

#include "feat/Feature.h"
#include "feat/FeatureDatabase.h"
#include "feat/FeatureInitializer.h"
#include "track/TrackAruco.h"
#include "track/TrackDescriptor.h"
#include "track/TrackKLT.h"
#include "track/TrackSIM.h"
#include "types/Landmark.h"
#include "types/LandmarkRepresentation.h"
#include "utils/opencv_lambda_body.h"
#include "utils/print.h"
#include "utils/quat_ops.h"
#include "utils/sensor_data.h"

#include "init/InertialInitializer.h"

#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "update/UpdaterMSCKF.h"
#include "update/UpdaterSLAM.h"
#include "update/UpdaterZeroVelocity.h"

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

VioManager::VioManager(VioManagerOptions &params_) : thread_init_running(false), thread_init_success(false) {

  // Nice startup message
  PRINT_DEBUG("=======================================\n");
  PRINT_DEBUG("OPENVINS ON-MANIFOLD EKF IS STARTING\n");
  PRINT_DEBUG("=======================================\n");

  // Nice debug
  this->params = params_;
  params.print_and_load_estimator();
  params.print_and_load_noise();
  params.print_and_load_state();
  params.print_and_load_trackers();

  // This will globally set the thread count we will use
  // -1 will reset to the system default threading (usually the num of cores)
  cv::setNumThreads(params.num_opencv_threads);
  cv::setRNGSeed(0);

  // Create the state!!
  state = std::make_shared<State>(params.state_options);

  // Set the IMU intrinsics
  state->_calib_imu_dw->set_value(params.vec_dw);
  state->_calib_imu_dw->set_fej(params.vec_dw);
  state->_calib_imu_da->set_value(params.vec_da);
  state->_calib_imu_da->set_fej(params.vec_da);
  state->_calib_imu_tg->set_value(params.vec_tg);
  state->_calib_imu_tg->set_fej(params.vec_tg);
  state->_calib_imu_GYROtoIMU->set_value(params.q_GYROtoIMU);
  state->_calib_imu_GYROtoIMU->set_fej(params.q_GYROtoIMU);
  state->_calib_imu_ACCtoIMU->set_value(params.q_ACCtoIMU);
  state->_calib_imu_ACCtoIMU->set_fej(params.q_ACCtoIMU);

  // Timeoffset from camera to IMU
  Eigen::VectorXd temp_camimu_dt;
  temp_camimu_dt.resize(1);
  temp_camimu_dt(0) = params.calib_camimu_dt;
  state->_calib_dt_CAMtoIMU->set_value(temp_camimu_dt);
  state->_calib_dt_CAMtoIMU->set_fej(temp_camimu_dt);

  // Loop through and load each of the cameras
  state->_cam_intrinsics_cameras = params.camera_intrinsics;
  for (int i = 0; i < state->_options.num_cameras; i++) {
    state->_cam_intrinsics.at(i)->set_value(params.camera_intrinsics.at(i)->get_value());
    state->_cam_intrinsics.at(i)->set_fej(params.camera_intrinsics.at(i)->get_value());
    state->_calib_IMUtoCAM.at(i)->set_value(params.camera_extrinsics.at(i));
    state->_calib_IMUtoCAM.at(i)->set_fej(params.camera_extrinsics.at(i));
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // If we are recording statistics, then open our file
  if (params.record_timing_information) {
    // If the file exists, then delete it
    if (boost::filesystem::exists(params.record_timing_filepath)) {
      boost::filesystem::remove(params.record_timing_filepath);
      PRINT_INFO(YELLOW "[STATS]: found old file found, deleted...\n" RESET);
    }
    // Create the directory that we will open the file in
    boost::filesystem::path p(params.record_timing_filepath);
    boost::filesystem::create_directories(p.parent_path());
    // Open our statistics file!
    of_statistics.open(params.record_timing_filepath, std::ofstream::out | std::ofstream::app);
    // Write the header information into it
    of_statistics << "# timestamp (sec),tracking,propagation,msckf update,";
    if (state->_options.max_slam_features > 0) {
      of_statistics << "slam update,slam delayed,";
    }
    of_statistics << "re-tri & marg,total" << std::endl;
  }

  //===================================================================================
  //===================================================================================
  //===================================================================================

  // Let's make a feature extractor
  // NOTE: after we initialize we will increase the total number of feature tracks
  // NOTE: we will split the total number of features over all cameras uniformly
  int init_max_features = std::floor((double)params.init_options.init_max_features / (double)params.state_options.num_cameras);
  if (params.use_klt) {
    trackFEATS = std::shared_ptr<TrackBase>(new TrackKLT(state->_cam_intrinsics_cameras, init_max_features,
                                                         state->_options.max_aruco_features, params.use_stereo, params.histogram_method,
                                                         params.fast_threshold, params.grid_x, params.grid_y, params.min_px_dist));
  } else {
    trackFEATS = std::shared_ptr<TrackBase>(new TrackDescriptor(
        state->_cam_intrinsics_cameras, init_max_features, state->_options.max_aruco_features, params.use_stereo, params.histogram_method,
        params.fast_threshold, params.grid_x, params.grid_y, params.min_px_dist, params.knn_ratio));
  }

  // Initialize our aruco tag extractor
  if (params.use_aruco) {
    trackARUCO = std::shared_ptr<TrackBase>(new TrackAruco(state->_cam_intrinsics_cameras, state->_options.max_aruco_features,
                                                           params.use_stereo, params.histogram_method, params.downsize_aruco));
  }

  // Initialize our state propagator
  propagator = std::make_shared<Propagator>(params.imu_noises, params.gravity_mag);

  // Our state initialize
  initializer = std::make_shared<ov_init::InertialInitializer>(params.init_options, trackFEATS->get_feature_database());

  // Make the updater!
  updaterMSCKF = std::make_shared<UpdaterMSCKF>(params.msckf_options, params.featinit_options);
  updaterSLAM = std::make_shared<UpdaterSLAM>(params.slam_options, params.aruco_options, params.featinit_options);

  // If we are using zero velocity updates, then create the updater
  if (params.try_zupt) {
    updaterZUPT = std::make_shared<UpdaterZeroVelocity>(params.zupt_options, params.imu_noises, trackFEATS->get_feature_database(),
                                                        propagator, params.gravity_mag, params.zupt_max_velocity,
                                                        params.zupt_noise_multiplier, params.zupt_max_disparity);
  }
}

void VioManager::feed_measurement_imu(const ov_core::ImuData &message) {

  // The oldest time we need IMU with is the last clone
  // We shouldn't really need the whole window, but if we go backwards in time we will
  double oldest_time = state->margtimestep();
  if (oldest_time > state->_timestamp) {
    oldest_time = -1;
  }
  if (!is_initialized_vio) {
    oldest_time = message.timestamp - params.init_options.init_window_time + state->_calib_dt_CAMtoIMU->value()(0) - 0.10;
  }
  propagator->feed_imu(message, oldest_time);

  // 【2026-10-21】累计陀螺旋转，供前端预测 KLT 初值（只在两帧图像之间累计）。
  // 符号约定取自 Propagator：R_GtoI(t+dt) = exp_so3(-w*dt) * R_GtoI(t)。
  if (gyro_pred_last_cam_t_ >= 0.0) {
    if (gyro_pred_last_imu_t_ > 0.0) {
      const double dt = message.timestamp - gyro_pred_last_imu_t_;
      if (dt > 0.0 && dt < 0.2)
        gyro_pred_R_ = ov_core::exp_so3(-message.wm * dt) * gyro_pred_R_;
    }
    gyro_pred_last_imu_t_ = message.timestamp;
  }

  // Push back to our initializer
  if (!is_initialized_vio) {
    initializer->feed_imu(message, oldest_time);
  }

  // Push back to the zero velocity updater if it is enabled
  // No need to push back if we are just doing the zv-update at the begining and we have moved
  if (is_initialized_vio && updaterZUPT != nullptr && (!params.zupt_only_at_beginning || !has_moved_since_zupt)) {
    updaterZUPT->feed_imu(message, oldest_time);
  }
}

void VioManager::feed_measurement_simulation(double timestamp, const std::vector<int> &camids,
                                             const std::vector<std::vector<std::pair<size_t, Eigen::VectorXf>>> &feats) {

  // Start timing
  rT1 = boost::posix_time::microsec_clock::local_time();

  // Check if we actually have a simulated tracker
  // If not, recreate and re-cast the tracker to our simulation tracker
  std::shared_ptr<TrackSIM> trackSIM = std::dynamic_pointer_cast<TrackSIM>(trackFEATS);
  if (trackSIM == nullptr) {
    // Replace with the simulated tracker
    trackSIM = std::make_shared<TrackSIM>(state->_cam_intrinsics_cameras, state->_options.max_aruco_features);
    trackFEATS = trackSIM;
    // Need to also replace it in init and zv-upt since it points to the trackFEATS db pointer
    initializer = std::make_shared<ov_init::InertialInitializer>(params.init_options, trackFEATS->get_feature_database());
    if (params.try_zupt) {
      updaterZUPT = std::make_shared<UpdaterZeroVelocity>(params.zupt_options, params.imu_noises, trackFEATS->get_feature_database(),
                                                          propagator, params.gravity_mag, params.zupt_max_velocity,
                                                          params.zupt_noise_multiplier, params.zupt_max_disparity);
    }
    PRINT_WARNING(RED "[SIM]: casting our tracker to a TrackSIM object!\n" RESET);
  }

  // Feed our simulation tracker
  trackSIM->feed_measurement_simulation(timestamp, camids, feats);
  rT2 = boost::posix_time::microsec_clock::local_time();

  // Check if we should do zero-velocity, if so update the state with it
  // Note that in the case that we only use in the beginning initialization phase
  // If we have since moved, then we should never try to do a zero velocity update!
  if (is_initialized_vio && updaterZUPT != nullptr && (!params.zupt_only_at_beginning || !has_moved_since_zupt)) {
    // If the same state time, use the previous timestep decision
    if (state->_timestamp != timestamp) {
      did_zupt_update = updaterZUPT->try_update(state, timestamp);
    }
    if (did_zupt_update) {
      assert(state->_timestamp == timestamp);
      propagator->clean_old_imu_measurements(timestamp + state->_calib_dt_CAMtoIMU->value()(0) - 0.10);
      updaterZUPT->clean_old_imu_measurements(timestamp + state->_calib_dt_CAMtoIMU->value()(0) - 0.10);
      propagator->invalidate_cache();
      return;
    }
  }

  // If we do not have VIO initialization, then return an error
  if (!is_initialized_vio) {
    PRINT_ERROR(RED "[SIM]: your vio system should already be initialized before simulating features!!!\n" RESET);
    PRINT_ERROR(RED "[SIM]: initialize your system first before calling feed_measurement_simulation()!!!!\n" RESET);
    std::exit(EXIT_FAILURE);
  }

  // Call on our propagate and update function
  // Simulation is either all sync, or single camera...
  ov_core::CameraData message;
  message.timestamp = timestamp;
  for (auto const &camid : camids) {
    int width = state->_cam_intrinsics_cameras.at(camid)->w();
    int height = state->_cam_intrinsics_cameras.at(camid)->h();
    message.sensor_ids.push_back(camid);
    message.images.push_back(cv::Mat::zeros(cv::Size(width, height), CV_8UC1));
    message.masks.push_back(cv::Mat::zeros(cv::Size(width, height), CV_8UC1));
  }
  do_feature_propagate_update(message);
}

void VioManager::get_depth_err_hist(long long &n, double &sum, long long hist[6]) const {
  n = 0;
  sum = 0.0;
  for (int i = 0; i < 6; ++i) hist[i] = 0;
  if (!updaterMSCKF) return;
  n = updaterMSCKF->diag_deperr_n;
  sum = updaterMSCKF->diag_deperr_sum;
  for (int i = 0; i < 6; ++i) hist[i] = updaterMSCKF->diag_deperr_hist[i];
}

long long VioManager::get_depth_used() const {
  return updaterMSCKF ? updaterMSCKF->diag_depth_used : 0;
}

void VioManager::set_depth_image(const cv::Mat &depth_mm, double timestamp, double fx, double fy,
                                 double cx, double cy, const double R_c2d[9],
                                 const double t_c2d_m[3]) {
  if (depth_mm.empty() || fx <= 0.0)
    return;
  // 留 40 帧历史：特征的锚点观测可能是几帧之前的
  depth_hist_.push_back({timestamp, depth_mm});
  while (depth_hist_.size() > 40)
    depth_hist_.pop_front();
  depth_fx_ = fx;
  depth_fy_ = fy;
  depth_cx_ = cx;
  depth_cy_ = cy;
  // 诊断开关：OV_DEPTH_NO_EXTR=1 时强制把两相机当重合，用来看外参到底贡献多少误差
  static const bool no_extr = (std::getenv("OV_DEPTH_NO_EXTR") != nullptr);
  if (R_c2d != nullptr && !no_extr) {
    for (int r = 0; r < 3; ++r)
      for (int c = 0; c < 3; ++c) R_c2d_(r, c) = R_c2d[r * 3 + c];
  }
  if (t_c2d_m != nullptr && !no_extr)
    t_c2d_m_ = Eigen::Vector3d(t_c2d_m[0], t_c2d_m[1], t_c2d_m[2]);

  if (!use_depth_init_)
    return;

  // 每 300 张深度图打一次门控统计（不依赖宿主程序的诊断开关）
  if ((++diag_depth_frames % 300) == 0) {
    std::printf("[DEPTH] 深度帧 %lld：无最近帧 %lld，太旧(>35ms) %lld，边缘误配拒绝 %lld\n",
                diag_depth_frames, diag_depth_no_frame, diag_depth_too_old, diag_depth_edge_rej);
    std::fflush(stdout);
  }

  // 安装查询回调：彩色像素 (u,v) + 时刻 t → 米制深度
  UpdaterMSCKF::DepthLookup::fn = [this](double t, float u, float v) -> float {
    // 1) 找时刻最接近的深度帧（30fps 下同一个锚点时刻能精确命中）
    const DepthFrameRec *best = nullptr;
    double best_dt = 1e9;
    for (const auto &rec : depth_hist_) {
      const double dt = std::fabs(rec.t - t);
      if (dt < best_dt) {
        best_dt = dt;
        best = &rec;
      }
    }
    //=====================================================================
    // 【2026-10-23】深度年龄门限 50ms -> 35ms，并统计拒绝率。
    //
    // 实测（相机侧）：彩色与深度的设备时间戳差 p50 17-30 ms、p95 33-35 ms
    // （--aggregate color 下有一半帧带的是上一个深度帧）。
    // 50 ms 在 0.9 m/s、f=452、Z=2 m 时是 15 px 的错配 —— 比 deperr 底还大。
    // 35 ms 覆盖实测 p95，只切掉最老的那一小部分。
    //=====================================================================
    constexpr double kMaxDepthAgeS = 0.035;
    if (best == nullptr) { ++diag_depth_no_frame; return -1.0f; }
    if (best_dt > kMaxDepthAgeS) { ++diag_depth_too_old; return -1.0f; }
    // 2) 彩色像素去畸变 → 归一化视线 → 用深度内参投到深度图上
    auto cam_it = state->_cam_intrinsics_cameras.find(0);
    if (cam_it == state->_cam_intrinsics_cameras.end())
      return -1.0f;
    Eigen::Vector2d uv_n = cam_it->second->undistort_d(Eigen::Vector2d(u, v));
    const Eigen::Vector3d b(uv_n(0), uv_n(1), 1.0);   // 彩色相机系视线
    const Eigen::Vector3d Rb = R_c2d_ * b;

    // 【2026-10-21】严格投影：彩色射线上的点 P_c = λ·b，投到深度相机系是
    //   P_d = R_c2d·(λ·b) + t_c2d
    // 但 λ 未知（正是要求深度）。所以迭代两次：先用 2 m 猜，查到深度后反解 λ。
    // 之前的版本直接按"两相机重合"投影，实测外参平移 10.11 mm —— 1 米处等于
    // 13.7 像素的系统偏差，查深度时会采到错误的表面（尤其深度不连续处）。
    double lam = 2.0;
    double z_m = -1.0;
    for (int iter = 0; iter < 2; ++iter) {
      const Eigen::Vector3d Pd = R_c2d_ * (lam * b) + t_c2d_m_;
      if (Pd.z() < 0.05) return -1.0f;
      const int cx = static_cast<int>(std::lround(depth_fx_ * Pd.x() / Pd.z() + depth_cx_));
      const int cy = static_cast<int>(std::lround(depth_fy_ * Pd.y() / Pd.z() + depth_cy_));
      if (cx < 2 || cy < 2 || cx >= best->depth_mm.cols - 2 || cy >= best->depth_mm.rows - 2)
        return -1.0f;
      // 5x5 取有效值中位，抑制结构光空洞/噪声，也能挡掉部分深度不连续处的错值
      std::vector<unsigned short> vals;
      vals.reserve(25);
      for (int dy = -2; dy <= 2; ++dy) {
        const unsigned short *row = best->depth_mm.ptr<unsigned short>(cy + dy);
        for (int dx = -2; dx <= 2; ++dx) {
          const unsigned short z = row[cx + dx];
          if (z != 0 && z != 0xFFFF) vals.push_back(z);
        }
      }
      if (vals.size() < 5) return -1.0f;
      std::nth_element(vals.begin(), vals.begin() + vals.size() / 2, vals.end());
      const double d_m = static_cast<double>(vals[vals.size() / 2]) * 1e-3;
      //=================================================================
      // 【2026-10-23】深度边缘门控（外审第 6 条的实测版）
      //
      // 实测（ver3/res1，各 800 万像素）：13.9% 的像素 5x5 窗内极差 >10 cm，
      // 但只有 2.2% 的中值真的落在"另一个面"上。对那 2% 直接把深度点
      // 当精确值塞进状态是有害的（一个错的三维锚会污染后续所有更新）。
      // 判据：窗内极差 >15 cm 且 |中值-中心像素| >3 cm → 不用。
      //=================================================================
      {
        const unsigned short z_center = best->depth_mm.ptr<unsigned short>(cy)[cx];
        unsigned short vmin = vals[0], vmax = vals[0];
        for (auto v : vals) { vmin = std::min(vmin, v); vmax = std::max(vmax, v); }
        const double spread_m = static_cast<double>(vmax - vmin) * 1e-3;
        const double center_m = static_cast<double>(z_center) * 1e-3;
        if (z_center != 0 && z_center != 0xFFFF && spread_m > 0.15 &&
            std::fabs(d_m - center_m) > 0.03) {
          ++diag_depth_edge_rej;
          return -1.0f;
        }
      }
      // P_d.z = (R·b).z·λ + t.z = d  ⇒  λ = (d − t.z)/(R·b).z
      if (std::fabs(Rb.z()) < 1e-6) return -1.0f;
      const double lam_new = (d_m - t_c2d_m_.z()) / Rb.z();
      if (lam_new < 0.05 || lam_new > 20.0) return -1.0f;
      z_m = lam_new;
      if (std::fabs(lam_new - lam) < 1e-3) break;
      lam = lam_new;
    }
    if (z_m < 0.0) return -1.0f;
    // 返回"沿彩色视线方向的尺度 λ"（不是深度相机系的 z），
    // 因为上层用 p_FinG = p_AinG + R_GtoAᵀ·(b·λ) 重建位置。
    return static_cast<float>(z_m);
  };
}

double VioManager::get_camera_timeoffset() const {
  if (!state || !state->_calib_dt_CAMtoIMU)
    return 0.0;
  return state->_calib_dt_CAMtoIMU->value()(0);
}

bool VioManager::inflate_covariance(double scale) {
  if (!state || !is_initialized_vio)
    return false;
  state->inflate_covariance(scale);
  return true;
}

double VioManager::get_msckf_tri_rate() const {
  if (!updaterMSCKF) return -1.0;
  if (updaterMSCKF->diag_feat_in <= 0) return -1.0;
  return static_cast<double>(updaterMSCKF->diag_tri_ok) / static_cast<double>(updaterMSCKF->diag_feat_in);
}

void VioManager::get_msckf_stats(long long &calls, long long &feat_in, long long &chi2_rej,
                                 long long &updates, long long &rows,
                                 long long &clean_drop, long long &tri_drop) const {
  calls = feat_in = chi2_rej = updates = rows = 0;
  clean_drop = tri_drop = 0;
  if (!updaterMSCKF) return;
  calls = updaterMSCKF->diag_calls;
  feat_in = updaterMSCKF->diag_feat_in;
  chi2_rej = updaterMSCKF->diag_feat_chi2_rej;
  updates = updaterMSCKF->diag_updates;
  rows = updaterMSCKF->diag_rows;
  clean_drop = updaterMSCKF->diag_clean_drop;
  tri_drop = updaterMSCKF->diag_tri_drop;
}

void VioManager::track_image_and_update(const ov_core::CameraData &message_const) {

  // Start timing
  rT1 = boost::posix_time::microsec_clock::local_time();

  // Assert we have valid measurement data and ids
  assert(!message_const.sensor_ids.empty());
  assert(message_const.sensor_ids.size() == message_const.images.size());
  for (size_t i = 0; i < message_const.sensor_ids.size() - 1; i++) {
    assert(message_const.sensor_ids.at(i) != message_const.sensor_ids.at(i + 1));
  }

  // Downsample if we are downsampling
  ov_core::CameraData message = message_const;
  for (size_t i = 0; i < message.sensor_ids.size() && params.downsample_cameras; i++) {
    cv::Mat img = message.images.at(i);
    cv::Mat mask = message.masks.at(i);
    cv::Mat img_temp, mask_temp;
    cv::pyrDown(img, img_temp, cv::Size(img.cols / 2.0, img.rows / 2.0));
    message.images.at(i) = img_temp;
    cv::pyrDown(mask, mask_temp, cv::Size(mask.cols / 2.0, mask.rows / 2.0));
    message.masks.at(i) = mask_temp;
  }

  // 【2026-10-21】把累计的陀螺旋转（IMU 系）转到相机系，交给 tracker 当 KLT 初值。
  // R_C1toC2 = R_ItoC * R_I1toI2 * R_ItoC^T
  // 默认关闭：实测 tri 率 +70%，但 3 个离线 session 里 2 个位移变差（3.78->6.41, 3.86->6.06）。
  // 想用/做对照就设 OV_GYRO_PREDICT=1。
  static const bool gyro_predict_on = (std::getenv("OV_GYRO_PREDICT") != nullptr);
  if (gyro_predict_on && gyro_pred_last_cam_t_ > 0.0 && !message.sensor_ids.empty()) {
    const int cam_id0 = message.sensor_ids.at(0);
    auto it_calib = state->_calib_IMUtoCAM.find(cam_id0);
    if (it_calib != state->_calib_IMUtoCAM.end()) {
      const Eigen::Matrix3d R_ItoC = it_calib->second->Rot();
      Eigen::Matrix3d R_pred = R_ItoC * gyro_pred_R_ * R_ItoC.transpose();
      const double tr = std::min(1.0, std::max(-1.0, (R_pred.trace() - 1.0) * 0.5));
      const double ang = std::acos(tr);
      auto klt = std::dynamic_pointer_cast<ov_core::TrackKLT>(trackFEATS);
      if (klt && std::isfinite(ang) && ang < 0.35) // < 20°，超过就不敢信
        klt->set_predicted_rotation(R_pred);
      else if (klt)
        klt->clear_predicted_rotation();
    }
  }
  // 取用完毕：清零，开始累计到下一帧
  gyro_pred_R_.setIdentity();
  gyro_pred_last_imu_t_ = -1.0;
  gyro_pred_last_cam_t_ = message.timestamp;

  // Perform our feature tracking!
  trackFEATS->feed_new_camera(message);

  // 【健康监测】每帧都采样：这一帧有多少特征、这帧有没有产生 MSCKF 更新。
  // 必须放在跟踪之后、任何 cleanup 之前；放在 do_feature_propagate_update 里会因为
  // 未初始化/ZUPT 提前 return 而漏采，放在函数末尾会被 cleanup() 删空。
  frame_health.features = static_cast<int>(
      trackFEATS->get_feature_database()->features_containing(message.timestamp, false, false).size());
  frame_health.msckf_updated = false;
  frame_health.time = message.timestamp;

  // 【可视化】把当前帧跟踪到的像素点抄出来，供实时查看器画到图像上。
  // 同样必须在 cleanup() 之前；它只读跟踪器的 last_obs，不改变估计行为。
  tracked_points.clear();
  {
    const auto last_obs = trackFEATS->get_last_obs();
    const auto last_ids = trackFEATS->get_last_ids();
    auto db = trackFEATS->get_feature_database();
    for (const auto &cam : last_obs) {
      const auto ids_it = last_ids.find(cam.first);
      for (size_t k = 0; k < cam.second.size(); ++k) {
        TrackedPoint tp;
        tp.u = cam.second[k].pt.x;
        tp.v = cam.second[k].pt.y;
        if (ids_it != last_ids.end() && k < ids_it->second.size()) {
          auto feat = db->get_feature(ids_it->second[k]);
          if (feat) {
            auto t_it = feat->timestamps.find(cam.first);
            if (t_it != feat->timestamps.end()) tp.len = static_cast<int>(t_it->second.size());
          }
        }
        tracked_points.push_back(tp);
      }
    }
  }

  // If the aruco tracker is available, the also pass to it
  // NOTE: binocular tracking for aruco doesn't make sense as we by default have the ids
  // NOTE: thus we just call the stereo tracking if we are doing binocular!
  if (is_initialized_vio && trackARUCO != nullptr) {
    trackARUCO->feed_new_camera(message);
  }
  rT2 = boost::posix_time::microsec_clock::local_time();

  // Check if we should do zero-velocity, if so update the state with it
  // Note that in the case that we only use in the beginning initialization phase
  // If we have since moved, then we should never try to do a zero velocity update!
  if (is_initialized_vio && updaterZUPT != nullptr && (!params.zupt_only_at_beginning || !has_moved_since_zupt)) {
    // If the same state time, use the previous timestep decision
    if (state->_timestamp != message.timestamp) {
      did_zupt_update = updaterZUPT->try_update(state, message.timestamp);
    }
    if (did_zupt_update) {
      assert(state->_timestamp == message.timestamp);
      propagator->clean_old_imu_measurements(message.timestamp + state->_calib_dt_CAMtoIMU->value()(0) - 0.10);
      updaterZUPT->clean_old_imu_measurements(message.timestamp + state->_calib_dt_CAMtoIMU->value()(0) - 0.10);
      propagator->invalidate_cache();
      return;
    }
  }

  // If we do not have VIO initialization, then try to initialize
  // TODO: Or if we are trying to reset the system, then do that here!
  if (!is_initialized_vio) {
    is_initialized_vio = try_to_initialize(message);
    if (!is_initialized_vio) {
      double time_track = (rT2 - rT1).total_microseconds() * 1e-6;
      PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for tracking\n" RESET, time_track);
      return;
    }
  }

  // Call on our propagate and update function
  do_feature_propagate_update(message);
}

void VioManager::do_feature_propagate_update(const ov_core::CameraData &message) {

  //===================================================================================
  // 【2026-10-21 诊断插桩】逐帧 CSV
  // 目的：定位"跑 ~10 秒后 MSCKF 更新率从 20+/30 塌到 0/30"到底是哪一步先变。
  // 只有设置环境变量 OV_DIAG_CSV=<路径> 时才写文件，默认零开销、不改变算法行为。
  //===================================================================================
  static std::ofstream diag_of;
  static bool diag_checked = false;
  if (!diag_checked) {
    diag_checked = true;
    const char *diag_path = getenv("OV_DIAG_CSV");
    if (diag_path != nullptr && diag_path[0] != '\0') {
      diag_of.open(diag_path, std::ios::out | std::ios::trunc);
      diag_of << "t_rel,ts,n_feat,feat_lost,feat_marg,feat_maxtrack,msckf_raw,msckf_cap,slam_upd,"
                 "calls,feat_in,clean_drop,tri_drop,chi2_rej,updates,rows,mean_obs,mean_tracklen,"
                 "tri_ok,chi2_ok,ctmeas,comp_zero,tri_fail_tri,tri_fail_refine,"
                 "chi2_rej_n,chi2_rej_ratio_mean,chi2_rej_ratio_max,chi2_ok_ratio_mean,"
                 "rej_lt2,rej_lt5,rej_lt20,rej_lt200,rej_ge200,"
                 "n_clones,n_slam_state,vx,vy,vz,pz,bgx,bgy,bgz,bax,bay,baz,"
                 "sig_p,sig_v,sig_bg,sig_ba,time_msckf,time_marg\n";
      diag_of.flush();
    }
  }
  int d_lost = 0, d_marg = 0, d_max = 0, d_raw = 0, d_cap = 0, d_slamupd = 0;
  double d_mean_obs = -1.0, d_mean_len = -1.0;
  long long d_c0 = 0, d_fi0 = 0, d_cd0 = 0, d_td0 = 0, d_cr0 = 0, d_u0 = 0, d_r0 = 0;
  long long d_tok0 = 0, d_chiok0 = 0, d_ctm0 = 0, d_cz0 = 0, d_tft0 = 0, d_tfr0 = 0;
  double d_rr0 = 0.0, d_ro0 = 0.0;
  long long d_rh0[5] = {0, 0, 0, 0, 0};

  //===================================================================================
  // State propagation, and clone augmentation
  //===================================================================================

  // Return if the camera measurement is out of order
  if (state->_timestamp > message.timestamp) {
    PRINT_WARNING(YELLOW "image received out of order, unable to do anything (prop dt = %3f)\n" RESET,
                  (message.timestamp - state->_timestamp));
    return;
  }

  // Propagate the state forward to the current update time
  // Also augment it with a new clone!
  // NOTE: if the state is already at the given time (can happen in sim)
  // NOTE: then no need to prop since we already are at the desired timestep
  if (state->_timestamp != message.timestamp) {
    propagator->propagate_and_clone(state, message.timestamp);
  }
  rT3 = boost::posix_time::microsec_clock::local_time();

  // If we have not reached max clones, we should just return...
  // This isn't super ideal, but it keeps the logic after this easier...
  // We can start processing things when we have at least 5 clones since we can start triangulating things...
  if ((int)state->_clones_IMU.size() < std::min(state->_options.max_clone_size, 5)) {
    PRINT_DEBUG("waiting for enough clone states (%d of %d)....\n", (int)state->_clones_IMU.size(),
                std::min(state->_options.max_clone_size, 5));
    return;
  }

  // Return if we where unable to propagate
  if (state->_timestamp != message.timestamp) {
    PRINT_WARNING(RED "[PROP]: Propagator unable to propagate the state forward in time!\n" RESET);
    PRINT_WARNING(RED "[PROP]: It has been %.3f since last time we propagated\n" RESET, message.timestamp - state->_timestamp);
    return;
  }
  has_moved_since_zupt = true;

  //===================================================================================
  // MSCKF features and KLT tracks that are SLAM features
  //===================================================================================

  // Now, lets get all features that should be used for an update that are lost in the newest frame
  // We explicitly request features that have not been deleted (used) in another update step
  std::vector<std::shared_ptr<Feature>> feats_lost, feats_marg, feats_slam;
  feats_lost = trackFEATS->get_feature_database()->features_not_containing_newer(state->_timestamp, false, true);

  // Don't need to get the oldest features until we reach our max number of clones
  if ((int)state->_clones_IMU.size() > state->_options.max_clone_size || (int)state->_clones_IMU.size() > 5) {
    feats_marg = trackFEATS->get_feature_database()->features_containing(state->margtimestep(), false, true);
    if (trackARUCO != nullptr && message.timestamp - startup_time >= params.dt_slam_delay) {
      feats_slam = trackARUCO->get_feature_database()->features_containing(state->margtimestep(), false, true);
    }
  }

  // Remove any lost features that were from other image streams
  // E.g: if we are cam1 and cam0 has not processed yet, we don't want to try to use those in the update yet
  // E.g: thus we wait until cam0 process its newest image to remove features which were seen from that camera
  auto it1 = feats_lost.begin();
  while (it1 != feats_lost.end()) {
    bool found_current_message_camid = false;
    for (const auto &camuvpair : (*it1)->uvs) {
      if (std::find(message.sensor_ids.begin(), message.sensor_ids.end(), camuvpair.first) != message.sensor_ids.end()) {
        found_current_message_camid = true;
        break;
      }
    }
    if (found_current_message_camid) {
      it1++;
    } else {
      it1 = feats_lost.erase(it1);
    }
  }

  // We also need to make sure that the max tracks does not contain any lost features
  // This could happen if the feature was lost in the last frame, but has a measurement at the marg timestep
  it1 = feats_lost.begin();
  while (it1 != feats_lost.end()) {
    if (std::find(feats_marg.begin(), feats_marg.end(), (*it1)) != feats_marg.end()) {
      // PRINT_WARNING(YELLOW "FOUND FEATURE THAT WAS IN BOTH feats_lost and feats_marg!!!!!!\n" RESET);
      it1 = feats_lost.erase(it1);
    } else {
      it1++;
    }
  }

  d_lost = (int)feats_lost.size();
  d_marg = (int)feats_marg.size();

  // Find tracks that have reached max length, these can be made into SLAM features
  std::vector<std::shared_ptr<Feature>> feats_maxtracks;
  auto it2 = feats_marg.begin();
  while (it2 != feats_marg.end()) {
    // See if any of our camera's reached max track
    bool reached_max = false;
    for (const auto &cams : (*it2)->timestamps) {
      if ((int)cams.second.size() > state->_options.max_clone_size) {
        reached_max = true;
        break;
      }
    }
    // If max track, then add it to our possible slam feature list
    if (reached_max) {
      feats_maxtracks.push_back(*it2);
      it2 = feats_marg.erase(it2);
    } else {
      it2++;
    }
  }

  // Count how many aruco tags we have in our state
  int curr_aruco_tags = 0;
  auto it0 = state->_features_SLAM.begin();
  while (it0 != state->_features_SLAM.end()) {
    if ((int)(*it0).second->_featid <= 4 * state->_options.max_aruco_features)
      curr_aruco_tags++;
    it0++;
  }

  // Append a new SLAM feature if we have the room to do so
  // Also check that we have waited our delay amount (normally prevents bad first set of slam points)
  if (state->_options.max_slam_features > 0 && message.timestamp - startup_time >= params.dt_slam_delay &&
      (int)state->_features_SLAM.size() < state->_options.max_slam_features + curr_aruco_tags) {
    // Get the total amount to add, then the max amount that we can add given our marginalize feature array
    int amount_to_add = (state->_options.max_slam_features + curr_aruco_tags) - (int)state->_features_SLAM.size();
    int valid_amount = (amount_to_add > (int)feats_maxtracks.size()) ? (int)feats_maxtracks.size() : amount_to_add;
    // If we have at least 1 that we can add, lets add it!
    // Note: we remove them from the feat_marg array since we don't want to reuse information...
    if (valid_amount > 0) {
      feats_slam.insert(feats_slam.end(), feats_maxtracks.end() - valid_amount, feats_maxtracks.end());
      feats_maxtracks.erase(feats_maxtracks.end() - valid_amount, feats_maxtracks.end());
    }
  }

  // Loop through current SLAM features, we have tracks of them, grab them for this update!
  // NOTE: if we have a slam feature that has lost tracking, then we should marginalize it out
  // NOTE: we only enforce this if the current camera message is where the feature was seen from
  // NOTE: if you do not use FEJ, these types of slam features *degrade* the estimator performance....
  // NOTE: we will also marginalize SLAM features if they have failed their update a couple times in a row
  for (std::pair<const size_t, std::shared_ptr<Landmark>> &landmark : state->_features_SLAM) {
    if (trackARUCO != nullptr) {
      std::shared_ptr<Feature> feat1 = trackARUCO->get_feature_database()->get_feature(landmark.second->_featid);
      if (feat1 != nullptr)
        feats_slam.push_back(feat1);
    }
    std::shared_ptr<Feature> feat2 = trackFEATS->get_feature_database()->get_feature(landmark.second->_featid);
    if (feat2 != nullptr)
      feats_slam.push_back(feat2);
    assert(landmark.second->_unique_camera_id != -1);
    bool current_unique_cam =
        std::find(message.sensor_ids.begin(), message.sensor_ids.end(), landmark.second->_unique_camera_id) != message.sensor_ids.end();
    if (feat2 == nullptr && current_unique_cam)
      landmark.second->should_marg = true;
    if (landmark.second->update_fail_count > 1)
      landmark.second->should_marg = true;
  }

  // Lets marginalize out all old SLAM features here
  // These are ones that where not successfully tracked into the current frame
  // We do *NOT* marginalize out our aruco tags landmarks
  StateHelper::marginalize_slam(state);

  // Separate our SLAM features into new ones, and old ones
  std::vector<std::shared_ptr<Feature>> feats_slam_DELAYED, feats_slam_UPDATE;
  for (size_t i = 0; i < feats_slam.size(); i++) {
    if (state->_features_SLAM.find(feats_slam.at(i)->featid) != state->_features_SLAM.end()) {
      feats_slam_UPDATE.push_back(feats_slam.at(i));
      // PRINT_DEBUG("[UPDATE-SLAM]: found old feature %d (%d
      // measurements)\n",(int)feats_slam.at(i)->featid,(int)feats_slam.at(i)->timestamps_left.size());
    } else {
      feats_slam_DELAYED.push_back(feats_slam.at(i));
      // PRINT_DEBUG("[UPDATE-SLAM]: new feature ready %d (%d
      // measurements)\n",(int)feats_slam.at(i)->featid,(int)feats_slam.at(i)->timestamps_left.size());
    }
  }

  d_max = (int)feats_maxtracks.size();
  d_slamupd = (int)feats_slam_UPDATE.size();

  // Concatenate our MSCKF feature arrays (i.e., ones not being used for slam updates)
  std::vector<std::shared_ptr<Feature>> featsup_MSCKF = feats_lost;
  featsup_MSCKF.insert(featsup_MSCKF.end(), feats_marg.begin(), feats_marg.end());
  featsup_MSCKF.insert(featsup_MSCKF.end(), feats_maxtracks.begin(), feats_maxtracks.end());

  //===================================================================================
  // Now that we have a list of features, lets do the EKF update for MSCKF and SLAM!
  //===================================================================================

  // Sort based on track length
  // TODO: we should have better selection logic here (i.e. even feature distribution in the FOV etc..)
  // TODO: right now features that are "lost" are at the front of this vector, while ones at the end are long-tracks
  auto compare_feat = [](const std::shared_ptr<Feature> &a, const std::shared_ptr<Feature> &b) -> bool {
    size_t asize = 0;
    size_t bsize = 0;
    for (const auto &pair : a->timestamps)
      asize += pair.second.size();
    for (const auto &pair : b->timestamps)
      bsize += pair.second.size();
    return asize < bsize;
  };
  std::sort(featsup_MSCKF.begin(), featsup_MSCKF.end(), compare_feat);

  d_raw = (int)featsup_MSCKF.size();

  // Pass them to our MSCKF updater
  // NOTE: if we have more then the max, we select the "best" ones (i.e. max tracks) for this update
  // NOTE: this should only really be used if you want to track a lot of features, or have limited computational resources
  if ((int)featsup_MSCKF.size() > state->_options.max_msckf_in_update)
    featsup_MSCKF.erase(featsup_MSCKF.begin(), featsup_MSCKF.end() - state->_options.max_msckf_in_update);
  d_cap = (int)featsup_MSCKF.size();

  // 诊断：这一批特征平均有几个观测、平均跨几帧（塌缩时这两个数会先变）
  if (!featsup_MSCKF.empty()) {
    double sum_obs = 0.0, sum_len = 0.0;
    for (const auto &feat : featsup_MSCKF) {
      size_t maxlen = 0;
      for (const auto &cam_ts : feat->timestamps) {
        sum_obs += (double)cam_ts.second.size();
        if (cam_ts.second.size() > maxlen)
          maxlen = cam_ts.second.size();
      }
      sum_len += (double)maxlen;
    }
    d_mean_obs = sum_obs / (double)featsup_MSCKF.size();
    d_mean_len = sum_len / (double)featsup_MSCKF.size();
  }

  if (updaterMSCKF) {
    d_c0 = updaterMSCKF->diag_calls;
    d_fi0 = updaterMSCKF->diag_feat_in;
    d_cd0 = updaterMSCKF->diag_clean_drop;
    d_td0 = updaterMSCKF->diag_tri_drop;
    d_cr0 = updaterMSCKF->diag_feat_chi2_rej;
    d_u0 = updaterMSCKF->diag_updates;
    d_r0 = updaterMSCKF->diag_rows;
    d_tok0 = updaterMSCKF->diag_tri_ok;
    d_chiok0 = updaterMSCKF->diag_chi2_ok;
    d_ctm0 = updaterMSCKF->diag_ctmeas;
    d_cz0 = updaterMSCKF->diag_comp_zero;
    d_tft0 = updaterMSCKF->diag_tri_fail_tri;
    d_tfr0 = updaterMSCKF->diag_tri_fail_refine;
    d_rr0 = updaterMSCKF->diag_chi2_rej_ratio_sum;
    d_ro0 = updaterMSCKF->diag_chi2_ok_ratio_sum;
    for (int i = 0; i < 5; i++)
      d_rh0[i] = updaterMSCKF->diag_chi2_rej_hist[i];
  }
  const long long _ekf_calls0 = StateHelper::total_ekf_update_calls;
  const long long _upd_before = updaterMSCKF ? updaterMSCKF->diag_updates : 0;
  updaterMSCKF->update(state, featsup_MSCKF);
  if (updaterMSCKF && updaterMSCKF->diag_updates > _upd_before) {
    frame_health.msckf_updated = true;   // 这一帧真的产生了 MSCKF 更新
  }
  propagator->invalidate_cache();
  rT4 = boost::posix_time::microsec_clock::local_time();

  // Perform SLAM delay init and update
  // NOTE: that we provide the option here to do a *sequential* update
  // NOTE: this will be a lot faster but won't be as accurate.
  std::vector<std::shared_ptr<Feature>> feats_slam_UPDATE_TEMP;
  while (!feats_slam_UPDATE.empty()) {
    // Get sub vector of the features we will update with
    std::vector<std::shared_ptr<Feature>> featsup_TEMP;
    featsup_TEMP.insert(featsup_TEMP.begin(), feats_slam_UPDATE.begin(),
                        feats_slam_UPDATE.begin() + std::min(state->_options.max_slam_in_update, (int)feats_slam_UPDATE.size()));
    feats_slam_UPDATE.erase(feats_slam_UPDATE.begin(),
                            feats_slam_UPDATE.begin() + std::min(state->_options.max_slam_in_update, (int)feats_slam_UPDATE.size()));
    // Do the update
    updaterSLAM->update(state, featsup_TEMP);
    feats_slam_UPDATE_TEMP.insert(feats_slam_UPDATE_TEMP.end(), featsup_TEMP.begin(), featsup_TEMP.end());
    propagator->invalidate_cache();
  }
  feats_slam_UPDATE = feats_slam_UPDATE_TEMP;
  rT5 = boost::posix_time::microsec_clock::local_time();
  updaterSLAM->delayed_init(state, feats_slam_DELAYED);
  // 【2026-10-21】统计这一帧的**真实视觉更新**：MSCKF 和 SLAM 任何一条走了 EKFUpdate 都算。
  // 原来只统计 MSCKF 批次，只走 SLAM 的帧会被健康门限当成"没更新"。
  frame_health.any_visual_update =
      (StateHelper::total_ekf_update_calls > _ekf_calls0);
  frame_health.slam_updated =
      frame_health.any_visual_update && !frame_health.msckf_updated;
  rT6 = boost::posix_time::microsec_clock::local_time();

  //===================================================================================
  // Update our visualization feature set, and clean up the old features
  //===================================================================================

  // Re-triangulate all current tracks in the current frame
  if (message.sensor_ids.at(0) == 0) {

    // Re-triangulate features
    retriangulate_active_tracks(message);

    // Clear the MSCKF features only on the base camera
    // Thus we should be able to visualize the other unique camera stream
    // MSCKF features as they will also be appended to the vector
    good_features_MSCKF.clear();
  }

  // Save all the MSCKF features used in the update
  for (auto const &feat : featsup_MSCKF) {
    good_features_MSCKF.push_back(feat->p_FinG);
    feat->to_delete = true;
  }

  //===================================================================================
  // Cleanup, marginalize out what we don't need any more...
  //===================================================================================

  // Remove features that where used for the update from our extractors at the last timestep
  // This allows for measurements to be used in the future if they failed to be used this time
  // Note we need to do this before we feed a new image, as we want all new measurements to NOT be deleted
  trackFEATS->get_feature_database()->cleanup();
  if (trackARUCO != nullptr) {
    trackARUCO->get_feature_database()->cleanup();
  }

  // First do anchor change if we are about to lose an anchor pose
  updaterSLAM->change_anchors(state);

  // Cleanup any features older than the marginalization time
  if ((int)state->_clones_IMU.size() > state->_options.max_clone_size) {
    trackFEATS->get_feature_database()->cleanup_measurements(state->margtimestep());
    if (trackARUCO != nullptr) {
      trackARUCO->get_feature_database()->cleanup_measurements(state->margtimestep());
    }
  }

  // Finally marginalize the oldest clone if needed
  StateHelper::marginalize_old_clone(state);
  rT7 = boost::posix_time::microsec_clock::local_time();



  //===================================================================================
  // Debug info, and stats tracking
  //===================================================================================

  // Get timing statitics information
  double time_track = (rT2 - rT1).total_microseconds() * 1e-6;
  double time_prop = (rT3 - rT2).total_microseconds() * 1e-6;
  double time_msckf = (rT4 - rT3).total_microseconds() * 1e-6;
  double time_slam_update = (rT5 - rT4).total_microseconds() * 1e-6;
  double time_slam_delay = (rT6 - rT5).total_microseconds() * 1e-6;
  double time_marg = (rT7 - rT6).total_microseconds() * 1e-6;
  double time_total = (rT7 - rT1).total_microseconds() * 1e-6;

  //===================================================================================
  // 【2026-10-21 诊断】逐帧写一行 CSV（仅在 OV_DIAG_CSV 被设置时）
  //===================================================================================
  if (diag_of.is_open() && updaterMSCKF) {
    const Eigen::Matrix<double, 3, 1> v = state->_imu->vel();
    const Eigen::Matrix<double, 3, 1> bg = state->_imu->bias_g();
    const Eigen::Matrix<double, 3, 1> ba = state->_imu->bias_a();
    const double sig_p =
        std::sqrt(std::max(0.0, state->cov_diag(3) + state->cov_diag(4) + state->cov_diag(5)));
    const double sig_v =
        std::sqrt(std::max(0.0, state->cov_diag(6) + state->cov_diag(7) + state->cov_diag(8)));
    const double sig_bg =
        std::sqrt(std::max(0.0, state->cov_diag(9) + state->cov_diag(10) + state->cov_diag(11)));
    const double sig_ba =
        std::sqrt(std::max(0.0, state->cov_diag(12) + state->cov_diag(13) + state->cov_diag(14)));
    diag_of << std::fixed << std::setprecision(6) << (message.timestamp - startup_time) << "," << message.timestamp << ","
            << frame_health.features << "," << d_lost << "," << d_marg << "," << d_max << "," << d_raw << "," << d_cap << ","
            << d_slamupd << "," << (updaterMSCKF->diag_calls - d_c0) << "," << (updaterMSCKF->diag_feat_in - d_fi0) << ","
            << (updaterMSCKF->diag_clean_drop - d_cd0) << "," << (updaterMSCKF->diag_tri_drop - d_td0) << ","
            << (updaterMSCKF->diag_feat_chi2_rej - d_cr0) << "," << (updaterMSCKF->diag_updates - d_u0) << ","
            << (updaterMSCKF->diag_rows - d_r0) << "," << d_mean_obs << "," << d_mean_len << ","
            << (updaterMSCKF->diag_tri_ok - d_tok0) << "," << (updaterMSCKF->diag_chi2_ok - d_chiok0) << ","
            << (updaterMSCKF->diag_ctmeas - d_ctm0) << "," << (updaterMSCKF->diag_comp_zero - d_cz0) << ","
            << (updaterMSCKF->diag_tri_fail_tri - d_tft0) << "," << (updaterMSCKF->diag_tri_fail_refine - d_tfr0) << ",";
    {
      const long long nrej = updaterMSCKF->diag_feat_chi2_rej - d_cr0;
      const double rr = updaterMSCKF->diag_chi2_rej_ratio_sum - d_rr0;
      const double ro = updaterMSCKF->diag_chi2_ok_ratio_sum - d_ro0;
      const long long nok = updaterMSCKF->diag_chi2_ok - d_chiok0;
      diag_of << nrej << "," << (nrej > 0 ? rr / (double)nrej : -1.0) << "," << updaterMSCKF->diag_chi2_rej_ratio_max
              << "," << (nok > 0 ? ro / (double)nok : -1.0) << ",";
      for (int i = 0; i < 5; i++)
        diag_of << (updaterMSCKF->diag_chi2_rej_hist[i] - d_rh0[i]) << ",";
    }
    diag_of << (int)state->_clones_IMU.size()
            << "," << (int)state->_features_SLAM.size() << "," << v(0) << "," << v(1) << "," << v(2) << "," << state->_imu->pos()(2)
            << "," << bg(0) << "," << bg(1) << "," << bg(2) << "," << ba(0) << "," << ba(1) << "," << ba(2) << "," << sig_p << ","
            << sig_v << "," << sig_bg << "," << sig_ba << "," << time_msckf << "," << time_marg << "\n";
    diag_of.flush();
  }

  // Timing information
  PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for tracking\n" RESET, time_track);
  PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for propagation\n" RESET, time_prop);
  PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for MSCKF update (%d feats)\n" RESET, time_msckf, (int)featsup_MSCKF.size());
  if (state->_options.max_slam_features > 0) {
    PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for SLAM update (%d feats)\n" RESET, time_slam_update, (int)state->_features_SLAM.size());
    PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for SLAM delayed init (%d feats)\n" RESET, time_slam_delay, (int)feats_slam_DELAYED.size());
  }
  PRINT_DEBUG(BLUE "[TIME]: %.4f seconds for re-tri & marg (%d clones in state)\n" RESET, time_marg, (int)state->_clones_IMU.size());

  std::stringstream ss;
  ss << "[TIME]: " << std::setprecision(4) << time_total << " seconds for total (camera";
  for (const auto &id : message.sensor_ids) {
    ss << " " << id;
  }
  ss << ")" << std::endl;
  PRINT_DEBUG(BLUE "%s" RESET, ss.str().c_str());

  // Finally if we are saving stats to file, lets save it to file
  if (params.record_timing_information && of_statistics.is_open()) {
    // We want to publish in the IMU clock frame
    // The timestamp in the state will be the last camera time
    double t_ItoC = state->_calib_dt_CAMtoIMU->value()(0);
    double timestamp_inI = state->_timestamp + t_ItoC;
    // Append to the file
    of_statistics << std::fixed << std::setprecision(15) << timestamp_inI << "," << std::fixed << std::setprecision(5) << time_track << ","
                  << time_prop << "," << time_msckf << ",";
    if (state->_options.max_slam_features > 0) {
      of_statistics << time_slam_update << "," << time_slam_delay << ",";
    }
    of_statistics << time_marg << "," << time_total << std::endl;
    of_statistics.flush();
  }

  // Update our distance traveled
  if (timelastupdate != -1 && state->_clones_IMU.find(timelastupdate) != state->_clones_IMU.end()) {
    Eigen::Matrix<double, 3, 1> dx = state->_imu->pos() - state->_clones_IMU.at(timelastupdate)->pos();
    distance += dx.norm();
  }
  timelastupdate = message.timestamp;

  // Debug, print our current state
  // 【2026-10-21 改 INFO->DEBUG】这几行是"每帧打印整个状态"，实时跑起来会把控制台
  // 刷成一片（30 帧/秒 × 4 行），真正有用的 [VIO]/[cam] 状态行全被埋掉。
  // 改成 DEBUG：默认 INFO 级别不打印，需要时把配置 verbosity 设成 DEBUG 再看。
  PRINT_DEBUG("q_GtoI = %.3f,%.3f,%.3f,%.3f | p_IinG = %.3f,%.3f,%.3f | dist = %.2f (meters)\n", state->_imu->quat()(0),
             state->_imu->quat()(1), state->_imu->quat()(2), state->_imu->quat()(3), state->_imu->pos()(0), state->_imu->pos()(1),
             state->_imu->pos()(2), distance);
  PRINT_DEBUG("bg = %.4f,%.4f,%.4f | ba = %.4f,%.4f,%.4f\n", state->_imu->bias_g()(0), state->_imu->bias_g()(1), state->_imu->bias_g()(2),
             state->_imu->bias_a()(0), state->_imu->bias_a()(1), state->_imu->bias_a()(2));

  // Debug for camera imu offset
  if (state->_options.do_calib_camera_timeoffset) {
    PRINT_DEBUG("camera-imu timeoffset = %.5f\n", state->_calib_dt_CAMtoIMU->value()(0));
  }

  // Debug for camera intrinsics
  if (state->_options.do_calib_camera_intrinsics) {
    for (int i = 0; i < state->_options.num_cameras; i++) {
      std::shared_ptr<Vec> calib = state->_cam_intrinsics.at(i);
      PRINT_INFO("cam%d intrinsics = %.3f,%.3f,%.3f,%.3f | %.3f,%.3f,%.3f,%.3f\n", (int)i, calib->value()(0), calib->value()(1),
                 calib->value()(2), calib->value()(3), calib->value()(4), calib->value()(5), calib->value()(6), calib->value()(7));
    }
  }

  // Debug for camera extrinsics
  if (state->_options.do_calib_camera_pose) {
    for (int i = 0; i < state->_options.num_cameras; i++) {
      std::shared_ptr<PoseJPL> calib = state->_calib_IMUtoCAM.at(i);
      PRINT_INFO("cam%d extrinsics = %.3f,%.3f,%.3f,%.3f | %.3f,%.3f,%.3f\n", (int)i, calib->quat()(0), calib->quat()(1), calib->quat()(2),
                 calib->quat()(3), calib->pos()(0), calib->pos()(1), calib->pos()(2));
    }
  }

  // Debug for imu intrinsics
  if (state->_options.do_calib_imu_intrinsics && state->_options.imu_model == StateOptions::ImuModel::KALIBR) {
    PRINT_INFO("q_GYROtoI = %.3f,%.3f,%.3f,%.3f\n", state->_calib_imu_GYROtoIMU->value()(0), state->_calib_imu_GYROtoIMU->value()(1),
               state->_calib_imu_GYROtoIMU->value()(2), state->_calib_imu_GYROtoIMU->value()(3));
  }
  if (state->_options.do_calib_imu_intrinsics && state->_options.imu_model == StateOptions::ImuModel::RPNG) {
    PRINT_INFO("q_ACCtoI = %.3f,%.3f,%.3f,%.3f\n", state->_calib_imu_ACCtoIMU->value()(0), state->_calib_imu_ACCtoIMU->value()(1),
               state->_calib_imu_ACCtoIMU->value()(2), state->_calib_imu_ACCtoIMU->value()(3));
  }
  if (state->_options.do_calib_imu_intrinsics && state->_options.imu_model == StateOptions::ImuModel::KALIBR) {
    PRINT_INFO("Dw = | %.4f,%.4f,%.4f | %.4f,%.4f | %.4f |\n", state->_calib_imu_dw->value()(0), state->_calib_imu_dw->value()(1),
               state->_calib_imu_dw->value()(2), state->_calib_imu_dw->value()(3), state->_calib_imu_dw->value()(4),
               state->_calib_imu_dw->value()(5));
    PRINT_INFO("Da = | %.4f,%.4f,%.4f | %.4f,%.4f | %.4f |\n", state->_calib_imu_da->value()(0), state->_calib_imu_da->value()(1),
               state->_calib_imu_da->value()(2), state->_calib_imu_da->value()(3), state->_calib_imu_da->value()(4),
               state->_calib_imu_da->value()(5));
  }
  if (state->_options.do_calib_imu_intrinsics && state->_options.imu_model == StateOptions::ImuModel::RPNG) {
    PRINT_INFO("Dw = | %.4f | %.4f,%.4f | %.4f,%.4f,%.4f |\n", state->_calib_imu_dw->value()(0), state->_calib_imu_dw->value()(1),
               state->_calib_imu_dw->value()(2), state->_calib_imu_dw->value()(3), state->_calib_imu_dw->value()(4),
               state->_calib_imu_dw->value()(5));
    PRINT_INFO("Da = | %.4f | %.4f,%.4f | %.4f,%.4f,%.4f |\n", state->_calib_imu_da->value()(0), state->_calib_imu_da->value()(1),
               state->_calib_imu_da->value()(2), state->_calib_imu_da->value()(3), state->_calib_imu_da->value()(4),
               state->_calib_imu_da->value()(5));
  }
  if (state->_options.do_calib_imu_intrinsics && state->_options.do_calib_imu_g_sensitivity) {
    PRINT_INFO("Tg = | %.4f,%.4f,%.4f |  %.4f,%.4f,%.4f | %.4f,%.4f,%.4f |\n", state->_calib_imu_tg->value()(0),
               state->_calib_imu_tg->value()(1), state->_calib_imu_tg->value()(2), state->_calib_imu_tg->value()(3),
               state->_calib_imu_tg->value()(4), state->_calib_imu_tg->value()(5), state->_calib_imu_tg->value()(6),
               state->_calib_imu_tg->value()(7), state->_calib_imu_tg->value()(8));
  }
}
