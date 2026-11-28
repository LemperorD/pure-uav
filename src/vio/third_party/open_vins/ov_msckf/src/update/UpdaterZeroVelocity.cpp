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

#include "UpdaterZeroVelocity.h"

#include <algorithm>
#include <cmath>

#include "UpdaterHelper.h"

#include "feat/FeatureDatabase.h"
#include "feat/FeatureHelper.h"
#include "state/Propagator.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/distributions/chi_squared.hpp>

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

UpdaterZeroVelocity::UpdaterZeroVelocity(UpdaterOptions &options, NoiseManager &noises, std::shared_ptr<ov_core::FeatureDatabase> db,
                                         std::shared_ptr<Propagator> prop, double gravity_mag, double zupt_max_velocity,
                                         double zupt_noise_multiplier, double zupt_max_disparity, double zupt_max_omega_deg,
                                         double zupt_max_acc_dev, double zupt_sigma_v, int zupt_min_still_frames,
                                         bool zupt_v_only)
    : _options(options), _noises(noises), _db(db), _prop(prop), _zupt_max_velocity(zupt_max_velocity),
      _zupt_noise_multiplier(zupt_noise_multiplier), _zupt_max_disparity(zupt_max_disparity),
      _zupt_max_omega_deg(zupt_max_omega_deg), _zupt_max_acc_dev(zupt_max_acc_dev), _zupt_sigma_v(zupt_sigma_v),
      _zupt_min_still_frames(zupt_min_still_frames), still_window_(zupt_min_still_frames), _zupt_v_only(zupt_v_only) {

  // Gravity
  _gravity << 0.0, 0.0, gravity_mag;

  // Save our raw pixel noise squared
  _noises.sigma_w_2 = std::pow(_noises.sigma_w, 2);
  _noises.sigma_a_2 = std::pow(_noises.sigma_a, 2);
  _noises.sigma_wb_2 = std::pow(_noises.sigma_wb, 2);
  _noises.sigma_ab_2 = std::pow(_noises.sigma_ab, 2);

  // Initialize the chi squared test table with confidence level 0.95
  // https://github.com/KumarRobotics/msckf_vio/blob/050c50defa5a7fd9a04c1eed5687b405f02919b5/src/msckf_vio.cpp#L215-L221
  for (int i = 1; i < 1000; i++) {
    boost::math::chi_squared chi_squared_dist(i);
    chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
  }
}

void UpdaterZeroVelocity::clear_still_window() {
  still_window_.reset();
  _still_streak = 0;
  last_zupt_state_timestamp = 0.0;
  last_zupt_count = 0;
  _zupt_skipped_dt = 0.0;
}

void UpdaterZeroVelocity::reset_detection() {
  clear_still_window();
  imu_data.clear();
  have_last_prop_time_offset = false;
  last_pending_log_ = -1e9;
  last_update_log_ = -1e9;
}

bool UpdaterZeroVelocity::try_update(std::shared_ptr<State> state, double timestamp) {

  // Return if we don't have any imu data yet
  if (imu_data.empty()) {
    clear_still_window();
    return false;
  }

  // Return if the state is already at the desired time
  if (state->_timestamp == timestamp) {
    last_zupt_state_timestamp = 0.0;
    return false;
  }

  // Set the last time offset value if we have just started the system up
  if (!have_last_prop_time_offset) {
    last_prop_time_offset = state->_calib_dt_CAMtoIMU->value()(0);
    have_last_prop_time_offset = true;
  }

  // assert that the time we are requesting is in the future
  // assert(timestamp > state->_timestamp);

  // Get what our IMU-camera offset should be (t_imu = t_cam + calib_dt)
  double t_off_new = state->_calib_dt_CAMtoIMU->value()(0);

  // First lets construct an IMU vector of measurements we need
  // double time0 = state->_timestamp+t_off_new;
  double time0 = state->_timestamp + last_prop_time_offset;
  double time1 = timestamp + t_off_new;

  // A camera gap or unbracketed interval cannot provide consecutive stillness
  // evidence. The propagator can extrapolate, but that is unsuitable for ZUPT.
  if (_zupt_v_only && (timestamp - state->_timestamp > 0.10 || !(time1 > time0) ||
                       imu_data.front().timestamp > time0 || imu_data.back().timestamp < time1)) {
    clear_still_window();
    last_prop_time_offset = t_off_new;
    if (timestamp - last_pending_log_ >= 2.0) {
      last_pending_log_ = timestamp;
      PRINT_INFO("[ZUPT] 静止待确认: 相机间隔过长或 IMU 未覆盖该帧\n");
    }
    return false;
  }

  // Select bounding inertial measurements
  std::vector<ov_core::ImuData> imu_recent = Propagator::select_imu_readings(imu_data, time0, time1);

  // Move forward in time
  last_prop_time_offset = t_off_new;

  // Check that we have at least one measurement to propagate with
  if (imu_recent.size() < 2) {
    PRINT_WARNING(RED "[ZUPT]: There are no IMU data to check for zero velocity with!!\n" RESET);
    clear_still_window();
    return false;
  }

  //===================================================================================
  // 【2026-10-25】IMU 静止判据（独立于视觉的第二证据）
  //
  // 为什么加：上游只看图像位移（disparity）和卡方。两个都不够：
  //   · 图像位移会被重复纹理 + 暗光骗 —— 天花板条纹上 KLT 能把错误匹配一路跟下去；
  //   · 卡方在"姿态已经错了 11°"时必然爆表，于是最该救场的那次零速更新被拒。
  // IMU 判据不受这两件事影响：静止时 |ω| 有界、‖a‖ 一定等于 g。而且它只读观测值，
  // 不含任何状态估计，所以滤波器错得再离谱也不会影响这条判据。
  // 实测（turn4/turn5 的安静段）：|ω| 0.4~0.6 °/s、| ‖a‖ − g | < 0.02 m/s²。
  //===================================================================================
  double mean_w_deg = 0.0, mean_acc_dev = 0.0;
  double max_w_deg = 0.0, max_acc_dev = 0.0;
  {
    double sw = 0.0, sa = 0.0;
    for (const auto &m : imu_recent) {
      const double w_deg = m.wm.norm() * 180.0 / M_PI;
      const double a_dev = std::abs(m.am.norm() - _gravity.norm());
      sw += w_deg;
      sa += a_dev;
      max_w_deg = std::max(max_w_deg, w_deg);
      max_acc_dev = std::max(max_acc_dev, a_dev);
    }
    mean_w_deg = sw / static_cast<double>(imu_recent.size());
    mean_acc_dev = sa / static_cast<double>(imu_recent.size());
  }
  const bool imu_still = (mean_w_deg < _zupt_max_omega_deg) && (mean_acc_dev < _zupt_max_acc_dev) &&
                         (max_w_deg < 1.8 * _zupt_max_omega_deg) && (max_acc_dev < 2.0 * _zupt_max_acc_dev);

  //===================================================================================
  // 【2026-10-25 / 2026-11-23 优化】轻量安全零速更新路径：
  //   1) 视觉必须连续静止；IMU 偶发轻微超限只影响窗口得票，明显运动立即清空窗口；
  //   2) 常态仅更新 [bg, v] 并先传播 Q_bias：不再用 (a_m - b_a - R*g) 在不含 ba 的状态下强扭姿态，
  //      避免残留 ba 零偏把 R 掰偏 1~2° 并在静止期通过交叉协方差拉偏位置/航向；
  //   3) 仅在原始加速度计与估计重力夹角 > 2.5°（确认发生真实倾角发散而非 ba 残留）时，才用原始
  //      a_raw 做保守重力救援；
  //   4) 速度残差与 P_vv 不一致时先对 v 注入对角过程噪声 Q_v，既把 v 拉回 0 又阻尼 K_{p,v}，
  //      彻底消除交叉协方差引起的位置瞬移，且不再硬写 set_value(0) 破坏 EKF/FEJ 一致性；
  //   5) 静止收敛后降频（6 Hz）更新并累计 dt 传播 Q_bias，防止长时间停放时协方差塌缩与缓慢漂移。
  //===================================================================================
  if (_zupt_v_only) {
    double disp_avg = 0.0, disp_var = 0.0;
    int num_features = 0;
    FeatureHelper::compute_disparity(_db, state->_timestamp, timestamp, disp_avg, disp_var, num_features);
    last_disp_avg = disp_avg;
    // 仅在桌面级极静时适度放宽像素位移上限（最大 2.0 px）以容忍 1080p MJPG 底噪
    const bool imu_ultra_still = (mean_w_deg < 0.8) && (max_w_deg < 1.5) && (mean_acc_dev < 0.08);
    const double eff_max_disp = imu_ultra_still ? std::min(2.0, 1.35 * _zupt_max_disparity) : _zupt_max_disparity;
    const bool vision_still = (disp_avg >= 0.0 && disp_avg < eff_max_disp && num_features > 20);
    const double v_now0 = state->_imu->vel().norm();

    using ImuEvidence = StationaryWindow::ImuEvidence;
    const bool imu_motion = (mean_w_deg >= 2.0 * _zupt_max_omega_deg) || (max_w_deg >= 2.7 * _zupt_max_omega_deg) ||
                            (mean_acc_dev >= 2.0 * _zupt_max_acc_dev) || (max_acc_dev >= 3.0 * _zupt_max_acc_dev);
    const auto imu_evidence = imu_still ? ImuEvidence::quiet : (imu_motion ? ImuEvidence::motion : ImuEvidence::marginal);
    if (!still_window_.observe(vision_still, imu_evidence)) {
      last_zupt_state_timestamp = 0.0;
      last_zupt_count = 0;
      _zupt_skipped_dt = 0.0;
      if (timestamp - last_pending_log_ >= 2.0) {
        last_pending_log_ = timestamp;
        const char *reason = !vision_still ? "视觉" : (imu_motion ? "IMU 运动" : "静止窗口未达标");
        PRINT_INFO("[ZUPT] 静止待确认 (%s) 窗口=%d/%d IMU 静止=%d/%d |w|均值/峰值=%.2f/%.2f deg/s "
                   "加速度偏差均值/峰值=%.3f/%.3f m/s² 位移=%.2f/%.2f px (%d 特征)\n",
                   reason, still_window_.size(), still_window_.capacity(), still_window_.quiet_count(),
                   still_window_.required_quiet(), mean_w_deg, max_w_deg, mean_acc_dev, max_acc_dev, disp_avg, eff_max_disp,
                   num_features);
      }
      return false;
    }

    double dt_summed = 0.0;
    for (size_t i = 0; i + 1 < imu_recent.size(); ++i) {
      dt_summed += imu_recent.at(i + 1).timestamp - imu_recent.at(i).timestamp;
    }
    dt_summed = std::max(dt_summed, 1e-3);

    Eigen::Vector3d a_raw_avg = Eigen::Vector3d::Zero();
    Eigen::Vector3d w_avg = Eigen::Vector3d::Zero();
    for (const auto &m : imu_recent) {
      a_raw_avg += m.am;
      w_avg += m.wm - state->_imu->bias_g();
    }
    a_raw_avg /= static_cast<double>(imu_recent.size());
    w_avg /= static_cast<double>(imu_recent.size());

    double tilt_raw_deg = 0.0;
    const Eigen::Vector3d g_est_I = state->_imu->Rot() * _gravity;
    if (a_raw_avg.norm() > 1e-3 && g_est_I.norm() > 1e-3) {
      const double c = std::max(-1.0, std::min(1.0, a_raw_avg.normalized().dot(g_est_I.normalized())));
      tilt_raw_deg = std::acos(c) * 180.0 / M_PI;
    }
    const bool need_gravity_rescue = (tilt_raw_deg > 2.5);

    if (last_zupt_count >= 2) {
      _db->cleanup_measurements_exact(last_zupt_state_timestamp);
    }

    // 静止收敛后降频到约 6 Hz 执行 EKF 更新，避免 30 Hz 反复收紧协方差造成状态僵化/慢漂
    const bool converged_still = (last_zupt_count >= 15) && (v_now0 < 0.005) && (w_avg.norm() < 0.002) && !need_gravity_rescue;
    if (converged_still && ((last_zupt_count % 5) != 0)) {
      _zupt_skipped_dt += dt_summed;
      state->_timestamp = timestamp;
      last_zupt_state_timestamp = timestamp;
      last_zupt_count++;
      return true;
    }

    const double dt_bias = std::max(dt_summed + _zupt_skipped_dt, 1e-3);
    _zupt_skipped_dt = 0.0;

    // 1) 传播陀螺与加计零偏随机游走协方差，保持静止期零偏可跟踪温度漂移
    {
      Eigen::MatrixXd Phi_bias = Eigen::MatrixXd::Identity(6, 6);
      Eigen::MatrixXd Q_bias = Eigen::MatrixXd::Zero(6, 6);
      Q_bias.block<3, 3>(0, 0) = (dt_bias * _noises.sigma_wb_2) * Eigen::Matrix3d::Identity();
      Q_bias.block<3, 3>(3, 3) = (dt_bias * _noises.sigma_ab_2) * Eigen::Matrix3d::Identity();
      std::vector<std::shared_ptr<Type>> Phi_order;
      Phi_order.push_back(state->_imu->bg());
      Phi_order.push_back(state->_imu->ba());
      StateHelper::EKFPropagation(state, Phi_order, Phi_order, Phi_bias, Q_bias);
    }

    // 2) 若当前速度估计与 P_vv 严重不一致（如异常发散后停机），先对 v 单独注入对角过程噪声 Q_v：
    //    K_v ≈ I（顺利归零），同时 K_{p,v} = P_{p,v}(P_{vv}+Q_v+R_v)^{-1} 被大幅阻尼（避免瞬移位置），
    //    且协方差严格保持半正定。
    const double sigma_v = (_zupt_sigma_v > 1e-4) ? _zupt_sigma_v : 0.05;
    {
      std::vector<std::shared_ptr<Type>> v_order{state->_imu->v()};
      const Eigen::Matrix3d P_vv = StateHelper::get_marginal_covariance(state, v_order);
      const Eigen::Matrix3d S_v = P_vv + (sigma_v * sigma_v) * Eigen::Matrix3d::Identity();
      const Eigen::Vector3d v_vec = state->_imu->vel();
      const double chi2_v = v_vec.dot(S_v.llt().solve(v_vec));
      if (chi2_v > 9.0 && v_now0 > 0.02) {
        const double q_v = std::pow(0.5 * v_now0, 2);
        Eigen::MatrixXd Phi_v = Eigen::MatrixXd::Identity(3, 3);
        Eigen::MatrixXd Q_v = q_v * Eigen::Matrix3d::Identity();
        StateHelper::EKFPropagation(state, v_order, v_order, Phi_v, Q_v);
      }
    }

    // 3) 构造观测方程：常态仅 [bg, v]；仅当倾角发散 > 2.5° 时追加原始重力方向救援 [q]
    const double mult = (_zupt_noise_multiplier > 1e-6) ? _zupt_noise_multiplier : 1.0;
    const double var_w = std::max(mult * _noises.sigma_w_2 / dt_summed, 0.002 * 0.002);

    std::vector<std::shared_ptr<Type>> Hx_order;
    const int dim = need_gravity_rescue ? 9 : 6;
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(dim, dim);
    Eigen::VectorXd res = Eigen::VectorXd::Zero(dim);
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(dim, dim);

    int col_bg = 0;
    int col_v = 3;
    if (need_gravity_rescue) {
      Hx_order.push_back(state->_imu->q());
      col_bg = 3;
      col_v = 6;
    }
    Hx_order.push_back(state->_imu->bg());
    Hx_order.push_back(state->_imu->v());

    // (a) v = 0
    res.block<3, 1>(0, 0) = -state->_imu->vel();
    H.block<3, 3>(0, col_v) = Eigen::Matrix3d::Identity();
    R.block<3, 3>(0, 0) = (sigma_v * sigma_v) * Eigen::Matrix3d::Identity();

    // (b) w_m - b_g = 0
    res.block<3, 1>(3, 0) = -w_avg;
    H.block<3, 3>(3, col_bg) = -Eigen::Matrix3d::Identity();
    R.block<3, 3>(3, 3) = var_w * Eigen::Matrix3d::Identity();

    // (c) 严重倾角发散救援（原始 a_raw 对齐重力，不引入估计 ba 偏差）
    if (need_gravity_rescue) {
      const Eigen::Matrix3d R_GtoI_j = (state->_options.do_fej) ? state->_imu->Rot_fej() : state->_imu->Rot();
      res.block<3, 1>(6, 0) = -(a_raw_avg - g_est_I);
      H.block<3, 3>(6, 0) = -skew_x(R_GtoI_j * _gravity);
      R.block<3, 3>(6, 6) = (0.35 * 0.35) * Eigen::Matrix3d::Identity();
    }

    StateHelper::EKFUpdate(state, Hx_order, H, res, R);
    state->_timestamp = timestamp;
    last_zupt_state_timestamp = timestamp;
    last_zupt_count++;

    if (timestamp - last_update_log_ >= 2.0) {
      last_update_log_ = timestamp;
      PRINT_INFO(CYAN "[ZUPT] t=%.2f 判静止→零速更新 |v|=%.3f→%.3f |w|=%.2f°/s ‖a‖偏差=%.3f 位移=%.2fpx(%d特征) tilt=%.2f°\n" RESET,
                 timestamp, v_now0, state->_imu->vel().norm(), mean_w_deg, mean_acc_dev, disp_avg, num_features, tilt_raw_deg);
    }
    return true;
  }

  // If we should integrate the acceleration and say the velocity should be zero
  // Also if we should still inflate the bias based on their random walk noises
  bool integrated_accel_constraint = false; // untested
  bool model_time_varying_bias = true;
  bool override_with_disparity_check = true;
  bool explicitly_enforce_zero_motion = false;

  // 【2026-10-25】显式零速残差：把速度状态也拉进更新，观测 "v = 0"。
  const bool add_zero_velocity = (_zupt_sigma_v > 0.0) && !integrated_accel_constraint;

  // Order of our Jacobian
  std::vector<std::shared_ptr<Type>> Hx_order;
  Hx_order.push_back(state->_imu->q());
  Hx_order.push_back(state->_imu->bg());
  Hx_order.push_back(state->_imu->ba());
  if (integrated_accel_constraint || add_zero_velocity) {
    Hx_order.push_back(state->_imu->v());
  }
  const int col_v = 9; // v 在 H 列里的起点（q/bg/ba 之后）

  // Large final matrices used for update (we will compress these)
  // 【2026-10-25】add_zero_velocity 时多 3 行 "v = 0"
  int h_size = (integrated_accel_constraint) ? 12 : (add_zero_velocity ? 12 : 9);
  int m_size = 6 * ((int)imu_recent.size() - 1) + (add_zero_velocity ? 3 : 0);
  Eigen::MatrixXd H = Eigen::MatrixXd::Zero(m_size, h_size);
  Eigen::VectorXd res = Eigen::VectorXd::Zero(m_size);

  // IMU intrinsic calibration estimates (static)
  Eigen::Matrix3d Dw = State::Dm(state->_options.imu_model, state->_calib_imu_dw->value());
  Eigen::Matrix3d Da = State::Dm(state->_options.imu_model, state->_calib_imu_da->value());
  Eigen::Matrix3d Tg = State::Tg(state->_calib_imu_tg->value());

  // Loop through all our IMU and construct the residual and Jacobian
  // TODO: should add jacobians here in respect to IMU intrinsics!!
  // State order is: [q_GtoI, bg, ba, v_IinG]
  // Measurement order is: [w_true = 0, a_true = 0 or v_k+1 = 0]
  // w_true = w_m - bw - nw
  // a_true = a_m - ba - R*g - na
  // v_true = v_k - g*dt + R^T*(a_m - ba - na)*dt
  double dt_summed = 0;
  for (size_t i = 0; i < imu_recent.size() - 1; i++) {

    // Precomputed values
    double dt = imu_recent.at(i + 1).timestamp - imu_recent.at(i).timestamp;
    Eigen::Vector3d a_hat = state->_calib_imu_ACCtoIMU->Rot() * Da * (imu_recent.at(i).am - state->_imu->bias_a());
    Eigen::Vector3d w_hat = state->_calib_imu_GYROtoIMU->Rot() * Dw * (imu_recent.at(i).wm - state->_imu->bias_g() - Tg * a_hat);

    // Measurement noise (convert from continuous to discrete)
    // NOTE: The dt time might be different if we have "cut" any imu measurements
    // NOTE: We are performing "whittening" thus, we will decompose R_meas^-1 = L*L^t
    // NOTE: This is then multiplied to the residual and Jacobian (equivalent to just updating with R_meas)
    // NOTE: See Maybeck Stochastic Models, Estimation, and Control Vol. 1 Equations (7-21a)-(7-21c)
    double w_omega = std::sqrt(dt) / _noises.sigma_w;
    double w_accel = std::sqrt(dt) / _noises.sigma_a;
    double w_accel_v = 1.0 / (std::sqrt(dt) * _noises.sigma_a);

    // Measurement residual (true value is zero)
    res.block(6 * i + 0, 0, 3, 1) = -w_omega * w_hat;
    if (!integrated_accel_constraint) {
      res.block(6 * i + 3, 0, 3, 1) = -w_accel * (a_hat - state->_imu->Rot() * _gravity);
    } else {
      res.block(6 * i + 3, 0, 3, 1) = -w_accel_v * (state->_imu->vel() - _gravity * dt + state->_imu->Rot().transpose() * a_hat * dt);
    }

    // Measurement Jacobian
    Eigen::Matrix3d R_GtoI_jacob = (state->_options.do_fej) ? state->_imu->Rot_fej() : state->_imu->Rot();
    H.block(6 * i + 0, 3, 3, 3) = -w_omega * Eigen::Matrix3d::Identity();
    if (!integrated_accel_constraint) {
      H.block(6 * i + 3, 0, 3, 3) = -w_accel * skew_x(R_GtoI_jacob * _gravity);
      H.block(6 * i + 3, 6, 3, 3) = -w_accel * Eigen::Matrix3d::Identity();
    } else {
      H.block(6 * i + 3, 0, 3, 3) = -w_accel_v * R_GtoI_jacob.transpose() * skew_x(a_hat) * dt;
      H.block(6 * i + 3, 6, 3, 3) = -w_accel_v * R_GtoI_jacob.transpose() * dt;
      H.block(6 * i + 3, 9, 3, 3) = w_accel_v * Eigen::Matrix3d::Identity();
    }
    dt_summed += dt;
  }

  //===================================================================================
  // 【2026-10-25】追加 "v = 0" 三行（白化后）
  // 细节：更新末尾会把 R 乘上 _zupt_noise_multiplier，所以这里先按 sqrt(mult) 反向
  // 缩放，保证最终等效噪声就是 _zupt_sigma_v，不受 multiplier 影响。
  //===================================================================================
  if (add_zero_velocity) {
    const double mult = (_zupt_noise_multiplier > 1e-6) ? _zupt_noise_multiplier : 1.0;
    const double w_v = std::sqrt(mult) / _zupt_sigma_v;
    res.block(m_size - 3, 0, 3, 1) = -w_v * state->_imu->vel();
    H.block(m_size - 3, col_v, 3, 3) = w_v * Eigen::Matrix3d::Identity();
  }

  // Compress the system (we should be over determined)
  UpdaterHelper::measurement_compress_inplace(H, res);
  if (H.rows() < 1) {
    return false;
  }

  // Multiply our noise matrix by a fixed amount
  // We typically need to treat the IMU as being "worst" to detect / not become overconfident
  Eigen::MatrixXd R = _zupt_noise_multiplier * Eigen::MatrixXd::Identity(res.rows(), res.rows());

  // Next propagate the biases forward in time
  // NOTE: G*Qd*G^t = dt*Qd*dt = dt*(1/dt*Qc)*dt = dt*Qc
  Eigen::MatrixXd Q_bias = Eigen::MatrixXd::Identity(6, 6);
  Q_bias.block(0, 0, 3, 3) *= dt_summed * _noises.sigma_wb_2;
  Q_bias.block(3, 3, 3, 3) *= dt_summed * _noises.sigma_ab_2;

  // Chi2 distance check
  // NOTE: we also append the propagation we "would do before the update" if this was to be accepted (just the bias evolution)
  // NOTE: we don't propagate first since if we fail the chi2 then we just want to return and do normal logic
  Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hx_order);
  if (model_time_varying_bias) {
    P_marg.block(3, 3, 6, 6) += Q_bias;
  }
  Eigen::MatrixXd S = H * P_marg * H.transpose() + R;
  double chi2 = res.dot(S.llt().solve(res));

  // Get our threshold (we precompute up to 1000 but handle the case that it is more)
  double chi2_check;
  if (res.rows() < 1000) {
    chi2_check = chi_squared_table[res.rows()];
  } else {
    boost::math::chi_squared chi_squared_dist(res.rows());
    chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
    PRINT_WARNING(YELLOW "[ZUPT]: chi2_check over the residual limit - %d\n" RESET, (int)res.rows());
  }

  // Check if the image disparity
  bool disparity_passed = false;
  if (override_with_disparity_check) {

    // Get the disparity statistics from this image to the previous
    double time0_cam = state->_timestamp;
    double time1_cam = timestamp;
    int num_features = 0;
    double disp_avg = 0.0;
    double disp_var = 0.0;
    FeatureHelper::compute_disparity(_db, time0_cam, time1_cam, disp_avg, disp_var, num_features);
    last_disp_avg = disp_avg;   // 仅用于日志

    // Check if this disparity is enough to be classified as moving
    disparity_passed = (disp_avg < _zupt_max_disparity && num_features > 20);
    if (disparity_passed) {
      PRINT_INFO(CYAN "[ZUPT]: passed disparity (%.3f < %.3f, %d features)\n" RESET, disp_avg, _zupt_max_disparity, (int)num_features);
    } else {
      PRINT_DEBUG(YELLOW "[ZUPT]: failed disparity (%.3f > %.3f, %d features)\n" RESET, disp_avg, _zupt_max_disparity, (int)num_features);
    }
  }

  //===================================================================================
  // 【2026-10-25】判据重写
  //
  // 上游逻辑：`if (!disparity_passed && (chi2 爆 || |v| 大)) reject`
  //   → 只要图像位移小就**无条件接受**，卡方和速度都不看。实测 turn4 每帧都进来，
  //     连 |v|=0.255 > 0.20 也照收。反过来，卡方在姿态错了 11° 时必然 2989 >> 84，
  //     如果图像位移刚好超一点，就变成"最该救场的时候偏偏拒绝"。
  //
  // 新逻辑：三条独立证据全部通过才接受
  //   ① 视觉：相邻帧平均位移 < zupt_max_disparity 且存活特征 > 20   （证明图像没动）
  //   ② IMU ：窗口内 |ω| 与 |‖a‖−g| 都有界                        （证明机体没动）
  //   ③ 状态：|v| < zupt_max_velocity                              （连状态自己都不认为在动）
  // 卡方只记录、不作为门限：姿态/零偏本身错了的时候卡方一定爆，而零速更新正是把
  // 它拉回来的唯一手段，用它当门限等于"错得越狠越不给改"。
  //===================================================================================
  const double v_now = state->_imu->vel().norm();
  const bool vel_ok = (v_now < _zupt_max_velocity);
  const bool still_now = disparity_passed && imu_still && vel_ok;

  // 【2026-10-25】去抖：静止条件必须连续满足 _zupt_min_still_frames 帧才动手。
  // 单帧判据在暗光下会闪断，而一次误触发（真在动却判静止）会被 v=0 残差硬拽，
  // 实测 turn5 只错 7 次就把 10.4 m 的轨迹搞成 44.6 m —— 宁可晚 0.2 s 也不能误触发。
  _still_streak = still_now ? (_still_streak + 1) : 0;
  if (!still_now || _still_streak < _zupt_min_still_frames) {
    last_zupt_state_timestamp = 0.0;
    last_zupt_count = 0;
    return false;
  }
  // 打印限流：每 2 s 最多一条，避免刷屏
  static double last_print_t = -1e9;
  const bool do_print = (timestamp - last_print_t) > 2.0;
  if (do_print) {
    last_print_t = timestamp;
    PRINT_INFO(CYAN "[ZUPT]: t=%.2f 判静止→零速更新 |v|=%.3f→0 |w|=%.2f°/s ‖a‖偏差=%.3f 位移=%.2fpx chi2=%.0f(门限%.0f，仅记录)\n" RESET,
               timestamp, v_now, mean_w_deg, mean_acc_dev, last_disp_avg, chi2, _options.chi2_multipler * chi2_check);
  }

  // Do our update, only do this update if we have previously detected
  // If we have succeeded, then we should remove the current timestamp feature tracks
  // This is because we will not clone at this timestep and instead do our zero velocity update
  // NOTE: We want to keep the tracks from the second time we have called the zv-upt since this won't have a clone
  // NOTE: All future times after the second call to this function will also *not* have a clone, so we can remove those
  if (last_zupt_count >= 2) {
    _db->cleanup_measurements_exact(last_zupt_state_timestamp);
  }

  // Else we are good, update the system
  // 1) update with our IMU measurements directly
  // 2) propagate and then explicitly say that our ori, pos, and vel should be zero
  if (!explicitly_enforce_zero_motion) {

    // Next propagate the biases forward in time
    // NOTE: G*Qd*G^t = dt*Qd*dt = dt*Qc
    if (model_time_varying_bias) {
      Eigen::MatrixXd Phi_bias = Eigen::MatrixXd::Identity(6, 6);
      std::vector<std::shared_ptr<Type>> Phi_order;
      Phi_order.push_back(state->_imu->bg());
      Phi_order.push_back(state->_imu->ba());
      StateHelper::EKFPropagation(state, Phi_order, Phi_order, Phi_bias, Q_bias);
    }

    // Finally move the state time forward
    StateHelper::EKFUpdate(state, Hx_order, H, res, R);
    state->_timestamp = timestamp;

  } else {

    // Propagate the state forward in time
    double time0_cam = last_zupt_state_timestamp;
    double time1_cam = timestamp;
    _prop->propagate_and_clone(state, time1_cam);

    // Create the update system!
    H = Eigen::MatrixXd::Zero(9, 15);
    res = Eigen::VectorXd::Zero(9);
    R = Eigen::MatrixXd::Identity(9, 9);

    // residual (order is ori, pos, vel)
    Eigen::Matrix3d R_GtoI0 = state->_clones_IMU.at(time0_cam)->Rot();
    Eigen::Vector3d p_I0inG = state->_clones_IMU.at(time0_cam)->pos();
    Eigen::Matrix3d R_GtoI1 = state->_clones_IMU.at(time1_cam)->Rot();
    Eigen::Vector3d p_I1inG = state->_clones_IMU.at(time1_cam)->pos();
    res.block(0, 0, 3, 1) = -log_so3(R_GtoI0 * R_GtoI1.transpose());
    res.block(3, 0, 3, 1) = p_I1inG - p_I0inG;
    res.block(6, 0, 3, 1) = state->_imu->vel();
    res *= -1;

    // jacobian (order is q0, p0, q1, p1, v0)
    Hx_order.clear();
    Hx_order.push_back(state->_clones_IMU.at(time0_cam));
    Hx_order.push_back(state->_clones_IMU.at(time1_cam));
    Hx_order.push_back(state->_imu->v());
    if (state->_options.do_fej) {
      R_GtoI0 = state->_clones_IMU.at(time0_cam)->Rot_fej();
    }
    H.block(0, 0, 3, 3) = Eigen::Matrix3d::Identity();
    H.block(0, 6, 3, 3) = -R_GtoI0;
    H.block(3, 3, 3, 3) = -Eigen::Matrix3d::Identity();
    H.block(3, 9, 3, 3) = Eigen::Matrix3d::Identity();
    H.block(6, 12, 3, 3) = Eigen::Matrix3d::Identity();

    // noise (order is ori, pos, vel)
    R.block(0, 0, 3, 3) *= std::pow(1e-2, 2);
    R.block(3, 3, 3, 3) *= std::pow(1e-1, 2);
    R.block(6, 6, 3, 3) *= std::pow(1e-1, 2);

    // finally update and remove the old clone
    StateHelper::EKFUpdate(state, Hx_order, H, res, R);
    StateHelper::marginalize(state, state->_clones_IMU.at(time1_cam));
    state->_clones_IMU.erase(time1_cam);
  }

  // Finally return
  last_zupt_state_timestamp = timestamp;
  last_zupt_count++;
  return true;
}
