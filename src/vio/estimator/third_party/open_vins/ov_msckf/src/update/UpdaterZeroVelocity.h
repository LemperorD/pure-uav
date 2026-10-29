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

#ifndef OV_MSCKF_UPDATER_ZEROVELOCITY_H
#define OV_MSCKF_UPDATER_ZEROVELOCITY_H

#include <memory>

#include "utils/sensor_data.h"

#include "UpdaterOptions.h"
#include "utils/NoiseManager.h"

namespace ov_core {
class Feature;
class FeatureDatabase;
} // namespace ov_core
namespace ov_type {
class Landmark;
} // namespace ov_type

namespace ov_msckf {

class State;
class Propagator;

/**
 * @brief Will try to *detect* and then update using zero velocity assumption.
 *
 * Consider the case that a VIO unit remains stationary for a period time.
 * Typically this can cause issues in a monocular system without SLAM features since no features can be triangulated.
 * Additional, if features could be triangulated (e.g. stereo) the quality can be poor and hurt performance.
 * If we can detect the cases where we are stationary then we can leverage this to prevent the need to do feature update during this period.
 * The main application would be using this on a **wheeled vehicle** which needs to stop (e.g. stop lights or parking).
 */
class UpdaterZeroVelocity {

public:
  /**
   * @brief Default constructor for our zero velocity detector and updater.
   * @param options Updater options (chi2 multiplier)
   * @param noises imu noise characteristics (continuous time)
   * @param db Feature tracker database with all features in it
   * @param prop Propagator class object which can predict the state forward in time
   * @param gravity_mag Global gravity magnitude of the system (normally 9.81)
   * @param zupt_max_velocity Max velocity we should consider to do a update with
   * @param zupt_noise_multiplier Multiplier of our IMU noise matrix (default should be 1.0)
   * @param zupt_max_disparity Max disparity we should consider to do a update with
   */
  /// 构造函数（2026-10-25 增加后 3 个参数：IMU 静止判据 + 显式零速残差）
  /// @param zupt_max_omega_deg 窗口内平均 |ω| 上限（deg/s），超了就不认为静止；<=0 或 999 表示不用这条
  /// @param zupt_max_acc_dev   窗口内平均 | ‖a‖ − g | 上限（m/s²）；同上
  /// @param zupt_sigma_v       零速残差 σ（m/s）；<=0 表示不加显式 v=0 约束（等价旧行为）
  UpdaterZeroVelocity(UpdaterOptions &options, NoiseManager &noises, std::shared_ptr<ov_core::FeatureDatabase> db,
                      std::shared_ptr<Propagator> prop, double gravity_mag, double zupt_max_velocity, double zupt_noise_multiplier,
                      double zupt_max_disparity, double zupt_max_omega_deg = 999.0, double zupt_max_acc_dev = 999.0,
                      double zupt_sigma_v = 0.0, int zupt_min_still_frames = 0, bool zupt_v_only = false);

  /**
   * @brief Feed function for inertial data
   * @param message Contains our timestamp and inertial information
   * @param oldest_time Time that we can discard measurements before
   */
  void feed_imu(const ov_core::ImuData &message, double oldest_time = -1) {

    // Append it to our vector
    imu_data.emplace_back(message);

    // Sort our imu data (handles any out of order measurements)
    // std::sort(imu_data.begin(), imu_data.end(), [](const IMUDATA i, const IMUDATA j) {
    //    return i.timestamp < j.timestamp;
    //});

    // Clean old measurements
    // std::cout << "ZVUPT: imu_data.size() " << imu_data.size() << std::endl;
    clean_old_imu_measurements(oldest_time - 0.10);
  }

  /**
   * @brief This will remove any IMU measurements that are older then the given measurement time
   * @param oldest_time Time that we can discard measurements before (in IMU clock)
   */
  void clean_old_imu_measurements(double oldest_time) {
    if (oldest_time < 0)
      return;
    auto it0 = imu_data.begin();
    while (it0 != imu_data.end()) {
      if (it0->timestamp < oldest_time) {
        it0 = imu_data.erase(it0);
      } else {
        it0++;
      }
    }
  }

  /**
   * @brief Will first detect if the system is zero velocity, then will update.
   * @param state State of the filter
   * @param timestamp Next camera timestamp we want to see if we should propagate to.
   * @return True if the system is currently at zero velocity
   */
  bool try_update(std::shared_ptr<State> state, double timestamp);

protected:
  /// Options used during update (chi2 multiplier)
  UpdaterOptions _options;

  /// Container for the imu noise values
  NoiseManager _noises;

  /// Feature tracker database with all features in it
  std::shared_ptr<ov_core::FeatureDatabase> _db;

  /// Our propagator!
  std::shared_ptr<Propagator> _prop;

  /// Gravity vector
  Eigen::Vector3d _gravity;

  /// Max velocity (m/s) that we should consider a zupt with
  double _zupt_max_velocity = 1.0;

  /// Multiplier of our IMU noise matrix (default should be 1.0)
  double _zupt_noise_multiplier = 1.0;

  /// Max disparity (pixels) that we should consider a zupt with
  double _zupt_max_disparity = 1.0;

  /// 【2026-10-25】IMU 静止判据：窗口内平均角速度上限（deg/s）
  /// 为什么必须有：只看图像位移会被"重复纹理 + 暗光"骗 —— 天花板条纹上 KLT 可以把
  /// 错误匹配一路跟下去，图像位移 0.3 px 而机器其实在动。IMU 是独立证据源。
  double _zupt_max_omega_deg = 999.0;

  /// 【2026-10-25】IMU 静止判据：平均 |‖a‖ − g| 上限（m/s²）
  double _zupt_max_acc_dev = 999.0;

  /// 【2026-10-25】连续满足静止条件的帧数门限（去抖）。0 = 不要求。
  /// 为什么需要：暗光下相邻帧像素位移有 0.4~0.5 px 的噪声底，单帧判据会闪断，而一次
  /// 误触发（真在动却判静止）用 σ=0.1 m/s 的 v=0 残差去拽，代价是整条轨迹崩。
  int _zupt_min_still_frames = 0;
  int _still_streak = 0;
  double last_disp_avg = 0.0;   ///< 仅日志：上一帧的平均像素位移

  /// 【2026-10-25】只做 "v = 0" 的轻量零速更新：不动 q / bg / ba。
  /// 为什么：上游那套 IMU 残差（ω=0、比力=重力）在状态已经错了的时候会把 ba 拽飞
  /// （实测 turn4：只触发 1 次，|ba| 从 1.84 → 5.07 m/s²，轨迹 32.7 → 70.6 m）。
  /// 而"停下来别再加速"这个需求只需要一条约束：速度为零。姿态错了不会让位置跑。
  bool _zupt_v_only = false;

  /// 【2026-10-25】显式零速残差 σ（m/s）。>0 时在更新里加 3 行 "v = 0"。
  /// 为什么必须加：上游 ZUPT 的 Hx_order 只有 [q, bg, ba]，它把"角速度=0、比力=重力"
  /// 当观测，却**不约束速度**。实测 turn4：静止 13 s，|v| 从 0.13 长到 5.92 m/s，
  /// 位置飘 32.7 m —— 因为状态里的 v 从头到尾没人管。
  double _zupt_sigma_v = 0.0;

  /// Chi squared 95th percentile table (lookup would be size of residual)
  std::map<int, double> chi_squared_table;

  /// Our history of IMU messages (time, angular, linear)
  std::vector<ov_core::ImuData> imu_data;

  /// Estimate for time offset at last propagation time
  double last_prop_time_offset = 0.0;
  bool have_last_prop_time_offset = false;

  /// Last timestamp we did zero velocity update with
  double last_zupt_state_timestamp = 0.0;

  /// Number of times we have called update
  int last_zupt_count = 0;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_ZEROVELOCITY_H
