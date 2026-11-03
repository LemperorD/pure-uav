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

#ifndef OV_MSCKF_UPDATER_MSCKF_H
#define OV_MSCKF_UPDATER_MSCKF_H

#include <Eigen/Eigen>
#include <functional>
#include <algorithm>
#include <cmath>
#include <memory>

#include "feat/FeatureInitializerOptions.h"

#include "UpdaterOptions.h"

namespace ov_core {
class Feature;
class FeatureInitializer;
} // namespace ov_core

namespace ov_msckf {

class State;

/**
 * @brief Will compute the system for our sparse features and update the filter.
 *
 * This class is responsible for computing the entire linear system for all features that are going to be used in an update.
 * This follows the original MSCKF, where we first triangulate features, we then nullspace project the feature Jacobian.
 * After this we compress all the measurements to have an efficient update and update the state.
 */
class UpdaterMSCKF {

public:
  /**
   * @brief 【2026-10-21】深度查询回调（由 VioManager 每帧安装；为空表示没有深度）。
   *
   * 给定特征**锚点观测**的时间 t 和彩色像素 (u,v)，返回米制深度（米）；无效返回 <0。
   *
   * 为什么要这个：单目 VIO 的致命处是"没有平移就没有视差 → 三角化失败 → 没有测量"。
   * 实测原地转时三角化率只有 0.04，滤波器退化成纯惯导。有了深度就能直接由
   * (锚点位姿, 视线, Z) 构造特征的三维位置，**纯旋转也能出测量**，
   * 而且尺度被深度钉死（不再靠 IMU 推，零偏不再乱漂）。
   */
  struct DepthLookup {
    static std::function<float(double t, float u, float v)> fn;
  };

  /**
   * @brief Default constructor for our MSCKF updater
   *
   * Our updater has a feature initializer which we use to initialize features as needed.
   * Also the options allow for one to tune the different parameters for update.
   *
   * @param options Updater options (include measurement noise value)
   * @param feat_init_options Feature initializer options
   */
  UpdaterMSCKF(UpdaterOptions &options, ov_core::FeatureInitializerOptions &feat_init_options);

  /**
   * @brief Given tracked features, this will try to use them to update the state.
   *
   * @param state State of the filter
   * @param feature_vec Features that can be used for update
   */
  void update(std::shared_ptr<State> state, std::vector<std::shared_ptr<ov_core::Feature>> &feature_vec);

  // 【2026-10-20 加】更新诊断。用来回答一个具体问题：
  //   视觉更新到底是"没特征可用"，还是"有特征但被卡方门限全拒了"？
  // IMU 噪声越小，预测协方差越小，残差越容易被判成外点 —— 如果拒得太狠，
  // 就等于退化成纯 IMU 积分，而日志上看不出来。
  // 【2026-10-21】自适应卡方门限（保证"持续输出"用）：
  // 卡方一旦把某帧的特征全拒掉，更新率归零 → 状态开环发散 → 更全拒，形成棘轮。
  // 这里保证每帧至少 min_accepted_per_frame 个特征能进更新：通过数不够就按倍数放宽门限，
  // 够用就把门限逐步收回标称值。min_accepted_per_frame = 0 即关闭（默认关闭，用 OV_MIN_ACCEPT 打开）。
  double eff_chi2_mult = -1.0;   // 当前生效的门限倍数（-1 = 还没初始化）
  int min_accepted_per_frame = 0;
  double max_chi2_mult = 4096.0; // 放宽上限，防止无限放大
  long long diag_eff_mult_max = 0; // 诊断：本次运行里用到过的最大倍数

  long long diag_calls = 0;           // update() 被调用次数
  long long diag_feat_in = 0;         // 进入更新的特征数
  long long diag_feat_chi2_rej = 0;   // 被卡方拒掉的特征数
  long long diag_updates = 0;         // 真正执行了 EKFUpdate 的次数
  long long diag_rows = 0;            // 实际喂给更新的压缩后残差行数
  long long diag_clean_drop = 0;      // 观测数<2 在 clean 阶段被丢的特征
  long long diag_tri_drop = 0;        // 三角化/高斯牛顿精化失败被丢的特征
  // 【2026-10-21 加】把"特征是怎么消失的"拆到每一步，定位 0 更新的具体出口
  long long diag_tri_ok = 0;
  long long diag_depth_used = 0;
  // 深度一致性诊断：由深度构造的点重投影回窗口内所有观测的误差分布
  long long diag_deperr_n = 0;
  double diag_deperr_sum = 0.0;
  long long diag_deperr_hist[6] = {0, 0, 0, 0, 0, 0}; // <1 <2 <5 <10 <25 >=25 px  // 由深度直接构造位置、跳过三角化的特征数          // 三角化成功的特征数（进入雅可比构建）
  long long diag_chi2_ok = 0;         // 通过卡方的特征数
  long long diag_ctmeas = 0;          // 本次拼接出的原始残差行数 ct_meas
  long long diag_comp_zero = 0;       // ct_meas>0 但压缩后 0 行的次数
  long long diag_tri_fail_tri = 0;    // 其中：三角化本身失败
  long long diag_tri_fail_refine = 0; // 其中：三角化成功但高斯牛顿精化失败
  // 卡方比值 = chi2 / (multiplier*chi2_check)，1.0 是门限。用来看"被拒的特征到底偏离多少倍"
  double diag_chi2_rej_ratio_sum = 0.0;
  double diag_chi2_ok_ratio_sum = 0.0;
  double diag_chi2_rej_ratio_max = 0.0;
  long long diag_chi2_rej_hist[5] = {0, 0, 0, 0, 0}; // <2x, <5x, <20x, <200x, >=200x

protected:
  /// Options used during update
  UpdaterOptions _options;

  /// Feature initializer class object
  std::shared_ptr<ov_core::FeatureInitializer> initializer_feat;

  /// Chi squared 95th percentile table (lookup would be size of residual)
  std::map<int, double> chi_squared_table;
};

} // namespace ov_msckf

#endif // OV_MSCKF_UPDATER_MSCKF_H
