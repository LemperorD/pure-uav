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

#include "UpdaterMSCKF.h"

#include <functional>
#include <cstdlib>
#include <fstream>
#include <iomanip>

#include <cstdlib>
#include <fstream>
#include <iomanip>

#include "UpdaterHelper.h"

#include "feat/Feature.h"
#include "feat/FeatureInitializer.h"
#include "state/State.h"
#include "state/StateHelper.h"
#include "types/LandmarkRepresentation.h"
#include "utils/colors.h"
#include "utils/print.h"
#include "utils/quat_ops.h"

#include <boost/date_time/posix_time/posix_time.hpp>
#include <boost/math/distributions/chi_squared.hpp>

using namespace ov_core;
using namespace ov_type;
using namespace ov_msckf;

std::function<float(double, float, float)> UpdaterMSCKF::DepthLookup::fn;

UpdaterMSCKF::UpdaterMSCKF(UpdaterOptions &options, ov_core::FeatureInitializerOptions &feat_init_options) : _options(options) {

  // Save our raw pixel noise squared
  _options.sigma_pix_sq = std::pow(_options.sigma_pix, 2);

  // Save our feature initializer
  initializer_feat = std::shared_ptr<ov_core::FeatureInitializer>(new ov_core::FeatureInitializer(feat_init_options));

  // Initialize the chi squared test table with confidence level 0.95
  // https://github.com/KumarRobotics/msckf_vio/blob/050c50defa5a7fd9a04c1eed5687b405f02919b5/src/msckf_vio.cpp#L215-L221
  for (int i = 1; i < 500; i++) {
    boost::math::chi_squared chi_squared_dist(i);
    chi_squared_table[i] = boost::math::quantile(chi_squared_dist, 0.95);
  }
}

void UpdaterMSCKF::update(std::shared_ptr<State> state, std::vector<std::shared_ptr<Feature>> &feature_vec) {

  // Return if no features
  if (feature_vec.empty())
    return;
  ++diag_calls;
  diag_feat_in += static_cast<long long>(feature_vec.size());

  // Start timing
  boost::posix_time::ptime rT0, rT1, rT2, rT3, rT4, rT5;
  rT0 = boost::posix_time::microsec_clock::local_time();

  // 0. Get all timestamps our clones are at (and thus valid measurement times)
  std::vector<double> clonetimes;
  for (const auto &clone_imu : state->_clones_IMU) {
    clonetimes.emplace_back(clone_imu.first);
  }

  // 1. Clean all feature measurements and make sure they all have valid clone times
  auto it0 = feature_vec.begin();
  while (it0 != feature_vec.end()) {

    // Clean the feature
    (*it0)->clean_old_measurements(clonetimes);

    // Count how many measurements
    int ct_meas = 0;
    for (const auto &pair : (*it0)->timestamps) {
      ct_meas += (*it0)->timestamps[pair.first].size();
    }

    // Remove if we don't have enough
    if (ct_meas < 2) {
      ++diag_clean_drop;
      (*it0)->to_delete = true;
      it0 = feature_vec.erase(it0);
    } else {
      it0++;
    }
  }
  rT1 = boost::posix_time::microsec_clock::local_time();

  // 2. Create vector of cloned *CAMERA* poses at each of our clone timesteps
  std::unordered_map<size_t, std::unordered_map<double, FeatureInitializer::ClonePose>> clones_cam;
  for (const auto &clone_calib : state->_calib_IMUtoCAM) {

    // For this camera, create the vector of camera poses
    std::unordered_map<double, FeatureInitializer::ClonePose> clones_cami;
    for (const auto &clone_imu : state->_clones_IMU) {

      // Get current camera pose
      Eigen::Matrix<double, 3, 3> R_GtoCi = clone_calib.second->Rot() * clone_imu.second->Rot();
      Eigen::Matrix<double, 3, 1> p_CioinG = clone_imu.second->pos() - R_GtoCi.transpose() * clone_calib.second->pos();

      // Append to our map
      clones_cami.insert({clone_imu.first, FeatureInitializer::ClonePose(R_GtoCi, p_CioinG)});
    }

    // Append to our map
    clones_cam.insert({clone_calib.first, clones_cami});
  }

  // 3. Try to triangulate all MSCKF or new SLAM features that have measurements
  auto it1 = feature_vec.begin();
  while (it1 != feature_vec.end()) {

    //=================================================================================
    // 【2026-10-21】有深度就直接构造三维位置，跳过三角化。
    // 修的是单目的致命处："没有平移就没有视差 → 三角化失败 → 没有测量"。
    // 实测原地转时三角化率只有 0.04；有深度后纯旋转也能出测量。
    //=================================================================================
    if (UpdaterMSCKF::DepthLookup::fn && !(*it1)->timestamps.empty()) {
      const size_t cam_id0 = (*it1)->timestamps.begin()->first;
      const auto &ts0 = (*it1)->timestamps.begin()->second;
      const auto &uv_list = (*it1)->uvs.at(cam_id0);
      if (!ts0.empty() && !uv_list.empty()) {
        const double t_anchor = ts0.front();
        const float u0 = uv_list.front()(0);
        const float v0 = uv_list.front()(1);
        const float z_meas = UpdaterMSCKF::DepthLookup::fn(t_anchor, u0, v0);
        if (z_meas > 0.05f && z_meas < 20.0f) {
          auto it_cam = clones_cam.find(cam_id0);
          if (it_cam != clones_cam.end()) {
            auto it_clone = it_cam->second.find(t_anchor);
            if (it_clone != it_cam->second.end()) {
              Eigen::Vector2d uv_d(u0, v0);
              Eigen::Vector2d uv_n = state->_cam_intrinsics_cameras.at(cam_id0)->undistort_d(uv_d);
              Eigen::Vector3d b(uv_n(0), uv_n(1), 1.0);
              const Eigen::Matrix3d &R_GtoA = it_clone->second.Rot();
              const Eigen::Vector3d &p_AinG = it_clone->second.pos();
              // p_FinG = p_AinG + R_GtoA^T * (b * Z)
              (*it1)->p_FinG = p_AinG + R_GtoA.transpose() * (b * static_cast<double>(z_meas));
              ++diag_depth_used;

              //=================================================================
              // 【2026-10-21 诊断】深度一致性：把"由深度构造的三维点"重新投影回
              // 它在窗口内的**所有**观测，看重投影误差。
              //   误差 ~1-2 px  → 深度关联是对的
              //   误差几十 px   → 深度采到了别的表面（深度不连续处的典型症状）
              // 这正是"尺度爆炸"最可能的来源，所以必须量出来。
              //=================================================================
              {
                std::vector<double> errs;
                for (const auto &pair : (*it1)->timestamps) {
                  const size_t cid = pair.first;
                  auto itc = clones_cam.find(cid);
                  if (itc == clones_cam.end()) continue;
                  auto iti = state->_cam_intrinsics_cameras.find(cid);
                  if (iti == state->_cam_intrinsics_cameras.end()) continue;
                  const auto &uvs_c = (*it1)->uvs.at(cid);
                  const auto &ts_c = pair.second;
                  for (size_t k = 0; k < ts_c.size() && k < uvs_c.size(); ++k) {
                    auto itcl = itc->second.find(ts_c[k]);
                    if (itcl == itc->second.end()) continue;
                    const Eigen::Vector3d P_C =
                        itcl->second.Rot() * ((*it1)->p_FinG - itcl->second.pos());
                    if (P_C.z() < 0.1) continue;
                    const Eigen::Vector2d uv_norm = P_C.head<2>() / P_C.z();
                    const Eigen::Vector2d uv_projected = iti->second->distort_d(uv_norm);
                    const double du = uv_projected(0) - uvs_c[k](0);
                    const double dv = uv_projected(1) - uvs_c[k](1);
                    errs.push_back(std::sqrt(du * du + dv * dv));
                  }
                }
                if (!errs.empty()) {
                  std::nth_element(errs.begin(), errs.begin() + errs.size() / 2, errs.end());
                  const double med = errs[errs.size() / 2];
                  // 逐特征导出（时间 + 中位重投影误差），用来区分
                  // "一开始就错（深度关联问题）" 和 "后来越来越错（滤波器先歪了）"
                  static std::ofstream derr_of;
                  static bool derr_checked = false;
                  if (!derr_checked) {
                    derr_checked = true;
                    const char *dp = std::getenv("OV_DIAG_DEPTHERR");
                    if (dp != nullptr && dp[0] != '\0') {
                      derr_of.open(dp, std::ios::out | std::ios::trunc);
                      derr_of << "t_anchor,errmodel_reproj_px,nobs\n";
                      derr_of.flush();
                    }
                  }
                  if (derr_of.is_open())
                    derr_of << std::fixed << std::setprecision(6) << t_anchor << "," << med << ","
                            << errs.size() << "\n";
                  ++diag_deperr_n;
                  diag_deperr_sum += med;
                  if (med < 1.0) ++diag_deperr_hist[0];
                  else if (med < 2.0) ++diag_deperr_hist[1];
                  else if (med < 5.0) ++diag_deperr_hist[2];
                  else if (med < 10.0) ++diag_deperr_hist[3];
                  else if (med < 25.0) ++diag_deperr_hist[4];
                  else ++diag_deperr_hist[5];
                }
              }

              ++diag_tri_ok;
              it1++;
              continue;   // 跳过三角化和精化
            }
          }
        }
      }
    }

    // Triangulate the feature and remove if it fails
    bool success_tri = true;
    if (initializer_feat->config().triangulate_1d) {
      success_tri = initializer_feat->single_triangulation_1d(*it1, clones_cam);
    } else {
      success_tri = initializer_feat->single_triangulation(*it1, clones_cam);
    }

    // Gauss-newton refine the feature
    bool success_refine = true;
    if (initializer_feat->config().refine_features) {
      success_refine = initializer_feat->single_gaussnewton(*it1, clones_cam);
    }

    // Remove the feature if not a success
    if (!success_tri || !success_refine) {
      ++diag_tri_drop;
      if (!success_tri)
        ++diag_tri_fail_tri;
      else
        ++diag_tri_fail_refine;
      (*it1)->to_delete = true;
      it1 = feature_vec.erase(it1);
      continue;
    }
    ++diag_tri_ok;
    it1++;
  }
  rT2 = boost::posix_time::microsec_clock::local_time();

  // Calculate the max possible measurement size
  size_t max_meas_size = 0;
  for (size_t i = 0; i < feature_vec.size(); i++) {
    for (const auto &pair : feature_vec.at(i)->timestamps) {
      max_meas_size += 2 * feature_vec.at(i)->timestamps[pair.first].size();
    }
  }

  // Calculate max possible state size (i.e. the size of our covariance)
  // NOTE: that when we have the single inverse depth representations, those are only 1dof in size
  size_t max_hx_size = state->max_covariance_size();
  for (auto &landmark : state->_features_SLAM) {
    max_hx_size -= landmark.second->size();
  }

  // Large Jacobian and residual of *all* features for this update
  Eigen::VectorXd res_big = Eigen::VectorXd::Zero(max_meas_size);
  Eigen::MatrixXd Hx_big = Eigen::MatrixXd::Zero(max_meas_size, max_hx_size);
  std::unordered_map<std::shared_ptr<Type>, size_t> Hx_mapping;
  std::vector<std::shared_ptr<Type>> Hx_order_big;
  size_t ct_jacob = 0;
  size_t ct_meas = 0;

  // 【2026-10-21】自适应门限：读一次环境变量，初始化当前倍数
  static const int min_accept_env = [] {
    const char *e = std::getenv("OV_MIN_ACCEPT");
    return e ? std::atoi(e) : 0;
  }();
  min_accepted_per_frame = min_accept_env;
  if (eff_chi2_mult < 0.0)
    eff_chi2_mult = _options.chi2_multipler;
  const double cur_mult = eff_chi2_mult;
  const long long ok_before = diag_chi2_ok;

  // 4. Compute linear system for each feature, nullspace project, and reject
  auto it2 = feature_vec.begin();
  while (it2 != feature_vec.end()) {

    // Convert our feature into our current format
    UpdaterHelper::UpdaterHelperFeature feat;
    feat.featid = (*it2)->featid;
    feat.uvs = (*it2)->uvs;
    feat.uvs_norm = (*it2)->uvs_norm;
    feat.timestamps = (*it2)->timestamps;

    // If we are using single inverse depth, then it is equivalent to using the msckf inverse depth
    feat.feat_representation = state->_options.feat_rep_msckf;
    if (state->_options.feat_rep_msckf == LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE) {
      feat.feat_representation = LandmarkRepresentation::Representation::ANCHORED_MSCKF_INVERSE_DEPTH;
    }

    // Save the position and its fej value
    if (LandmarkRepresentation::is_relative_representation(feat.feat_representation)) {
      feat.anchor_cam_id = (*it2)->anchor_cam_id;
      feat.anchor_clone_timestamp = (*it2)->anchor_clone_timestamp;
      feat.p_FinA = (*it2)->p_FinA;
      feat.p_FinA_fej = (*it2)->p_FinA;
    } else {
      feat.p_FinG = (*it2)->p_FinG;
      feat.p_FinG_fej = (*it2)->p_FinG;
    }

    // Our return values (feature jacobian, state jacobian, residual, and order of state jacobian)
    Eigen::MatrixXd H_f;
    Eigen::MatrixXd H_x;
    Eigen::VectorXd res;
    std::vector<std::shared_ptr<Type>> Hx_order;

    // Get the Jacobian for this feature
    UpdaterHelper::get_feature_jacobian_full(state, feat, H_f, H_x, res, Hx_order);

    //=================================================================================
    // 【2026-10-21 诊断】逐观测记录原始残差 (u, v, res_u, res_v)。
    // 目的：区分"卷帘快门"和"相机-IMU 时延错位"——
    //   卷帘：残差随图像行号 v 线性变化（每行曝光时刻不同 → 整帧倾斜）
    //   时延：残差是整帧统一的一个平移（与 v 无关）
    // 需要 OV_DIAG_FEATS=<路径>，默认零开销。
    //=================================================================================
    static std::ofstream feat_of;
    static bool feat_checked = false;
    if (!feat_checked) {
      feat_checked = true;
      const char *fp = std::getenv("OV_DIAG_FEATS");
      if (fp != nullptr && fp[0] != '\0') {
        feat_of.open(fp, std::ios::out | std::ios::trunc);
        feat_of << "t,u,v,res_u,res_v\n";
        feat_of.flush();
      }
    }
    if (feat_of.is_open() && !feat.uvs.empty() && !feat.timestamps.empty()) {
      const auto &uv_vec = feat.uvs.begin()->second;
      const auto &ts_vec = feat.timestamps.begin()->second;
      const int nobs = (int)std::min(uv_vec.size(), ts_vec.size());
      for (int k = 0; k < nobs && 2 * k + 1 < res.rows(); k++) {
        feat_of << std::fixed << std::setprecision(6) << ts_vec[k] << ',' << uv_vec[k](0) << ',' << uv_vec[k](1) << ','
                << res(2 * k) << ',' << res(2 * k + 1) << '\n';
      }
    }

    // Nullspace project
    UpdaterHelper::nullspace_project_inplace(H_f, H_x, res);

    /// Chi2 distance check
    Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, Hx_order);
    Eigen::MatrixXd S = H_x * P_marg * H_x.transpose();
    S.diagonal() += _options.sigma_pix_sq * Eigen::VectorXd::Ones(S.rows());
    double chi2 = res.dot(S.llt().solve(res));

    // Get our threshold (we precompute up to 500 but handle the case that it is more)
    double chi2_check;
    if (res.rows() < 500) {
      chi2_check = chi_squared_table[res.rows()];
    } else {
      boost::math::chi_squared chi_squared_dist(res.rows());
      chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
      PRINT_WARNING(YELLOW "chi2_check over the residual limit - %d\n" RESET, (int)res.rows());
    }

    // Check if we should delete or not
    //=================================================================================
    // 【2026-10-21 诊断】逐特征记录卡方判定 + 跟踪历史长度。
    // 目的：转弯锁死时，被拒的是"刚检出的短轨迹"还是"跟了很久的长轨迹"？
    //   短轨迹被拒 = 三角化质量差（视差小）；
    //   长轨迹被拒 = 滤波器状态/时序错，问题在模型侧。
    // OV_DIAG_FEATDEC=<路径>，默认零开销。
    //=================================================================================
    static std::ofstream dec_of;
    static bool dec_checked = false;
    if (!dec_checked) {
      dec_checked = true;
      const char *dp = std::getenv("OV_DIAG_FEATDEC");
      if (dp != nullptr && dp[0] != '\0') {
        dec_of.open(dp, std::ios::out | std::ios::trunc);
        dec_of << "t,featid,nobs,tspan,chi2,chi2_check,rejected\n";
        dec_of.flush();
      }
    }
    if (dec_of.is_open() && !feat.timestamps.empty()) {
      const auto &tsv = feat.timestamps.begin()->second;
      const int nobs = (int)tsv.size();
      const double tspan = (nobs > 1) ? (tsv.back() - tsv.front()) : 0.0;
      const bool rej = (chi2 > cur_mult * chi2_check);
      dec_of << std::fixed << std::setprecision(6) << state->_timestamp << "," << feat.featid << "," << nobs << ","
             << tspan << "," << chi2 << "," << chi2_check << "," << (rej ? 1 : 0) << '\n';
    }

    const double ratio = chi2 / std::max(cur_mult * chi2_check, 1e-12);
    if (chi2 > cur_mult * chi2_check) {
      ++diag_feat_chi2_rej;
      diag_chi2_rej_ratio_sum += ratio;
      diag_chi2_rej_ratio_max = std::max(diag_chi2_rej_ratio_max, ratio);
      ++diag_chi2_rej_hist[(ratio < 2.0) ? 0 : (ratio < 5.0) ? 1 : (ratio < 20.0) ? 2 : (ratio < 200.0) ? 3 : 4];
      (*it2)->to_delete = true;
      it2 = feature_vec.erase(it2);
      // PRINT_DEBUG("featid = %d\n", feat.featid);
      // PRINT_DEBUG("chi2 = %f > %f\n", chi2, _options.chi2_multipler*chi2_check);
      // std::stringstream ss;
      // ss << "res = " << std::endl << res.transpose() << std::endl;
      // PRINT_DEBUG(ss.str().c_str());
      continue;
    }
    diag_chi2_ok_ratio_sum += ratio;

    // We are good!!! Append to our large H vector
    size_t ct_hx = 0;
    for (const auto &var : Hx_order) {

      // Ensure that this variable is in our Jacobian
      if (Hx_mapping.find(var) == Hx_mapping.end()) {
        Hx_mapping.insert({var, ct_jacob});
        Hx_order_big.push_back(var);
        ct_jacob += var->size();
      }

      // Append to our large Jacobian
      Hx_big.block(ct_meas, Hx_mapping[var], H_x.rows(), var->size()) = H_x.block(0, ct_hx, H_x.rows(), var->size());
      ct_hx += var->size();
    }

    // Append our residual and move forward
    res_big.block(ct_meas, 0, res.rows(), 1) = res;
    ct_meas += res.rows();
    ++diag_chi2_ok;
    it2++;
  }
  rT3 = boost::posix_time::microsec_clock::local_time();

  // We have appended all features to our Hx_big, res_big
  // Delete it so we do not reuse information
  for (size_t f = 0; f < feature_vec.size(); f++) {
    feature_vec[f]->to_delete = true;
  }

  // 【2026-10-21】根据这一帧实际通过的数量调整下一帧的门限
  if (min_accepted_per_frame > 0) {
    const long long ok_now = diag_chi2_ok - ok_before;
    if (ok_now < min_accepted_per_frame) {
      eff_chi2_mult = std::min(max_chi2_mult, eff_chi2_mult * 2.0);
    } else {
      eff_chi2_mult = std::max(_options.chi2_multipler, eff_chi2_mult * 0.7);
    }
    diag_eff_mult_max = std::max(diag_eff_mult_max, static_cast<long long>(eff_chi2_mult * 100));
  }

  diag_ctmeas += static_cast<long long>(ct_meas);

  // Return if we don't have anything and resize our matrices
  if (ct_meas < 1) {
    return;
  }
  assert(ct_meas <= max_meas_size);
  assert(ct_jacob <= max_hx_size);
  res_big.conservativeResize(ct_meas, 1);
  Hx_big.conservativeResize(ct_meas, ct_jacob);

  // 5. Perform measurement compression
  UpdaterHelper::measurement_compress_inplace(Hx_big, res_big);
  if (Hx_big.rows() < 1) {
    ++diag_comp_zero;
    return;
  }
  rT4 = boost::posix_time::microsec_clock::local_time();

  // Our noise is isotropic, so make it here after our compression
  Eigen::MatrixXd R_big = _options.sigma_pix_sq * Eigen::MatrixXd::Identity(res_big.rows(), res_big.rows());

  // 6. With all good features update the state
  ++diag_updates;
  diag_rows += static_cast<long long>(res_big.rows());
  StateHelper::EKFUpdate(state, Hx_order_big, Hx_big, res_big, R_big);
  rT5 = boost::posix_time::microsec_clock::local_time();

  // Debug print timing information
  PRINT_ALL("[MSCKF-UP]: %.4f seconds to clean\n", (rT1 - rT0).total_microseconds() * 1e-6);
  PRINT_ALL("[MSCKF-UP]: %.4f seconds to triangulate\n", (rT2 - rT1).total_microseconds() * 1e-6);
  PRINT_ALL("[MSCKF-UP]: %.4f seconds create system (%d features)\n", (rT3 - rT2).total_microseconds() * 1e-6, (int)feature_vec.size());
  PRINT_ALL("[MSCKF-UP]: %.4f seconds compress system\n", (rT4 - rT3).total_microseconds() * 1e-6);
  PRINT_ALL("[MSCKF-UP]: %.4f seconds update state (%d size)\n", (rT5 - rT4).total_microseconds() * 1e-6, (int)res_big.rows());
  PRINT_ALL("[MSCKF-UP]: %.4f seconds total\n", (rT5 - rT1).total_microseconds() * 1e-6);
}
