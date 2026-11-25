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
#include "utils/opencv_lambda_body.h"
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
  // 特征数 >= 8 时分发到 OpenCV 线程池并行计算各特征三角化/高斯牛顿精化/深度回退，
  // 再按原下标顺序归约统计与压缩向量，保证数值与单线程逐位一致（设 OV_MSCKF_SERIAL=1 可强制串行做 A/B）。
  constexpr size_t kParallelMinFeatures = 8;
  static const bool force_serial_msckf = (std::getenv("OV_MSCKF_SERIAL") != nullptr);
  const auto &depth_fn = depth_lookup_ ? depth_lookup_ : UpdaterMSCKF::DepthLookup::fn;

  auto eval_depth_med_feat = [&](const std::shared_ptr<Feature> &feat_ptr, const Eigen::Vector3d &p_G, size_t &n_obs_out) -> double {
    std::vector<double> errs;
    for (const auto &pair : feat_ptr->timestamps) {
      const size_t cid = pair.first;
      auto itc = clones_cam.find(cid);
      if (itc == clones_cam.end()) continue;
      auto iti = state->_cam_intrinsics_cameras.find(cid);
      if (iti == state->_cam_intrinsics_cameras.end()) continue;
      const double fx_cur = std::max(452.5, iti->second->get_K()(0, 0));
      const double scale_ref = 452.5 / fx_cur;
      const auto &uvs_c = feat_ptr->uvs.at(cid);
      const auto &ts_c = pair.second;
      for (size_t k = 0; k < ts_c.size() && k < uvs_c.size(); ++k) {
        auto itcl = itc->second.find(ts_c[k]);
        if (itcl == itc->second.end()) continue;
        const Eigen::Vector3d P_C = itcl->second.Rot() * (p_G - itcl->second.pos());
        if (P_C.z() < 0.1) continue;
        const Eigen::Vector2d uv_norm = P_C.head<2>() / P_C.z();
        const Eigen::Vector2d uv_projected = iti->second->distort_d(uv_norm);
        const double du = (uv_projected(0) - uvs_c[k](0)) * scale_ref;
        const double dv = (uv_projected(1) - uvs_c[k](1)) * scale_ref;
        errs.push_back(std::sqrt(du * du + dv * dv));
      }
    }
    n_obs_out = errs.size();
    if (errs.empty()) return -1.0;
    std::nth_element(errs.begin(), errs.begin() + errs.size() / 2, errs.end());
    return errs[errs.size() / 2];
  };

  auto record_deperr = [&](double med) {
    if (med < 0.0) return;
    ++diag_deperr_n;
    diag_deperr_sum += std::min(med, 30.0);
    if (med < 1.0) ++diag_deperr_hist[0];
    else if (med < 2.0) ++diag_deperr_hist[1];
    else if (med < 5.0) ++diag_deperr_hist[2];
    else if (med < 10.0) ++diag_deperr_hist[3];
    else if (med < 25.0) ++diag_deperr_hist[4];
    else ++diag_deperr_hist[5];
  };

  struct TriSlot {
    bool keep = false;
    bool depth_used = false;
    bool tri_ok = false;
    bool fail_tri = false;
    double deperr_med = -1.0;
  };

  const size_t n_tri = feature_vec.size();
  std::vector<TriSlot> tri_slots(n_tri);

  auto tri_worker = [&](size_t idx) {
    auto &feat_ptr = feature_vec[idx];
    TriSlot &slot = tri_slots[idx];

    //=================================================================================
    // 【2026-11-23】优先使用多视图三角化 + 高斯牛顿精化（视差充足时线性化点最精确，
    // 不会因深度传感器 2~5% 噪声或前景/背景遮挡边缘外点引入虚假杠杆臂残差）。
    // 仅在多视图三角化失败（纯旋转/弱视差）时回退至深度初值，并施加严格一致性门控
    // （>=3 帧观测、深度 [0.30, 4.5]m、折算 640p 中位重投影误差 <= 1.2 px）。
    //=================================================================================
    bool success_tri = true;
    if (initializer_feat->config().triangulate_1d) {
      success_tri = initializer_feat->single_triangulation_1d(feat_ptr, clones_cam);
    } else {
      success_tri = initializer_feat->single_triangulation(feat_ptr, clones_cam);
    }

    // Gauss-newton refine the feature
    bool success_refine = true;
    if (success_tri && initializer_feat->config().refine_features) {
      success_refine = initializer_feat->single_gaussnewton(feat_ptr, clones_cam);
    }

    if (success_tri && success_refine) {
      // 保留深度一致性诊断统计（供 HealthMonitor 识别特征跟踪崩塌），但不篡改已精化的三角化位置
      if (depth_fn && !feat_ptr->timestamps.empty()) {
        const size_t cid0 = feat_ptr->anchor_cam_id;
        auto it_ts = feat_ptr->timestamps.find(cid0);
        auto it_uv = feat_ptr->uvs.find(cid0);
        if (it_ts != feat_ptr->timestamps.end() && it_uv != feat_ptr->uvs.end() && !it_ts->second.empty() && !it_uv->second.empty()) {
          const double t_a = it_ts->second.back();
          const float z_meas = depth_fn(t_a, it_uv->second.back()(0), it_uv->second.back()(1));
          if (z_meas >= 0.30f && z_meas <= 4.5f) {
            auto it_cam = clones_cam.find(cid0);
            if (it_cam != clones_cam.end()) {
              auto it_clone = it_cam->second.find(t_a);
              if (it_clone != it_cam->second.end()) {
                Eigen::Vector2d uv_n = state->_cam_intrinsics_cameras.at(cid0)->undistort_d(
                    Eigen::Vector2d(it_uv->second.back()(0), it_uv->second.back()(1)));
                Eigen::Vector3d b(uv_n(0), uv_n(1), 1.0);
                const Eigen::Vector3d p_depth =
                    it_clone->second.pos() + it_clone->second.Rot().transpose() * (b * static_cast<double>(z_meas));
                size_t n_obs = 0;
                slot.deperr_med = eval_depth_med_feat(feat_ptr, p_depth, n_obs);
              }
            }
          }
        }
      }
      slot.tri_ok = true;
      slot.keep = true;
      return;
    }

    // 三角化失败（弱视差/纯旋转）：尝试深度回退初始化
    if (depth_fn && !feat_ptr->timestamps.empty()) {
      const size_t cam_id0 = feat_ptr->timestamps.begin()->first;
      const auto &ts0 = feat_ptr->timestamps.begin()->second;
      const auto &uv_list = feat_ptr->uvs.at(cam_id0);
      if (ts0.size() >= 3 && ts0.size() == uv_list.size()) {
        const double t_anchor = ts0.back();
        const float u0 = uv_list.back()(0);
        const float v0 = uv_list.back()(1);
        const float z_meas = depth_fn(t_anchor, u0, v0);
        if (z_meas >= 0.30f && z_meas <= 4.5f) {
          auto it_cam = clones_cam.find(cam_id0);
          if (it_cam != clones_cam.end()) {
            auto it_clone = it_cam->second.find(t_anchor);
            if (it_clone != it_cam->second.end()) {
              Eigen::Vector2d uv_n = state->_cam_intrinsics_cameras.at(cam_id0)->undistort_d(Eigen::Vector2d(u0, v0));
              Eigen::Vector3d b(uv_n(0), uv_n(1), 1.0);
              const Eigen::Matrix3d &R_GtoA = it_clone->second.Rot();
              const Eigen::Vector3d &p_AinG = it_clone->second.pos();
              const Eigen::Vector3d p_FinA_depth = b * static_cast<double>(z_meas);
              const Eigen::Vector3d p_depth = p_AinG + R_GtoA.transpose() * p_FinA_depth;
              feat_ptr->anchor_cam_id = cam_id0;
              feat_ptr->anchor_clone_timestamp = t_anchor;
              feat_ptr->p_FinA = p_FinA_depth;
              feat_ptr->p_FinG = p_depth;
              if (initializer_feat->config().refine_features) {
                if (!initializer_feat->single_gaussnewton(feat_ptr, clones_cam)) {
                  feat_ptr->p_FinA = p_FinA_depth;
                  feat_ptr->p_FinG = p_depth;
                } else {
                  const Eigen::Vector3d p_FinA_ref = R_GtoA * (feat_ptr->p_FinG - p_AinG);
                  if (!p_FinA_ref.allFinite() || p_FinA_ref.z() < 0.25 || p_FinA_ref.z() > 15.0 ||
                      std::abs(p_FinA_ref.z() - static_cast<double>(z_meas)) > 0.25 * static_cast<double>(z_meas)) {
                    feat_ptr->p_FinA = p_FinA_depth;
                    feat_ptr->p_FinG = p_depth;
                  }
                }
              }
              size_t n_obs = 0;
              const double med = eval_depth_med_feat(feat_ptr, feat_ptr->p_FinG, n_obs);
              slot.deperr_med = med;
              if (n_obs >= 3 && med >= 0.0 && med <= 2.0) {
                slot.depth_used = true;
                slot.tri_ok = true;
                slot.keep = true;
                return;
              }
            }
          }
        }
      }
    }

    // Remove the feature if not a success
    slot.keep = false;
    slot.fail_tri = !success_tri;
    feat_ptr->to_delete = true;
  };

  if (!force_serial_msckf && n_tri >= kParallelMinFeatures) {
    cv::parallel_for_(cv::Range(0, static_cast<int>(n_tri)), LambdaBody([&](const cv::Range &range) {
                        for (int i = range.start; i < range.end; ++i) {
                          tri_worker(static_cast<size_t>(i));
                        }
                      }));
  } else {
    for (size_t i = 0; i < n_tri; ++i) {
      tri_worker(i);
    }
  }

  size_t write_tri = 0;
  for (size_t i = 0; i < n_tri; ++i) {
    const TriSlot &slot = tri_slots[i];
    if (slot.deperr_med >= 0.0) {
      record_deperr(slot.deperr_med);
    }
    if (slot.keep) {
      if (slot.depth_used) ++diag_depth_used;
      if (slot.tri_ok) ++diag_tri_ok;
      if (write_tri != i) {
        feature_vec[write_tri] = std::move(feature_vec[i]);
      }
      ++write_tri;
    } else {
      ++diag_tri_drop;
      if (slot.fail_tri)
        ++diag_tri_fail_tri;
      else
        ++diag_tri_fail_refine;
    }
  }
  feature_vec.resize(write_tri);
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
  // 诊断文件在并行区外单线程初始化一次，避免静态初始化与文件流竞争
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

  struct JacSlot {
    Eigen::MatrixXd H_x;
    Eigen::VectorXd res;
    Eigen::VectorXd res_raw;
    std::vector<std::shared_ptr<Type>> Hx_order;
    double chi2 = 0.0;
    double chi2_check = 0.0;
    double ratio = 0.0;
    bool rejected = false;
    bool over_limit_warn = false;
    int over_limit_rows = 0;
  };

  const size_t n_jac = feature_vec.size();
  std::vector<JacSlot> jac_slots(n_jac);
  const bool save_raw_res = feat_of.is_open();

  auto jac_worker = [&](size_t idx) {
    const auto &src_feat = feature_vec[idx];
    JacSlot &slot = jac_slots[idx];

    UpdaterHelper::UpdaterHelperFeature feat;
    feat.featid = src_feat->featid;
    feat.uvs = src_feat->uvs;
    feat.uvs_norm = src_feat->uvs_norm;
    feat.timestamps = src_feat->timestamps;

    feat.feat_representation = state->_options.feat_rep_msckf;
    if (state->_options.feat_rep_msckf == LandmarkRepresentation::Representation::ANCHORED_INVERSE_DEPTH_SINGLE) {
      feat.feat_representation = LandmarkRepresentation::Representation::ANCHORED_MSCKF_INVERSE_DEPTH;
    }

    if (LandmarkRepresentation::is_relative_representation(feat.feat_representation)) {
      feat.anchor_cam_id = src_feat->anchor_cam_id;
      feat.anchor_clone_timestamp = src_feat->anchor_clone_timestamp;
      feat.p_FinA = src_feat->p_FinA;
      feat.p_FinA_fej = src_feat->p_FinA;
    } else {
      feat.p_FinG = src_feat->p_FinG;
      feat.p_FinG_fej = src_feat->p_FinG;
    }

    Eigen::MatrixXd H_f;
    UpdaterHelper::get_feature_jacobian_full(state, feat, H_f, slot.H_x, slot.res, slot.Hx_order);
    if (save_raw_res) {
      slot.res_raw = slot.res;
    }

    UpdaterHelper::nullspace_project_inplace(H_f, slot.H_x, slot.res);

    Eigen::MatrixXd P_marg = StateHelper::get_marginal_covariance(state, slot.Hx_order);
    Eigen::MatrixXd S = slot.H_x * P_marg * slot.H_x.transpose();
    S.diagonal() += _options.sigma_pix_sq * Eigen::VectorXd::Ones(S.rows());
    slot.chi2 = slot.res.dot(S.llt().solve(slot.res));

    auto it_chi = (slot.res.rows() < 500) ? chi_squared_table.find(static_cast<int>(slot.res.rows())) : chi_squared_table.end();
    if (it_chi != chi_squared_table.end()) {
      slot.chi2_check = it_chi->second;
    } else {
      boost::math::chi_squared chi_squared_dist(slot.res.rows());
      slot.chi2_check = boost::math::quantile(chi_squared_dist, 0.95);
      slot.over_limit_warn = (slot.res.rows() >= 500);
      slot.over_limit_rows = static_cast<int>(slot.res.rows());
    }

    slot.ratio = slot.chi2 / std::max(cur_mult * slot.chi2_check, 1e-12);
    slot.rejected = (slot.chi2 > cur_mult * slot.chi2_check);
  };

  if (!force_serial_msckf && n_jac >= kParallelMinFeatures) {
    cv::parallel_for_(cv::Range(0, static_cast<int>(n_jac)), LambdaBody([&](const cv::Range &range) {
                        for (int i = range.start; i < range.end; ++i) {
                          jac_worker(static_cast<size_t>(i));
                        }
                      }));
  } else {
    for (size_t i = 0; i < n_jac; ++i) {
      jac_worker(i);
    }
  }

  size_t write_jac = 0;
  for (size_t i = 0; i < n_jac; ++i) {
    auto &src_feat = feature_vec[i];
    const JacSlot &slot = jac_slots[i];

    if (save_raw_res && !src_feat->uvs.empty() && !src_feat->timestamps.empty()) {
      const auto &uv_vec = src_feat->uvs.begin()->second;
      const auto &ts_vec = src_feat->timestamps.begin()->second;
      const int nobs = static_cast<int>(std::min(uv_vec.size(), ts_vec.size()));
      for (int k = 0; k < nobs && 2 * k + 1 < slot.res_raw.rows(); ++k) {
        feat_of << std::fixed << std::setprecision(6) << ts_vec[k] << ',' << uv_vec[k](0) << ',' << uv_vec[k](1) << ','
                << slot.res_raw(2 * k) << ',' << slot.res_raw(2 * k + 1) << '\n';
      }
    }

    if (slot.over_limit_warn) {
      PRINT_WARNING(YELLOW "chi2_check over the residual limit - %d\n" RESET, slot.over_limit_rows);
    }

    if (dec_of.is_open() && !src_feat->timestamps.empty()) {
      const auto &tsv = src_feat->timestamps.begin()->second;
      const int nobs = static_cast<int>(tsv.size());
      const double tspan = (nobs > 1) ? (tsv.back() - tsv.front()) : 0.0;
      dec_of << std::fixed << std::setprecision(6) << state->_timestamp << "," << src_feat->featid << "," << nobs << ","
             << tspan << "," << slot.chi2 << "," << slot.chi2_check << "," << (slot.rejected ? 1 : 0) << '\n';
    }

    if (slot.rejected) {
      ++diag_feat_chi2_rej;
      diag_chi2_rej_ratio_sum += slot.ratio;
      diag_chi2_rej_ratio_max = std::max(diag_chi2_rej_ratio_max, slot.ratio);
      ++diag_chi2_rej_hist[(slot.ratio < 2.0) ? 0 : (slot.ratio < 5.0) ? 1 : (slot.ratio < 20.0) ? 2 : (slot.ratio < 200.0) ? 3 : 4];
      src_feat->to_delete = true;
      continue;
    }

    diag_chi2_ok_ratio_sum += slot.ratio;

    size_t ct_hx = 0;
    for (const auto &var : slot.Hx_order) {
      if (Hx_mapping.find(var) == Hx_mapping.end()) {
        Hx_mapping.insert({var, ct_jacob});
        Hx_order_big.push_back(var);
        ct_jacob += var->size();
      }
      Hx_big.block(ct_meas, Hx_mapping[var], slot.H_x.rows(), var->size()) = slot.H_x.block(0, ct_hx, slot.H_x.rows(), var->size());
      ct_hx += var->size();
    }

    res_big.block(ct_meas, 0, slot.res.rows(), 1) = slot.res;
    ct_meas += slot.res.rows();
    ++diag_chi2_ok;
    if (write_jac != i) {
      feature_vec[write_jac] = std::move(src_feat);
    }
    ++write_jac;
  }
  feature_vec.resize(write_jac);
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
