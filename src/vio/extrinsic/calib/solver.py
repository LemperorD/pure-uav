"""Sparse joint reprojection + gyroscope + accelerometer optimization.

Camera trajectory is parameterized in CAMERA time. For an IMU sample at ti,
the trajectory is evaluated at ti - shift, hence ti = tc + shift.
"""
import json
from dataclasses import asdict
from pathlib import Path

import numpy as np
import yaml
from scipy.optimize import least_squares
from scipy.sparse import lil_matrix
from scipy.spatial.transform import Rotation, RotationSpline

from .data import CalibrationError
from .spline import PoseSpline, invert_transform, mv, predict_imu, skew


def rms(a):
    return float(np.sqrt(np.mean(np.asarray(a)**2)))


def optimizer_info(result):
    return {"success": bool(result.success), "status": int(result.status),
            "message": str(result.message), "nfev": int(result.nfev),
            "cost": float(result.cost), "optimality": float(result.optimality)}


class Problem:
    # Global vector = rotvec_CI, p_CI, dt, bg_I, ba_I, gravity_W.
    ng = 16

    def __init__(self, camera, observations, tc, ti, imu, settings):
        self.camera, self.obs, self.tc, self.cfg = camera, observations, tc, settings
        self.check_visual()
        # Keep only IMU times whose evaluations stay inside the detected sequence
        # for EVERY permitted offset, with spline endpoint derivative padding.
        margin = settings.max_offset_s + settings.knot_spacing_s
        selected = np.flatnonzero((ti >= tc[0]+margin) & (ti <= tc[-1]-margin))
        if len(selected) < 100:
            raise CalibrationError("相机/IMU 时间重叠不足；请先同步到同一时钟域。")
        # Time-based decimation, retaining original values and original timestamps.
        keep, last = [], -np.inf
        for j in selected:
            if ti[j]-last >= 0.99/settings.imu_rate_hz:
                keep.append(j)
                last = ti[j]
        self.ti, self.imu = ti[keep], imu[keep]
        if np.max(np.diff(self.ti)) > settings.max_imu_gap_s:
            raise CalibrationError("IMU 数据空洞过大；请检查丢包或调大 max_imu_gap_s。")
        if np.percentile(np.linalg.norm(self.imu[:, :3], axis=1), 50) < 2:
            raise CalibrationError("IMU 加速度过小：需输入 m/s² 的比力（含静止重力响应），不能输入去重力加速度或 g 单位。")
        h = settings.knot_spacing_s
        start = tc[0]-2*h
        n = int(np.ceil((tc[-1]-start)/h))+4
        self.spline = PoseSpline(start, h, n)
        self.frame_ids = np.concatenate([np.full(len(o.uv), i, int) for i, o in enumerate(observations)])
        self.xyz = np.concatenate([o.xyz for o in observations])
        self.uv = np.concatenate([o.uv for o in observations])
        self.npix = self.uv.size
        self.edge_ids = np.array([0, 1, n-2, n-1])
        self.history = []
        self.initial_controls = self.fit_initial_controls()
        self.edge_ref = self.initial_controls[self.edge_ids].copy()

    def check_visual(self):
        s = self.cfg
        if len(self.obs) < s.min_frames or self.tc[-1]-self.tc[0] < s.min_duration_s:
            raise CalibrationError(f"有效帧数或序列长度不足：{len(self.obs)} 帧 / "
                                   f"{self.tc[-1]-self.tc[0]:.2f}s，要求 ≥{s.min_frames} 帧、"
                                   f"≥{s.min_duration_s:g}s。")
        if np.max(np.diff(self.tc)) > s.max_camera_gap_s:
            raise CalibrationError("成功检测标定板帧之间存在大空洞；请截取连续可见的窗口"
                                   "（Settings.auto_window=1 时程序会自动挑最长连续段，"
                                   "走到这里说明选窗之后仍不连续）。")

    def fit_initial_controls(self):
        spline = self.spline
        t_ctrl = spline.start+(np.arange(spline.n)-1)*spline.h
        rotations = Rotation.from_matrix(np.array([o.r_wc for o in self.obs]))
        positions = np.array([o.p_wc for o in self.obs])
        rs = RotationSpline(self.tc, rotations)
        rv = rs(t_ctrl).as_rotvec()
        # Solve the linear translation B-spline collocation system. A small
        # second-difference prior stabilizes unobserved padding, only at init.
        b = spline.translation_design(self.tc)
        smooth = np.diff(np.eye(spline.n), n=2, axis=0)
        p = np.linalg.lstsq(np.vstack([b, 0.01*smooth]),
                            np.vstack([positions, np.zeros((len(smooth), 3))]), rcond=None)[0]
        guess = np.column_stack([rv, p])
        ref = guess.copy()
        # Correct the SO(3) control pose attenuation using pose observations.
        def residual(flat):
            ctrl = flat.reshape(-1, 6)
            r, pos, *_ = spline.evaluate(self.tc, ctrl[:, :3], ctrl[:, 3:])
            er = Rotation.from_matrix(np.swapaxes(rotations.as_matrix(), -1, -2) @ r).as_rotvec()
            return np.r_[er.ravel()/0.005, (pos-positions).ravel()/0.003,
                         (ctrl[self.edge_ids]-ref[self.edge_ids]).ravel()/0.1]
        nr = len(self.tc)*6+len(self.edge_ids)*6
        pattern = lil_matrix((nr, spline.n*6), dtype=np.int8)
        support = spline.support(self.tc)[0]
        for k, i in enumerate(support):
            cols = np.arange(i*6, (i+4)*6)
            pattern[k*3:k*3+3, cols] = 1
            off = len(self.tc)*3+k*3
            pattern[off:off+3, cols] = 1
        off = len(self.tc)*6
        for k, i in enumerate(self.edge_ids):
            pattern[off+k*6:off+(k+1)*6, i*6:(i+1)*6] = 1
        result = least_squares(residual, guess.ravel(), jac_sparsity=pattern.tocsr(),
                               max_nfev=35, ftol=1e-5, xtol=1e-7, gtol=1e-6,
                               tr_options={"atol": 1e-6, "btol": 1e-6})
        self.history.append({"stage": "pnp_spline_initialization", **optimizer_info(result)})
        return result.x.reshape(-1, 6)

    def trajectory(self, times, controls):
        return self.spline.evaluate(times, controls[:, :3], controls[:, 3:])

    def initialize_globals(self):
        ctrl = self.initial_controls
        best = None
        for dt in np.linspace(-self.cfg.max_offset_s*0.98, self.cfg.max_offset_s*0.98, 81):
            _, _, _, w, _ = self.trajectory(self.ti-dt, ctrl)
            wc, wi = w-w.mean(0), self.imu[:, 3:]-self.imu[:, 3:].mean(0)
            r_ic, _ = Rotation.align_vectors(wi, wc)
            bg = self.imu[:, 3:].mean(0)-r_ic.apply(w.mean(0))
            score = rms(r_ic.apply(w)+bg-self.imu[:, 3:])
            if best is None or score < best[0]:
                best = score, dt, r_ic.inv(), bg
        _, dt, r_ci, bg = best
        r, _, a, w, alpha = self.trajectory(self.ti-dt, ctrl)
        singular = np.linalg.svd(w-w.mean(0), compute_uv=False)/np.sqrt(len(w))
        if singular[1] < self.cfg.min_rotation_std_rad_s:
            raise CalibrationError(f"多轴旋转激励不足，角速度标准差奇异值={singular.tolist()}。")
        # f_I - R_IC R_CW a_W = [R_IC L, -R_IC R_CW, I] [p_CI,g_W,ba_I]
        r_ic = r_ci.as_matrix().T
        l = skew(alpha)+skew(w) @ skew(w)
        r_cw = np.swapaxes(r, -1, -2)
        ap = r_ic @ l
        nuisance = np.concatenate([-r_ic @ r_cw, np.broadcast_to(np.eye(3), r.shape)], axis=2)
        design = np.concatenate([ap, nuisance], axis=2).reshape(-1, 9)
        target = (self.imu[:, :3]-mv(r_ic @ r_cw, a)).ravel()
        coeff = np.linalg.lstsq(design, target, rcond=None)[0]
        # Marginalize gravity+bias before assessing lever arm observability.
        nn = nuisance.reshape(-1, 6)
        pp = ap.reshape(-1, 3)
        projected = pp-nn @ np.linalg.lstsq(nn, pp, rcond=None)[0]
        lever_sv = np.linalg.svd(projected, compute_uv=False)/np.sqrt(len(w))
        if lever_sv[-1] < self.cfg.min_lever_singular:
            raise CalibrationError(f"平移外参不可观：去除重力/零偏后的杠杆臂奇异值={lever_sv.tolist()}。请增加多轴转动。")
        g = coeff[3:6]
        if np.linalg.norm(g) < 1:
            raise CalibrationError("重力初值异常；检查加速度单位、坐标轴、时间同步。")
        g = g*self.cfg.gravity_m_s2/np.linalg.norm(g)
        p = np.clip(coeff[:3], -self.cfg.max_translation_m*0.8, self.cfg.max_translation_m*0.8)
        ba = np.clip(coeff[6:9], -self.cfg.max_accel_bias_m_s2*0.8, self.cfg.max_accel_bias_m_s2*0.8)
        bg = np.clip(bg, -self.cfg.max_gyro_bias_rad_s*0.8, self.cfg.max_gyro_bias_rad_s*0.8)
        self.excitation = {"angular_velocity_std_singular_values_rad_s": singular.tolist(),
                           "marginal_lever_singular_values_s_inv2": lever_sv.tolist()}
        return np.r_[r_ci.as_rotvec(), p, dt, bg, ba, g]

    def parts(self, x, fixed_controls=None):
        g = x[:self.ng]
        ctrl = x[self.ng:].reshape(-1, 6) if fixed_controls is None else fixed_controls
        r, p, *_ = self.trajectory(self.tc, ctrl)
        fid = self.frame_ids
        xyz_c = mv(np.swapaxes(r[fid], -1, -2), self.xyz-p[fid])
        pixels = self.camera.project(xyz_c, trial_depth_floor=self.cfg.min_target_depth_m)-self.uv
        ri, _, ai, w, alpha = self.trajectory(self.ti-g[6], ctrl)
        gyro, accel = predict_imu(ri, ai, w, alpha, Rotation.from_rotvec(g[:3]).as_matrix(),
                                  g[3:6], g[13:16], g[7:10], g[10:13])
        return pixels, gyro-self.imu[:, 3:], accel-self.imu[:, :3], xyz_c[:, 2]

    def residual(self, x, fixed_controls=None):
        s = self.cfg
        pix, gyro, accel, depth = self.parts(x, fixed_controls)
        imu_res = np.column_stack([gyro/s.gyro_sigma_rad_s, accel/s.accel_sigma_m_s2])
        gnorm = (np.linalg.norm(x[13:16])-s.gravity_m_s2)/s.gravity_norm_sigma
        # Independent cheirality residuals accompany the finite projection
        # continuation; the final solution still must satisfy the depth bound.
        depth_res = np.maximum(s.min_target_depth_m-depth, 0)/s.depth_sigma_m
        res = np.r_[pix.ravel()/s.image_sigma_px, depth_res, imu_res.ravel(), gnorm]
        if fixed_controls is None:
            ctrl = x[self.ng:].reshape(-1, 6)
            res = np.r_[res, ((ctrl[self.edge_ids]-self.edge_ref)/0.3).ravel()]
        return res

    def bounds(self, joint):
        s = self.cfg
        lower = np.r_[[-np.inf]*3, [-s.max_translation_m]*3, -s.max_offset_s,
                      [-s.max_gyro_bias_rad_s]*3, [-s.max_accel_bias_m_s2]*3, [-15.]*3]
        upper = -lower
        if joint:
            lower = np.r_[lower, np.full(self.spline.n*6, -np.inf)]
            upper = np.r_[upper, np.full(self.spline.n*6, np.inf)]
        return lower, upper

    def sparsity(self):
        nr = self.npix+len(self.xyz)+6*len(self.ti)+1+6*len(self.edge_ids)
        sp = lil_matrix((nr, self.ng+self.spline.n*6), dtype=np.int8)
        starts = self.spline.support(self.tc)[0]
        at = 0
        for obs, i in zip(self.obs, starts):
            n = obs.uv.size
            sp[at:at+n, self.ng+i*6:self.ng+(i+4)*6] = 1
            at += n
        for obs, i in zip(self.obs, starts):
            n = len(obs.xyz)
            sp[at:at+n, self.ng+i*6:self.ng+(i+4)*6] = 1
            at += n
        # Time shift moves a sample across knots: use the UNION over its complete
        # allowed interval, not merely the current support at the initial shift.
        lo = self.spline.support(self.ti-self.cfg.max_offset_s)[0]
        hi = self.spline.support(self.ti+self.cfg.max_offset_s)[0]
        for k, (i, j) in enumerate(zip(lo, hi)):
            rows = slice(at+k*6, at+(k+1)*6)
            sp[rows, :self.ng] = 1
            sp[rows, self.ng+i*6:self.ng+(j+4)*6] = 1
        at += 6*len(self.ti)
        sp[at, 13:16] = 1
        at += 1
        for k, i in enumerate(self.edge_ids):
            sp[at+k*6:at+(k+1)*6, self.ng+i*6:self.ng+(i+1)*6] = 1
        return sp.tocsr()

    def metrics(self, x):
        pix, gyro, accel, depth = self.parts(x)
        return {"reprojection_rms_px": rms(np.linalg.norm(pix, axis=1)),
                "reprojection_median_px": float(np.median(np.linalg.norm(pix, axis=1))),
                "reprojection_p95_px": float(np.percentile(np.linalg.norm(pix, axis=1), 95)),
                "gyro_rms_rad_s": rms(gyro), "gyro_rms_axes_rad_s": np.sqrt(np.mean(gyro**2, axis=0)).tolist(),
                "accel_rms_m_s2": rms(accel), "accel_rms_axes_m_s2": np.sqrt(np.mean(accel**2, axis=0)).tolist(),
                "min_target_depth_m": float(depth.min())}

    def solve(self):
        s = self.cfg
        initial_g = self.initialize_globals()
        global_result = least_squares(lambda g: self.residual(g, self.initial_controls), initial_g,
                                       bounds=self.bounds(False), loss="soft_l1", f_scale=s.robust_scale,
                                       max_nfev=s.init_max_nfev, x_scale="jac", ftol=s.optimizer_ftol,
                                       xtol=s.optimizer_xtol, gtol=s.optimizer_gtol)
        self.history.append({"stage": "fixed_trajectory_initialization", **optimizer_info(global_result)})
        x0 = np.r_[global_result.x, self.initial_controls.ravel()]
        initial_metrics = self.metrics(x0)
        print(f"联合优化：{len(self.obs)} 帧，{len(self.ti)} IMU，{self.spline.n} 样条控制位姿，{len(x0)} 变量", flush=True)
        result = least_squares(self.residual, x0, jac_sparsity=self.sparsity(),
                               bounds=self.bounds(True), loss="soft_l1", f_scale=s.robust_scale,
                               max_nfev=s.joint_max_nfev, x_scale="jac", ftol=s.optimizer_ftol,
                               xtol=s.optimizer_xtol, gtol=s.optimizer_gtol,
                               verbose=s.verbose, tr_options={"atol": 1e-6, "btol": 1e-6, "maxiter": 400})
        self.history.append({"stage": "joint_reprojection_imu", **optimizer_info(result)})
        metrics = self.metrics(result.x)
        errors = []
        if not result.success:
            errors.append("联合优化未满足终止条件（可能迭代上限不足/结点过密/数据不一致）")
        for key, limit in [("reprojection_rms_px", s.max_reprojection_rms_px),
                           ("gyro_rms_rad_s", s.max_gyro_rms_rad_s),
                           ("accel_rms_m_s2", s.max_accel_rms_m_s2)]:
            if metrics[key] > limit:
                errors.append(f"{key}={metrics[key]:.6g} 超过门槛 {limit}")
        if metrics["min_target_depth_m"] < s.min_target_depth_m:
            errors.append("标定板角点深度低于可见性下界（相机后方或过近）")
        g = result.x[:self.ng]
        if abs(g[6]) > s.max_offset_s*0.97:
            errors.append("时间偏移贴近搜索边界；请先检查时钟或扩大合理边界")
        if np.max(np.abs(g[3:6])) > s.max_translation_m*0.97:
            errors.append("平移外参贴近边界")
        if np.max(np.abs(g[7:10])) > s.max_gyro_bias_rad_s*0.97 or np.max(np.abs(g[10:13])) > s.max_accel_bias_m_s2*0.97:
            errors.append("IMU 零偏贴近边界")
        t_ci = np.eye(4)
        t_ci[:3, :3], t_ci[:3, 3] = Rotation.from_rotvec(g[:3]).as_matrix(), g[3:6]
        report = {"status": "failed" if errors else "passed_residual_and_excitation_checks",
                  "failure_reasons": errors, "settings": asdict(s),
                  "conventions": {"transform": "p_C = T_cam_imu @ p_I; T_imu_cam = inverse(T_cam_imu)",
                                  "time": "t_imu = t_cam + timeshift_cam_imu (seconds)",
                                  "acceleration": "specific force, m/s^2, not gravity-removed acceleration"},
                  "counts": {"frames": len(self.obs), "corners": len(self.uv), "imu": len(self.ti), "control_poses": self.spline.n},
                  "duration_s": float(self.tc[-1]-self.tc[0]), "excitation": self.excitation,
                  "initial_metrics": initial_metrics, "final_metrics": metrics, "stages": self.history,
                  "T_cam_imu": t_ci.tolist(), "T_imu_cam": invert_transform(t_ci).tolist(),
                  "timeshift_cam_imu": float(g[6]), "gyro_bias_rad_s": g[7:10].tolist(),
                  "accel_bias_m_s2": g[10:13].tolist(), "gravity_world_m_s2": g[13:16].tolist(),
                  "limitations": ["Residual/optimizer checks do not establish real-world accuracy.",
                                   "Single constant bias, constant time offset, ideal pinhole-radtan model; no rolling-shutter or IMU scale/misalignment calibration.",
                                   "No formal global observability or uncertainty certification; repeat on independent windows and inspect results."]}
        return report, result.x, (self.tc, self.ti)


class OpenCvDumper(yaml.SafeDumper):
    def increase_indent(self, flow=False, indentless=False):
        return super().increase_indent(flow, False)


def _opencv_sequence(dumper, values):
    return dumper.represent_sequence("tag:yaml.org,2002:seq", values,
                                     flow_style=not any(isinstance(v, (list, dict)) for v in values))


OpenCvDumper.add_representer(list, _opencv_sequence)


def opencv_yaml(entries):
    return "%YAML:1.0\n---\n"+yaml.dump(entries, Dumper=OpenCvDumper, sort_keys=False, width=200)


def save_result(output, camera, report, x=None, problem=None):
    output = Path(output)
    output.mkdir(parents=True, exist_ok=True)
    if len(camera.dist) != 4:
        reason = "skipped: Kalibr/OpenVINS radtan requires exactly 4 coefficients; k3 is NOT silently dropped"
        report["kalibr_export"] = report["openvins_export"] = reason
    (output / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False), encoding="utf-8")
    if x is not None:
        pix, gyro, accel, _ = problem.parts(x)
        np.savez_compressed(output / "residuals.npz", pixels_px=pix, gyro_rad_s=gyro, accel_m_s2=accel,
                            imu_times_s=problem.ti, camera_times_s=problem.tc)
        ctrl = x[problem.ng:].reshape(-1, 6)
        np.savez_compressed(output / "trajectory.npz", controls=ctrl, start_s=problem.spline.start,
                            spacing_s=problem.spline.h, globals=x[:problem.ng])
    if report["status"].startswith("failed"):
        return
    entry = {k: v for k, v in camera.entry.items()
             if k not in ("T_cam_imu", "T_imu_cam", "timeshift_cam_imu", "T_cn_cnm1", "cam_overlaps")}
    entry["T_cam_imu"] = report["T_cam_imu"]
    entry["timeshift_cam_imu"] = report["timeshift_cam_imu"]
    if len(camera.dist) != 4:
        (output / "camchain-opencv.yaml").write_text(yaml.safe_dump({camera.name: entry}, sort_keys=False), encoding="utf-8")
        return
    (output / "camchain-kalibr.yaml").write_text(yaml.safe_dump({camera.name: entry}, sort_keys=False), encoding="utf-8")
    ov = dict(entry)
    del ov["T_cam_imu"]
    ov["T_imu_cam"] = report["T_imu_cam"]
    ov["cam_overlaps"] = []
    (output / "camchain-openvins.yaml").write_text(opencv_yaml({camera.name: ov}), encoding="utf-8")
