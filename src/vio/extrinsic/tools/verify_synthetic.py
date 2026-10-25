#!/usr/bin/env python3
"""SYNTHETIC rendered-image+IMU test. Not a physical-camera test.
Generator: analytic Euler motion + finite differences of world IMU poses.
Estimator: independent cumulative SO(3) spline. Truth is never an input to it.
"""
import argparse
import csv
import json
import subprocess
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import yaml
from scipy.spatial.transform import Rotation
from scipy.signal import butter, sosfilt

ROOT = Path(__file__).resolve().parents[1]  # extrinsic/：唯一需要上 sys.path 的路径
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from calib.data import Camera                    # noqa: E402
from calib.spline import invert_transform        # noqa: E402

EPOCH_NS = 1700000000000000000


def camera_trajectory(t):
    t = np.atleast_1d(t)
    angles = np.column_stack([0.25*np.sin(1.7*t)+0.07*np.sin(3.1*t),
                             0.24*np.cos(1.3*t)+0.08*np.sin(2.7*t),
                             0.22*np.sin(1.1*t)+0.10*np.cos(2.3*t)])
    r = np.diag([1., -1., -1.]) @ Rotation.from_euler("xyz", angles).as_matrix()
    p = np.column_stack([0.14+0.08*np.sin(1.4*t), 0.10+0.07*np.sin(1.8*t+0.3),
                         1.00+0.07*np.cos(1.3*t)+0.03*np.sin(2.8*t)])
    return r, p


def fast_camera_trajectory(t):
    """Stress fixture with 3.7/4.3/4.9 Hz rotation, not a measured hand motion."""
    t = np.atleast_1d(t)
    angles = np.column_stack([
        0.13*np.sin(2*np.pi*1.6*t)+0.05*np.sin(2*np.pi*4.3*t+0.7),
        0.12*np.cos(2*np.pi*2.1*t)+0.04*np.sin(2*np.pi*3.7*t),
        0.11*np.sin(2*np.pi*1.3*t)+0.05*np.cos(2*np.pi*4.9*t)])
    r = np.diag([1., -1., -1.]) @ Rotation.from_euler("xyz", angles).as_matrix()
    p = np.column_stack([0.14+0.05*np.sin(2*np.pi*1.7*t),
                         0.10+0.04*np.sin(2*np.pi*2.3*t+0.3),
                         1.00+0.05*np.cos(2*np.pi*1.9*t)])
    return r, p


def imu_truth(t, t_ci, bg, ba, gravity, trajectory=camera_trajectory):
    eps = 0.0001
    def pose(tt):
        r, p = trajectory(tt)
        return r @ t_ci[:3, :3], p+np.einsum("nij,j->ni", r, t_ci[:3, 3])
    r, p = pose(t)
    rp, pp = pose(t+eps)
    rm, pm = pose(t-eps)
    r2p, p2p = pose(t+2*eps)
    r2m, p2m = pose(t-2*eps)
    dr = (-r2p+8*rp-8*rm+r2m)/(12*eps)
    ddp = (-p2p+16*pp-30*p+16*pm-p2m)/(12*eps**2)
    body = np.swapaxes(r, -1, -2) @ dr
    gyro = np.column_stack([body[:, 2, 1]-body[:, 1, 2], body[:, 0, 2]-body[:, 2, 0],
                            body[:, 1, 0]-body[:, 0, 1]])*0.5+bg
    accel = np.einsum("nji,nj->ni", r, ddp-gravity)+ba
    return np.column_stack([accel, gyro])


def render_setup(camera, cols, rows, square, kind="checkerboard"):
    texels = 100
    if kind == "checkerboard":
        texture = np.full(((rows+2)*texels, (cols+2)*texels), 235, np.uint8)
        left, top = -1.5*square, -1.5*square
        for row in range(rows+1):
            for col in range(cols+1):
                x, y = int((col+0.5)*texels), int((row+0.5)*texels)
                texture[y:y+texels, x:x+texels] = 20 if (row+col) % 2 == 0 else 235
    else:
        spacing = 0.3
        texture = np.full((int((rows*1.3+1)*texels), int((cols*1.3+1)*texels)), 255, np.uint8)
        left, top = -0.5*square, -0.5*square
        dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
        for row in range(rows):
            for col in range(cols):
                tag_id = row*cols+col
                tag = (cv2.aruco.generateImageMarker(dictionary, tag_id, texels)
                       if hasattr(cv2.aruco, "generateImageMarker")
                       else cv2.aruco.drawMarker(dictionary, tag_id, texels))
                x, y = round((col*(1+spacing)+0.5)*texels), round((row*(1+spacing)+0.5)*texels)
                # Texture y increases with world y; tag top is at larger world y.
                texture[y:y+texels, x:x+texels] = tag[::-1]
    w, h = camera.size
    pixels = np.stack(np.meshgrid(np.arange(w), np.arange(h)), -1).astype(float).reshape(-1, 1, 2)
    xy = cv2.undistortPoints(pixels, camera.k, camera.dist).reshape(-1, 2)
    return texture, left, top, texels/square, np.column_stack([xy, np.ones(len(xy))])


def render_frame(camera, setup, t, rng, trajectory=camera_trajectory):
    texture, left, top, scale, rays_c = setup
    r, p = trajectory(t)
    r, p = r[0], p[0]
    rays_w = rays_c @ r.T
    depth = -p[2]/rays_w[:, 2]
    xyz = p+rays_w*depth[:, None]
    w, h = camera.size
    mx = ((xyz[:, 0]-left)*scale).reshape(h, w).astype(np.float32)
    my = ((xyz[:, 1]-top)*scale).reshape(h, w).astype(np.float32)
    img = cv2.remap(texture, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_CONSTANT, borderValue=160)
    img = cv2.GaussianBlur(img, (3, 3), 0.55)
    return np.clip(img.astype(float)+rng.normal(0, 1.2, img.shape), 0, 255).astype(np.uint8)


def generate(folder, duration=12., seed=7, shift=0.012, kind="checkerboard",
             motion="slow", gyro_lowpass_hz=None, accel_lowpass_hz=None):
    if motion not in ("slow", "fast"):
        raise ValueError("motion must be slow or fast")
    for cutoff in (gyro_lowpass_hz, accel_lowpass_hz):
        if cutoff is not None and not 0 < cutoff < 100:
            raise ValueError("Synthetic filter cutoff must be between 0 and 100 Hz")
    trajectory = camera_trajectory if motion == "slow" else fast_camera_trajectory
    folder = Path(folder)
    folder.mkdir(parents=True, exist_ok=True)
    (folder/"synthetic_images").mkdir(exist_ok=True)
    entry = {"camera_model": "pinhole", "intrinsics": [600., 610., 480., 360.],
             "distortion_model": "radtan", "distortion_coeffs": [-0.12, 0.025, 0.001, -0.0008],
             "resolution": [960, 720], "rostopic": "/cam0/image_raw"}
    camera = Camera(entry)
    target = {"target_type": "checkerboard", "cols": 8, "rows": 6, "square_size": 0.04}
    if kind == "aprilgrid":
        target = {"target_type": "aprilgrid", "tagCols": 3, "tagRows": 3, "tagSize": 0.08,
                  "tagSpacing": 0.3, "tag_rotation_deg": 0}
    t_ci = np.eye(4)
    t_ci[:3, :3] = Rotation.from_euler("xyz", [22., -13., 31.], degrees=True).as_matrix()
    t_ci[:3, 3] = [0.065, -0.035, 0.045]
    bg, ba = np.array([0.008, -0.006, 0.004]), np.array([0.04, -0.03, 0.02])
    gravity = np.array([0., 0., -9.80665])
    rng = np.random.default_rng(seed)
    setup = render_setup(camera, 8, 6, 0.04) if kind == "checkerboard" else render_setup(camera, 3, 3, 0.08, kind)
    with (folder/"images.csv").open("w", newline="") as f:
        out = csv.writer(f)
        out.writerow(["timestamp", "filename"])
        for index, t in enumerate(np.arange(0, duration+1e-6, 1/15)):
            name = f"synthetic_images/synthetic_{index:05d}.png"
            cv2.imencode(".png", render_frame(camera, setup, t, rng, trajectory))[1].tofile(str(folder/name))
            out.writerow([EPOCH_NS+round(t*1e9), name])
    times = np.arange(-0.20, duration+0.20, 1/200)
    imu = imu_truth(times-shift, t_ci, bg, ba, gravity, trajectory)
    imu[:, :3] += rng.normal(0, 0.035, imu[:, :3].shape)
    imu[:, 3:] += rng.normal(0, 0.0015, imu[:, 3:].shape)
    # Negative-control fixture: causal DIGITAL Butterworth, zero-state startup.
    # Preserve physical timestamps to expose the unmodelled processing delay.
    for sl, cutoff in ((slice(3, 6), gyro_lowpass_hz), (slice(0, 3), accel_lowpass_hz)):
        if cutoff is not None:
            imu[:, sl] = sosfilt(butter(2, cutoff, fs=200., output="sos"), imu[:, sl], axis=0)
    with (folder/"imu.csv").open("w", newline="") as f:
        out = csv.writer(f)
        out.writerow(["timestamp", "ax", "ay", "az", "gx", "gy", "gz"])
        out.writerows([EPOCH_NS+round(t*1e9), *v] for t, v in zip(times, imu))
    truth = {"data_type": f"SYNTHETIC: rendered {kind}, simulated IMU, NOT real hardware",
             "T_cam_imu": t_ci.tolist(), "timeshift_cam_imu": shift,
             "gyro_bias_rad_s": bg.tolist(), "accel_bias_m_s2": ba.tolist(), "seed": seed, "duration_s": duration,
             "motion_profile": motion,
             "synthetic_causal_filter": {"gyro_cutoff_hz": gyro_lowpass_hz, "accel_cutoff_hz": accel_lowpass_hz,
                                         "model": "digital second-order Butterworth, fs=200Hz, zero-state, no timestamp correction"},
             "noise": {"image_intensity_sigma": 1.2, "gyro_sample_sigma_rad_s": 0.0015, "accel_sample_sigma_m_s2": 0.035}}
    (folder/"synthetic_truth.json").write_text(json.dumps(truth, indent=2))
    (folder/"camera.yaml").write_text(yaml.safe_dump({"cam0": entry}, sort_keys=False))
    (folder/"target.yaml").write_text(yaml.safe_dump(target, sort_keys=False))
    return truth


def compare(report, truth):
    estimate, target = np.array(report["T_cam_imu"]), np.array(truth["T_cam_imu"])
    angle = float(np.rad2deg(Rotation.from_matrix(estimate[:3, :3] @ target[:3, :3].T).magnitude()))
    position = float(np.linalg.norm(estimate[:3, 3]-target[:3, 3]))
    offset = float(abs(report["timeshift_cam_imu"]-truth["timeshift_cam_imu"])*1000)
    inverse_position = float(np.linalg.norm(invert_transform(estimate)[:3, 3]-invert_transform(target)[:3, 3]))
    return {"rotation_error_deg": angle, "translation_error_m": position,
            "inverse_transform_translation_error_m": inverse_position, "time_offset_error_ms": offset,
            "thresholds": {"rotation_deg": 0.5, "translation_m": 0.005, "time_ms": 2.0},
            "passed": bool(angle < 0.5 and position < 0.005 and offset < 2.0)}


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--output", type=Path, default=ROOT/"validation/synthetic_seed7")
    p.add_argument("--duration", type=float, default=12.)
    p.add_argument("--seed", type=int, default=7)
    p.add_argument("--target-type", choices=["checkerboard", "aprilgrid"], default="checkerboard")
    p.add_argument("--time-shift", type=float, default=0.012, help="synthetic truth offset, seconds")
    p.add_argument("--generate-only", action="store_true")
    p.add_argument("--reuse-data", action="store_true")
    p.add_argument("--result-name", default="calibration")
    p.add_argument("--max-nfev", type=int)
    p.add_argument("--knot-spacing", type=float)
    p.add_argument("--motion", choices=["slow", "fast"], default="slow")
    p.add_argument("--gyro-lowpass-hz", type=float, help="Synthetic negative control ONLY; not a production correction")
    p.add_argument("--accel-lowpass-hz", type=float)
    args = p.parse_args()
    truth = json.loads((args.output/"synthetic_truth.json").read_text()) if args.reuse_data else generate(args.output, args.duration, args.seed, args.time_shift, args.target_type,
                                                                                    args.motion, args.gyro_lowpass_hz, args.accel_lowpass_hz)
    if args.generate_only:
        return 0
    result_dir = args.output/args.result_name
    cmd = [sys.executable, str(ROOT/"calibrate_extrinsics.py"), "--camera", str(args.output/"camera.yaml"),
           "--target", str(args.output/"target.yaml"), "--images", str(args.output/"images.csv"),
           "--imu", str(args.output/"imu.csv"), "--output", str(result_dir), "--duration", str(args.duration),
           ]
    for flag, value in (("--max-nfev", args.max_nfev), ("--knot-spacing", args.knot_spacing)):
        if value is not None:
            cmd.extend([flag, str(value)])
    started = time.perf_counter()
    run = subprocess.run(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
    (args.output/f"{args.result_name}.log").write_text(run.stdout, encoding="utf-8")
    print(run.stdout, flush=True)
    report = json.loads((result_dir/"report.json").read_text())
    if "T_cam_imu" not in report:
        return run.returncode or 2
    comparison = {"data_type": truth["data_type"], "seed": truth["seed"],
                  "solver_status": report["status"], "solver_returncode": run.returncode,
                  "calibration_elapsed_s": time.perf_counter()-started, **compare(report, truth)}
    (args.output/f"{args.result_name}_comparison.json").write_text(json.dumps(comparison, indent=2))
    print(json.dumps(comparison, indent=2))
    return run.returncode or (0 if comparison["passed"] else 1)


if __name__ == "__main__":
    sys.exit(main())
