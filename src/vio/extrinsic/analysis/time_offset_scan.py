#!/usr/bin/env python3
"""Independent camera-IMU time-offset estimate.

The magnitude of angular velocity is invariant to the (unknown) camera-IMU rotation,
so |omega| from PnP frame-to-frame rotations can be aligned to |omega| from the gyro
without knowing the extrinsic.  This gives an independent check on the timeshift
reported by Kalibr and by the in-house solver, and lets us test the two candidate
camera timestamp sources (host receive time vs. V4L2 buffer timestamp).
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))

from calib.data import Camera, Target, make_observation, read_yaml  # noqa: E402


def load_session(session: str):
    d = ROOT / "data" / session
    manifest = json.loads((d / "capture_manifest.json").read_text())
    saved = [s for s in (manifest.get("saved") or []) if s.get("v4l2_buffer_timestamp_ns")]
    by_name = {Path(s["file"]).name: s for s in saved}
    rows = []
    import csv
    with (d / "images.csv").open(newline="") as fh:
        for row in csv.DictReader(fh):
            name = Path(row["filename"]).name
            rows.append((int(row["timestamp"]), d / row["filename"], by_name.get(name)))
    rows.sort(key=lambda r: r[0])
    imu = np.loadtxt(d / "imu.csv", delimiter=",", skiprows=1)
    return rows, imu, manifest


def pnp_poses(rows, camera, target):
    t_host, t_v4l2, omegas = [], [], []
    prev = None
    for stamp, path, meta in rows:
        gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            continue
        det = target.detect(gray)
        obs = None
        if det is not None:
            obs = make_observation(stamp, *det, camera, prev, checker=False, max_points=0)
        if obs is not None and obs.pnp_rms <= 3.0:
            prev = obs
            t_host.append(stamp * 1e-9)
            if meta is not None:
                # map driver MONOTONIC timestamp to host REALTIME using the same frame
                off = meta["host_receive_time_ns"] - meta["host_receive_monotonic_ns"]
                t_v4l2.append((meta["v4l2_buffer_timestamp_ns"] + off) * 1e-9)
            else:
                t_v4l2.append(np.nan)
            omegas.append((obs.r_wc, obs.pnp_rms))
        else:
            t_host.append(np.nan)
            t_v4l2.append(np.nan)
            omegas.append(None)
    return np.array(t_host), np.array(t_v4l2), omegas


def frame_rates(t, poses):
    """theta/dt between consecutive valid frames, timestamped at the interval midpoint."""
    mid, rate, rms = [], [], []
    for k in range(len(t) - 1):
        if poses[k] is None or poses[k + 1] is None:
            continue
        if not (np.isfinite(t[k]) and np.isfinite(t[k + 1])):
            continue
        dt = t[k + 1] - t[k]
        if not (0.02 < dt < 0.06):
            # still allow a couple of dropped frames
            if not (0.06 <= dt < 0.12):
                continue
        R1 = poses[k][0]
        R2 = poses[k + 1][0]
        dR = R1.T @ R2
        theta = np.degrees(np.arccos(np.clip((np.trace(dR) - 1) / 2, -1, 1))) * np.pi / 180
        rate.append(theta / dt)
        mid.append(0.5 * (t[k] + t[k + 1]))
        rms.append(max(poses[k][1], poses[k + 1][1]))
    return np.array(mid), np.array(rate), np.array(rms)


def gyro_mag_at(t, tq, gyro):
    """linear-interpolated |omega| at query times"""
    n = tq.shape[0]
    idx = np.searchsorted(t, tq)
    idx = np.clip(idx, 1, len(t) - 1)
    t0, t1 = t[idx - 1], t[idx]
    w = (tq - t0) / (t1 - t0)
    g = np.linalg.norm(gyro, axis=1)
    return (1 - w) * g[idx - 1] + w * g[idx]


def scan(mid, rate, t_imu, gyro, dmin=-0.12, dmax=0.12, step=0.0002, rms=None):
    deltas = np.arange(dmin, dmax + step / 2, step)
    # robust trimmed cost; only use intervals with decent camera rate (good SNR)
    sel = rate > 0.25
    if rms is not None:
        sel &= rms < 2.0
    m, r = mid[sel], rate[sel]
    imu_at = m[None, :] + deltas[:, None]
    cost = np.empty(len(deltas))
    for i in range(len(deltas)):
        gm = gyro_mag_at(t_imu, imu_at[i], gyro)
        cost[i] = np.sqrt(np.mean((gm - r) ** 2))
    d = deltas[np.argmin(cost)]
    return d, cost, deltas, sel.sum()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("sessions", nargs="+")
    ap.add_argument("--max-frames", type=int, default=0)
    args = ap.parse_args()

    cam = Camera.load(ROOT / "configs" / "gemini_pro_rgb.yaml")
    tgt = Target(read_yaml(ROOT / "configs" / "aprilgrid.yaml"))

    print(f"camera fx={cam.params[0]:.2f} cx={cam.params[2]:.2f}")
    print(f"target {tgt.kind} {tgt.rows}x{tgt.cols} size={tgt.size} spacing={tgt.spacing} roll={tgt.roll}")
    print()
    print(f"{'session':<10}{'det valid':>10}{'pairs':>7}{'used':>6}"
          f"{'delta_host[ms]':>16}{'delta_v4l2[ms]':>16}{'diff[ms]':>10}")
    results = {}
    for s in args.sessions:
        rows, imu, manifest = load_session(s)
        if args.max_frames:
            rows = rows[:args.max_frames]
        th, tv, poses = pnp_poses(rows, cam, tgt)
        n_valid = sum(p is not None for p in poses)
        mid_h, rate_h, rms_h = frame_rates(th, poses)
        d_h, cost_h, grid, used = scan(mid_h, rate_h, imu[:, 0] * 1e-9, imu[:, 4:7], rms=rms_h)
        # v4l2 variant
        good = np.isfinite(tv)
        # rebuild rate with v4l2 timestamps
        t_use = tv.copy()
        mid_v, rate_v, rms_v = frame_rates(t_use, poses)
        d_v, cost_v, _, used_v = scan(mid_v, rate_v, imu[:, 0] * 1e-9, imu[:, 4:7], rms=rms_v)
        print(f"{s:<10}{n_valid:>10}{len(mid_h):>7}{used:>6}{1000*d_h:>16.2f}{1000*d_v:>16.2f}"
              f"{1000*(d_v-d_h):>10.2f}")
        results[s] = {"delta_host_ms": 1000 * d_h, "delta_v4l2_ms": 1000 * d_v,
                      "n_pairs": int(len(mid_h)), "n_used": int(used),
                      "cost_at_min_host": float(cost_h.min()), "cost_at_min_v4l2": float(cost_v.min())}
    print()
    print("Interpretation: delta is the shift to ADD to the recorded camera timestamp so that")
    print("the gyro timeline matches the image timeline, i.e. the pipeline's timeshift_cam_imu.")
    outp = ROOT / "analysis" / "time_offset_scan.json"
    outp.write_text(json.dumps(results, indent=2))
    print(f"saved {outp}")


if __name__ == "__main__":
    main()
