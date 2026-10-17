#!/usr/bin/env python3
"""Map IMU sensor clock into image clock using measured synchronization anchors.
Does NOT infer clock alignment by making first samples coincide.
Anchors CSV: imu_timestamp,camera_timestamp (both explicitly typed by CLI units).
Input IMU CSV: timestamp,ax,ay,az,gx,gy,gz; units m/s² and rad/s.
"""
import argparse
import csv
import json
import sys
from pathlib import Path

import numpy as np

from calib.data import CalibrationError, read_imu_csv, stamp_ns


def fit_clock(imu_ns, camera_ns, max_rms_ms=2.):
    # Subtract integer epochs before floating point fitting.
    ti, tc = np.asarray(imu_ns, np.int64), np.asarray(camera_ns, np.int64)
    if len(ti) < 5 or len(ti) != len(tc):
        raise CalibrationError("时钟拟合至少需要5个测得的同步锚点。")
    if np.any(np.diff(ti) <= 0) or np.any(np.diff(tc) <= 0):
        raise CalibrationError("同步锚点时间戳必须严格递增。")
    x, y = (ti-ti[0])*1e-9, (tc-tc[0])*1e-9
    if x[-1] < 5:
        raise CalibrationError("同步锚点需覆盖至少5秒，才能拟合时钟漂移。")
    design = np.column_stack([x, np.ones(len(x))])
    scale, intercept = np.linalg.lstsq(design, y, rcond=None)[0]
    residual_ms = (design @ [scale, intercept]-y)*1000
    rms = float(np.sqrt(np.mean(residual_ms**2)))
    if rms > max_rms_ms or abs(scale-1) > 0.005:
        raise CalibrationError(f"时钟拟合不可靠：RMS={rms:.3f}ms，scale={scale:.9f}。")
    return float(scale), float(intercept), int(ti[0]), int(tc[0]), residual_ms


def main():
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--input", type=Path, required=True)
    p.add_argument("--anchors", type=Path, required=True)
    p.add_argument("--imu-unit", choices=["ns", "us", "ms", "s"], required=True)
    p.add_argument("--camera-unit", choices=["ns", "us", "ms", "s"], required=True)
    p.add_argument("--max-rms-ms", type=float, default=2.)
    p.add_argument("--output", type=Path, required=True)
    args = p.parse_args()
    if args.output.exists():
        p.error("输出文件已存在，请使用新文件。")
    try:
        ti, tc = [], []
        with args.anchors.open(encoding="utf-8-sig", newline="") as f:
            for row in csv.DictReader(f):
                ti.append(stamp_ns(row["imu_timestamp"], args.imu_unit))
                tc.append(stamp_ns(row["camera_timestamp"], args.camera_unit))
        scale, offset, t0i, t0c, residual = fit_clock(ti, tc, args.max_rms_ms)
        stamps, data = read_imu_csv(args.input, args.imu_unit)
        if stamps[0] < ti[0] or stamps[-1] > ti[-1]:
            raise CalibrationError("同步锚点必须覆盖全部 IMU 时间范围；禁止未经验证地外推时钟。")
        out_stamps = np.rint(scale*(stamps-t0i)+offset*1e9).astype(np.int64)+t0c
        args.output.parent.mkdir(parents=True, exist_ok=True)
        with args.output.open("w", newline="") as f:
            writer = csv.writer(f)
            writer.writerow(["timestamp", "ax", "ay", "az", "gx", "gy", "gz"])
            writer.writerows([int(t), *v] for t, v in zip(out_stamps, data))
        args.output.with_suffix(".clock.json").write_text(json.dumps({
            "scale": scale, "drift_ppm": (scale-1)*1e6, "origin_imu_ns": t0i,
            "origin_camera_ns": t0c, "intercept_relative_s": offset,
            "residuals_ms": residual.tolist(), "rms_ms": float(np.sqrt(np.mean(residual**2))),
            "equation": "camera_clock_ns = origin_camera_ns + scale*(imu_ns-origin_imu_ns) + intercept_relative_s*1e9",
            "note": "Measured anchors are required. Receive timestamps with variable network delay are not exact sample synchronization."}, indent=2))
        return 0
    except (CalibrationError, ValueError, KeyError, OSError) as exc:
        print(f"时钟转换失败：{exc}", file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())
