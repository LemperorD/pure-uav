#!/usr/bin/env python3
"""跑 VIO 之前先检查这段采集能不能用。

看三件事：
  1. 运动速度（从 IMU 陀螺算）→ 估计 20 ms 曝光下的拖影像素；
  2. 图像清晰度（Laplacian 方差）和亮度；
  3. FAST 角点数（粗略判断有没有纹理）。

用法：
    python3 tools/check_frames.py ~/calib/camera_imu_calibration/data/session20
    python3 tools/check_frames.py <session> --ref ~/calib/camera_imu_calibration/data/session16

参考量级（本机 1920x1080、曝光 200=20 ms）：
  · 拖影 p50 < 5 px、p90 < 15 px 才比较舒服；
  · 陀螺 |ω| 中位 ≤ 0.3 rad/s（≈17°/s），峰值别超过 1 rad/s（≈57°/s）。
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
from pathlib import Path

import cv2
import numpy as np

FOCAL_PX = 1367.0  # 1920x1080 下的 fx，用于把角速度换算成像素拖影


def read_images(session: Path):
    rows = list(csv.DictReader((session / "images.csv").open(encoding="utf-8-sig")))
    out = []
    for r in rows:
        ts = int(r["timestamp"])
        f = Path(r["filename"])
        out.append((ts, f if f.is_absolute() else session / f))
    out.sort()
    return out


def read_imu(session: Path):
    d = np.loadtxt(session / "imu.csv", delimiter=",", skiprows=1)
    d = np.atleast_2d(d)
    ts = d[:, 0]
    # 兼容 ns / µs
    scale = 1e-9 if ts[0] > 1e17 else 1e-6
    return ts * scale, d[:, 1:7]


def exposure_s(session: Path) -> float:
    mf = session / "capture_manifest.json"
    if mf.is_file():
        try:
            m = json.loads(mf.read_text())
            exp = (m.get("locked_controls") or {}).get("exposure")
            if exp:
                return float(exp) * 100e-6   # V4L2 exposure 单位 100 µs
        except Exception:
            pass
    return 0.020


def stats(session: Path, n: int):
    imgs = read_images(session)
    t, imu = read_imu(session)
    g = np.linalg.norm(imu[:, 3:6], axis=1)
    idx = np.linspace(0, len(imgs) - 1, min(n, len(imgs))).astype(int)
    fast = cv2.FastFeatureDetector_create(threshold=20, nonmaxSuppression=True)
    sh, p90, nf = [], [], []
    for i in idx:
        gray = cv2.imread(str(imgs[i][1]), cv2.IMREAD_GRAYSCALE)
        if gray is None:
            continue
        sh.append(cv2.Laplacian(gray, cv2.CV_64F).var())
        p90.append(np.percentile(gray, 90))
        nf.append(len(fast.detect(gray, None)))
    dur = (imgs[-1][0] - imgs[0][0]) / 1e9
    exp = exposure_s(session)
    blur = g * exp * FOCAL_PX
    return {
        "name": session.name,
        "n_img": len(imgs),
        "fps": len(imgs) / dur,
        "dur": dur,
        "sharp_med": float(np.median(sh)),
        "sharp_p10": float(np.percentile(sh, 10)),
        "bright_med": float(np.median(p90)),
        "feat_med": float(np.median(nf)),
        "exposure_ms": exp * 1e3,
        "gyro_med": float(np.median(g)),
        "gyro_p90": float(np.percentile(g, 90)),
        "gyro_max": float(g.max()),
        "blur_p50": float(np.percentile(blur, 50)),
        "blur_p90": float(np.percentile(blur, 90)),
        "blur_p99": float(np.percentile(blur, 99)),
        "frac_blur10": float(np.mean(blur > 10) * 100),
        "frac_blur30": float(np.mean(blur > 30) * 100),
    }


def show(s):
    print(f"=== {s['name']} ===")
    print(f"  图像 {s['n_img']} 张 / {s['dur']:.1f} s（{s['fps']:.1f} fps），曝光 {s['exposure_ms']:.0f} ms")
    print(f"  清晰度 Laplacian: 中位 {s['sharp_med']:.0f}，p10 {s['sharp_p10']:.0f}；亮度 p90 中位 {s['bright_med']:.0f}；FAST 角点中位 {s['feat_med']:.0f}")
    print(f"  陀螺 |ω|: 中位 {s['gyro_med']:.2f} rad/s（{np.degrees(s['gyro_med']):.0f}°/s），p90 {s['gyro_p90']:.2f}，最大 {s['gyro_max']:.2f}")
    print(f"  估计拖影（{s['exposure_ms']:.0f} ms 曝光）: p50 {s['blur_p50']:.0f} px，p90 {s['blur_p90']:.0f} px，p99 {s['blur_p99']:.0f} px；>10px 的帧 {s['frac_blur10']:.0f}%，>30px 的帧 {s['frac_blur30']:.0f}%")
    bad = []
    if s["blur_p90"] > 25:
        bad.append("转动太快/曝光太长：p90 拖影 >25 px")
    if s["frac_blur30"] > 10:
        bad.append(f"{s['frac_blur30']:.0f}% 的帧拖影 >30 px")
    if s["gyro_p90"] > 1.0:
        bad.append("角速度 p90 >1 rad/s（57°/s）")
    if s["feat_med"] < 150:
        bad.append("可跟踪角点太少（中位 <150），场景纹理不足")
    print("  结论: " + ("✅ 看起来可以跑 VIO" if not bad else "⚠️  " + "；".join(bad)))
    return bad


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("session", type=Path)
    ap.add_argument("--ref", type=Path, default=None)
    ap.add_argument("--n", type=int, default=80)
    args = ap.parse_args()
    s = stats(args.session.resolve(), args.n)
    show(s)
    if args.ref:
        r = stats(args.ref.resolve(), args.n)
        print()
        show(r)
        print(f"\n相对参考: 清晰度 {s['sharp_med']/max(r['sharp_med'],1):.2f}×，拖影 p90 {s['blur_p90']/max(r['blur_p90'],1):.2f}×，角速度中位 {s['gyro_med']/max(r['gyro_med'],1):.2f}×")
    return 0


if __name__ == "__main__":
    sys.exit(main())
