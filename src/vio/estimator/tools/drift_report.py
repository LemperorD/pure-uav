#!/usr/bin/env python3
"""原地绕圈 VIO 测试的漂移报告。

输入 vio_online 的 TUM（timestamp tx ty tz qx qy qz qw，LOCAL_FRD）。
典型用法（原地转 360° 后回到原朝向）：

    python3 tools/drift_report.py out/session18_traj.tum --plot out/session18_drift.png

报告：
  · 位置漂移 = |p_end - p_start|（原地转圈，真值接近 0）
  · 最大偏离 = max |p - p_start|
  · 路径长度
  · 累计转角（|Δyaw| 积分）与首尾航向差（转 360° 后应≈0）
  · 末段漂移速率（判断是不是还在发散）
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np


def load_tum(path: Path):
    rows = []
    for line in path.read_text().splitlines():
        s = line.strip()
        if not s or s.startswith("#"):
            continue
        f = s.split()
        if len(f) < 8:
            continue
        try:
            rows.append([float(x) for x in f[:8]])
        except ValueError:
            continue
    return np.array(rows)


def yaw(q: np.ndarray) -> np.ndarray:
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.degrees(np.arctan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z)))


def wrap(d):
    return (d + 180.0) % 360.0 - 180.0


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("tum", type=Path)
    ap.add_argument("--plot", type=Path, default=None)
    args = ap.parse_args()

    d = load_tum(args.tum)
    if len(d) < 2:
        raise SystemExit("轨迹点不足")
    t = d[:, 0] - d[0, 0]
    p = d[:, 1:4]
    y = yaw(d[:, 4:8])

    p0 = p[0]
    rel = p - p0
    drift = float(np.linalg.norm(rel[-1]))
    max_exc = float(np.max(np.linalg.norm(rel, axis=1)))
    path_len = float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))
    dyaw = wrap(np.diff(y))
    total_rot = float(np.sum(np.abs(dyaw)))
    final_yaw = float(wrap(y[-1] - y[0]))
    dur = float(t[-1])
    # 末段（最后 20% 时间）的位置漂移速率
    k = max(1, int(0.2 * len(t)))
    tail_rate = float(np.linalg.norm(rel[-1] - rel[-k]) / max(t[-1] - t[-k], 1e-6)) * 1000

    print("=" * 62)
    print(f"轨迹: {args.tum}")
    print(f"  样本 {len(d)}，时长 {dur:.2f} s（{len(d)/dur:.1f} Hz）")
    print(f"  位置漂移（首尾）      : {drift*100:8.2f} cm")
    print(f"  最大偏离              : {max_exc*100:8.2f} cm")
    print(f"  路径长度              : {path_len:8.2f} m")
    print(f"  累计转角 Σ|Δyaw|       : {total_rot:8.1f}°")
    print(f"  首尾航向差（wrapped） : {final_yaw:+8.2f}°")
    print(f"  末段位置漂移速率      : {tail_rate:8.2f} mm/s")
    print(f"  末位姿 p_end          : [{rel[-1,0]*100:+.2f} {rel[-1,1]*100:+.2f} {rel[-1,2]*100:+.2f}] cm")
    print("=" * 62)
    if abs(total_rot) < 30:
        print("提示：累计转角很小，这次录制不像“原地绕圈”，漂移数字参考意义有限。")

    if args.plot:
        import os
        os.environ.setdefault("MPLCONFIGDIR", str(Path(__file__).resolve().parent.parent / "out" / ".mplconfig"))
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, ax = plt.subplots(1, 2, figsize=(11, 4.6))
        ax[0].plot(rel[:, 0] * 100, rel[:, 1] * 100, lw=1.2, color="#1f77b4")
        ax[0].plot(0, 0, "o", ms=8, color="#2ca02c", label="start")
        ax[0].plot(rel[-1, 0] * 100, rel[-1, 1] * 100, "s", ms=8, color="#d62728", label="end")
        ax[0].set_xlabel("x [cm]"); ax[0].set_ylabel("y [cm]")
        ax[0].set_title(f"top view (x-y), final drift {drift*100:.2f} cm")
        ax[0].axis("equal"); ax[0].grid(alpha=.3); ax[0].legend(fontsize=8)

        ax[1].plot(t, y - y[0], color="#ff7f0e", lw=1.2)
        ax[1].set_xlabel("time [s]"); ax[1].set_ylabel("yaw - yaw0 [deg]")
        ax[1].set_title(f"heading, total |Δyaw| {total_rot:.0f}°")
        ax[1].grid(alpha=.3)
        fig.tight_layout()
        args.plot.parent.mkdir(parents=True, exist_ok=True)
        fig.savefig(args.plot, dpi=140)
        print(f"图: {args.plot}")


if __name__ == "__main__":
    main()
