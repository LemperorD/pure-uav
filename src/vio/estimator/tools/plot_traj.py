#!/usr/bin/env python3
"""画轨迹图（TUM 或 VisionPose CSV），用于快速检查 VIO 结果。

用法:
  python plot_traj.py --tum ours.tum [--tum ref.tum] --out out.png
  python plot_traj.py --pose-csv vio_pose.csv --out out.png
"""

import argparse
import sys

import numpy as np


def load_tum(path):
    data = []
    with open(path) as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            c = s.split()
            if len(c) >= 8:
                data.append([float(x) for x in c[:8]])
    return np.asarray(data)


def load_pose_csv(path):
    rows = np.loadtxt(path, delimiter=",", skiprows=1)
    # timestamp_us,frame,child,x,y,z,qw,qx,qy,qz,vx,vy,vz
    return rows


def plot_xy(ax, p, label, style="-"):
    ax.plot(p[:, 0], p[:, 1], style, label=label, linewidth=1.2)
    ax.scatter([p[0, 0]], [p[0, 1]], marker="o", s=20)
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    ax.axis("equal")
    ax.grid(True, alpha=0.3)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--tum", default="", help="TUM 轨迹（可重复）")
    ap.add_argument("--tum-ref", default="", help="参考 TUM")
    ap.add_argument("--pose-csv", default="", help="VisionPose CSV")
    ap.add_argument("--out", required=True)
    args = ap.parse_args()

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(11, 8))
    ax_xy, ax_z, ax_v, ax_xy_pose = axes[0, 0], axes[0, 1], axes[1, 0], axes[1, 1]

    if args.tum:
        a = load_tum(args.tum)
        plot_xy(ax_xy, a[:, 1:4], "ours (global)")
        ax_z.plot(a[:, 0] - a[0, 0], a[:, 3], label="ours z")
        ax_z.set_xlabel("t [s]")
        ax_z.set_ylabel("z [m]")
        ax_z.grid(True, alpha=0.3)

    if args.tum_ref:
        b = load_tum(args.tum_ref)
        plot_xy(ax_xy, b[:, 1:4], "reference (global)", "--")
        ax_z.plot(b[:, 0] - b[0, 0], b[:, 3], "--", label="reference z")
        ax_xy.legend()
        ax_z.legend()

    if args.pose_csv:
        rows = load_pose_csv(args.pose_csv)
        t = (rows[:, 0] - rows[0, 0]) * 1e-6
        pose_xy = rows[:, 3:5]
        plot_xy(ax_xy_pose, rows[:, 3:6], "VisionPose (LOCAL_FRD)")
        ax_xy_pose.set_title("VisionPose: LOCAL_FRD x-y")
        ax_v.plot(t, rows[:, 10], label="vx (body)")
        ax_v.plot(t, rows[:, 11], label="vy (body)")
        ax_v.plot(t, rows[:, 12], label="vz (body)")
        ax_v.set_xlabel("t [s]")
        ax_v.set_ylabel("v [m/s]")
        ax_v.grid(True, alpha=0.3)
        ax_v.legend()
        ax_z.plot(t, rows[:, 5], label="VisionPose z (down)")
        ax_z.legend()

    ax_xy.set_title("OpenVINS global x-y")
    ax_z.set_title("z(t)")
    fig.tight_layout()
    fig.savefig(args.out, dpi=140)
    print("saved", args.out)
    return 0


if __name__ == "__main__":
    sys.exit(main())
