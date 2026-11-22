#!/usr/bin/env python3
"""比较两条 TUM 轨迹（本模块输出 vs OpenVINS 参考输出）。

只做时间最近邻匹配，不重新做对齐；因为两边是同一套 OpenVINS 核心、
同一份配置，理想情况下应该几乎逐帧重合（差异只来自图像编码/线程）。

用法：
    python compare_traj.py a.tum b.tum [--tol 0.005]
"""

import argparse
import sys

import numpy as np


def load_tum(path):
    data = []
    with open(path, "r") as f:
        for line in f:
            s = line.strip()
            if not s or s.startswith("#"):
                continue
            cols = s.split()
            if len(cols) < 8:
                continue
            data.append([float(x) for x in cols[:8]])
    return np.asarray(data, dtype=np.float64)


def quat_angle_deg(qa, qb):
    """两个 (x,y,z,w) 四元数之间的角度（度）。"""
    dot = abs(float(np.dot(qa, qb)))
    dot = min(1.0, max(-1.0, dot))
    return np.degrees(2.0 * np.arccos(dot))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("a", help="轨迹 A（我们的输出）")
    ap.add_argument("b", help="轨迹 B（参考）")
    ap.add_argument("--tol", type=float, default=0.005, help="时间匹配容差 (s)")
    args = ap.parse_args()

    a = load_tum(args.a)
    b = load_tum(args.b)
    if len(a) == 0 or len(b) == 0:
        print("空轨迹")
        return 1

    # 时间最近邻（两边都按时间排好序）
    idx = np.searchsorted(b[:, 0], a[:, 0])
    idx = np.clip(idx, 0, len(b) - 1)
    dt = np.abs(b[idx, 0] - a[:, 0])
    # 检查前后哪个更近
    left = np.clip(idx - 1, 0, len(b) - 1)
    choose_left = np.abs(b[left, 0] - a[:, 0]) < dt
    idx[choose_left] = left[choose_left]
    dt = np.abs(b[idx, 0] - a[:, 0])

    mask = dt <= args.tol
    n = int(mask.sum())
    if n == 0:
        print(f"没有时间差 < {args.tol}s 的匹配帧（A {len(a)} 帧, B {len(b)} 帧）")
        return 1

    ta = a[mask, 0]
    pa = a[mask, 1:4]
    pb = b[idx[mask], 1:4]
    qa = a[mask, 4:8]
    qb = b[idx[mask], 4:8]

    dp = np.linalg.norm(pa - pb, axis=1)
    ang = np.array([quat_angle_deg(qa[i], qb[i]) for i in range(n)])

    def report(name, v):
        print(f"  {name:<14}: mean={v.mean():.4f}  median={np.median(v):.4f}  "
              f"p95={np.percentile(v, 95):.4f}  max={v.max():.4f}")

    print(f"匹配帧: {n}/{len(a)} (tol={args.tol}s), 时间范围 "
          f"[{ta[0]:.3f}, {ta[-1]:.3f}]")
    report("位置差 (m)", dp)
    report("姿态差 (deg)", ang)
    print(f"  时间差 (ms)  : mean={dt[mask].mean()*1e3:.2f}  max={dt[mask].max()*1e3:.2f}")

    # 起终点位移（判断尺度/漂移的粗指标）
    print(f"  A 起点/终点 : {pa[0]} -> {pa[-1]}")
    print(f"  B 起点/终点 : {pb[0]} -> {pb[-1]}")
    print(f"  A 轨迹长度  : {np.sum(np.linalg.norm(np.diff(pa, axis=0), axis=1)):.3f} m")
    print(f"  B 轨迹长度  : {np.sum(np.linalg.norm(np.diff(pb, axis=0), axis=1)):.3f} m")
    return 0


if __name__ == "__main__":
    sys.exit(main())
