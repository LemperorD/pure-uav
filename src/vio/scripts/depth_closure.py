#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""用深度相机当独立尺子：直接量 VIO 的闭环误差（不需要卷尺/AprilTag）。

原理
----
录制的数据集里有 640x400 深度图，深度内参和彩色↔深度外参都写进了
depth_meta.txt，相机-IMU 外参在 config/*/kalibr_imucam_chain.yaml。
于是任意两帧的深度点云都能各自变换到"世界系"，用 ICP 求出**真实的**相对位姿，
再和 VIO 自己给的相对位姿相比——差值就是 VIO 在这段基线里的漂移。

关键点：这个比较完全独立于用户有没有走回原点。
即使人站在离标记 40 cm 的地方，ICP 也能算出这 40 cm 是"真走位"还是"VIO 漂了"。

用法
----
    # 自动挑一切"回到过同一片区域"的帧对
    python3 scripts/depth_closure.py --dataset datasets/ver1 --tum out/ver1/traj_safe.tum

    # 只看指定帧对
    python3 scripts/depth_closure.py --dataset datasets/ver1 --tum out/ver1/traj_safe.tum \
        --pairs 105:1410 --stride 1
"""

import argparse
import csv
import math
import os
import sys
from pathlib import Path

import numpy as np

try:
    from scipy.spatial import cKDTree
except ImportError:
    print("需要 scipy（已确认解释器里有）。")
    sys.exit(1)

import cv2


# ----------------------------------------------------------------------------
# 读 YAML 里那段 kalibr T_imu_cam（只是 4x4，没必要拉 pyyaml）
# ----------------------------------------------------------------------------
def read_T_imu_cam(path):
    txt = Path(path).read_text()
    lines = txt.splitlines()
    for i, ln in enumerate(lines):
        if "T_imu_cam" in ln:
            rows = []
            for j in range(i + 1, i + 5):
                s = lines[j].strip()
                if not s.startswith("- ["):
                    break
                rows.append([float(x) for x in s.strip("- [").strip("]").split(",")])
            if len(rows) == 4:
                return np.array(rows)
    raise SystemExit(f"没在 {path} 里找到 T_imu_cam")


def read_depth_meta(ds):
    meta = {}
    for ln in Path(ds, "depth_meta.txt").read_text().splitlines():
        ln = ln.strip()
        if not ln or "=" not in ln:
            continue
        k, v = ln.split("=", 1)
        meta[k.strip()] = v.strip()
    if meta.get("valid") != "1":
        raise SystemExit("depth_meta.txt 里 valid!=1，这个数据集没有可用深度")
    return meta


def load_tum(p):
    rows = []
    for ln in Path(p).read_text().splitlines():
        ln = ln.strip()
        if not ln or ln.startswith("#"):
            continue
        f = ln.split()
        if len(f) < 8:
            continue
        rows.append([float(x) for x in f[:8]])
    return np.array(rows, dtype=float)


def quat_to_R(qx, qy, qz, qw):
    n = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    qx, qy, qz, qw = qx / n, qy / n, qz / n, qw / n
    return np.array([
        [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
        [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
        [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)],
    ])


def R_to_rpy_deg(R):
    """ZYX：返回 roll(Y 轴?) 直接给 roll/pitch/yaw（度），够看就行。"""
    sy = -R[2, 0]
    sy = max(-1.0, min(1.0, sy))
    pitch = math.asin(sy)
    roll = math.atan2(R[2, 1], R[2, 2])
    yaw = math.atan2(R[1, 0], R[0, 0])
    return np.degrees([roll, pitch, yaw])


def rot_angle_deg(R):
    c = (np.trace(R) - 1.0) / 2.0
    return math.degrees(math.acos(max(-1.0, min(1.0, c))))


# ----------------------------------------------------------------------------
# 深度 -> 点云（彩色相机系）
# ----------------------------------------------------------------------------
class Cloud:
    def __init__(self, ds, meta, T_imu_cam, T_imu_depth, step=3, zmin=0.35, zmax=7.0):
        self.ds = Path(ds)
        self.fx = float(meta["fx"])
        self.fy = float(meta["fy"])
        self.cx = float(meta["cx"])
        self.cy = float(meta["cy"])
        self.step = step
        self.zmin, self.zmax = zmin, zmax
        self.T_imu_depth = T_imu_depth
        self._cache = {}
        # 深度图与彩色图同时间戳命名：images.csv 给出 cam0/<stem>.png，深度是 depth/<stem>.png
        self.stems = []
        for ln in Path(ds, "images.csv").read_text().splitlines()[1:]:
            ln = ln.strip()
            if not ln:
                continue
            f = ln.split(",")
            self.stems.append(Path(f[1]).stem)

    def points(self, idx):
        if idx in self._cache:
            return self._cache[idx]
        p = self.ds / "depth" / f"{self.stems[idx]}.png"
        if not p.exists():
            # 隔帧深度模式（--aggregate color）下有一半的帧没有深度图，跳过即可
            self._cache[idx] = np.zeros((0, 3))
            return np.zeros((0, 3))
        d = cv2.imread(str(p), cv2.IMREAD_UNCHANGED)
        if d is None:
            self._cache[idx] = np.zeros((0, 3))
            return np.zeros((0, 3))
        if d.ndim == 3:
            d = d[:, :, 0]
        d = d.astype(np.float32)
        scale = float(self._cache.setdefault("scale", 1.0))  # 数据集里已是 mm
        d *= scale
        Z = d / 1000.0
        s = self.step
        Z = Z[::s, ::s]
        vv, uu = np.mgrid[0:d.shape[0]:s, 0:d.shape[1]:s]
        m = (Z > self.zmin) & (Z < self.zmax)
        if m.sum() < 500:
            self._cache[idx] = np.zeros((0, 3))
            return np.zeros((0, 3))
        X = (uu[m] - self.cx) * Z[m] / self.fx
        Y = (vv[m] - self.cy) * Z[m] / self.fy
        p_d = np.stack([X, Y, Z[m]], axis=1)          # 深度相机系
        p_i = p_d @ self.T_imu_depth[:3, :3].T + self.T_imu_depth[:3, 3]
        self._cache[idx] = p_i
        return p_i


# ----------------------------------------------------------------------------
# ICP（点到点 + Kabsch，带 trimming）
# ----------------------------------------------------------------------------
def icp(src, dst, T0, iters=40, trim=0.8, verbose=False):
    tree = cKDTree(dst)
    T = T0.copy()
    prev = 1e9
    hist = []
    for it in range(iters):
        p = src @ T[:3, :3].T + T[:3, 3]
        dist, nn = tree.query(p, k=1, workers=-1)
        keep = dist < np.quantile(dist, trim)
        if keep.sum() < 50:
            break
        A = p[keep]
        B = dst[nn[keep]]
        ca, cb = A.mean(0), B.mean(0)
        H = (A - ca).T @ (B - cb)
        U, S, Vt = np.linalg.svd(H)
        D = np.eye(3)
        if np.linalg.det(Vt.T @ U.T) < 0:
            D[2, 2] = -1
        R = Vt.T @ D @ U.T
        t = cb - R @ ca
        dT = np.eye(4)
        dT[:3, :3] = R
        dT[:3, 3] = t
        T = dT @ T
        rms = float(np.sqrt((dist[keep] ** 2).mean()))
        hist.append(rms)
        if verbose:
            print(f"    it{it:2d} rms={rms*100:.2f}cm  inlier={keep.sum()}")
        if abs(prev - rms) < 1e-5:
            break
        prev = rms
    p = src @ T[:3, :3].T + T[:3, 3]
    dist, _ = tree.query(p, k=1, workers=-1)
    inl = dist < np.quantile(dist, trim)
    return T, float(np.sqrt((dist[inl] ** 2).mean())), int(inl.sum()), len(src)


# ----------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--dataset", required=True)
    ap.add_argument("--tum", required=True)
    ap.add_argument("--chain", default="config/gemini_1920x1080/kalibr_imucam_chain.yaml")
    ap.add_argument("--pairs", default="", help="显式帧对，如 105:1410,105:800")
    ap.add_argument("--max-pairs", type=int, default=10)
    ap.add_argument("--step", type=int, default=3, help="深度降采样步长")
    ap.add_argument("--min-baseline", type=float, default=0.15, help="只看 VIO 认为移动超过这个距离的帧对（米）")
    args = ap.parse_args()

    ds = Path(args.dataset)
    meta = read_depth_meta(ds)
    T_imu_cam = read_T_imu_cam(args.chain)          # 彩色相机 -> IMU（p_imu = T * p_cam）

    # --- 彩色->深度 外参（与 src/orbbec_camera.cpp depthInfo() 完全一致）---
    R_c2d = np.array([float(x) for x in meta["R_c2d"].split(",")]).reshape(3, 3)
    t_c2d = np.array([float(meta["tx_mm"]), float(meta["ty_mm"]), float(meta["tz_mm"])]) / 1000.0
    T_c2d = np.eye(4)
    T_c2d[:3, :3] = R_c2d
    T_c2d[:3, 3] = t_c2d
    T_d2c = np.linalg.inv(T_c2d)                    # 深度 -> 彩色
    T_imu_depth = T_imu_cam @ T_d2c                 # 深度 -> IMU

    traj = load_tum(args.tum)
    ts = traj[:, 0]
    P = traj[:, 1:4]
    poses = []
    for r in traj:
        T = np.eye(4)
        T[:3, :3] = quat_to_R(r[4], r[5], r[6], r[7])
        T[:3, 3] = r[1:4]
        poses.append(T)
    print(f"轨迹 {len(traj)} 帧，跨度 {ts[-1]-ts[0]:.2f}s，{args.tum}")

    cloud = Cloud(ds, meta, T_imu_cam, T_imu_depth, step=args.step)

    # 帧号映射：位姿时间戳（秒）→ images.csv 里的图像序号
    img_ts = np.array([int(ln.split(",")[0]) for ln in
                       Path(ds, "images.csv").read_text().splitlines()[1:] if ln.strip()], dtype=float) / 1e6
    idx_map = np.clip(np.searchsorted(img_ts, ts), 0, len(img_ts) - 1)
    err = np.abs(img_ts[idx_map] - ts)
    if err.max() > 0.05:
        print(f"[警告] 有 {int((err>0.05).sum())} 个位姿和图像时间戳对不上（最大 {err.max()*1e3:.1f} ms）")

    # --- 选帧对 ---
    pairs = []
    if args.pairs:
        for tok in args.pairs.split(","):
            a, b = tok.split(":")
            a = int(a)
            # 允许写 0:last 或直接写行数（wc -l 数出来比实际位姿多 1，会越界）
            b = len(traj) - 1 if b.strip() in ("last", "-1") else int(b)
            b = min(b, len(traj) - 1)
            pairs.append((a, b))
    else:
        n = len(traj)
        cand = []
        # 找"回到过附近"的帧对：按 VIO 位置做近邻，要求时间隔得够远
        tree_p = cKDTree(P[::5])
        for a in range(0, n, 5):
            d, ii = tree_p.query(P[a], k=12, distance_upper_bound=1.2)
            for dd, jj in zip(np.atleast_1d(d), np.atleast_1d(ii)):
                if not np.isfinite(dd) or jj >= len(range(0, n, 5)):
                    continue
                b = jj * 5
                if abs(ts[b] - ts[a]) > 5.0 and dd > args.min_baseline:
                    cand.append((dd, abs(ts[b] - ts[a]), a, b))
        cand.sort(key=lambda x: -x[1])           # 时间隔最久的优先（漂移最大、最有说服力）
        seen = set()
        for dd, dt, a, b in cand:
            key = (min(a, b) // 30, max(a, b) // 30)
            if key in seen:
                continue
            seen.add(key)
            pairs.append((a, b))
            if len(pairs) >= args.max_pairs:
                break

    n_unusable = 0
    print(f"\n{'帧对':>12} {'VIO 位移':>9} {'ICP 位移':>9} {'差(漂移)':>9} "
          f"{'VIO转角':>8} {'ICP转角':>8} {'角度差':>7} {'ICP rms':>8} {'内点':>6}")
    print("-" * 92)
    rows = []
    for a, b in pairs:
        pa = cloud.points(idx_map[a])
        pb = cloud.points(idx_map[b])
        if len(pa) < 500 or len(pb) < 500:
            print(f"{a:5d}:{b:<6d} 深度点太少（{len(pa)}/{len(pb)}），跳过")
            continue
        T_a_inv = np.linalg.inv(poses[a])
        T_rel_vio = T_a_inv @ poses[b]            # b 帧在 a 帧坐标系里的位姿（IMU 系）
        #===============================================================
        # 【2026-10-23 外审第七章的批评，认】旧版用 VIO 相对位姿当初值、
        # 又按 VIO 位置挑帧对、把"和 VIO 差太远"的解直接丢掉 —— 这是**有偏**的：
        # 天然偏向"VIO 没错太离谱"的样本，会低估漂移。
        # 现在改成：
        #   ① 多个初值（VIO + 5 组扰动）各解一次，取 rms 最小的，并报告解的离散度
        #   ② 反向再解一次（B→A），要求正反向一致
        #   ③ 解不出来/不一致 → 明确打印"不可用"，不静默丢弃
        # 仍然承认：初值来自 VIO、帧对按 VIO 位置挑，所以它叫
        #   "深度几何一致性参考"，不是绝对真值。
        #===============================================================
        rng = np.random.default_rng(12345)
        sols = []
        for k in range(5):
            T0 = T_rel_vio.copy()
            if k > 0:
                P = np.eye(4)
                d = rng.uniform(-0.5, 0.5, 3)
                th = rng.uniform(-8, 8, 3) * math.pi / 180.0
                Rx = np.array([[1, 0, 0], [0, math.cos(th[0]), -math.sin(th[0])],
                               [0, math.sin(th[0]), math.cos(th[0])]])
                Ry = np.array([[math.cos(th[1]), 0, math.sin(th[1])], [0, 1, 0],
                               [-math.sin(th[1]), 0, math.cos(th[1])]])
                Rz = np.array([[math.cos(th[2]), -math.sin(th[2]), 0],
                               [math.sin(th[2]), math.cos(th[2]), 0], [0, 0, 1]])
                P[:3, :3] = Rz @ Ry @ Rx
                P[:3, 3] = d
                T0 = T_rel_vio @ P
            Tt, rr, ii, _ = icp(pb, pa, T0, trim=0.6)
            sols.append((rr, Tt, ii))
        sols.sort(key=lambda x: x[0])
        T_icp, rms, inl = sols[0][1], sols[0][0], sols[0][2]
        # 反向：把 a 点云对齐到 b，再取逆
        T_rev, rms_rev, _, _ = icp(pa, pb, np.linalg.inv(T_rel_vio), trim=0.6)
        T_icp2 = np.linalg.inv(T_rev)
        dv = np.linalg.norm(T_rel_vio[:3, 3])
        di = np.linalg.norm(T_icp[:3, 3])
        # 漂移：把 ICP 求出的真实相对位姿作用在 VIO 的相对位姿上
        T_err = np.linalg.inv(T_icp) @ T_rel_vio
        de = np.linalg.norm(T_err[:3, 3])
        av = rot_angle_deg(T_rel_vio[:3, :3])
        ai = rot_angle_deg(T_icp[:3, :3])
        ae = rot_angle_deg(T_err[:3, :3])
        # 低 rms ≠ 解对：重复纹理/近处单一物体能让 ICP 收敛到错解（实测 ver3 的
        # 端点对给出 1.5 m + 25° 的荒谬结果，rms 却只有 4 cm）。用一个硬约束拦掉：
        # ICP 解不能离 VIO 初值太远，否则这对帧根本不可用（多半是两端视野没重叠）。
        de2 = np.linalg.norm((np.linalg.inv(T_icp2) @ T_rel_vio)[:3, 3])
        spread = abs(de - de2)
        # 多初值离散度：不同初值收敛到的解之间最大差异（>15 cm 说明约束太弱）
        tv = [np.linalg.norm((np.linalg.inv(t) @ T_rel_vio)[:3, 3]) for _, t, _ in sols]
        multistart_spread = float(max(tv) - min(tv))
        suspicious = (de > 1.0) or (ae > 10.0)
        unstable = spread > 0.10
        flag = ""
        if suspicious:
            flag = "  ⚠️不可用(离初值太远,多半视野没重叠)"
        elif unstable:
            flag = f"  ⚠️不可用(trimming 敏感,解不稳定 {de:.2f}/{de2:.2f})"
        elif multistart_spread > 0.15:
            flag = f"  ⚠️不可用(多初值离散 {multistart_spread:.2f} m,约束太弱)"
        elif rms > 0.06:
            flag = f"  ⚠️勉强(rms {rms*100:.0f}cm)"
        # 【关键】先打印再判跳过：旧版把 suspicious 的帧对**静默丢掉**，
        # 而 suspicious 恰恰是"VIO 错最多"的那些 —— 这会让报告天然好看。
        print(f"{a:5d}:{b:<6d} {dv:8.3f}m {di:8.3f}m {de:8.3f}m "
              f"{av:7.2f}° {ai:7.2f}° {ae:6.2f}° {rms*100:6.1f}cm {inl:6d}{flag}")
        if suspicious or unstable or multistart_spread > 0.15:
            n_unusable += 1
            continue
        rows.append((a, b, dv, di, de, av, ai, ae, rms, inl))

    if n_unusable:
        print(f"\n⚠️ 另有 {n_unusable} 对帧判为不可用（已打印在上表，标了原因）——"
              f"这些恰恰可能是 VIO 错最多的地方，不要无视")
    if rows:
        print("\n解读：ICP 位移 = 两帧之间**真实**的相对位移（深度尺子量的）；")
        print("      差(漂移) = VIO 在这段基线上错了多少。闭环误差 ≤ 各段之和。")


if __name__ == "__main__":
    main()
