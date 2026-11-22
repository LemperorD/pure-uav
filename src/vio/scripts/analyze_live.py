#!/usr/bin/env python3
"""事后分析一次实时 VIO 的记录（run_vio_live.sh 的输出目录）。

用法:
    python3 scripts/analyze_live.py out/live_20261021_1030

它回答四个问题（不需要真值）：
  1. 这次走了多久、报了多少位移、最大速度多少 —— 人走路不可能超过 ~2 m/s，
     超过就说明已经发散，不用量真值。
  2. 倾角误差（滤波器认为的重力方向 vs 加速度计实测的重力方向）随时间怎么变。
     这是最灵敏的指标：g·sinθ 就是被当成"真实加速度"积分的那个量。
  3. 实测速度斜坡 dv/dt 和 g·sinθ 对不对得上。
  4. 存一张图。

需要的文件（缺哪个就少算哪项）:
    pose.csv  位姿 + 零偏（--csv-out）
    traj.tum  轨迹（--tum-out）
    imu.csv   原始 IMU（--imu-log），没有就算不了倾角误差
"""

import argparse
import os
import sys

import numpy as np


def load(outdir):
    d = {}
    p = os.path.join(outdir, "pose.csv")
    if os.path.exists(p) and os.path.getsize(p) > 0:
        d["csv"] = np.genfromtxt(p, delimiter=",", names=True)
    p = os.path.join(outdir, "traj.tum")
    if os.path.exists(p) and os.path.getsize(p) > 0:
        a = np.loadtxt(p, usecols=range(8))
        if a.ndim == 1:
            a = a[None, :]
        d["tum"] = a
    p = os.path.join(outdir, "imu.csv")
    if os.path.exists(p) and os.path.getsize(p) > 0:
        a = np.loadtxt(p, comments="#", delimiter=",")
        if a.ndim == 1:
            a = a[None, :]
        d["imu"] = a
    return d


def quat_to_R(q):
    """q = [x,y,z,w] -> R (body->world)"""
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - z * w)
    R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w)
    R[:, 2, 1] = 2 * (y * z + x * w)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return R


def tilt_series(d):
    """返回 (t, tilt_deg, is_static_mask) —— 只对准静止时刻可信。"""
    if "imu" not in d or "tum" not in d:
        return None
    im, tv = d["imu"], d["tum"]
    ti = im[:, 0] * 1e-6
    acc = im[:, 1:4]
    gyr = im[:, 4:7]
    tq, q = tv[:, 0], tv[:, 4:8]
    t0 = tq[0]
    if ti[-1] < tq[0] or ti[0] > tq[-1]:
        return None
    R = quat_to_R(q)
    idx = np.clip(np.searchsorted(tq, ti), 1, len(tq) - 1)
    ok = (ti > tq[0]) & (ti < tq[-1])
    down_est = np.einsum("nij,j->ni", R[idx], np.array([0.0, 0.0, 1.0]))
    down_meas = -acc / np.maximum(np.linalg.norm(acc, axis=1, keepdims=True), 1e-9)
    cos = np.clip(np.sum(down_est * down_meas, axis=1), -1, 1)
    deg = np.degrees(np.arccos(cos))
    gyro_deg = np.degrees(np.linalg.norm(gyr, axis=1))
    an = np.linalg.norm(acc, axis=1)
    still = ok & (gyro_deg < 1.5) & (np.abs(an - 9.81) < 0.15)
    return ti - t0, deg, still


def main():
    ap = argparse.ArgumentParser(description="分析一次实时 VIO 记录")
    ap.add_argument("outdir", help="run_vio_live.sh 的输出目录")
    ap.add_argument("--no-plot", action="store_true", help="只打印，不存图")
    ap.add_argument("--walk-speed", type=float, default=2.0,
                    help="人走路的速度上限 (m/s)，超过就判发散（默认 2.0）")
    args = ap.parse_args()

    d = load(args.outdir)
    if "csv" not in d and "tum" not in d:
        print(f"{args.outdir} 里没有 pose.csv / traj.tum，没有可分析的位姿", file=sys.stderr)
        return 1

    print(f"=== {args.outdir} ===")

    # ---------- 1. 基本量 ----------
    if "csv" in d:
        c = d["csv"]
        t = c["timestamp_us"] * 1e-6
        t = t - t[0]
        p = np.c_[c["x"], c["y"], c["z"]]
        v = np.c_[c["vx"], c["vy"], c["vz"]]
        vn = np.linalg.norm(v, axis=1)
    else:
        tv = d["tum"]
        t = tv[:, 0] - tv[0, 0]
        p = tv[:, 1:4]
        vn = np.linalg.norm(np.diff(p, axis=0), axis=1) / np.maximum(np.diff(t), 1e-3)

    pn = np.linalg.norm(p, axis=1)
    dt = np.maximum(np.diff(t), 1e-3)
    fs = np.linalg.norm(np.diff(p, axis=0), axis=1) / dt
    print(f"位姿数 {len(t)}   时长 {t[-1]:.1f} s")
    print(f"终点位移 {pn[-1]:.1f} m   最大 |p| {pn.max():.1f} m")
    print(f"速度(状态) 中位 {np.median(vn):.2f} m/s   最大 {vn.max():.1f} m/s")
    # 发散判据用【状态速度】，不要用帧间位移差分：
    # 位置会被视觉更新踢出单帧尖峰（实测单帧 8.9 cm ≈ 2.7 m/s，而当时状态速度只有 0.42 m/s），
    # 拿帧差当速度会把正常段误报成发散。
    if "csv" in d:
        over = np.where(vn > args.walk_speed)[0]
        label = "状态速度"
    else:
        over = np.where(fs > args.walk_speed)[0]
        label = "帧间位移（没有 pose.csv，只能用它，注意有单帧尖峰）"
    if len(over):
        print(f"⚠️  从 t={t[over[0]]:.1f}s 起，{label}已超过 {args.walk_speed:.1f} m/s"
              f"（人走路到不了），共 {len(over)}/{len(vn)} 帧")
    else:
        print(f"✅ 全程{label}都没超过 {args.walk_speed:.1f} m/s")

    # 速度斜坡
    m = t > 2.0
    if m.sum() > 10 and vn[m].max() > 0.3:
        slope = np.polyfit(t[m], vn[m], 1)[0]
        print(f"速度斜坡 dv/dt ≈ {slope:.3f} m/s²")

    if "csv" in d and "bax" in d["csv"].dtype.names:
        c = d["csv"]
        print(f"末态加速度零偏 ba = ({c['bax'][-1]:+.3f}, {c['bay'][-1]:+.3f}, {c['baz'][-1]:+.3f}) m/s²"
              f"   |ba| = {np.sqrt(c['bax'][-1]**2+c['bay'][-1]**2+c['baz'][-1]**2):.3f}")

    # ---------- 2. 倾角误差 ----------
    ts = tilt_series(d)
    if ts is None:
        print("\n(没有 imu.csv 或时间不重叠 —— 算不了倾角误差，加 --imu-log 再跑一次)")
        return 0
    ti, deg, still = ts
    if still.sum() < 50:
        print(f"\n准静止样本只有 {still.sum()} 个，倾角误差不可靠")
        return 0
    print(f"\n--- 倾角误差（准静止样本 {still.sum()} 个，占 {100*still.mean():.0f}%）---")
    print(f"{'时段':>10} {'中位':>8} {'g·sinθ':>10}")
    dur = ti[still].max()
    edges = np.arange(0, dur + 10, 10)
    for lo, hi in zip(edges[:-1], edges[1:]):
        mk = still & (ti >= lo) & (ti < hi)
        if mk.sum() > 20:
            th = np.median(deg[mk])
            print(f"{lo:4.0f}-{hi:3.0f}s {th:7.2f}° {9.81*np.sin(np.radians(th)):9.2f} m/s²")
    print("参考：1° → 0.17 m/s²，5° → 0.86，20° → 3.35。人走路平均加速度才 0.3~0.5 m/s²。")

    if args.no_plot:
        return 0

    # ---------- 3. 图 ----------
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib import font_manager
    except Exception as e:  # noqa: BLE001
        print(f"(画不了图: {e})")
        return 0
    for fp in ("/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",
               "/usr/share/fonts/truetype/wqy/wqy-microhei.ttc"):
        if os.path.exists(fp):
            font_manager.fontManager.addfont(fp)
            plt.rcParams["font.family"] = font_manager.FontProperties(fname=fp).get_name()
            break
    plt.rcParams["axes.unicode_minus"] = False

    tt = ti - ti[0]
    th_s = np.convolve(np.interp(tt, ti[still] - ti[0], deg[still]), np.ones(31) / 31, mode="same")
    fig, ax = plt.subplots(2, 1, figsize=(11, 6.5), sharex=True)
    ax[0].plot(t, vn if "csv" in d else np.r_[0, vn], color="#cc2222", lw=2)
    ax[0].set_yscale("log")
    ax[0].set_ylabel("速度 |v| (m/s, 对数)")
    ax[0].grid(alpha=0.3)
    ax[0].set_title(f"实时 VIO：{t[-1]:.0f} s，终点位移 {pn[-1]:.1f} m")
    ax[1].plot(tt, th_s, color="#117755", lw=2)
    ax[1].set_ylabel("倾角误差 (°)")
    ax[1].set_xlabel("时间 (s)")
    ax[1].grid(alpha=0.3)
    ax[1].set_title("滤波器重力方向 vs 加速度计实测重力方向（准静止样本）")
    png = os.path.join(args.outdir, "analysis.png")
    plt.tight_layout()
    plt.savefig(png, dpi=110)
    print(f"\n图: {png}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
