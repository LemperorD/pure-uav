#!/usr/bin/env python3
"""track_check.py —— 在录数据/起飞之前，先量一下"这幅画面跟不跟得住"。

背景（2026-10-21 实测）：
  session29 发散到 5 倍，根因不是"特征点太少"，而是 18 ms 曝光 + 高角速度带来的
  运动模糊把角点糊掉了。KLT 存活率从 84%（模糊 <2 px）掉到 38%（20~40 px），
  FAST 检出数同时从 400 掉到 25。跟踪一崩，三角化就失败（55%），MSCKF 更新率
  掉到 26%，速度就靠 IMU 空积分跑飞（+120 mm/s²）。

  所以"画面够不够亮"不是判据，"跟踪活不活得下来"才是。

判据（本工具直接量）：
  - 存活率 survival  = 第 i 帧检出的角点里，KLT 跟到 i+1 再跟到 i+2 的比例
                      （MSCKF 至少要 2~3 次观测才能三角化）
  - 每帧成熟特征 mature ≈ 检出数 × survival²，至少要 ≥80 个才有得挑

用法：
  python3 tools/track_check.py --session ~/calib/datasets/session29_vio
  python3 tools/track_check.py --session <采集目录> --gyro imu.csv   # 顺带算模糊
  python3 tools/track_check.py --camera 0 --frames 60                # 现场实测
  python3 tools/track_check.py --self-test
"""

import argparse
import sys

import numpy as np

try:
    import cv2
except ImportError:  # 自检之外都要用；缺了在 main 里给提示
    cv2 = None

FAST_THRESH = 20
FOCAL_PX = 1367.5  # 与 kalibr_imucam_chain.yaml 的 fx 一致
MAX_PTS = 300       # 与 estimator_config.yaml 的 num_pts 一致：跟踪器最多用这么多点


def klt(prev, cur, pts):
    if pts is None or len(pts) < 5:
        return np.zeros(0, bool), np.zeros((0, 2), np.float32)
    p0 = pts.reshape(-1, 1, 2).astype(np.float32)
    p1, st, _ = cv2.calcOpticalFlowPyrLK(
        prev, cur, p0, None, winSize=(15, 15), maxLevel=4,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    pb, st2, _ = cv2.calcOpticalFlowPyrLK(
        cur, prev, p1, None, winSize=(15, 15), maxLevel=4,
        criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 30, 0.01))
    fb = np.linalg.norm(p0 - pb, axis=2).ravel()
    ok = (st.ravel() == 1) & (st2.ravel() == 1) & (fb < 1.0)
    return ok, p1.reshape(-1, 2)


def analyse(frames, gyro=None, exposure_ms=None):
    """frames: list of (timestamp_s, gray). 返回逐对统计。"""
    fast = cv2.FastFeatureDetector_create(FAST_THRESH, True)
    rows = []
    for i in range(len(frames) - 2):
        t0, a = frames[i]
        t1, b = frames[i + 1]
        t2, c = frames[i + 2]
        kp = fast.detect(a, None)
        pts = np.array([k.pt for k in kp], np.float32)
        n0 = len(pts)
        if n0 < 5:
            rows.append((t0, n0, 0.0, 0.0, 0.0))
            continue
        ok1, p1 = klt(a, b, pts)
        ok2, _ = klt(b, c, p1)
        # 两段都成功的点 = 活过两个间隔（MSCKF 至少要 2~3 次观测才能三角化）
        m = min(len(ok1), len(ok2), n0)
        both = ok1[:m] & ok2[:m]
        surv2 = both.sum() / n0
        surv1 = ok1[:m].sum() / n0
        blur = np.nan
        if gyro is not None and exposure_ms is not None:
            blur = FOCAL_PX * float(np.interp(t0, gyro[0], gyro[1])) * exposure_ms * 1e-3
        rows.append((t0, n0, surv1, surv2, blur))
    return np.array(rows, dtype=float)


def verdict(rows, min_mature=80.0, min_surv=0.55):
    if len(rows) == 0:
        return "no-data", "一帧都没处理到"
    n = rows[:, 1]
    surv = rows[:, 3]
    # 跟踪器最多只用 num_pts 个点，所以"成熟特征数"要按上限截断
    # （纯噪声画面 FAST 能检出上万点，但存活率只有 1%，不能算作可用）
    mature = np.minimum(n, MAX_PTS) * surv
    med_s = float(np.median(surv))
    med_m = float(np.median(mature))
    med_n = float(np.median(n))
    used = float(np.median(np.minimum(n, MAX_PTS)))
    if med_m >= min_mature and med_s >= min_surv:
        v = "ok"
        msg = f"可跟踪：每帧约 {med_m:.0f} 个能活过两帧的特征"
    elif med_m >= min_mature * 0.5:
        v = "marginal"
        msg = f"勉强：每帧只有约 {med_m:.0f} 个成熟特征，更新率会偏低"
    else:
        v = "unusable"
        msg = f"不可用：每帧只有约 {med_m:.0f} 个成熟特征（门槛 {min_mature:.0f}）"
    return v, (f"{msg}｜检出中位 {med_n:.0f}（实际用 {used:.0f}）"
               f"｜两帧存活率中位 {med_s*100:.0f}%")


def load_session(path, max_frames):
    from pathlib import Path
    root = Path(path).expanduser()
    csv = root / "images.csv"
    if not csv.is_file():
        for cand in ("images.csv",):
            if (root / "cam0").is_dir() and (root / cand).is_file():
                break
        else:
            raise SystemExit(f"{root} 里没有 images.csv / cam0")
    lines = csv.read_text().splitlines()[1:]
    out = []
    for ln in lines[:max_frames]:
        ts, fn = ln.split(",")
        img = cv2.imread(str(root / fn), cv2.IMREAD_GRAYSCALE)
        if img is None:
            continue
        out.append((int(ts) * 1e-6, img))
    return out


def load_gyro(path, max_frames):
    a = np.genfromtxt(path, delimiter=",", skip_header=1)
    t = a[:, 0].astype(np.float64) * 1e-6
    w = np.linalg.norm(a[:, 1:4], axis=1)
    w = np.convolve(w, np.ones(5) / 5, mode="same")
    return (t, w)


def self_test():
    print("=== 自检 ===")
    rng = np.random.default_rng(7)
    ok = True
    # 造一张有大量清晰角点的图（棋盘 + 随机纹理）
    base = np.zeros((480, 640), np.uint8)
    base[::16, ::16] = 255
    base[8::16, 8::16] = 255
    for _ in range(200):
        x, y = int(rng.integers(10, 620)), int(rng.integers(10, 460))
        cv2.line(base, (x, y), (x + int(rng.integers(4, 18)), y + int(rng.integers(4, 18))),
                 160, 1)
    seq = [base.copy() for _ in range(3)]
    r = analyse([(i * 0.033, f) for i, f in enumerate(seq)])
    v, msg = verdict(r)
    print(f"  静止清晰：{v}  {msg}")
    ok &= (v == "ok")

    # 模糊掉
    blurred = [cv2.GaussianBlur(b, (41, 41), 0) for b in seq]
    r = analyse([(i * 0.033, f) for i, f in enumerate(blurred)])
    v2, msg2 = verdict(r)
    print(f"  静止但很糊：{v2}  {msg2}")
    ok &= (v2 != "ok")

    # 纯噪声（没有可跟踪结构）
    noise = [rng.integers(0, 255, (480, 640), dtype=np.uint8) for _ in range(3)]
    r = analyse([(i * 0.033, f) for i, f in enumerate(noise)])
    v3, msg3 = verdict(r)
    print(f"  纯噪声：{v3}  {msg3}")
    ok &= (v3 in ("unusable", "no-data"))

    print("自检", "通过" if ok else "失败")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description="量一下当前画面跟不跟得住（KLT 存活率）")
    ap.add_argument("--session", help="VIO 数据集或采集目录（含 images.csv + cam0）")
    ap.add_argument("--camera", type=int, help="现场用 /dev/videoN 的编号")
    ap.add_argument("--frames", type=int, default=120, help="最多处理多少帧（默认 120）")
    ap.add_argument("--start-sec", type=float, default=0.0, help="只看第 S 秒之后（分析录像用）")
    ap.add_argument("--end-sec", type=float, default=None, help="只看第 E 秒之前")
    ap.add_argument("--gyro", help="imu.csv，给了就能顺带算运动模糊")
    ap.add_argument("--exposure-ms", type=float, help="录制时的曝光时长，用于算模糊")
    ap.add_argument("--shake", action="store_true",
                    help="摇晃测试：提示你按飞行角速度转动相机，测转动中的存活率"
                         "（静止时测不出运动模糊）")
    ap.add_argument("--min-mature", type=float, default=80.0)
    ap.add_argument("--min-survival", type=float, default=0.55)
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()

    if args.self_test:
        return self_test()

    gyro = load_gyro(args.gyro, args.frames) if args.gyro else None
    if args.shake:
        import time
        dev = args.camera if args.camera is not None else 0
        cap = cv2.VideoCapture(dev, cv2.CAP_V4L2)
        if not cap.isOpened():
            print("打不开相机")
            return 2
        print("静止自检 1 秒（先丢 5 帧等驱动/AE 稳定）……")
        for _ in range(5):
            cap.read()
        still = []
        t_end = time.monotonic() + 1.0
        while time.monotonic() < t_end:
            okf, bgr = cap.read()
            if okf:
                still.append((time.monotonic(), cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)))
        print("")
        print(">>> 现在开始按飞行时的速率绕三个轴各转 2 秒（要快，别慢慢晃）<<<")
        print("")
        move = []
        t_end = time.monotonic() + 6.0
        while time.monotonic() < t_end:
            okf, bgr = cap.read()
            if okf:
                move.append((time.monotonic(), cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)))
        cap.release()

        for tag, fr in (("静止", still), ("转动", move)):
            if len(fr) < 3:
                continue
            a2 = analyse(fr)
            if len(a2) == 0:
                continue
            surv = float(np.median(a2[:, 3]))
            mature = float(np.median(np.minimum(a2[:, 1], MAX_PTS) * a2[:, 3]))
            print(f"{tag}：检出中位 {np.median(a2[:, 1]):.0f}｜"
                  f"两帧存活率 {surv * 100:.0f}%｜成熟特征 {mature:.0f}")

        if len(move) >= 3:
            a2 = analyse(move)
            if len(a2):
                surv = float(np.median(a2[:, 3]))
                if surv < 0.55:
                    print("")
                    print("❌ 转动时跟踪崩了 —— 曝光太长或增益太高。")
                    print("   按 t_exp <= blur_budget / (f * omega) 压曝光、加灯，别指望估计器能救。")
                    return 1
                if surv < 0.70:
                    print("")
                    print("⚠️ 转动时跟踪打折：更新率会偏低，尺度可能偏。")
                    return 1
                print("")
                print("✅ 转动时跟踪站得住。")
        return 0

    if args.session:
        frames = load_session(args.session, args.frames)
    elif args.camera is not None:
        cap = cv2.VideoCapture(args.camera, cv2.CAP_V4L2)
        if not cap.isOpened():
            print(f"打不开 /dev/video{args.camera}")
            return 2
        frames = []
        import time
        for _ in range(args.frames + 5):
            okf, bgr = cap.read()
            if not okf:
                break
            frames.append((time.monotonic(), cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)))
        cap.release()
        frames = frames[5:]
    else:
        ap.error("要么给 --session / --camera，要么 --self-test")

    if len(frames) < 3:
        print("帧数不足")
        return 2

    if args.start_sec > 0 or args.end_sec is not None:
        t_first = frames[0][0]
        lo = t_first + args.start_sec
        hi = t_first + (args.end_sec if args.end_sec is not None else 1e18)
        frames = [f for f in frames if lo <= f[0] <= hi]
        if len(frames) < 3:
            print("该时间窗内帧数不足")
            return 2
    rows = analyse(frames, gyro, args.exposure_ms)
    v, msg = verdict(rows, args.min_mature, args.min_survival)
    print(f"判定: {v}")
    print(f"  {msg}")
    if not np.all(np.isnan(rows[:, 4])):
        print(f"  预测运动模糊 中位 {np.nanmedian(rows[:,4]):.1f} px"
              f"  p90 {np.nanpercentile(rows[:,4],90):.1f} px")
    print()
    print(f"  {'秒':>6} {'检出':>6} {'1帧存活':>8} {'2帧存活':>8} {'成熟特征':>8} {'模糊px':>7}")
    t0 = rows[0, 0]
    step = max(1, len(rows) // 25)
    for r in rows[::step]:
        b = "-" if np.isnan(r[4]) else f"{r[4]:7.1f}"
        print(f"  {r[0]-t0:6.2f} {r[1]:6.0f} {r[2]*100:7.0f}% {r[3]*100:7.0f}%"
              f" {min(r[1], MAX_PTS)*r[3]:8.0f} {b:>7}")
    return 0 if v == "ok" else 1


if __name__ == "__main__":
    if cv2 is None:
        sys.exit("需要 opencv-python")
    sys.exit(main())
