#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""一键跑离线 VIO：采集目录 → VIO 数据集 → vio_offline → 漂移/尺度报告。

用法（改下面那 4 行就行）：
    python3 src/vio/estimator/scripts/run_vio_offline.py

也可以临时覆盖，不改文件：
    python3 src/vio/estimator/scripts/run_vio_offline.py --session session25 --distance 4.3

它替你处理了以前踩过的坑：
  · 系统里没有 `python` 这个命令 → 自动挑一个装了 numpy 的解释器去跑子工具
  · `vio_offline` 不存在 → 真实路径是 estimator/build/bin/vio_offline，
    找不到还会自动帮你编译
  · 输出路径写相对路径会跑飞 → 全部用绝对路径
  · 走完直线只有"漂移 428 cm"这种看不懂的数 → 直接把实测距离换算成尺度误差
"""

import argparse
import math
import os
import shutil
import subprocess
import sys
from pathlib import Path

# ┌───────────────────────── 改这里 ─────────────────────────┐
SESSION = "session24"                 # data/ 下的会话名；也可写绝对路径
TRUE_DISTANCE_M = 0.0                 # 【直线走】实测距离（米）；走闭环就留 0
PATH_LENGTH_M = 0.0                   # 【闭环】卷尺量的路径总长（米）；如 2x2 方框 = 8.0
CONFIG_DIR = "config/gemini_1920x1080"  # 唯一生产配置：60Hz（一帧不丢）/ 300 特征
AUTO_BUILD = True                     # 找不到 vio_offline 就自动编译
# └──────────────────────────────────────────────────────────┘

# 代码在仓库里，数据仍在采集工作区（~/calib）；可用环境变量 CALIB_WORKSPACE 换。
DATA_WS = Path(os.environ.get("CALIB_WORKSPACE", str(Path.home() / "calib")))
VIO_ROOT = Path(__file__).resolve().parents[1]        # src/vio/estimator
REPO_DATA = VIO_ROOT.parent / "data"                  # src/vio/data：新录制（calib/ 标定板、vio/ 场景）
DATA_ROOT = DATA_WS / "camera_imu_calibration" / "data"
DS_ROOT = DATA_WS / "datasets"
OUT_DIR = VIO_ROOT / "out"
BIN = VIO_ROOT / "build" / "bin" / "vio_offline"
TUM_TOOL = VIO_ROOT / "tools" / "session_to_dataset.py"
DRIFT_TOOL = VIO_ROOT / "tools" / "drift_report.py"


# --------------------------------------------------------------------------
# 小工具
# --------------------------------------------------------------------------

def hr(title=""):
    print("\n" + "=" * 74)
    if title:
        print(title)
        print("=" * 74)


def pick_python():
    """挑一个能 import numpy 的解释器（drift_report 依赖 numpy）。"""
    cands = [
        sys.executable,
        "/home/kxy/miniforge3/envs/vins/bin/python",
        shutil.which("python3"),
        shutil.which("python"),
    ]
    seen = set()
    for c in cands:
        if not c or c in seen:
            continue
        seen.add(c)
        if not Path(c).exists():
            continue
        try:
            r = subprocess.run([c, "-c", "import numpy"],
                               capture_output=True, timeout=90)
            if r.returncode == 0:
                return c
        except Exception:
            pass
    return None


def run(cmd, log_path=None, tail=30):
    """跑一条命令：打印命令、保存完整日志、屏幕只显示尾部。返回是否成功。"""
    print(f"\n$ {' '.join(str(x) for x in cmd)}")
    try:
        p = subprocess.run([str(x) for x in cmd],
                           capture_output=True, text=True, timeout=3600)
    except FileNotFoundError:
        print(f"  ❌ 找不到可执行文件：{cmd[0]}")
        return False, ""
    out = (p.stdout or "") + (p.stderr or "")
    if log_path:
        Path(log_path).parent.mkdir(parents=True, exist_ok=True)
        Path(log_path).write_text(out, encoding="utf-8")
    lines = [ln for ln in out.splitlines() if ln.strip()]
    for ln in lines[-tail:]:
        print("  " + ln)
    if len(lines) > tail:
        print(f"  （上面是最后 {tail} 行；完整日志：{log_path}）")
    if p.returncode != 0:
        print(f"  ❌ 退出码 {p.returncode}")
        return False, out
    return True, out


def image_health(session_dir, n=8):
    """采 8 帧看图像质量。这是"现实场景"该有的门限 —— 不是靠调参兜底。

    返回 None 表示没装 cv2，跳过检查。
    """
    try:
        import cv2
        import numpy as np
    except Exception:
        return None

    files = sorted((Path(session_dir) / "cam0").glob("*.png"))
    if len(files) < 4:
        return None
    idx = [int(i) for i in np.linspace(0, len(files) - 1, n)]
    fast = cv2.FastFeatureDetector.create(20, True)
    means, feats = [], []
    for i in idx:
        g = cv2.imread(str(files[i]), cv2.IMREAD_GRAYSCALE)
        if g is None:
            continue
        means.append(float(g.mean()))
        feats.append(len(fast.detect(g, None)))
    if not feats:
        return None
    return {
        "n": len(files),
        "mean_first": means[0], "mean_last": means[-1],
        "mean_avg": sum(means) / len(means),
        "feat_first": feats[0], "feat_last": feats[-1],
        "feat_min": min(feats), "feat_median": int(sorted(feats)[len(feats) // 2]),
    }


def report_health(h):
    """打印采集质量 + 给出"这份数据能不能信"的判断。

    【2026-10-20 改】以前只打印警告就照跑 —— session25/26/27 三份明显发废的数据
    全都完整跑了一遍 VIO 才在最后看到"发散"。现在返回结论，暗到一定程度的
    直接拒绝跑，除非 --force。判死不是看不起数据，是省时间。
    """
    if h is None:
        print("  采集质量    跳过（当前解释器没有 cv2）")
        return "unknown"
    print(f"  采集质量    亮度 mean {h['mean_avg']:.0f}"
          f"（首 {h['mean_first']:.0f} → 末 {h['mean_last']:.0f}）")
    print(f"              特征 FAST {h['feat_first']} → {h['feat_last']}"
          f"（中位 {h['feat_median']}，最少 {h['feat_min']}）")
    warns = []
    fatal = []
    if h["mean_avg"] < 38:
        fatal.append(f"整体太暗（均值 {h['mean_avg']:.0f} < 38）")
    elif h["mean_avg"] < 45:
        warns.append("整体偏暗（mean < 45）→ 抓到的特征会明显变少")
    if h["feat_median"] < 200:
        fatal.append(f"特征太少（中位 {h['feat_median']} < 200），MSCKF 根本组不成约束")
    if h["feat_median"] < 400:
        warns.append(f"特征太少（中位 {h['feat_median']} < 400）→ MSCKF 更新会掉线")
    if h["feat_first"] > 0 and h["feat_last"] < 0.5 * h["feat_first"]:
        warns.append(f"特征在衰减（{h['feat_first']} → {h['feat_last']}）→ "
                     "走到了暗处/白墙，后半程容易发散")
    if fatal:
        for w in fatal:
            print(f"              ❌ {w}")
        print("              => 这种亮度下 VIO 不是精度差，是直接发散（session25/26/27 都是）")
        print("                 采集端现在会自动测光；如果测光已经顶到曝光/增益上限，")
        print("                 就是这段录像当时的光确实不够 —— 重录，别硬跑")
        return "unusable"
    if warns:
        for w in warns:
            print(f"              ⚠️  {w}")
        print("              => 数据质量偏弱，结论不稳；有条件就重录一次更踏实")
        return "weak"
    print("              ✅ 数据质量合格")
    return "ok"


def attitude_check(session_dir, rows):
    """用纯陀螺积分（不碰相机）独立算一遍转角，跟 VIO 对比。

    这样"姿态准不准"不依赖你回程时朝向摆得正不正 —— 你自己量的朝向只能到 ±10°，
    而陀螺积分是拿 200 Hz 原始数据积出来的。
    """
    try:
        import csv as _csv
        import numpy as _np
    except Exception:
        return None

    imu_csv = Path(session_dir) / "imu.csv"
    if not imu_csv.is_file() or len(rows) < 10:
        return None
    try:
        with imu_csv.open(newline="", encoding="utf-8-sig") as fh:
            recs = list(_csv.DictReader(fh))
        if not recs:
            return None
        keys = recs[0].keys()
        # 采集侧列序是 timestamp,ax,ay,az,gx,gy,gz
        if not {"gx", "gy", "gz"} <= set(keys):
            return None
        t = _np.array([int(r["timestamp"]) for r in recs]) / 1e9
        gyr = _np.array([[float(r["gx"]), float(r["gy"]), float(r["gz"])] for r in recs])
        acc = _np.array([[float(r["ax"]), float(r["ay"]), float(r["az"])] for r in recs])
    except Exception:
        return None

    A = _np.array(rows)
    qx, qy, qz, qw = A[:, 4], A[:, 5], A[:, 6], A[:, 7]
    yaw = _np.degrees(_np.arctan2(2 * (qw * qz + qx * qy),
                                  1 - 2 * (qy * qy + qz * qz)))
    dy = _np.diff(yaw)
    dy = (dy + 180) % 360 - 180
    vio_net, vio_acc = yaw[-1] - yaw[0], _np.abs(dy).sum()

    sel = (t >= A[0, 0]) & (t <= A[-1, 0])
    ts, gs, acs = t[sel], gyr[sel], acc[sel]
    if len(ts) < 10:
        return None

    def _rod(w, dt):
        th = _np.linalg.norm(w) * dt
        if th < 1e-12:
            return _np.eye(3)
        k = w / _np.linalg.norm(w)
        K = _np.array([[0, -k[2], k[1]], [k[2], 0, -k[0]], [-k[1], k[0], 0]])
        return _np.eye(3) + _np.sin(th) * K + (1 - _np.cos(th)) * (K @ K)

    up = -acs[:min(25, len(acs))].mean(0)          # 加速度计静止测的是 -g
    up /= _np.linalg.norm(up)
    e3 = up
    e1 = _np.cross(e3, [0.0, 0.0, 1.0])
    if _np.linalg.norm(e1) < 1e-6:
        e1 = _np.cross(e3, [1.0, 0.0, 0.0])
    e1 /= _np.linalg.norm(e1)
    R = _np.array([e1, _np.cross(e3, e1), e3]).copy()
    psi = [_np.degrees(_np.arctan2(R[1, 0], R[0, 0]))]
    for k in range(len(ts) - 1):
        dt = ts[k + 1] - ts[k]
        if dt > 0:
            R = R @ _rod(gs[k], dt)
        psi.append(_np.degrees(_np.arctan2(R[1, 0], R[0, 0])))
    psi = _np.unwrap(_np.radians(psi)) * 180 / _np.pi
    psi -= psi[0]
    gy_net, gy_acc = psi[-1], _np.abs(_np.diff(psi)).sum()
    return {"vio_net": vio_net, "vio_acc": vio_acc,
            "gy_net": gy_net, "gy_acc": gy_acc}


def report_attitude(a):
    if a is None:
        print("  姿态交叉校验 跳过（缺 imu.csv 或列不是 gx/gy/gz）")
        return
    print(f"  姿态交叉校验 （纯陀螺积分 vs VIO，不依赖你回程朝向）")
    print(f"    净转角   陀螺 {a['gy_net']:+6.1f}°   VIO {a['vio_net']:+6.1f}°"
          f"   差 {a['vio_net'] - a['gy_net']:+.1f}°")
    print(f"    累计转角 陀螺 {a['gy_acc']:6.1f}°   VIO {a['vio_acc']:6.1f}°"
          f"   差 {a['vio_acc'] - a['gy_acc']:+.1f}°")
    d = abs(a["vio_acc"] - a["gy_acc"])
    if d < 5:
        print("    ✅ 姿态一致（累计差 < 5°）—— 朝向那项不用你操心")
    elif d < 15:
        print("    🟡 姿态有偏差（5–15°），走久了航向会累积误差")
    else:
        print("    🟠 姿态差 > 15°：查时间偏移、外参旋转、或者数据质量")


def load_tum(path):
    """读 TUM：timestamp tx ty tz qx qy qz qw"""
    rows = []
    for line in Path(path).read_text(encoding="utf-8").splitlines():
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
    return rows


def norm(v):
    return math.sqrt(sum(x * x for x in v))


# --------------------------------------------------------------------------
# 主流程
# --------------------------------------------------------------------------

def main():
    ap = argparse.ArgumentParser(description="一键跑离线 VIO 并出尺度报告")
    ap.add_argument("--session", default=None,
                    help=f"会话名或目录（默认 {SESSION}）")
    ap.add_argument("--distance", type=float, default=None,
                    help="直线走的实测距离（米），用于算尺度误差")
    ap.add_argument("--path-length", type=float, default=None,
                    help="闭环的路径总长（米），用于算回环误差率和尺度")
    ap.add_argument("--config-dir", default=None, help="VIO 配置目录")
    ap.add_argument("--no-build", action="store_true", help="不自动编译")
    ap.add_argument("--force", action="store_true",
                    help="画面明显太暗也照样跑（只用于排查，结果基本不可信）")
    args = ap.parse_args()

    session_arg = args.session or SESSION
    true_d = TRUE_DISTANCE_M if args.distance is None else args.distance
    loop_len = PATH_LENGTH_M if args.path_length is None else args.path_length
    cfg_rel = args.config_dir or CONFIG_DIR
    auto_build = AUTO_BUILD and not args.no_build

    # ---- 解析会话目录 ----
    cand = Path(session_arg).expanduser()
    if not cand.is_absolute() and not cand.exists():
        # 按名字找：先查仓库 data/vio/（新录制），再查旧数据工作区（历史数据，原地不动）
        for base in (REPO_DATA / "vio", DATA_ROOT):
            if (base / session_arg).is_dir():
                cand = base / session_arg
                break
        else:
            cand = DATA_ROOT / session_arg
    session_dir = cand.resolve()
    name = session_dir.name

    hr("0/3  预检")
    print(f"  代码        {VIO_ROOT}")
    print(f"  数据工作区  {DATA_WS}")
    print(f"  会话        {session_dir}")
    print(f"  会话名      {name}")

    if not session_dir.is_dir():
        print(f"\n❌ 会话目录不存在：{session_dir}")
        found = sorted(p.name for p in (REPO_DATA / "vio").glob("session*") if p.is_dir())
        found += sorted(p.name for p in DATA_ROOT.glob("session*") if p.is_dir())
        print(f"   现有会话：{found}")
        return 2
    for need in ("images.csv", "imu.csv", "cam0"):
        if not (session_dir / need).exists():
            print(f"\n❌ {session_dir} 里缺 {need} —— 采集没完成？")
            return 2

    cfg_dir = (VIO_ROOT / cfg_rel) if not Path(cfg_rel).is_absolute() else Path(cfg_rel)
    if not (cfg_dir / "estimator_config.yaml").is_file():
        print(f"\n❌ 配置目录里没有 estimator_config.yaml：{cfg_dir}")
        return 2
    print(f"  VIO 配置    {cfg_dir}")

    py = pick_python()
    if not py:
        print("\n❌ 没找到能 import numpy 的 python 解释器，")
        print("   试试显式指定：/home/kxy/miniforge3/envs/vins/bin/python "
              "src/vio/estimator/scripts/run_vio_offline.py")
        return 2
    print(f"  Python      {py}")
    if loop_len > 0:
        print(f"  闭环路径    {loop_len:.2f} m（用来判回环误差率和尺度）")
    elif true_d > 0:
        print(f"  实测距离    {true_d:.2f} m（直线走，用来判尺度）")
    else:
        print("  判据        未填（只报位移和发散自检）")

    # ---- 二进制 ----
    if not BIN.is_file():
        print(f"\n  找不到 {BIN}")
        if not auto_build:
            print("  ❌ 关掉了自动编译，请先手动编译")
            return 2
        print("  → 自动编译……")
        build_dir = VIO_ROOT / "build"
        if not (build_dir / "CMakeCache.txt").is_file():
            run(["cmake", "-S", str(VIO_ROOT), "-B", str(build_dir),
                 "-DCMAKE_BUILD_TYPE=Release"],
                log_path=OUT_DIR / "build.log", tail=15)
        ok, _ = run(["cmake", "--build", str(build_dir), "-j",
                     str(max(1, (os.cpu_count() or 2) - 1))],
                    log_path=OUT_DIR / "build.log", tail=15)
        if not BIN.is_file():
            print(f"  ❌ 编译后仍然没有 {BIN}")
            return 2
    print(f"  可执行      {BIN}")

    verdict = report_health(image_health(session_dir))
    if verdict == "unusable" and not args.force:
        print()
        print("  ❌ 这份数据的画面太暗，VIO 跑下去只会发散 —— 先不跑了。")
        print("     重录之前先跑一次：python3 tools/light_check.py")
        print("     （它会自动测光，并告诉你曝光/增益还有多少余量）")
        print("     确实要硬跑加 --force。")
        return 3

    ds_dir = DS_ROOT / f"{name}_vio"
    OUT_DIR.mkdir(parents=True, exist_ok=True)

    # ---- 1/3 转数据集 ----
    hr("1/3  采集目录 → VIO 数据集")
    ok, _ = run([py, TUM_TOOL, "--session", session_dir, "--out", ds_dir])
    if not ok:
        return 1

    # ---- 2/3 跑 VIO ----
    hr("2/3  跑离线 VIO")
    tum = OUT_DIR / f"{name}_traj.tum"
    ok, _ = run([
        BIN,
        "--dataset", ds_dir,
        "--config-dir", cfg_dir,
        "--csv-out", OUT_DIR / f"{name}_pose.csv",
        "--tum-out", tum,
        "--raw-tum-out", OUT_DIR / f"{name}_raw.tum",
    ], log_path=OUT_DIR / f"{name}_vio.log", tail=20)
    if not ok:
        return 1
    if not tum.is_file():
        print(f"\n❌ VIO 没产出轨迹文件 {tum}")
        return 1

    # ---- 3/3 报告 ----
    hr("3/3  漂移 / 尺度报告")
    run([py, "-W", "ignore", DRIFT_TOOL, tum, "--plot", OUT_DIR / f"{name}_drift.png"], tail=40)

    rows = load_tum(tum)
    if len(rows) < 2:
        print("\n❌ 轨迹点太少，没法算")
        return 1

    # 录制窗口（数据集 images.csv 是微秒）—— 用来判断"你量的距离"和"轨迹"是不是同一段
    span_img, first_img = None, None
    try:
        import csv as _csv
        with (ds_dir / "images.csv").open(newline="", encoding="utf-8-sig") as fh:
            ts_us = [int(r["timestamp_us"]) for r in _csv.DictReader(fh)]
        first_img = ts_us[0] / 1e6
        span_img = (ts_us[-1] - ts_us[0]) / 1e6
    except Exception:
        pass

    p0 = rows[0][1:4]
    pn = rows[-1][1:4]
    d = [pn[i] - p0[i] for i in range(3)]
    disp = norm(d)
    path = sum(norm([rows[i + 1][k] - rows[i][k] for k in (1, 2, 3)])
               for i in range(len(rows) - 1))
    dur = rows[-1][0] - rows[0][0]
    straight = disp / path if path > 1e-9 else 0.0

    # 每步速度，用来做发散自检：真走路时首段和末段速度应该差不多；
    # 末段比首段大很多 = 视觉更新掉线、退化成纯 IMU 积分在跑飞。
    speeds = [norm([rows[i + 1][k] - rows[i][k] for k in (1, 2, 3)])
              / max(rows[i + 1][0] - rows[i][0], 1e-9) for i in range(len(rows) - 1)]
    n = len(speeds)
    head_v = tail_v = None
    diverged = False
    if n >= 6:
        m = max(1, n // 3)
        head_v = sum(speeds[:m]) / m
        tail_v = sum(speeds[-m:]) / m
        diverged = tail_v > 1.7 * max(head_v, 1e-6)

    print()
    print("-" * 74)
    print(f"  轨迹点      {len(rows)} 个 / {dur:.2f} s（{len(rows) / max(dur, 1e-9):.1f} Hz）")
    print(f"  首尾位移    {disp * 100:.1f} cm        ← 直线走的时候这个才是'走了多远'")
    print(f"  路径长度    {path * 100:.1f} cm        ← 含来回摇摆，比位移略长才对")
    print(f"  直线度      {straight * 100:.1f}%       （位移/路径，直线应 > 90%）")
    print(f"  平均速度    {path / max(dur, 1e-9) * 100:.1f} cm/s")
    if span_img:
        lead = rows[0][0] - first_img
        print(f"  录制窗口    {span_img:.2f} s ；VIO 只覆盖 {dur:.2f} s"
              f"（{dur / span_img * 100:.0f}%）")
        if lead > 0.2:
            print(f"             开头 {lead:.2f} s 在初始化，没有轨迹")
    if head_v is not None:
        print(f"  首段/末段   {head_v:.2f} / {tail_v:.2f} m/s"
              f"（比值 {tail_v / max(head_v, 1e-6):.1f}）")
    print("-" * 74)

    if diverged:
        print()
        print("  ⚠️  末段速度远大于首段 —— 这是「视觉更新掉了、退化成纯 IMU 积分」的")
        print("     典型发散特征（人不会走着走着越走越快）。")
        print("     => 下面的位移 **和** 尺度都不可信，先修数据再测：")
        print("        · 场景太暗：capture_config.py 把 GAIN 从 10 提到 30")
        print("        · 开场别立刻走：先静止 2 秒、镜头对着有纹理的墙面，再起步")
        print("        · 走的过程中别对着白墙/天花板")
        print()

    if true_d > 0:
        cov = (dur / span_img) if span_img else 1.0
        corrected = span_img is not None and cov < 0.95
        raw_scale = disp / true_d
        if corrected:
            expect = true_d * cov          # 整段录制里，落在 VIO 窗口内的那段
            scale = disp / expect
        else:
            expect = true_d
            scale = raw_scale
        err = (scale - 1.0) * 100
        print(f"  实测距离    {true_d:.2f} m" + ("（整段录制，含初始化前的部分）" if corrected else ""))
        if corrected:
            print(f"  同窗口折算  {dur:.2f}/{span_img:.2f} s = {cov * 100:.0f}% "
                  f"-> {expect:.2f} m")
            print(f"              直接用 {true_d:.2f} m 去除会得到 {raw_scale:.3f}"
                  f"（那是拿两段不同的时间去比，错的）")
        print(f"  尺度比      {scale:.3f}   （VIO 位移 ÷ 同窗口距离，理想 = 1.000）")
        print(f"  尺度误差    {err:+.1f}%" + ("   <- 已做窗口校正" if corrected else ""))
        print()
        if diverged:
            print("  ⚠️  这一次轨迹已经在发散，数字没有参考价值 —— 别拿它下结论。")
            print("     先把上面的数据问题（亮度/增益/开场静止）修掉，重录一遍再比。")
        elif abs(err) < 3:
            print("  ✅ 尺度很好（<3%）。VIO 的尺度没问题，别再折腾硬件了。")
            print("     剩下能优化的只有长时间漂移和姿态，跟 RGB/IR、跟双目都无关。")
        elif abs(err) < 8:
            print("  🟢 尺度在单目 VIO 的正常范围内（3–8%）。")
            print("     单目+IMU 的尺度本来就靠运动激励，这个量级不用改任何东西；")
            print("     想再准一点：走长一点（>8 m）、或者来回走一个闭环。")
        elif abs(err) < 15:
            print("  🟠 尺度偏大（8–15%）。按顺序查：")
            print("     1) 这条线真的直吗？旁边有椅子绕了就重新走一遍")
            print("     2) 起步别从静止慢慢加速，先正常走两步")
            print("     3) 相机拆装过就得重标外参")
        else:
            print("  🔴 尺度崩了（>15%）。这不是 RGB/IR 的问题，是估计器或数据的问题：")
            print("     - 看上面 VIO 日志里有没有 'no update' / ba 乱跳")
            print("     - 图像是不是太暗、特征太少（FAST 数 < 300 就该加光/加增益）")
    elif loop_len > 0:
        # ---------- 闭环：一次同时验尺度、漂移、姿态 ----------
        loop_err = disp                       # 回到原点，首尾位移就是回环误差
        err_rate = loop_err / max(path, 1e-9) * 100
        scale = path / loop_len               # 路径长度比 = 尺度
        print(f"  回环误差    {loop_err * 100:.1f} cm"
              f"   （首尾位移；回到原点时真值 = 0）")
        print(f"  回环误差率  {err_rate:.2f}%   = 回环误差 ÷ VIO 路径长度 ← 核心指标")
        print(f"  路径长度比  {scale:.3f}   VIO {path:.2f} m ÷ 实测 {loop_len:.2f} m"
              f"（尺度，理想 1.000）")
        if head_v is not None:
            print(f"  速度首末比  {tail_v / max(head_v, 1e-6):.2f}   （<1.7 才算不发散）")
        print()
        if diverged:
            print("  ⚠️  轨迹在发散，回环数字没有意义，先修数据。")
        elif err_rate < 5 and abs(scale - 1) < 0.05:
            print("  ✅ 闭环通过：回环误差 < 5%、尺度 < 5%。")
            print("     这说明尺度、漂移、姿态三样同时对 —— 这才是能拿去用的证据。")
        elif err_rate < 10:
            print(f"  🟡 回环误差 {err_rate:.1f}%：能跑，但走远了会累积。")
            print("     走更大的圈（路径 > 15 m）再测一次，看误差是线性涨还是发散。")
        else:
            print(f"  🟠 回环误差 {err_rate:.1f}% 偏大。按顺序查：")
            print("     1) 走的时候有没有绕、有没有原地转圈（姿态误差会放大成回环误差）")
            print("     2) 数据质量（看上面的采集质量门限）")
            print("     3) 走完是不是真的回到了同一个点、朝向也对齐了")
    else:
        print("  想知道准不准：把 TRUE_DISTANCE_M 填成卷尺量出来的米数，再跑一次。")

    report_attitude(attitude_check(session_dir, rows))

    print()
    print(f"  轨迹 {tum}")
    print(f"  图   {OUT_DIR / f'{name}_drift.png'}")
    print(f"  日志 {OUT_DIR / f'{name}_vio.log'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
