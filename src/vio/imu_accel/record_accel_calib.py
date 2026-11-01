#!/usr/bin/env python3
"""record_accel_calib.py —— 带节奏提示的六面加速度计标定采集。

属于「⑤ 加速度计尺度/不正交」模块（vio/imu_accel/）。
依赖 ② 的 imu_noise/record_static_imu.py 里的 Recorder。

为什么要有这个脚本
------------------
`imu/src/record_static_imu.py` 是给 Allan 方差采长时静置数据用的，它**不会
提示你什么时候翻面**——你只能盯着秒表自己数，而且它每 10 秒打印一次进度，
根本没法用来做 6 个面的分段采集。

这个脚本做的事：把 6 个面排成时间表，逐秒倒数，到点大喊换面，
录完自动调 `tools/accel_calib.py` 出结果。全程只需要跟着屏幕做动作。

用法
----
    python3 imu_accel/record_accel_calib.py

    # 只想看流程不录音（10 秒跑完全程，用来确认节奏）
    python3 imu_accel/record_accel_calib.py --dry-run

    # 录完不自动标定
    python3 imu_accel/record_accel_calib.py --no-calibrate

    # 每个面保持更久（默认 15 秒，工具要求 ≥4 秒）
    python3 imu_accel/record_accel_calib.py --hold 20

摆放顺序固定为：肚子朝下 → 背朝下 → 机头朝下 → 机尾朝下 → 右侧朝下 → 左侧朝下
（机体系 FRD：X 前、Y 右、Z 下）。要换顺序改下面的 SCHEDULE。
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import threading
import time
from datetime import datetime
from pathlib import Path

HERE = Path(__file__).resolve().parent          # vio/imu_accel
VIO_ROOT = HERE.parent                          # vio/


def _find_imu_src() -> Path:
    """兼容两种布局：本项目 vio/imu_noise/ 与开发时的 calib/imu/src/。"""
    for c in (VIO_ROOT / "imu_noise", VIO_ROOT / "imu" / "src"):
        if (c / "record_static_imu.py").is_file():
            return c
    raise SystemExit(f"找不到 record_static_imu.py（试过 {VIO_ROOT}/imu_noise 和 "
                     f"{VIO_ROOT}/imu/src）")


IMU_SRC = _find_imu_src()
DATA_DIR = IMU_SRC / "data"
CALIB_TOOL = HERE / "accel_calib.py"

# (代码名, 中文提示, 重力在机体系的方向)
SCHEDULE = [
    ("belly", "肚子朝下（正常正放）", "+Z"),
    ("back", "背朝下（完全翻过来）", "−Z"),
    ("nose", "机头朝下（垫书，别压镜头）", "+X"),
    ("tail", "机尾朝下（别压天线）", "−X"),
    ("right", "右侧朝下（横躺）", "+Y"),
    ("left", "左侧朝下（横躺反向）", "−Y"),
]

BAR_W = 22


def _bar(frac: float) -> str:
    n = int(round(max(0.0, min(1.0, frac)) * BAR_W))
    return "█" * n + "░" * (BAR_W - n)


def _clear_line():
    sys.stdout.write("\r" + " " * 100 + "\r")
    sys.stdout.flush()


def run_schedule(hold_s: float, trans_s: float, prep_s: float):
    """只跑提示节奏，不录音。返回实际耗时。"""
    t0 = time.monotonic()

    # 准备
    print(f"\n  准备：{prep_s:.0f} 秒后开始。请先把飞机摆成【{SCHEDULE[0][1]}】。")
    for k in range(int(prep_s), 0, -1):
        sys.stdout.write(f"\r  倒计时 {k:2d} 秒 … 现在摆好：{SCHEDULE[0][1]}   ")
        sys.stdout.flush()
        time.sleep(1.0)
    _clear_line()

    for i, (name, desc, axis) in enumerate(SCHEDULE):
        # 保持段
        print(f"\n━━━ {i + 1}/{len(SCHEDULE)}  【{desc}】  重力方向 {axis} ━━━")
        print("    保持不动，别碰桌子。")
        n = max(1, int(round(hold_s)))
        for k in range(n, 0, -1):
            frac = 1.0 - k / n
            sys.stdout.write(f"\r    {_bar(frac)}  剩 {k:2d} 秒   ")
            sys.stdout.flush()
            time.sleep(hold_s / n)
        _clear_line()
        print("    ✅ 这一段完成")

        # 换面段
        if i + 1 < len(SCHEDULE):
            nxt = SCHEDULE[i + 1]
            print(f"\n  ⟳  换面！拿起飞机，改成【{nxt[1]}】")
            n = max(1, int(round(trans_s)))
            for k in range(n, 0, -1):
                sys.stdout.write(f"\r    换面 {k:2d} 秒 … 下一个：{nxt[1]}   ")
                sys.stdout.flush()
                time.sleep(trans_s / n)
            _clear_line()

    print(f"\n  全部完成，用时 {time.monotonic() - t0:.0f} 秒。")
    return time.monotonic() - t0


def main() -> int:
    ap = argparse.ArgumentParser(description="带节奏提示的六面加速度计标定采集")
    ap.add_argument("--hold", type=float, default=15.0, help="每个面保持几秒（默认 15）")
    ap.add_argument("--trans", type=float, default=6.0, help="换面给几秒（默认 6）")
    ap.add_argument("--prep", type=float, default=5.0, help="开始前倒计时（默认 5）")
    ap.add_argument("--dry-run", action="store_true", help="不录音，只走一遍节奏")
    ap.add_argument("--no-calibrate", action="store_true", help="录完不自动标定")
    ap.add_argument("--port", default="auto", help="串口，默认自动找")
    args = ap.parse_args()

    if args.dry_run:
        print("=== dry-run：只验证节奏提示，不录音 ===")
        run_schedule(hold_s=2, trans_s=1, prep_s=2)
        print("\n(dry-run 结束。真跑请去掉 --dry-run)")
        return 0

    sys.path.insert(0, str(IMU_SRC))
    try:
        import record_static_imu as rsm
        from serial_port import find_mavlink_port
    except ImportError as exc:
        print(f"❌ 找不到 imu/src 里的模块：{exc}")
        return 2

    port = args.port
    if port == "auto":
        port = find_mavlink_port()
        if not port:
            return 3

    DATA_DIR.mkdir(parents=True, exist_ok=True)
    out_csv = DATA_DIR / f"accelcal_{datetime.now():%Y%m%d_%H%M%S}.csv"

    total_s = args.prep + len(SCHEDULE) * args.hold + (len(SCHEDULE) - 1) * args.trans
    print("=" * 74)
    print("  六面加速度计标定采集")
    print("=" * 74)
    print(f"  串口      {port}")
    print(f"  输出      {out_csv}")
    print(f"  总时长    约 {total_s:.0f} 秒"
          f"（每面 {args.hold:.0f} s + 换面 {args.trans:.0f} s × {len(SCHEDULE) - 1}）")
    print()
    print("【开始前】")
    print("  □ 卸桨、不接电池（USB 供电就够）")
    print("  □ 飞控/飞机放在平整桌面上，附近别有人走动")
    print("  □ 机头朝下那一步垫本书，别让镜头受力")
    print()

    # 关掉 Recorder 的 10 秒进度打印，避免和节奏提示抢同一行
    rsm.PROGRESS_EVERY_S = 1e9
    rec = rsm.Recorder(port, out_csv, rsm.REQUEST_HZ)

    # 给足余量，实际由 schedule 结束后置 stop
    rec_duration_h = (total_s + 120.0) / 3600.0
    th = threading.Thread(target=rec.run, args=(rec_duration_h,), daemon=True)
    th.start()
    time.sleep(2.0)  # 等串口打开 + 首帧

    try:
        run_schedule(args.hold, args.trans, args.prep)
    except KeyboardInterrupt:
        print("\n\n  收到 Ctrl-C，提前结束。已录到的数据会保留。")
    finally:
        rec.stop = True
        th.join(timeout=10.0)

    n = rec.stats.get("samples", 0)
    wall = rec.stats.get("wall_seconds", 0.0)
    print(f"\n  样本 {n:,}｜时长 {wall:.1f} s"
          f"｜速率 {n / wall if wall > 0 else 0:.1f} Hz")
    print(f"  CSV  {out_csv}")

    if args.no_calibrate:
        print(f"\n下一步：python3 {CALIB_TOOL} --csv '{out_csv}' --mode ferraris")
        return 0

    print("\n" + "=" * 74)
    print("  自动标定")
    print("=" * 74)
    cmd = [sys.executable, str(CALIB_TOOL),
           "--csv", str(out_csv), "--mode", "ferraris"]
    rc = subprocess.call(cmd)
    if rc != 0:
        print(f"\n标定失败（退出码 {rc}）。可以手工重试：")
        print("  " + " ".join(f"'{c}'" if " " in c else c for c in cmd))
    return rc


if __name__ == "__main__":
    sys.exit(main())
