#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""亮度时间线：盯住一个不动的画面录 N 秒，看亮度会不会自己变。

用来判断"画面莫名其妙变暗"到底是
    A. 相机/驱动/USB 链路的问题   —— 亮度会自己漂移或跳变
    B. 场景照度或取景变了         —— 亮度基本恒定，只在切换场景时变

做法：每秒存一张图（不动相机、不动灯），事后画亮度曲线。

    python3 tools/brightness_timeline.py                # 60 秒
    python3 tools/brightness_timeline.py --seconds 120
    python3 tools/brightness_timeline.py --tag with_lamp  # 给这一次起个名，图会分开存
"""
from __future__ import annotations

import argparse
import datetime
import json
import shutil
import subprocess
import sys
from pathlib import Path

CALIB = Path(__file__).resolve().parent.parent / "camera_imu_calibration"
CAM_SCRIPT = CALIB / "capture" / "vendor" / "capture_calibration.py"
GAMMA = 2.4


def find_camera(expected="2bc5:0511"):
    vendor, product = expected.lower().split(":")
    for node in sorted(Path("/sys/class/video4linux").glob("video*")):
        try:
            index = (node / "index").read_text().strip()
            dev = (node / "device").resolve()
        except OSError:
            continue
        if index != "0":
            continue
        ids = None
        for d in (dev, *dev.parents):
            if (d / "idVendor").is_file():
                ids = ((d / "idVendor").read_text().strip().lower(),
                       (d / "idProduct").read_text().strip().lower())
                break
        if ids == (vendor, product):
            return f"/dev/{node.name}"
    return None


def pick_python():
    for c in (sys.executable, "/home/kxy/miniforge3/envs/vins/bin/python",
              shutil.which("python3")):
        if not c or not Path(c).exists():
            continue
        try:
            if subprocess.run([c, "-c", "import cv2, numpy"],
                              capture_output=True, timeout=90).returncode == 0:
                return c
        except Exception:
            pass
    return None


def main() -> int:
    ap = argparse.ArgumentParser(description="亮度时间线")
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--interval", type=float, default=1.0, help="每秒存几张")
    ap.add_argument("--tag", default="", help="这一次的名字，用来区分不同的图")
    ap.add_argument("--device", default=None)
    args = ap.parse_args()

    import capture_config as cfg

    dev = args.device or find_camera()
    if dev is None:
        print("❌ 没找到 2bc5:0511 的相机")
        return 2
    py = pick_python()
    if py is None:
        print("❌ 没找到装了 cv2 的 python")
        return 2

    stamp = datetime.datetime.now().strftime("%H%M%S")
    tag = f"{args.tag or 'run'}_{stamp}"
    out = Path(__file__).resolve().parent.parent / "vio_online" / "out" / "_timeline" / tag
    out.mkdir(parents=True, exist_ok=True)

    print(f"相机 {dev}  曝光 {cfg.EXPOSURE}  增益 {cfg.GAIN}")
    print(f"录 {args.seconds:g} 秒，每 {args.interval:g} 秒一张 -> {out}")
    print("⚠️ 这段时间里【相机别动、灯别动、人也别挡光】")
    print("   如果是测补光效果，先录一段黑的，再开灯录一段，两次分别给 --tag\n")

    cmd = [py, str(CAM_SCRIPT), "--device", dev, "--output", str(out),
           "--width", "1920", "--height", "1080", "--fps", "30",
           "--exposure", str(cfg.EXPOSURE), "--gain", str(cfg.GAIN),
           "--auto-interval", str(args.interval), "--duration", str(args.seconds),
           # 【2026-10-20 修】以前这里没传 --max-images，于是落回
           # calibration_config.py 的默认 MAX_IMAGES=40 —— 1 秒存 1 张，
           # 存到第 40 张就 break，看起来像"相机在 40 秒处断流"。
           # 那次"60 秒请求只交付 39.84 秒"就是这个假象，不是相机的问题。
           "--max-images", str(max(1, int(args.seconds / max(args.interval, 1e-3)) + 10)),
           "--auto-expose-max-exposure-ms", str(cfg.AUTO_EXPOSE_MAX_EXPOSURE_MS),
           "--auto-expose-target", str(cfg.AUTO_EXPOSE_TARGET),
           "--auto-expose-min-mean", str(cfg.AUTO_EXPOSE_MIN_MEAN),
    ] + ([] if cfg.AUTO_EXPOSE else ["--no-auto-expose"]) + (["--allow-dark"] if cfg.ALLOW_DARK else []) + [
           "--no-preview"]
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode != 0:
        print(p.stdout[-1500:])
        print(p.stderr[-1500:])
        print(f"❌ 采集失败（退出码 {p.returncode}）")
        return 1

    import cv2
    import numpy as np

    files = sorted((out / "cam0").glob("*.png"))
    if not files:
        print("❌ 一张图都没存下来")
        return 1

    t, means, laps = [], [], []
    t0 = None
    for f in files:
        g = cv2.imread(str(f), cv2.IMREAD_GRAYSCALE)
        if g is None:
            continue
        ns = int(f.stem) / 1e9
        t0 = ns if t0 is None else t0
        t.append(ns - t0)
        means.append(float(g.mean()))
        laps.append(float(cv2.Laplacian(g, cv2.CV_64F).var()))
    means = np.array(means)
    laps = np.array(laps)
    lin = (means / 255.0) ** GAMMA

    print()
    print("=" * 70)
    print(f"  样本 {len(means)} 张，跨度 {t[-1]:.1f} s（请求 {args.seconds:g} s）")
    shortfall = args.seconds - t[-1]
    if shortfall > max(3.0, 0.15 * args.seconds):
        print(f"  ⚠️ 只拿到 {t[-1]:.1f} s，比请求少 {shortfall:.1f} s")
    print(f"  亮度 min {means.min():.1f} / 中位 {np.median(means):.1f} / max {means.max():.1f}")
    print(f"  波动 (max-min)/中位 = {100 * (means.max() - means.min()) / max(np.median(means), 1e-9):.1f}%")
    print(f"  线性光 min {lin.min():.4f} / max {lin.max():.4f}")
    print(f"  局部对比 median {np.median(laps):.1f}")
    print("=" * 70)
    print("  时间线（每格 = 1 个样本，1..9 = 相对档位，. = 最暗那档）")
    # 绝对刻度：以中位数为 5（中间档），每档 = 5% 变化。
    # 之前用 min~max 归一化，0.3 个灰度级的噪声会被放大成满屏 1->9，误导。
    med = float(np.median(means))
    row = ""
    for m in means:
        dev = (m - med) / max(med, 1e-9) * 20      # 5% == 1 档
        lv = int(round(5 + max(-5, min(4, dev))))
        row += str(max(0, min(9, lv)))
    for i in range(0, len(row), 60):
        print(f"    {row[i:i + 60]}  （中间=5，左暗右亮，1 档=5%）")
    print()

    # 判据
    swing = (means.max() - means.min()) / max(np.median(means), 1e-9)
    half = len(means) // 2
    first_half = np.median(means[:half]) if half else np.median(means)
    second_half = np.median(means[half:]) if half else np.median(means)
    drift = (second_half - first_half) / max(first_half, 1e-9)

    if swing > 0.25:
        print("  ❌ 亮度自己在动（波动 %.0f%%）—— 相机不带这么玩，是链路问题。" % (swing * 100))
        print("     下一步：这说明【曝光锁定】没有被稳定执行，或者 USB 供电/带宽有问题。")
        print("     把上面这条曲线截图给我，我按这个去改采集端的锁定逻辑（fail-fast + 重试）。")
    elif abs(drift) > 0.12:
        print(f"  ❌ 前半段和后半段差 {drift * 100:+.0f}% —— 有单向漂移，不正常。")
        print("     典型的嫌疑：相机自己回到自动曝光、或传感器/供电过热降增益。")
    else:
        print(f"  ✅ 一分钟内亮度稳定（波动 {swing * 100:.0f}%，前后半段差 {drift * 100:+.0f}%）。")
        print("     说明相机和曝光锁定是好的 —— 那之前的变暗是取景/照度/进光量变了。")

    lc = json.loads((out / "capture_manifest.json").read_text(encoding="utf-8"))
    print()
    print("  manifest 记录：锁定 " + json.dumps(lc.get("locked_controls"), ensure_ascii=False))
    print("                 开录前 " + json.dumps(lc.get("original_controls"), ensure_ascii=False))
    ae = lc.get("auto_exposure") or {}
    if ae.get("achieved_mean") is not None:
        print(f"                 自动测光 曝光 {ae.get('exposure_ms')} ms / 增益 {ae.get('gain')}"
              f" → 灰度 {ae.get('achieved_mean')}（增益范围 {ae.get('gain_range')}）")
    else:
        print("                 自动测光 未启用（固定曝光/增益）")
    print(f"                 读到 {lc.get('frames_read')} 帧 / "
          f"首末帧跨度 {lc.get('elapsed_between_first_last_frames_s')} s / "
          f"{lc.get('observed_read_fps')} fps")
    lastseq = (lc.get("last_frame_timestamp_metadata") or {}).get("v4l2_sequence")
    if lastseq is not None and lc.get("frames_read"):
        # 序列号从 0 开始，frames_read 不含被丢掉的 warmup 帧 —— 不减掉它，
        # "丢帧"会恒等于 warmup-1，看着像相机在丢帧，其实是热身帧。
        warmup = int(lc.get("warmup_frames") or 0)
        verify = int((lc.get("exposure_verification") or {}).get("frames_consumed") or 0)
        dropped = max(0, lastseq + 1 - lc["frames_read"] - warmup - verify)
        print(f"                 末尾 v4l2_sequence {lastseq}，丢帧 {dropped}"
              f"（已扣除 {warmup} 帧 warmup + {verify} 帧曝光验证）")
    print(f"                 结束状态 {lc.get('termination')}"
          f"；交付 {lc.get('delivered_duration_s')} s / 请求 {lc.get('requested_duration_s')} s")
    # 【2026-10-20】把真正的原因说清楚：以前只报"少了多少秒"，看起来永远像
    # "相机断流"。实际上最常见的是采集循环自己提前 break（存图达到上限）。
    if shortfall > max(3.0, 0.15 * args.seconds):
        why = str(lc.get("termination") or "")
        if why.startswith("max_images"):
            print("  ↳ 原因：存图达到 --max-images 上限提前结束，不是相机断流。")
        elif why.startswith("failed"):
            print("  ↳ 原因：" + why)
        else:
            print("  ↳ 原因不明，看上面 frames_read / v4l2_sequence 是否连续。")
    return 0


if __name__ == "__main__":
    sys.path.insert(0, str(CALIB / "capture"))
    raise SystemExit(main())
