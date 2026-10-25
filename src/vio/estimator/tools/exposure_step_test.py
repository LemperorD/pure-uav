#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""曝光步进测试：判断手动曝光到底有没有写进相机固件。

为什么需要这个测试
------------------
`v4l2-ctl --get-ctrl` 的读回值走 uvcvideo 的驱动缓存，`S_CTRL` 之后读回一致
**不能证明固件真的执行了**。所以「控件已锁定」这行打印不能当证据。

唯一可信的验证是看**图像本身**：同一个场景、同样的灯，曝光翻一倍，亮度就该翻一倍。

    L200 / L100 ≈ 2      -> 手动曝光真的在管事，锁是有效的
    L200 / L100 ≈ 1      -> 值只进了驱动/缓存，相机还在自动曝光
    中间值（1.3~1.7）    -> 部分生效，或场景里有光源在变

用法
----
    python3 tools/exposure_step_test.py            # 用 /dev/video4，增益 10
    python3 tools/exposure_step_test.py --gain 30
    python3 tools/exposure_step_test.py --device /dev/video2

测试时**别动相机**，镜头对着一个稳定的场景（开灯的房间、有纹理的墙）。
"""
from __future__ import annotations

import argparse
import json
import shutil
import subprocess
import sys
from pathlib import Path

CALIB = Path(__file__).resolve().parent.parent / "camera_imu_calibration"
CAM_SCRIPT = CALIB / "capture" / "vendor" / "capture_calibration.py"



def find_camera(expected="2bc5:0511"):
    """按 USB 身份在 sysfs 里找相机节点，返回 /dev/videoN 或 None。

    不能写死 /dev/video4 —— 换个 USB 口、或者别的摄像头插拔一次，节点号就变了
    （实测过：同一台相机先后在 video4 和 video0 上）。
    """
    vendor, product = expected.lower().split(":")
    for node in sorted(Path("/sys/class/video4linux").glob("video*")):
        try:
            index = (node / "index").read_text().strip()
            dev = (node / "device").resolve()
        except OSError:
            continue
        if index != "0":              # index=1 是 metadata 节点，不能拿来采集
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
    """挑一个同时装了 cv2 和 numpy 的解释器来跑采集子进程。

    采集脚本要用 cv2.imencode 存 PNG；用错解释器会在录完图那一刻才报错，
    白等 8 秒。所以开跑前先筛一遍。
    """
    cands = [sys.executable, "/home/kxy/miniforge3/envs/vins/bin/python",
             shutil.which("python3")]
    seen = set()
    for c in cands:
        if not c or c in seen:
            continue
        seen.add(c)
        if not Path(c).exists():
            continue
        try:
            r = subprocess.run([c, "-c", "import cv2, numpy"],
                               capture_output=True, timeout=90)
            if r.returncode == 0:
                return c
        except Exception:
            pass
    return None


def run_capture(py, device, exposure, gain, out_dir, seconds, width, height, fps):
    out_dir.mkdir(parents=True, exist_ok=True)
    cmd = [
        py, str(CAM_SCRIPT),
        "--device", str(device),          # 必须显式传：采集脚本自己的默认值是 /dev/video4
        "--output", str(out_dir),
        "--width", str(width), "--height", str(height), "--fps", str(fps),
        "--exposure", str(exposure), "--gain", str(gain),
        "--auto-interval", f"{1.0 / fps:.6f}",
        "--duration", str(seconds),
        # 【2026-10-20 修】不传 --max-images 会落回默认 MAX_IMAGES=40：
        # 30 fps 下 4 秒的"曝光步进"实际只采了 1.3 秒，L1/L2 样本量被砍掉 2/3。
        "--max-images", str(int(seconds * fps) + 20),
        # 这个工具的全部意义就是"我指定两个曝光比亮度"，不能让自动测光改掉
        "--no-auto-expose",
        "--skip-exposure-check",
        "--no-preview",
    ]
    print(f"\n$ {' '.join(cmd)}")
    p = subprocess.run(cmd, capture_output=True, text=True)
    if p.returncode != 0:
        print(p.stdout[-2000:])
        print(p.stderr[-2000:])
        raise SystemExit(f"❌ 采集失败（退出码 {p.returncode}）")
    return out_dir


def analyze(d_lo, d_hi, gamma, skip_first=10):
    """比较两组图。

    关键：PNG 里存的是 **gamma 编码** 后的值，均值不能直接比值。
    相机 gamma≈2.4 时，光翻一倍只让均值涨 2^(1/2.4)=1.33 倍 ——
    不还原 gamma 就会误判成"曝光没生效"（我第一版就栽在这）。
    """
    import cv2
    import numpy as np

    def stack(d):
        files = sorted((d / "cam0").glob("*.png"))
        if len(files) <= skip_first:
            raise SystemExit(f"❌ {d} 里只有 {len(files)} 张图，不够算")
        ims = [cv2.imread(str(f), cv2.IMREAD_GRAYSCALE).astype("float32")
               for f in files[skip_first:]]
        return np.mean(ims, axis=0), len(files)

    lo, n1 = stack(d_lo)
    hi, n2 = stack(d_hi)
    use = (lo > 15) & (hi < 250)              # 去掉噪声底和饱和像素
    if use.sum() < 500:
        raise SystemExit("❌ 两组图差异太大或几乎全饱和，换场景重测")
    raw = float(np.median(hi[use] / lo[use]))
    a = (lo[use] / 255.0) ** gamma
    b = (hi[use] / 255.0) ** gamma
    lin = float(np.median(b / a))
    return dict(lo_mean=float(lo.mean()), hi_mean=float(hi.mean()), raw=raw,
                lin=lin, sat=100 * float((hi >= 250).mean()), n1=n1, n2=n2,
                used=100 * float(use.mean()))


def readback(out_dir):
    m = json.loads((out_dir / "capture_manifest.json").read_text(encoding="utf-8"))
    lc = m.get("locked_controls") or {}
    oc = m.get("original_controls") or {}
    return lc, oc


def main() -> int:
    ap = argparse.ArgumentParser(description="曝光步进测试")
    ap.add_argument("--device", default=None,
                    help="默认按 USB 身份 2bc5:0511 自动找，不用手填")
    ap.add_argument("--gain", type=int, default=10)
    ap.add_argument("--low", type=int, default=100, help="低曝光值（100 微秒单位）")
    ap.add_argument("--high", type=int, default=200, help="高曝光值")
    ap.add_argument("--seconds", type=float, default=4.0)
    ap.add_argument("--width", type=int, default=1920)
    ap.add_argument("--height", type=int, default=1080)
    ap.add_argument("--fps", type=int, default=30)
    ap.add_argument("--gamma", type=float, default=2.4,
                    help="相机的 gamma 编码值；实测 2.4 时线性域比值=2.00")
    args = ap.parse_args()

    if args.device is None:
        args.device = find_camera()
        if args.device is None:
            print("❌ 没找到 USB 身份为 2bc5:0511 的相机节点。")
            print("   先确认：lsusb | grep 2bc5  应该有 0511 和 0614 两个")
            print("   如果有 0511 但没有 /dev/video 节点 → 拔插一次相机（uvcvideo 没绑上）")
            return 2
        print(f"  自动找到相机: {args.device}")
    if not Path(args.device).exists():
        print(f"❌ 找不到 {args.device}")
        return 2
    if not CAM_SCRIPT.is_file():
        print(f"❌ 找不到采集脚本 {CAM_SCRIPT}")
        return 2

    base = Path(__file__).resolve().parent.parent / "vio_online" / "out" / "_exp_test"
    print(f"相机 {args.device}  增益 {args.gain}  曝光 {args.low} vs {args.high}")
    print("⚠️ 测试期间别动相机，镜头对着稳定的场景")

    py = pick_python()
    if py is None:
        print("❌ 没找到同时装了 cv2 + numpy 的 python。")
        print("   用这个跑：/home/kxy/miniforge3/envs/vins/bin/python tools/exposure_step_test.py")
        return 2
    print(f"  用解释器: {py}")

    d_lo = run_capture(py, args.device, args.low, args.gain, base / f"exp{args.low}",
                       args.seconds, args.width, args.height, args.fps)
    d_hi = run_capture(py, args.device, args.high, args.gain, base / f"exp{args.high}",
                       args.seconds, args.width, args.height, args.fps)

    st = analyze(d_lo, d_hi, args.gamma)
    ratio, expect = st["lin"], args.high / args.low

    print()
    print("=" * 74)
    print(f"  曝光 {args.low:3d} → mean {st['lo_mean']:6.2f}  （{st['n1']} 张取后 {st['n1'] - 10} 张）")
    print(f"  曝光 {args.high:3d} → mean {st['hi_mean']:6.2f}  （{st['n2']} 张取后 {st['n2'] - 10} 张）")
    print(f"  饱和像素 {st['sat']:.2f}%   可用像素 {st['used']:.1f}%   gamma {args.gamma}")
    print()
    print(f"  原始比值（gamma 编码域）  {st['raw']:.3f}   ← 直接比均值会得到这个，别用它判断")
    print(f"  线性域比值（还原 gamma）  {ratio:.3f}   ← 拿这个判断，理论值 {expect:.2f}")
    print("=" * 74)

    lc_lo, oc_lo = readback(d_lo)
    lc_hi, oc_hi = readback(d_hi)
    print(f"  manifest 记录（低）: 锁定 exposure={lc_lo.get('exposure')} "
          f"先读到的 exposure={oc_lo.get('exposure')} 先 EA={oc_lo.get('exposure_auto')}")
    print(f"  manifest 记录（高）: 锁定 exposure={lc_hi.get('exposure')} "
          f"先读到的 exposure={oc_hi.get('exposure')} 先 EA={oc_hi.get('exposure_auto')}")
    if oc_hi.get("exposure_auto") == 1:
        print("  注意：高曝光那一次的 original EA=1，说明上一轮退出没能恢复成自动 —— "
              "状态会跨进程残留")
    print()

    if ratio > 0.85 * expect:
        print("  ✅ 手动曝光真的在管事（锁有效）。")
        print("     那么之前画面变暗就不是「锁没上」，而是场景照度/参数本身的问题。")
        return 0
    if ratio < 1.15:
        print("  ❌ 曝光翻倍但亮度几乎没变 —— 值只进了驱动，相机还在自动曝光。")
        print("     「控件已锁定」那行是不可信的，采集必须改成锁定后做这个步进验证。")
        return 1
    print(f"  🟡 部分生效（{ratio:.2f} vs 期望 {expect:.2f}）。")
    print("     可能是场景里的光源在变（窗户、灯），或者曝光被上限截断。")
    print("     换个稳定的场景、把 --high 调小一点再测一次。")
    return 1


if __name__ == "__main__":
    raise SystemExit(main())
