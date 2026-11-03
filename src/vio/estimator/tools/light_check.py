#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""光照体检：录 3 秒，判断"现在这个光够不够跑 VIO"。

为什么不能只看平均亮度
----------------------
相机输出的是 **gamma 编码**（γ≈2.4）后的图，均值差 1.8 倍，实际光量可能差 4.4 倍。
所以这里折算到线性光再和已知能跑的档位比。

判据基准（都是实测过的）
------------------------
    session24  线性光 0.019  / 特征 519  -> 尺度 -1.9%，能跑   ✅
    session25  线性光 0.0047 / 特征  52  -> 发散 5.5 倍       ❌
    session26  线性光 0.0063 / 特征  38  -> 发散 3.9 倍       ❌
    session27  线性光 0.0016 / 特征   2  -> 全黑               ❌

用法
----
    python3 tools/light_check.py
    python3 tools/light_check.py --seconds 5 --gain 30
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
GAMMA = 2.4

# 判据基准（实测标定）
#   session24  亮度48.4 线性光0.019  覆盖13/25 = 52%  -> 尺度 -1.9%，能跑  ✅
#   当前白板    亮度20.8 线性光0.002  覆盖 4/25 = 16%  -> 不行（大片无纹理）
#   session25  亮度31.6              覆盖 2/25 =  8%  -> 发散 5.5 倍
#   session26  亮度26.3              覆盖 1/25 =  4%  -> 发散 3.9 倍
# 关键结论：决定成败的是【特征在画面上的空间分布】，不是总数、也不只是亮度。
COV_OK = 40.0        # 覆盖率 %（有 >=10 个候选点的格子占比）
COV_MARGINAL = 20.0
LIN_OK = 0.018       # 线性光，session24 档位
FEAT_OK = 400


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
    ap = argparse.ArgumentParser(description="光照体检")
    ap.add_argument("--device", default=None)
    ap.add_argument("--seconds", type=float, default=3.0)
    ap.add_argument("--exposure", type=int, default=None,
                    help="默认用 capture_config.py 里的值")
    ap.add_argument("--gain", type=int, default=None)
    args = ap.parse_args()

    import capture_config as cfg  # 需要把 capture 目录加进 sys.path
    exposure = args.exposure if args.exposure is not None else cfg.EXPOSURE
    gain = args.gain if args.gain is not None else cfg.GAIN

    dev = args.device or find_camera()
    if dev is None:
        print("❌ 没找到 2bc5:0511 的相机节点。lsusb | grep 2bc5 确认一下，或拔插一次")
        return 2
    py = pick_python()
    if py is None:
        print("❌ 没找到装了 cv2 的 python")
        return 2

    import datetime
    stamp = datetime.datetime.now().strftime("%H%M%S")
    out = (Path(__file__).resolve().parent.parent / "vio_online" / "out"
           / "_light_check" / stamp)
    out.mkdir(parents=True, exist_ok=True)
    print(f"相机 {dev}  自动测光（曝光上限 {cfg.AUTO_EXPOSE_MAX_EXPOSURE_MS:g} ms，"
          f"目标灰度 {cfg.AUTO_EXPOSE_TARGET:g}）  录 {args.seconds:g} s")
    print("（这会儿别动相机，镜头对着你要走的那个场景）\n")
    cmd = [py, str(CAM_SCRIPT), "--device", dev, "--output", str(out),
           "--width", "1920", "--height", "1080", "--fps", "30",
           "--exposure", str(exposure), "--gain", str(gain),
           "--auto-interval", "0.033333", "--duration", str(args.seconds),
           # 【2026-10-20 修】不传 --max-images 会落回默认 MAX_IMAGES=40，
           # 30 fps 下等于只采 1.3 秒 —— 后面的统计其实全是这 1.3 秒的。
           "--max-images", str(int(args.seconds * 60) + 20),
           # 和 record_session 用完全同一套测光参数，否则这里量到的亮度和真录制不一致
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
    files = sorted((out / "cam0").glob("*.png"))[5:]
    fast = cv2.FastFeatureDetector.create(20, True)
    means, feats = [], []
    for f in files:
        g = cv2.imread(str(f), cv2.IMREAD_GRAYSCALE)
        if g is None:
            continue
        means.append(float(g.mean()))
        feats.append(len(fast.detect(g, None)))
    if not means:
        print("❌ 一张图都没拿到")
        return 1

    mean = float(np.mean(means))
    lin = (mean / 255.0) ** GAMMA
    feat = int(np.median(feats))

    # 5x5 网格覆盖：OpenVINS 按网格取点，空格子等于不参与
    grid_sum = None
    for f in files[len(files) // 2:len(files) // 2 + 5]:
        g = cv2.imread(str(f), cv2.IMREAD_GRAYSCALE)
        if g is None:
            continue
        kp = fast.detect(g, None)
        h, w = g.shape
        gr = np.zeros((5, 5), int)
        for k in kp:
            x, y = k.pt
            gr[min(4, int(y / h * 5)), min(4, int(x / w * 5))] += 1
        grid_sum = gr if grid_sum is None else grid_sum + gr
    if grid_sum is not None:
        grid_sum = grid_sum // 5
        cells = int((grid_sum >= 10).sum())
        cov = cells / 25.0 * 100
    else:
        grid_sum, cells, cov = None, 0, 0.0

    print("=" * 66)
    print(f"  平均亮度   {mean:6.1f}      线性光 {lin:.4f}   （session24: 49.3 / 0.0194）")
    print(f"  FAST 总数  {feat:6d}      但总数会骗人，看下面的分布")
    print(f"  ★ 网格覆盖 {cov:5.0f}%  ({cells}/25 格有 >=10 个点)   （session24: 52%）")
    if grid_sum is not None:
        print()
        print("  5x5 网格特征分布（OpenVINS 按这个网格取点）:")
        for r in grid_sum:
            bar = "".join("#" if v >= 100 else "+" if v >= 20 else "." if v > 0 else " " for v in r)
            print(f"     [{bar}]  " + " ".join(f"{v:4d}" for v in r))
    print("=" * 66)

    try:
        mm = json.loads((out / "capture_manifest.json").read_text(encoding="utf-8"))
        print(f"  实际锁定  {json.dumps(mm.get('locked_controls'), ensure_ascii=False)}")
        print(f"  锁定之前  {json.dumps(mm.get('original_controls'), ensure_ascii=False)}")
        ae = mm.get("auto_exposure") or {}
        if ae.get("achieved_mean") is not None:
            print(f"  自动测光  曝光 {ae.get('exposure_ms')} ms / 增益 {ae.get('gain')}"
                  f" → 灰度 {ae.get('achieved_mean')}"
                  f"（增益可用范围 {ae.get('gain_range')}）")
            gr = ae.get("gain_range") or [0, 0]
            er = ae.get("exposure_range") or [0, 0]
            gain_maxed = ae.get("gain") is not None and ae["gain"] >= gr[1]
            exp_maxed = (ae.get("exposure") or 0) >= min(er[1], 1e6 / (30 * 100))
            if gain_maxed:
                print("            ⚠️ 增益已经拉到上限 —— 这个场景没有余量了。"
                      "赛场如果比这里再暗一档，就会直接被判太暗。")
                print("               想留余量：加一点光，把增益降到 50~60，"
                      "顺带能把曝光从 18 ms 降到 10~12 ms（少一半运动模糊）。")
            elif ae.get("achieved_mean", 0) >= (ae.get("target") or 100) * 0.9:
                print("            ✅ 增益没到上限，还有余量")
        print(f"  本次目录  {out}")
        print()
    except Exception:
        pass

    if cov >= COV_OK and lin >= LIN_OK * 0.6:
        print("  ✅ 够。构图和亮度都到位，可以录。")
        return 0
    if cov < COV_MARGINAL or feat < FEAT_OK * 0.5:
        print("  ❌ 不够 —— 但大概率不是灯的锅，是【构图】。")
        print("     看上面网格：' ' 和 '.' 的格子 = 那片画面没有纹理（白板/白墙/天花板平板）。")
        print("     光只能提高信噪比，长不出纹理来。")
        print()
        print("     怎么改：把镜头抬高/转向有东西的方向 —— 椅子、桌腿、门框、黑板边框、")
        print("     投影仪。让画面上中下都填满，别让白板/地面占满整屏。")
        print("     改完再跑一次这条命令，覆盖率上到 40% 以上就能录。")
        return 1
    print("  🟡 勉强够。能录，但边走边扫到无纹理区域时可能掉特征。")
    print("     建议把镜头朝向再调一点，或者把移动路线限制在有家具的那半边。")
    return 1


if __name__ == "__main__":
    sys.path.insert(0, str(CALIB / "capture"))
    raise SystemExit(main())
