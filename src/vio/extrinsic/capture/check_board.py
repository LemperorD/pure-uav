#!/usr/bin/env python3
"""标定板体检 —— 只看"板子认得出来吗、板面参数填对了吗"，不碰 IMU。

为什么需要它：板子只要有一处不对，标定程序只会回你一句
「有效标定板帧只有 0」，看不出是哪一处。这个脚本把三个量拆开逐个验证：

    1) 检出    每帧认得出几个 tag（<min_tags 就是 0，最常见的是对比度不够）
    2) 大小    每个 tag 在画面里多少像素（36h11 至少要 ~30 px，舒服要 45 px）
    3) 几何    PnP 残差。**这一项才是真正致命的**：行列数、tagSize、tagSpacing、
               tag_rotation_deg 只要有一个填错，检出照样成功，PnP 残差会到 100 px 以上，
               标定必然失败。

它还会反解 tagSpacing 和 tag_rotation_deg（对 AprilGrid），直接告诉你该填什么。

用法：
    python capture/check_board.py                      # 用最近一次取景体检拍的图
    python capture/check_board.py --dir ../data/calib/session09/cam0
    python capture/check_board.py --dir ../data/calib/session09/cam0 --target configs/checkerboard.yaml
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent                # extrinsic/：唯一需要上 sys.path 的路径
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from capture import image_health                          # noqa: E402

DEFAULT_DIR = ROOT.parent / "data" / "_framing_check" / "cam0"


def images_of(path: Path):
    if path.is_dir():
        return sorted(p for p in path.iterdir() if p.suffix.lower() in (".png", ".jpg", ".jpeg"))
    return sorted(path.parent.glob(path.name))


def make_target(entry, min_tags, window_px):
    from calib.data import Target
    return Target(entry, min_tags=min_tags, april_corner_window_px=window_px)


def pnp_rms(target, camera, gray, stamp=0):
    """返回 (角点数, 每 tag 边长中位, PnP 残差) 或 None。"""
    from calib.data import make_observation
    result = target.detect(gray)
    if result is None:
        return None
    xyz, uv = result
    obs = make_observation(stamp, xyz, uv, camera, None, checker=False, max_points=0)
    if obs is None:
        return None
    tag_px = None
    if target.kind == "aprilgrid":
        quads = np.asarray(uv, float).reshape(-1, 4, 2)
        tag_px = float(np.median(np.linalg.norm(quads[:, 1] - quads[:, 0], axis=1)))
    return len(xyz) // 4, tag_px, float(obs.pnp_rms)


def scan_entry(entry, camera, grays, min_tags, window_px, key, values):
    """把 entry[key] 逐个换成 values 试，返回 (最优值, 中位残差, 表)。"""
    rows, best = [], None
    for v in values:
        trial = dict(entry, **{key: v})
        try:
            target = make_target(trial, min_tags, window_px)
        except Exception:                                  # noqa: BLE001
            continue
        rms = [r for r in (pnp_rms(target, camera, g, i) for i, g in enumerate(grays)) if r]
        if len(rms) < 2:
            continue
        med = float(np.median([r[2] for r in rms]))
        rows.append((v, med, len(rms)))
        if best is None or med < best[1]:
            best = (v, med, len(rms))
    return best, rows


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dir", type=Path, default=DEFAULT_DIR, help="图片目录（默认最近一次取景体检）")
    ap.add_argument("--target", type=Path, default=ROOT / "configs" / "aprilgrid.yaml")
    ap.add_argument("--camera", type=Path, default=ROOT / "configs" / "gemini_pro_rgb.yaml")
    ap.add_argument("--sample", type=int, default=12, help="最多分析多少张")
    ap.add_argument("--max-pnp-px", type=float, default=None,
                    help="PnP 残差门槛（median）；默认取 extrinsics_config 里的 max_pnp_rms_px")
    args = ap.parse_args(argv)
    if args.max_pnp_px is None:
        try:
            import extrinsics_config as ec
            args.max_pnp_px = float(getattr(ec.SETTINGS, "max_pnp_rms_px", 2.0))
        except Exception:                                  # noqa: BLE001
            args.max_pnp_px = 2.0

    from calib.data import Camera, read_yaml
    entry = read_yaml(args.target)
    kind = entry.get("target_type")
    camera = Camera.load(args.camera)
    files = images_of(args.dir)
    if not files:
        print(f"❌ {args.dir} 里没有图片。先跑 capture/framing_check.py，或 --dir 指到 session 的 cam0。")
        return 2
    if len(files) > args.sample:
        idx = np.linspace(0, len(files) - 1, args.sample).astype(int)
        files = [files[i] for i in idx]
    grays = [g for g in (cv2.imread(str(f), cv2.IMREAD_GRAYSCALE) for f in files) if g is not None]
    print("=" * 78)
    print(f"  图片      {len(grays)} 张 {files[0].parent}")
    print(f"  分辨率    {grays[0].shape[1]}x{grays[0].shape[0]}")
    print(f"  相机内参  {args.camera}")
    print(f"  标定板    {args.target}")
    p = [int(v) for v in np.percentile(grays[0], [50, 90, 99])]
    print(f"  亮度      p50={p[0]} p90={p[1]} p99={p[2]}")
    print("=" * 78)

    # 先看默认设置下 image_health 自己怎么判 —— 那就是 framing_check / record_session 的结论
    res = image_health.analyze([str(f) for f in files], args.target, sample=len(files))
    level, actions = image_health.report(res)
    d = res["detection"]
    min_tags = d.get("min_tags") or 4
    window_px = 4
    print()
    print("=" * 78)
    print("  逐帧几何（这才是标定程序真正在意的东西）")
    print("=" * 78)

    target = make_target(entry, min_tags, window_px)
    rms_list = []
    for i, g in enumerate(grays):
        r = pnp_rms(target, camera, g, i)
        if r is None:
            print(f"  帧 {i:>2}  检出失败")
            continue
        ntag, tag_px, rms = r
        rms_list.append(rms)
        extra = f"tag {tag_px:5.0f} px" if tag_px else ""
        flag = "" if rms <= args.max_pnp_px else "  ← 残差偏大，板面参数不对"
        print(f"  帧 {i:>2}  {ntag:>2} tag  {extra:>12}  PnP {rms:6.2f} px{flag}")

    if not rms_list:
        print()
        print("❌ 一帧都没解出几何。先解决检出（看上面前面的判读），再来查板面参数。")
        return 1

    med = float(np.median(rms_list))
    print()
    print(f"  PnP 残差中位 {med:.2f} px（门槛 {args.max_pnp_px:g} px）")

    if kind == "aprilgrid":
        print()
        print("=" * 78)
        print("  反解板面参数")
        print("=" * 78)
        best, rows = scan_entry(entry, camera, grays, min_tags, window_px,
                                "tagSpacing", [round(0.1 + 0.01 * i, 2) for i in range(0, 71)])
        if rows:
            print("  tagSpacing 扫描（当前 %.2f）:" % float(entry["tagSpacing"]))
            for v, m, n in rows:
                if v < 0.2:
                    continue
                mark = "  ← 最优" if best and abs(v - best[0]) < 1e-9 else ""
                if mark or abs(v - round(float(entry["tagSpacing"]), 2)) < 1e-9 or int(v * 100) % 10 == 0:
                    print(f"    {v:.2f} → 中位 {m:6.2f} px ({n} 帧){mark}")
        best_rot, rot_rows = scan_entry(entry, camera, grays, min_tags, window_px,
                                        "tag_rotation_deg", [0, 90, 180, 270])
        print()
        print("  tag_rotation_deg 扫描（当前 %s）:" % entry.get("tag_rotation_deg", 180))
        for v, m, n in rot_rows:
            mark = "  ← 最优" if best_rot and v == best_rot[0] else ""
            print(f"    {v:>3}° → 中位 {m:6.2f} px ({n} 帧){mark}")

        if best and best_rot:
            print()
            print("-" * 78)
            print("  建议写进 %s：" % args.target)
            print(f"    tagSpacing: {best[0]:.2f}")
            print(f"    tag_rotation_deg: {best_rot[0]}")
            print("  （tagSize 用卡尺量黑框外沿填，反解不出来 —— 它只影响平移的绝对尺度）")
            print("-" * 78)

    ok = med <= args.max_pnp_px
    print()
    print("✅ 板面参数对得上，可以采正式数据" if ok else
          "❌ PnP 残差超标 —— 把上面建议的 tagSpacing / tag_rotation_deg 填进 yaml 再跑一次")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
