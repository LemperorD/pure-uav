#!/usr/bin/env python3
"""一键标定 —— 自动挑出连续可用窗口，然后调 calibrate_extrinsics.py。

为什么需要这个：
    `calibrate_extrinsics.py` 默认取【整段】录像。只要中间有一段 >0.35 秒
    什么都认不出（"大空洞"），它就整段拒绝，报：
        「成功检测标定板帧之间存在大空洞；请截取连续可见的窗口。」
    而它自己不会去挑窗口 —— 要你手动把 start_s / max_duration_s 填进
    extrinsics_config.py。2026-10-16 一晚上在这上面来回折了五次。

    这个脚本把"挑窗口"自动化：先逐帧复算一遍，找出最长的一段连续可用区间，
    写成临时 settings YAML，再带着 --settings 去调标定。你只需要点一下绿三角。

它做什么：
    1. 找 session（默认取最新的）
    2. 逐帧跑标定程序自己的检测+PnP，算出它真正会接受哪些帧（约 1~2 分钟）
    3. 按 max_camera_gap_s 切段，挑出最长的一段 ≥ min_duration_s 且 ≥ min_frames
    4. 无空洞 → 用整段；有空洞但有可用窗口 → 用那个窗口；都没有 → 叫你别浪费时间
    5. 调 calibrate_extrinsics.py --settings <临时文件> --output results/<session>[_window]
    6. 把三个残差和最终外参打印出来

用法（PyCharm 里直接点运行）：
    python capture/run_calibration.py                     # 最新的 session
    python capture/run_calibration.py --session session06
    python capture/run_calibration.py --list
    python capture/run_calibration.py --gap 0.6           # 自定义空洞门槛
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

import image_health                                     # noqa: E402

# 这几个跟着上一轮实测定下来的甜点走，可以用命令行覆盖
KNOT_SPACING_S = 0.10
CAMERA_RATE_HZ = 60.0
MAX_PNP_RMS_PX = 3.5


def find_sessions() -> list[str]:
    data = ROOT / "data"
    if not data.is_dir():
        return []
    out = [(p.stat().st_mtime, p.name) for p in data.iterdir()
           if p.is_dir() and (p / "images.csv").is_file() and (p / "imu.csv").is_file()]
    return [n for _, n in sorted(out, reverse=True)]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session", help="session 名（默认取最新）")
    ap.add_argument("--list", action="store_true", help="列出可用 session")
    ap.add_argument("--gap", type=float, default=0.35,
                    help="空洞门槛，秒（要跟 extrinsics_config.py 的 max_camera_gap_s 一致）")
    ap.add_argument("--min-frames", type=int, default=35)
    ap.add_argument("--min-duration", type=float, default=5.0)
    ap.add_argument("--knot-spacing", type=float, default=KNOT_SPACING_S)
    ap.add_argument("--dry-run", action="store_true", help="只挑窗口，不跑标定")
    args = ap.parse_args()

    sessions = find_sessions()
    if args.list:
        print("可用的 session（从新到旧）：")
        for n in sessions:
            print(f"  {n}")
        return 0
    if not sessions:
        print(f"❌ {ROOT/'data'} 下没有找到 session")
        return 2
    name = args.session or sessions[0]
    session = ROOT / "data" / name
    if not session.is_dir():
        print(f"❌ 没有这个 session：{session}\n   可用的：{'  '.join(sessions)}")
        return 2

    images = sorted((session / "cam0").glob("*.png"))
    if not images:
        print(f"❌ {session/'cam0'} 下没有图片")
        return 2

    # 相机 / 标定板配置从 extrinsics_config.py 读，保证和标定侧完全一致
    import extrinsics_config as ec
    camera_yaml = str(ec.PATHS["camera"])
    target_yaml = str(ec.PATHS["target"])

    print("=" * 72)
    print(" 一键标定")
    print("=" * 72)
    print(f"  session   {name}（{len(images)} 张图）")
    print(f"  相机配置  {camera_yaml}")
    print(f"  标定板    {target_yaml}")

    from calib.data import read_yaml
    target = read_yaml(target_yaml)
    cols = int(target["cols"])
    rows = int(target["rows"])
    print(f"  板子      {cols} x {rows} 内角点，格边长 {target['square_size']} m")
    print()

    print("逐帧复算（跑标定程序自己的检测+PnP，约 1~2 分钟）……")
    tl = image_health.full_timeline(
        images, camera_yaml, target_yaml,
        max_pnp_rms=MAX_PNP_RMS_PX, gap_s=args.gap,
        min_frames=args.min_frames, min_duration_s=args.min_duration,
        log=lambda m: print(m))
    print(f"  {tl['frames']} 帧里：检出 {tl['detected']}，"
          f"标定程序会接受 {tl['accepted']}（PnP 中位 {tl.get('pnp_median', float('nan')):.2f} px）")
    if tl["accepted"] < args.min_frames:
        print()
        print(f"❌ 只有 {tl['accepted']} 帧被接受，低于下限 {args.min_frames} 帧 —— 数据不够，重录。")
        return 2

    gaps = tl.get("gaps_over", [])
    max_gap = tl.get("max_gap_ms", 0.0)
    print(f"  最大空洞  {max_gap:.0f} ms（门槛 {args.gap*1000:.0f} ms）")
    if gaps:
        print(f"  超标空洞  {len(gaps)} 处：" +
              "  ".join(f"{a:.2f}s起{g:.0f}ms" for a, g in gaps[:6]))

    if not gaps:
        start, span, n = 0.0, tl.get("span_s", 0.0), tl["accepted"]
        print(f"\n★ 整段没有超标空洞 —— 直接标定（{n} 帧 / {span:.2f}s）")
        suffix = ""
    elif tl.get("best"):
        start, span, n = tl["best"]
        print(f"\n★ 最长可用窗口：{start:.2f}s 起，{span:.2f}s，{n} 帧")
        print(f"  （整段有 {len(gaps)} 处空洞，自动切到这一段，不用你手填 start_s）")
        suffix = "_window"
    else:
        print()
        print("❌ 没有一段【连续】够长又够密的窗口 —— 带着这份数据去跑标定只会报大空洞。")
        print("   直接重录。录制时：让棋盘全程留在画面中央、别转到它出画、慢一点。")
        return 2

    settings_path = session / "auto_window.yaml"
    settings_path.write_text(
        f"# 由 capture/run_calibration.py 自动生成 —— 别手改，重跑会覆盖\n"
        f"start_s: {start:.3f}\n"
        f"max_duration_s: {span:.3f}\n"
        f"knot_spacing_s: {args.knot_spacing}\n"
        f"camera_rate_hz: {CAMERA_RATE_HZ}\n"
        f"max_pnp_rms_px: {MAX_PNP_RMS_PX}\n")
    print(f"  窗口设置已写到 {settings_path.relative_to(ROOT)}")

    output = ROOT / "results" / f"{name}{suffix}"
    print(f"  输出目录  results/{name}{suffix}")
    if args.dry_run:
        print("\n（--dry-run，不跑标定）")
        return 0

    cmd = [sys.executable, str(ROOT / "calibrate_extrinsics.py"),
           "--images", str(session / "images.csv"),
           "--imu", str(session / "imu.csv"),
           "--camera", camera_yaml,
           "--target", target_yaml,
           "--settings", str(settings_path),
           "--output", str(output)]
    print()
    print("运行标定（几分钟）……")
    print("  " + " ".join(cmd[1:]))
    print()
    rc = subprocess.call(cmd, cwd=str(ROOT))

    report = output / "report.json"
    if report.is_file():
        d = json.loads(report.read_text())
        m = d.get("final_metrics", {})
        print()
        print("=" * 72)
        print(f" 结果：{d.get('status')}")
        print("=" * 72)
        if m:
            for key, limit, label in (("reprojection_rms_px", 1.0, "重投影 RMS"),
                                      ("gyro_rms_rad_s", 0.04, "陀螺 RMS"),
                                      ("accel_rms_m_s2", 0.40, "加计 RMS")):
                v = m.get(key)
                if v is None:
                    continue
                flag = "✅" if v <= limit else "❌"
                print(f"  {flag} {label:<12} {v:>10.4f}   门槛 {limit}")
        for r in d.get("failure_reasons", []):
            print(f"  ❌ {r}")
        if d.get("status") == "passed_residual_and_excitation_checks":
            yml = output / "camchain-kalibr.yaml"
            print(f"\n  外参：{yml.relative_to(ROOT)}")
            print("  下一步 —— 生成 OpenVINS 配置：")
            print(f"    python prepare_openvins.py --calibration results/{name}{suffix} \\")
            print("        --template-config ~/vio_ws/final_config \\")
            print(f"        --output ~/calib/ov_config_{name} \\")
            print("        --imu-topic /imu0 --cam-topic /cam0/image_raw")
    return rc


if __name__ == "__main__":
    sys.exit(main())
