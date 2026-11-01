#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""kalibr/compare.py —— 对比「自研标定」和「Kalibr」在同一段数据上的结果

两个 yaml 的**格式**不同只是排版：
    自研   T_cam_imu 写成块式   - - 0.0179 …  （yaml.dump 默认）
    Kalibr T_cam_imu 写成流式   - [0.0179, …]
键名、含义、矩阵形状完全一样（都是 cam0 + T_cam_imu(IMU→相机) + timeshift_cam_imu）。
本脚本用 yaml.safe_load 读进来比数值，不受排版影响。

用法：
    conda activate vins
    cd src/vio/extrinsic

    python kalibr/compare.py session16        # 自研 vs Kalibr，同一 session
    python kalibr/compare.py --all            # Kalibr 最新结果 vs 所有自研成功会话
    python kalibr/compare.py session16 --own results/session14/camchain-kalibr.yaml
"""
from __future__ import annotations

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent.parent      # extrinsic/
OWN_RES = ROOT / "results"                          # 自研结果
KALIB_RES = ROOT / "kalibr" / "results"              # 本脚本产出的结果
KALIB_BAGS = ROOT / "kalibr" / "bags"


def find_kalibr_yaml(session: str) -> Path | None:
    for d in (KALIB_RES / session, KALIB_BAGS):
        hits = sorted(d.glob(f"{session}*-camchain-imucam.yaml")) if d.is_dir() else []
        if hits:
            return hits[0]
    return None


def load_T(path: Path) -> tuple[np.ndarray, float | None]:
    d = yaml.safe_load(path.read_text(encoding="utf-8"))
    cam = d["cam0"]
    T = np.asarray(cam["T_cam_imu"], dtype=float)
    ts = cam.get("timeshift_cam_imu")
    return T, (float(ts) if ts is not None else None)


def rot_deg(A: np.ndarray, B: np.ndarray) -> float:
    R = A[:3, :3].T @ B[:3, :3]
    c = (np.trace(R) - 1.0) / 2.0
    return float(np.degrees(np.arccos(np.clip(c, -1.0, 1.0))))


def own_residuals(session: str) -> dict | None:
    rep = OWN_RES / session / "report.json"
    if not rep.is_file():
        return None
    d = json.loads(rep.read_text(encoding="utf-8"))
    m = d.get("final_metrics") or {}
    return {
        "reproj": m.get("reprojection_rms_px"),
        "gyro": m.get("gyro_rms_rad_s"),
        "accel": m.get("accel_rms_m_s2"),
        "status": d.get("status"),
    }


def kalibr_residuals(path: Path) -> dict | None:
    """从 Kalibr 的 *-results-imucam.txt 里抠残差。"""
    txt = path.parent / (path.name.replace("-camchain-imucam.yaml", "-results-imucam.txt"))
    if not txt.is_file():
        return None
    s = txt.read_text(encoding="utf-8", errors="replace")

    def grab(pattern: str) -> float | None:
        m = re.search(pattern, s)
        return float(m.group(1)) if m else None

    return {
        "reproj": grab(r"Reprojection error \(cam0\) \[px\]:\s*mean\s*([0-9.eE+-]+)"),
        "gyro": grab(r"Gyroscope error \(imu0\) \[rad/s\]:\s*mean\s*([0-9.eE+-]+)"),
        "accel": grab(r"Accelerometer error \(imu0\) \[m/s\^2\]:\s*mean\s*([0-9.eE+-]+)"),
        "status": "ok",
    }


def rel(p: Path) -> str:
    """能相对就相对显示，否则给绝对路径。"""
    try:
        return str(p.relative_to(ROOT))
    except ValueError:
        return str(p)


def compare(kalib_yaml: Path, own_yaml: Path, label: str) -> None:
    Tk, tsk = load_T(kalib_yaml)
    To, tso = load_T(own_yaml)
    dR = rot_deg(Tk, To)
    dt = To[:3, 3] - Tk[:3, 3]

    print(f"\n=== {label} ===")
    print(f"  Kalibr: {rel(kalib_yaml)}")
    print(f"  自研  : {rel(own_yaml)}")
    print(f"  旋转差 : {dR:8.2f} °")
    print(f"  平移差 : {np.linalg.norm(dt) * 1000:8.1f} mm"
          f"   (dx/dy/dz = {dt[0]*1000:+.1f} / {dt[1]*1000:+.1f} / {dt[2]*1000:+.1f} mm)")
    if tsk is not None and tso is not None:
        print(f"  时偏差 : {(tso - tsk) * 1000:+8.1f} ms"
              f"   (自研 {tso*1000:+.1f} ms, Kalibr {tsk*1000:+.1f} ms)")
    print("  T_cam_imu（自研 / Kalibr）：")
    for i in range(3):
        a = "  ".join(f"{v: .6f}" for v in To[i])
        b = "  ".join(f"{v: .6f}" for v in Tk[i])
        print(f"    [{a}]   [{b}]")


def main() -> int:
    ap = argparse.ArgumentParser(description="对比自研标定与 Kalibr 的外参结果")
    ap.add_argument("session", nargs="?", help="session 名（如 session16）")
    ap.add_argument("--own", type=Path, help="自研 camchain-kalibr.yaml 路径")
    ap.add_argument("--kalibr", type=Path, help="Kalibr *-camchain-imucam.yaml 路径")
    ap.add_argument("--all", action="store_true",
                    help="用最新的 Kalibr 结果对比所有自研成功过的 session")
    args = ap.parse_args()

    # 1) 指定了路径
    if args.own and args.kalibr:
        compare(args.kalibr, args.own, "自定义对比")
        return 0

    # 2) --all：拿最新 Kalibr 结果，对每个有 camchain-kalibr.yaml 的 session
    if args.all:
        kalibs = sorted(KALIB_RES.glob("*/*-camchain-imucam.yaml"),
                        key=lambda p: p.stat().st_mtime, reverse=True)
        if not kalibs:
            sys.exit("kalibr/results/ 下还没有 Kalibr 结果，先跑 run_kalibr.py")
        kalib_yaml = kalibs[0]
        print(f"Kalibr 结果：{rel(kalib_yaml)}")
        for d in sorted(OWN_RES.glob("session*")):
            own = d / "camchain-kalibr.yaml"
            if own.is_file():
                compare(kalib_yaml, own, f"{kalib_yaml.stem.split('-')[0]} vs {d.name}")
        return 0

    # 3) 同一 session
    if not args.session:
        sys.exit("给一个 session，例如：python kalibr/compare.py session16")
    s = args.session
    kalib_yaml = args.kalibr or find_kalibr_yaml(s)
    own_yaml = args.own or (OWN_RES / s / "camchain-kalibr.yaml")

    if kalib_yaml is None:
        sys.exit(f"没找到 {s} 的 Kalibr 结果。先跑：\n"
                 f"    python kalibr/run_kalibr.py {s}")
    if not own_yaml.is_file():
        sys.exit(f"没找到自研结果 {own_yaml}")
    compare(kalib_yaml, own_yaml, s)

    ka = kalibr_residuals(kalib_yaml)
    oa = own_residuals(s)
    print("\n  残差（注意：两套定义/筛选不同，只作参考）：")
    print(f"    {'':<10}{'reproj[px]':>12}{'gyro[rad/s]':>14}{'accel[m/s²]':>14}   status")
    for name, r in (("自研", oa), ("Kalibr", ka)):
        if not r:
            print(f"    {name:<10}{'—':>12}{'—':>14}{'—':>14}")
            continue
        f = lambda v, fmt: ("—" if v is None else format(v, fmt))
        print(f"    {name:<10}{f(r['reproj'], '.3f'):>12}{f(r['gyro'], '.5f'):>14}"
              f"{f(r['accel'], '.4f'):>14}   {r['status']}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
