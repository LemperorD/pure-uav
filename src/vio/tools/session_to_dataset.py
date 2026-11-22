#!/usr/bin/env python3
"""把 camera_imu_calibration 的采集目录（ns、cam0/）转成 vio_online 的离线数据集格式。

vio_online 的 OfflineDataset 要求：
    images.csv: timestamp_us,filename          （相对数据集目录）
    imu.csv   : timestamp_us,gx,gy,gz,ax,ay,az
    images/   : 图片（本项目直接用 cam0/ 软链）

采集目录（capture/record_session.py 产出）：
    images.csv: timestamp,filename             （纳秒，cam0/xxx.png）
    imu.csv   : timestamp,ax,ay,az,gx,gy,gz    ← 注意列顺序和 VIO 数据集不同

用法：
    python tools/session_to_dataset.py \
        --session ~/calib/camera_imu_calibration/data/session18 \
        --out ~/calib/datasets/session18_vio
"""
from __future__ import annotations

import argparse
import csv
import os
from pathlib import Path


def read_rows(path: Path):
    with path.open(newline="", encoding="utf-8-sig") as fh:
        return list(csv.DictReader(fh))


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--session", required=True, type=Path, help="data/sessionXX 目录")
    ap.add_argument("--out", required=True, type=Path, help="输出数据集目录")
    ap.add_argument("--link-name", default="cam0", help="图像子目录名（默认 cam0）")
    args = ap.parse_args()

    session: Path = args.session.expanduser().resolve()
    out: Path = args.out.expanduser().resolve()

    img_csv = session / "images.csv"
    imu_csv = session / "imu.csv"
    if not img_csv.is_file() or not imu_csv.is_file():
        raise SystemExit(f"❌ {session} 缺 images.csv 或 imu.csv")
    images_rows = read_rows(img_csv)
    imu_rows = read_rows(imu_csv)
    if not images_rows or not imu_rows:
        raise SystemExit("❌ 图像或 IMU 为空")

    # 时间戳：ns → µs。拒绝非整数/乱序。
    def to_us(s: str) -> int:
        ns = int(s)
        return ns // 1000

    img = sorted(((to_us(r["timestamp"]), r["filename"].strip()) for r in images_rows))
    imu = sorted((to_us(r["timestamp"]), r) for r in imu_rows)

    if any(b[0] <= a[0] for a, b in zip(img, img[1:])):
        raise SystemExit("❌ 图像时间戳不是严格递增")
    if any(b[0] <= a[0] for a, b in zip(imu, imu[1:])):
        raise SystemExit("❌ IMU 时间戳不是严格递增")

    span_img = (img[-1][0] - img[0][0]) / 1e6
    span_imu = (imu[-1][0] - imu[0][0]) / 1e6
    if imu[0][0] > img[0][0] + 50_000 or imu[-1][0] < img[-1][0] - 50_000:
        print("⚠️ IMU 没有完整覆盖图像时间范围："
              f"img [{img[0][0]}, {img[-1][0]}] imu [{imu[0][0]}, {imu[-1][0]}]")
    # 【2026-10-20 改】原来这里卡"起始时间相差 > 5 s"，但采集流程里 IMU 本来就先起，
    # 之后还要锁控件 + 自动测光 + 曝光验证，第一张图往往在 8~10 s 之后才出现 ——
    # 这个检查会误报"不是同一时钟域"（session28 就是这样被拦下的）。
    # 真正"不同时钟域"会差到 10^15 µs 量级（飞控 time_usec vs 主机 epoch），
    # 所以 60 s 既严得能抓错，又容得下正常的前置时间。同时要求两段有重叠。
    head_gap = abs(img[0][0] - imu[0][0]) / 1e6
    if head_gap > 60.0:
        raise SystemExit(
            f"❌ 图像与 IMU 的起始时间相差 {head_gap:.1f} s（> 60 s），不像同一时钟域")
    if imu[0][0] > img[-1][0] or imu[-1][0] < img[0][0]:
        raise SystemExit("❌ 图像与 IMU 的时间范围完全没有重叠")
    if head_gap > 5.0:
        print(f"ℹ️ IMU 比第一张图早 {head_gap:.1f} s"
              "（预热 + 锁控件 + 自动测光 + 曝光验证），正常；VIO 从第一张图开始初始化")

    out.mkdir(parents=True, exist_ok=True)
    link = out / args.link_name
    if link.exists() or link.is_symlink():
        if not link.is_symlink():
            raise SystemExit(f"❌ {link} 已存在且不是软链，拒绝覆盖")
        link.unlink()
    src_dir = session / args.link_name
    if not src_dir.is_dir():
        raise SystemExit(f"❌ 找不到图像目录 {src_dir}")
    os.symlink(src_dir, link)

    with (out / "images.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["timestamp_us", "filename"])
        for ts, name in img:
            w.writerow([ts, name])

    with (out / "imu.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.writer(fh)
        w.writerow(["timestamp_us", "gx", "gy", "gz", "ax", "ay", "az"])
        for ts, r in imu:
            w.writerow([ts, r["gx"], r["gy"], r["gz"], r["ax"], r["ay"], r["az"]])

    print(f"✅ {out}")
    print(f"   图像 {len(img)} 张 / {span_img:.2f} s（{len(img)/span_img:.1f} Hz）")
    print(f"   IMU  {len(imu)} 条 / {span_imu:.2f} s（{len(imu)/span_imu:.1f} Hz）")
    print(f"   图像目录软链 → {src_dir}")
    print(f"   下一步：./build/bin/vio_offline --dataset {out} --config-dir config/gemini_1920x1080")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
