#!/usr/bin/env python3
"""把 ROS1 bag 转成 vio_offline 使用的数据集。

输出目录结构：
    <out>/
      images.csv : timestamp_us,filename
      imu.csv    : timestamp_us,gx,gy,gz,ax,ay,az
      images/000000.png ...

时间戳保持 bag 里 header.stamp 的原始值（session05 是 Unix 微秒），
不做任何时钟重映射——离线验证估计器和坐标转换已经够用。

依赖：conda activate vins  （rosbags + opencv + numpy）

用法：
    python bag_to_dataset.py --bag ~/calib/rosbags/session05.bag \
                             --out ~/calib/datasets/session05
"""

import argparse
import os
import sys
from pathlib import Path

import cv2
import numpy as np
from rosbags.rosbag1 import Reader
from rosbags.typesys import Stores, get_typestore


def stamp_us(stamp) -> int:
    """ROS time -> 微秒（四舍五入）。"""
    return int(round(stamp.sec * 1_000_000 + stamp.nanosec * 1e-3))


def main() -> int:
    ap = argparse.ArgumentParser(description="ROS1 bag -> VIO offline dataset")
    ap.add_argument("--bag", required=True, help="输入 ROS1 bag")
    ap.add_argument("--out", required=True, help="输出数据集目录")
    ap.add_argument("--imu-topic", default="/imu0")
    ap.add_argument("--image-topic", default="/cam0/image_raw")
    ap.add_argument("--image-format", choices=["png", "jpg"], default="png")
    ap.add_argument("--max-images", type=int, default=0, help="0 表示全部")
    ap.add_argument("--start-sec", type=float, default=0.0,
                    help="从 bag 开始后多少秒开始导出（默认 0）")
    args = ap.parse_args()

    out_dir = Path(args.out).expanduser().resolve()
    img_dir = out_dir / "images"
    img_dir.mkdir(parents=True, exist_ok=True)

    out_dir.mkdir(parents=True, exist_ok=True)

    store = get_typestore(Stores.ROS1_NOETIC)

    imu_rows = []
    image_rows = []
    first_img_ts = None
    first_imu_ts = None
    shape = None

    with Reader(args.bag) as reader:
        topics = {c.topic for c in reader.connections}
        if args.imu_topic not in topics:
            print(f"[错误] bag 里没有 {args.imu_topic}，实际 topics: {sorted(topics)}")
            return 1
        if args.image_topic not in topics:
            print(f"[错误] bag 里没有 {args.image_topic}，实际 topics: {sorted(topics)}")
            return 1

        # 先扫一遍确定起始时间
        bag_start_ns = None
        for conn, t_ns, raw in reader.messages():
            bag_start_ns = t_ns
            break
        if bag_start_ns is None:
            print("[错误] 空 bag")
            return 1
        t_begin_ns = bag_start_ns + int(args.start_sec * 1e9)

        for conn, t_ns, raw in reader.messages():
            if t_ns < t_begin_ns:
                continue
            if conn.topic == args.imu_topic:
                msg = store.deserialize_ros1(raw, conn.msgtype)
                ts = stamp_us(msg.header.stamp)
                av = msg.angular_velocity
                la = msg.linear_acceleration
                imu_rows.append((ts, av.x, av.y, av.z, la.x, la.y, la.z))
                if first_imu_ts is None:
                    first_imu_ts = ts
            elif conn.topic == args.image_topic:
                if args.max_images > 0 and len(image_rows) >= args.max_images:
                    continue
                msg = store.deserialize_ros1(raw, conn.msgtype)
                ts = stamp_us(msg.header.stamp)
                if msg.height * msg.width <= 0:
                    continue
                h, w = msg.height, msg.width
                shape = (h, w)
                enc = msg.encoding.lower()
                buf = np.frombuffer(msg.data, dtype=np.uint8)

                if enc in ("mono8", "8uc1"):
                    img = buf.reshape(h, w)
                elif enc in ("bgr8", "rgb8"):
                    img = buf.reshape(h, w, 3)
                    if enc == "rgb8":
                        img = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    img = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
                elif enc in ("mono16", "16uc1"):
                    img16 = buf.view(np.uint16).reshape(h, w)
                    img = cv2.convertScaleAbs(img16, alpha=1.0 / 256.0)
                else:
                    print(f"[错误] 不支持的图像编码: {msg.encoding}")
                    return 1

                name = f"{len(image_rows):06d}.{args.image_format}"
                path = img_dir / name
                if args.image_format == "png":
                    ok = cv2.imwrite(str(path), img, [cv2.IMWRITE_PNG_COMPRESSION, 3])
                else:
                    ok = cv2.imwrite(str(path), img, [cv2.IMWRITE_JPEG_QUALITY, 95])
                if not ok:
                    print(f"[错误] 写图失败: {path}")
                    return 1
                image_rows.append((ts, f"images/{name}"))
                if first_img_ts is None:
                    first_img_ts = ts

    if not image_rows:
        print("[错误] 没有导出任何图像")
        return 1
    if not imu_rows:
        print("[错误] 没有导出任何 IMU")
        return 1

    image_rows.sort()
    imu_rows.sort()

    with open(out_dir / "images.csv", "w") as f:
        f.write("timestamp_us,filename\n")
        for ts, name in image_rows:
            f.write(f"{ts},{name}\n")

    with open(out_dir / "imu.csv", "w") as f:
        f.write("timestamp_us,gx,gy,gz,ax,ay,az\n")
        for row in imu_rows:
            f.write(",".join(str(x) if i == 0 else f"{x:.9f}" for i, x in enumerate(row)) + "\n")

    dur = (max(image_rows[-1][0], imu_rows[-1][0]) - min(image_rows[0][0], imu_rows[0][0])) * 1e-6
    print("完成:")
    print(f"  输出目录 : {out_dir}")
    print(f"  图像     : {len(image_rows)} 帧, {shape[1]}x{shape[0]}, {len(image_rows)/dur if dur else 0:.1f} fps")
    print(f"  IMU      : {len(imu_rows)} 帧, {len(imu_rows)/dur if dur else 0:.1f} Hz")
    print(f"  时长     : {dur:.2f} s")
    print(f"  图像时间 : [{image_rows[0][0]*1e-6:.6f}, {image_rows[-1][0]*1e-6:.6f}] s")
    print(f"  IMU 时间 : [{imu_rows[0][0]*1e-6:.6f}, {imu_rows[-1][0]*1e-6:.6f}] s")
    return 0


if __name__ == "__main__":
    sys.exit(main())
