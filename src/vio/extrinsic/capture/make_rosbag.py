#!/usr/bin/env python3
"""把一个采集 session 打成 ROS 1 bag，喂给 OpenVINS。

为什么是 ROS 1 bag（不是 ROS 2）：
    OpenVINS 有两种跑法：
      · subscribe 模式：ros2 bag play + 订阅话题 —— 你自己在
        ~/vio_exp/run_openvins_serial.sh 的注释里记过，实测会发散
        （ROS2 版 IMU 订阅队列深度只有 5，EuRoC 每 50ms 突发 10 条正好撑爆）
      · serial 模式：节点直接读 bag 文件，绕开消息传输 —— 官方评估 EuRoC 用的就是这个
    而 serial 模式（ros1_serial_msckf）只认 ROS 1 bag。所以打 ROS1 的。

为什么 mono8：
    OpenVINS 的特征跟踪只用灰度。彩色存成 bgr8 体积是 3 倍，白占磁盘。

为什么时间戳直接用 CSV 里的：
    采集时相机和 IMU 都打在宿主 CLOCK_REALTIME 上，标定用的就是这两个时间戳，
    估出来的 timeshift_cam_imu 就是针对它们的。bag 里必须原样保留，
    否则标定出来的时移立刻失效。

用法（PyCharm 里直接点运行；也可以在终端跑）：
    python capture/make_rosbag.py                    # 用最新的 session
    python capture/make_rosbag.py --session session05
    python capture/make_rosbag.py --list             # 看有哪些 session
"""
from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import cv2
import numpy as np

ROOT = Path(__file__).resolve().parent.parent
DATA_ROOT = ROOT.parent / "data"       # src/vio/data/
DATA = DATA_ROOT / "calib"             # 打 bag 只为 Kalibr → 只看标定板数据
DEFAULT_SESSION = None          # None = 自动挑最新的
BAG_DIR = ROOT / "bags"          # 容器里看到的是 extrinsic/bags/
CAM_TOPIC = "/cam0/image_raw"
IMU_TOPIC = "/imu0"


def find_sessions() -> list[str]:
    """data/calib/ 下同时有 images.csv 和 imu.csv 的目录，按时间从新到旧。"""
    if not DATA.is_dir():
        return []
    out = [(p.stat().st_mtime, p.name) for p in DATA.iterdir()
           if p.is_dir() and (p / "images.csv").is_file() and (p / "imu.csv").is_file()]
    return [name for _, name in sorted(out, reverse=True)]


def read_images_csv(session: Path) -> list[tuple[int, Path]]:
    rows: list[tuple[int, Path]] = []
    with (session / "images.csv").open(newline="") as fh:
        for row in csv.DictReader(fh):
            stamp = int(row["timestamp"])
            name = row["filename"].strip()
            path = Path(name)
            if not path.is_absolute():
                path = session / path
            rows.append((stamp, path))
    rows.sort(key=lambda r: r[0])
    return rows


def read_imu_csv(session: Path) -> np.ndarray:
    """返回 (N,7)：timestamp_ns, ax, ay, az, gx, gy, gz。单位 m/s^2 和 rad/s。"""
    data = np.loadtxt(session / "imu.csv", delimiter=",", skiprows=1)
    data = np.atleast_2d(data)
    order = np.argsort(data[:, 0], kind="stable")
    return data[order]


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session", default=DEFAULT_SESSION,
                    help="session 名（默认取最新）")
    ap.add_argument("--output", type=Path, default=None,
                    help=f"bag 输出路径（默认 {BAG_DIR}/<session>.bag）")
    ap.add_argument("--list", action="store_true", help="列出可用的 session 后退出")
    args = ap.parse_args()

    sessions = find_sessions()
    if args.list:
        print("可用的 session（从新到旧）：")
        for name in sessions:
            print(f"  {name}")
        return 0
    if not sessions:
        print(f"❌ {DATA} 下没有找到同时含 images.csv 和 imu.csv 的 session")
        return 2

    name = args.session or sessions[0]
    session = DATA / name
    if not session.is_dir():
        print(f"❌ 没有这个 session：{session}")
        print(f"   可用的：{'  '.join(sessions)}")
        return 2
    out = args.output or (BAG_DIR / f"{name}.bag")

    images = read_images_csv(session)
    imu = read_imu_csv(session)
    if not images or len(imu) == 0:
        print(f"❌ 数据为空：{len(images)} 张图 / {len(imu)} 条 IMU")
        return 2

    print("=" * 72)
    print(" 打包 ROS 1 bag（喂 OpenVINS）")
    print("=" * 72)
    print(f"  session   {name}")
    print(f"  图像      {len(images)} 张")
    print(f"  IMU       {len(imu)} 条")
    print(f"  输出      {out}")

    # --- 检查第一张图，确定编码参数 ---
    first = cv2.imread(str(images[0][1]), cv2.IMREAD_GRAYSCALE)
    if first is None:
        print(f"❌ 读不出第一张图：{images[0][1]}")
        return 2
    height, width = first.shape[:2]
    print(f"  分辨率    {width}x{height} mono8")

    # --- 时间跨度检查：相机和 IMU 必须落在同一个时钟域 ---
    cam_first, cam_last = images[0][0], images[-1][0]
    imu_first, imu_last = int(imu[0, 0]), int(imu[-1, 0])
    span = (cam_last - cam_first) / 1e9
    print(f"  相机跨度  {span:.2f} s")
    print(f"  IMU 跨度  {(imu_last - imu_first)/1e9:.2f} s")
    if imu_first > cam_first or imu_last < cam_last:
        print("  ⚠️ IMU 没有完整覆盖相机的时间范围；OpenVINS 首尾可能初始化不了")
    # 两端差超过 1e15 ns（约 11.6 天）说明根本不是一个时钟域
    if abs(cam_first - imu_first) > 1e15:
        print("❌ 相机和 IMU 的时间戳不在同一个时钟域（相差超过 11 天）")
        print("   检查 capture 时是不是用了飞控的 time_usec 而不是宿主 time.time_ns()")
        return 2

    from rosbags.rosbag1 import Writer
    from rosbags.typesys import Stores, get_typestore

    ts = get_typestore(Stores.ROS1_NOETIC)
    Image = ts.types["sensor_msgs/msg/Image"]
    Imu = ts.types["sensor_msgs/msg/Imu"]
    Header = ts.types["std_msgs/msg/Header"]
    Time = ts.types["builtin_interfaces/msg/Time"]
    Vec3 = ts.types["geometry_msgs/msg/Vector3"]
    Quat = ts.types["geometry_msgs/msg/Quaternion"]

    def header(stamp_ns: int, frame_id: str = ""):
        return Header(seq=0,
                      stamp=Time(sec=stamp_ns // 1_000_000_000,
                                 nanosec=stamp_ns % 1_000_000_000),
                      frame_id=frame_id)

    out.parent.mkdir(parents=True, exist_ok=True)
    if out.exists():
        out.unlink()

    n_img = n_imu = 0
    with Writer(out) as writer:
        cam_conn = writer.add_connection(CAM_TOPIC, Image.__msgtype__, typestore=ts)
        imu_conn = writer.add_connection(IMU_TOPIC, Imu.__msgtype__, typestore=ts)

        # IMU 先写（时间戳更早），bag 里按时间顺序更整洁
        for row in imu:
            stamp = int(row[0])
            msg = Imu(
                header=header(stamp, "imu0"),
                orientation=Quat(x=0.0, y=0.0, z=0.0, w=1.0),
                # 第 0 项置 -1 = "本字段无效"。OpenVINS 不看它，
                # 用的是 kalibr_imu_chain.yaml 里的噪声参数。
                orientation_covariance=np.array([-1.0] + [0.0] * 8, dtype=np.float64),
                angular_velocity=Vec3(x=float(row[4]), y=float(row[5]), z=float(row[6])),
                angular_velocity_covariance=np.array([-1.0] + [0.0] * 8, dtype=np.float64),
                linear_acceleration=Vec3(x=float(row[1]), y=float(row[2]), z=float(row[3])),
                linear_acceleration_covariance=np.array([-1.0] + [0.0] * 8, dtype=np.float64),
            )
            writer.write(imu_conn, stamp, ts.serialize_ros1(msg, Imu.__msgtype__))
            n_imu += 1

        for i, (stamp, path) in enumerate(images):
            gray = cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)
            if gray is None:
                print(f"  ⚠️ 跳过读不出的图：{path.name}")
                continue
            if gray.shape[:2] != (height, width):
                print(f"  ⚠️ 跳过分辨率不一致的图：{path.name} {gray.shape}")
                continue
            msg = Image(
                header=header(stamp, "cam0"),
                height=height, width=width,
                encoding="mono8", is_bigendian=0, step=width,
                data=gray.reshape(-1),
            )
            writer.write(cam_conn, stamp, ts.serialize_ros1(msg, Image.__msgtype__))
            n_img += 1
            if (i + 1) % 50 == 0:
                print(f"    写入 {i+1}/{len(images)} …")

    size_mb = out.stat().st_size / 1024 / 1024
    print()
    print(f"✅ 写完：{n_img} 张图 + {n_imu} 条 IMU，{size_mb:.0f} MB")

    # --- 回读校验（打不开的 bag 比没有 bag 更浪费时间）---
    from rosbags.rosbag1 import Reader
    with Reader(out) as reader:
        topics = {c.topic: c.msgcount for c in reader.connections}
        print(f"   回读校验：{topics}")

    print()
    print("下一步 —— 生成 OpenVINS 配置（如果还没生成）：")
    print(f"  python tools/prepare_openvins.py --calibration results/{name}_window \\")
    print("      --template-config ~/vio_ws/final_config \\")
    print(f"      --output ~/calib/ov_config_{name} \\")
    print(f"      --imu-topic {IMU_TOPIC} --cam-topic {CAM_TOPIC}")
    print()
    print("再下一步 —— 跑 OpenVINS：用上面生成的配置（导出参数的含义见 tools/prepare_openvins.py 顶部说明）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
