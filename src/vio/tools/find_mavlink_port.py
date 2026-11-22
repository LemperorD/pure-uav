#!/usr/bin/env python3
"""找出哪个串口在讲 MAVLink（CRC 校验过的真帧，不是靠猜字节）。

用法：  python3 tools/find_mavlink_port.py [每个口监听秒数，默认 3]

为什么需要它：CUAV 7-Nano 会同时枚举出 ttyACM0 和 ttyACM1，但 MAVLink 不一定在
ACM0 上；而且飞控刚上电的头一两秒可能还在初始化。采集脚本的 preflight 每个口
只听 1.5 秒，撞上这两种情况就会报"静默"，看起来像"东西坏了"。

不依赖 pyserial：和 camera_imu_calibration 的采集脚本一样用 termios + os.read，
MAVLink 解析复用仓库里的 mavlink_min.MavlinkParser（带 CRC 校验）。
"""
from __future__ import annotations

import glob
import os
import select
import sys
import termios
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "camera_imu_calibration" / "capture"))
sys.path.insert(0, str(HERE.parent / "camera_imu_calibration" / "capture" / "vendor"))
try:
    from mavlink_min import MavlinkParser  # noqa: E402
except Exception as exc:  # noqa: BLE001
    print(f"导入 mavlink_min 失败：{exc}")
    print("（应该从 ~/calib 这个位置运行，或者仓库被挪过）")
    raise SystemExit(2)


def open_port(path: str, baud: int = 115200) -> int:
    fd = os.open(path, os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    attrs = termios.tcgetattr(fd)
    attrs[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    speed = getattr(termios, f"B{baud}")
    attrs[4] = attrs[5] = speed
    termios.tcsetattr(fd, termios.TCSANOW, attrs)
    termios.tcflush(fd, termios.TCIOFLUSH)
    return fd


def probe(path: str, seconds: float) -> dict:
    out = {"port": path, "bytes": 0, "frames": 0, "crc_bad": 0, "ids": set(), "err": None}
    try:
        fd = open_port(path)
    except Exception as exc:  # noqa: BLE001
        out["err"] = f"{type(exc).__name__}: {exc}"
        return out

    parser = MavlinkParser(verify_crc=True)
    t0 = time.monotonic()
    try:
        while time.monotonic() - t0 < seconds:
            ready, _, _ = select.select([fd], [], [], 0.1)
            if not ready:
                continue
            try:
                chunk = os.read(fd, 4096)
            except BlockingIOError:
                continue
            except OSError:
                break
            if not chunk:
                continue
            out["bytes"] += len(chunk)
            for frame in parser.feed(chunk):
                out["frames"] += 1
                out["ids"].add(frame.msgid)
    finally:
        try:
            os.close(fd)
        except OSError:
            pass
    out["crc_bad"] = parser.stats.get("crc_bad", 0)
    return out


def main() -> int:
    seconds = float(sys.argv[1]) if len(sys.argv) > 1 else 3.0
    ports = sorted(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*"))
    if not ports:
        print("没有 ttyACM*/ttyUSB* —— 飞控插好了吗？换数据线/换 USB 口。")
        return 2

    print(f"找到 {len(ports)} 个串口：{', '.join(ports)}")
    print(f"每个监听 {seconds:g} 秒（USB CDC 下波特率无意义，统一 115200）……\n")

    good = []
    for p in ports:
        s = probe(p, seconds)
        if s["err"]:
            print(f"  {p:<16} ✗ 打不开：{s['err']}（权限？被占用？）")
            continue
        if s["frames"] == 0:
            print(f"  {p:<16} ✗ {s['bytes']} 字节，0 个有效 MAVLink 帧"
                  f"（CRC 坏 {s['crc_bad']}）")
            continue
        ids = sorted(s["ids"])
        rate = s["frames"] / seconds
        mark = "✅" if 0 in s["ids"] else "?"
        print(f"  {p:<16} {mark} {s['frames']} 帧 / {rate:.0f} Hz，msgid={ids[:8]}"
              f"（含心跳={'是' if 0 in s['ids'] else '否'}）")
        if 0 in s["ids"]:
            good.append(p)

    print()
    if not good:
        print("没有端口在发 HEARTBEAT。按顺序查：")
        print("  1) 飞控刚上电？等 5 秒重跑（初始化期间不发）")
        print("  2) 有程序占着串口？ps aux | grep -E 'vio_live|record_session'")
        print("  3) 换 USB 数据线 / 换口（避开 hub）")
        print("  4) 飞控是不是进了 bootloader（灯）")
        return 1

    print("在 camera_imu_calibration/capture/capture_config.py 里写死：")
    print(f'    IMU_PORT = "{good[0]}"')
    if len(good) > 1:
        print(f"（另外 {', '.join(good[1:])} 也在发心跳，挑一个即可）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
