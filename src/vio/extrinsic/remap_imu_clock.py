#!/usr/bin/env python3
"""兼容入口 —— 实现在 tools/remap_imu_clock.py（用同步锚点把 IMU 时钟映射到图像时钟）。

保留本文件只为老命令不变：`python remap_imu_clock.py --input ... --anchors ...`
等价于 `python tools/remap_imu_clock.py ...`。
"""
import sys

from tools.remap_imu_clock import fit_clock  # noqa: F401  兼容旧的模块级引用（tests 等）
from tools.remap_imu_clock import main

if __name__ == "__main__":
    sys.exit(main())
