#!/usr/bin/env python3
"""兼容入口 —— 实现在 tools/prepare_openvins.py（把标定结果导出成 OpenVINS 配置）。

保留本文件只为老命令不变：`python prepare_openvins.py --calibration ... --template-config ... --output ...`
等价于 `python tools/prepare_openvins.py ...`。
"""
import sys

from tools.prepare_openvins import main
from tools.prepare_openvins import prepare  # noqa: F401  兼容旧的模块级引用（tests 等）

if __name__ == "__main__":
    sys.exit(main())
