#!/usr/bin/env python3
"""兼容入口 —— 实现在 tools/verify_synthetic.py（合成数据端到端自检，真值独立生成）。

保留本文件只为老命令不变：`python verify_synthetic.py`
等价于 `python tools/verify_synthetic.py`。
"""
import sys

from tools.verify_synthetic import main
from tools.verify_synthetic import ROOT, compare, generate  # noqa: F401  兼容旧的模块级引用（verify_review.py）

if __name__ == "__main__":
    sys.exit(main())
