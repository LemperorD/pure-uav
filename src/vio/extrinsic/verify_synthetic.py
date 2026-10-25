#!/usr/bin/env python3
"""兼容入口 —— 实现在 tools/verify_synthetic.py（合成数据端到端自检，真值独立生成）。

保留本文件只为老命令不变：`python verify_synthetic.py`
等价于 `python tools/verify_synthetic.py`。
"""
import sys

from tools.verify_synthetic import main

if __name__ == "__main__":
    sys.exit(main())
