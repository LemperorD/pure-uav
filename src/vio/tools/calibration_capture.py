"""Locate the optional calibration repository's direct-capture tools."""

import os
from pathlib import Path


VIO_ROOT = Path(__file__).resolve().parents[1]
REPO_ROOT = VIO_ROOT.parents[1]
_REPO = os.environ.get("VIO_CALIBRATION_REPO")
CALIB_REPO = Path(_REPO).expanduser() if _REPO else (REPO_ROOT.parent / "calibration_for_vio")
CAPTURE_DIR = CALIB_REPO / "extrinsic" / "capture"
CAPTURE_SCRIPT = CAPTURE_DIR / "capture_calibration.py"


def require_capture_dir():
    if not CAPTURE_SCRIPT.is_file():
        raise SystemExit(f"找不到采集脚本 {CAPTURE_SCRIPT}；"
                         "请将 VIO_CALIBRATION_REPO 指向 calibration_for_vio 的仓库根目录。")
    return CAPTURE_DIR
