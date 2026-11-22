"""Locate the optional calibration repository's direct-capture tools."""

import os
from pathlib import Path


VIO_ROOT = Path(__file__).resolve().parents[1]
_REPO = os.environ.get("VIO_CALIBRATION_REPO")
CAPTURE_DIR = (Path(_REPO).expanduser() / "extrinsic" / "capture"
               if _REPO else VIO_ROOT / "camera_imu_calibration" / "capture")
CAPTURE_SCRIPT = CAPTURE_DIR / "vendor" / "capture_calibration.py"


def require_capture_dir():
    if not CAPTURE_SCRIPT.is_file():
        raise SystemExit(f"找不到采集脚本 {CAPTURE_SCRIPT}；"
                         "请将 VIO_CALIBRATION_REPO 指向 calibration_for_vio 的仓库根目录。")
    return CAPTURE_DIR
