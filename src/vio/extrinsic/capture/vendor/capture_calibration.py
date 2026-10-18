#!/home/kxy/miniforge3/envs/vins/bin/python
"""Collect mono8 calibration images from the Orbbec RGB UVC camera.

Kalibr folder layout: OUTPUT/cam0/<host_wall_clock_nanoseconds>.png
Defaults are in calibration_config.py. Driver buffer timestamps are recorded
separately; a monotonic clock flag is NOT proof of a sensor hardware timestamp.
"""

import argparse
from collections import deque
import ctypes
import fcntl
import json
import mmap
import os
from pathlib import Path
import re
import select
import sys
import time

import cv2
import numpy as np

import calibration_config as config


EXPECTED_VENDOR, EXPECTED_PRODUCT = "2bc5", "0511"


class V4L2Control(ctypes.Structure):
    _fields_ = [("id", ctypes.c_uint32), ("value", ctypes.c_int32)]


def _iowr(number, size):
    return 0xC0000000 | (size << 16) | (ord("V") << 8) | number


VIDIOC_G_CTRL = _iowr(27, ctypes.sizeof(V4L2Control))
VIDIOC_S_CTRL = _iowr(28, ctypes.sizeof(V4L2Control))


class Timeval(ctypes.Structure):
    _fields_ = [("seconds", ctypes.c_long), ("microseconds", ctypes.c_long)]


class Timecode(ctypes.Structure):
    _fields_ = [("type", ctypes.c_uint32), ("flags", ctypes.c_uint32),
                ("frames", ctypes.c_uint8), ("seconds", ctypes.c_uint8),
                ("minutes", ctypes.c_uint8), ("hours", ctypes.c_uint8),
                ("userbits", ctypes.c_uint8 * 4)]


class BufferMemory(ctypes.Union):
    _fields_ = [("offset", ctypes.c_uint32), ("userptr", ctypes.c_ulong),
                ("planes", ctypes.c_void_p), ("fd", ctypes.c_int32)]


class V4L2Buffer(ctypes.Structure):
    _fields_ = [("index", ctypes.c_uint32), ("type", ctypes.c_uint32),
                ("bytesused", ctypes.c_uint32), ("flags", ctypes.c_uint32),
                ("field", ctypes.c_uint32), ("timestamp", Timeval),
                ("timecode", Timecode), ("sequence", ctypes.c_uint32),
                ("memory", ctypes.c_uint32), ("m", BufferMemory),
                ("length", ctypes.c_uint32), ("reserved2", ctypes.c_uint32),
                ("request_fd", ctypes.c_int32)]


class PixelFormat(ctypes.Structure):
    _fields_ = [(name, ctypes.c_uint32) for name in
                ("width", "height", "pixelformat", "field", "bytesperline", "sizeimage",
                 "colorspace", "priv", "flags", "ycbcr_enc", "quantization", "xfer_func")]


class FormatUnion(ctypes.Union):
    # v4l2_format's union contains pointer-bearing members; preserve native alignment.
    _fields_ = [("pix", PixelFormat), ("raw", ctypes.c_uint8 * 200), ("align", ctypes.c_void_p)]


class V4L2Format(ctypes.Structure):
    _fields_ = [("type", ctypes.c_uint32), ("fmt", FormatUnion)]


class RequestBuffers(ctypes.Structure):
    _fields_ = [("count", ctypes.c_uint32), ("type", ctypes.c_uint32),
                ("memory", ctypes.c_uint32), ("capabilities", ctypes.c_uint32),
                ("flags", ctypes.c_uint8), ("reserved", ctypes.c_uint8 * 3)]


class Fraction(ctypes.Structure):
    _fields_ = [("numerator", ctypes.c_uint32), ("denominator", ctypes.c_uint32)]


class CaptureParm(ctypes.Structure):
    _fields_ = [("capability", ctypes.c_uint32), ("capturemode", ctypes.c_uint32),
                ("timeperframe", Fraction), ("extendedmode", ctypes.c_uint32),
                ("readbuffers", ctypes.c_uint32), ("reserved", ctypes.c_uint32 * 4)]


class ParmUnion(ctypes.Union):
    _fields_ = [("capture", CaptureParm), ("raw", ctypes.c_uint8 * 200)]


class StreamParm(ctypes.Structure):
    _fields_ = [("type", ctypes.c_uint32), ("parm", ParmUnion)]


def buffer_metadata(buffer):
    clock_bits = buffer.flags & 0xE000
    source_bits = buffer.flags & 0x70000
    return {"v4l2_buffer_timestamp_ns": (int(buffer.timestamp.seconds) * 1_000_000_000
                                          + int(buffer.timestamp.microseconds) * 1000),
            "v4l2_clock": {0x2000: "monotonic", 0x4000: "copied"}.get(clock_bits, "unknown"),
            "v4l2_timestamp_event": {0: "end_of_frame", 0x10000: "start_of_exposure"}.get(
                source_bits, "unknown"), "v4l2_flags": int(buffer.flags),
            "v4l2_sequence": int(buffer.sequence), "sensor_timestamp_ns": None,
            "timestamp_source": "v4l2_driver_buffer_unverified_sensor_time"}


class V4L2Camera:
    """Single-planar MJPEG MMAP capture with metadata from the same dequeued frame."""

    def __init__(self, device, width, height, fps):
        self.fd = None
        self.buffers = []
        self.streaming = False
        try:
            self.fd = os.open(device, os.O_RDWR | os.O_NONBLOCK)
            fmt = V4L2Format(type=1)
            fmt.fmt.pix.width, fmt.fmt.pix.height = width, height
            fmt.fmt.pix.pixelformat = cv2.VideoWriter_fourcc(*"MJPG")
            fmt.fmt.pix.field = 0
            fcntl.ioctl(self.fd, _iowr(5, ctypes.sizeof(fmt)), fmt)
            fcntl.ioctl(self.fd, _iowr(4, ctypes.sizeof(fmt)), fmt)
            parm = StreamParm(type=1)
            parm.parm.capture.timeperframe = Fraction(1, fps)
            fcntl.ioctl(self.fd, _iowr(22, ctypes.sizeof(parm)), parm)
            fcntl.ioctl(self.fd, _iowr(21, ctypes.sizeof(parm)), parm)
            fraction = parm.parm.capture.timeperframe
            actual_fps = fraction.denominator / fraction.numerator if fraction.numerator else 0
            self.mode = {"fourcc": fourcc_text(fmt.fmt.pix.pixelformat),
                         "width": int(fmt.fmt.pix.width), "height": int(fmt.fmt.pix.height),
                         "fps_requested_mode": actual_fps}
            if (self.mode["fourcc"] != "MJPG" or self.mode["width"] != width
                    or self.mode["height"] != height or abs(actual_fps - fps) > 0.51):
                raise RuntimeError(f"V4L2 模式不匹配: {self.mode}")
            req = RequestBuffers(count=4, type=1, memory=1)
            fcntl.ioctl(self.fd, _iowr(8, ctypes.sizeof(req)), req)
            if req.count < 2:
                raise RuntimeError("V4L2 缓冲区不足")
            for index in range(req.count):
                buffer = V4L2Buffer(index=index, type=1, memory=1)
                fcntl.ioctl(self.fd, _iowr(9, ctypes.sizeof(buffer)), buffer)
                self.buffers.append(mmap.mmap(self.fd, buffer.length, flags=mmap.MAP_SHARED,
                                             prot=mmap.PROT_READ | mmap.PROT_WRITE, offset=buffer.m.offset))
                fcntl.ioctl(self.fd, _iowr(15, ctypes.sizeof(buffer)), buffer)
            fcntl.ioctl(self.fd, 0x40045612, ctypes.c_uint32(1))
            self.streaming = True
        except BaseException:
            self.release()
            raise

    def read(self):
        if not select.select([self.fd], [], [], 2.0)[0]:
            raise RuntimeError("V4L2 等待图像超时")
        buffer = V4L2Buffer(type=1, memory=1)
        fcntl.ioctl(self.fd, _iowr(17, ctypes.sizeof(buffer)), buffer)
        host_time = time.time_ns()
        host_mono = time.monotonic_ns()
        try:
            if buffer.flags & 0x40 or not 0 < buffer.bytesused <= buffer.length:
                raise RuntimeError("V4L2 返回损坏或空帧")
            payload = self.buffers[buffer.index][:buffer.bytesused]
            metadata = buffer_metadata(buffer)
        finally:
            fcntl.ioctl(self.fd, _iowr(15, ctypes.sizeof(buffer)), buffer)
        frame = cv2.imdecode(np.frombuffer(payload, dtype=np.uint8), cv2.IMREAD_COLOR)
        if frame is None:
            raise RuntimeError("MJPG 帧解码失败")
        metadata.update(host_receive_time_ns=host_time, host_receive_monotonic_ns=host_mono)
        return frame, metadata

    def release(self):
        if self.fd is not None and self.streaming:
            try:
                fcntl.ioctl(self.fd, 0x40045613, ctypes.c_uint32(1))
            except OSError:
                pass
        self.streaming = False
        for buffer in self.buffers:
            buffer.close()
        self.buffers.clear()
        if self.fd is not None:
            os.close(self.fd)
            self.fd = None


class OpenCVCamera:
    def __init__(self, device, width, height, fps):
        self.cap, self.mode = open_camera(device, width, height, fps)

    def read(self):
        ok, frame = self.cap.read()
        if not ok or frame is None:
            raise RuntimeError("OpenCV 读取图像失败")
        return frame, {"host_receive_time_ns": time.time_ns(),
                       "host_receive_monotonic_ns": time.monotonic_ns(),
                       "sensor_timestamp_ns": None, "v4l2_buffer_timestamp_ns": None,
                       "timestamp_source": "host_receive_only"}

    def release(self):
        self.cap.release()


def open_backend(device, args):
    if args.backend != "opencv":
        try:
            camera = V4L2Camera(device, args.width, args.height, args.fps)
            print(f"直接 V4L2 模式回读: {camera.mode}")
            return camera, "v4l2"
        except (OSError, RuntimeError, ValueError) as exc:
            if args.backend == "v4l2":
                raise
            print(f"警告: 直接 V4L2 不可用 ({exc})，退回 OpenCV，仅记录主机时间", file=sys.stderr)
    return OpenCVCamera(device, args.width, args.height, args.fps), "opencv"

CONTROLS = {
    "exposure_auto": 0x009A0901,  # V4L2_EXPOSURE_MANUAL=1
    "exposure": 0x009A0902,       # absolute exposure: units of 100 us
    "auto_white_balance": 0x0098090C,
    "white_balance_temperature": 0x0098091A,
    "gain": 0x00980913,
    "brightness": 0x00980900,
    "contrast": 0x00980901,
    "sharpness": 0x0098091B,
    "power_line_frequency": 0x00980918,  # 50 Hz = 1
}


def get_ctrl(fd, cid):
    c = V4L2Control(id=cid)
    fcntl.ioctl(fd, VIDIOC_G_CTRL, c)
    return int(c.value)


def set_ctrl_checked(fd, cid, value, name):
    c = V4L2Control(id=cid, value=value)
    fcntl.ioctl(fd, VIDIOC_S_CTRL, c)
    actual = get_ctrl(fd, cid)  # S_CTRL return alone is not a verification.
    if actual != value:
        raise RuntimeError(f"{name} 请求 {value}，实际 {actual}；拒绝继续采集")
    print(f"  {name}: {actual}")


def identify_camera(device):
    resolved = device.expanduser().resolve(strict=True)
    match = re.fullmatch(r"video(\d+)", resolved.name)
    if not match or resolved.parent != Path("/dev"):
        raise ValueError(f"应指定 /dev/videoN 或指向它的符号链接: {device}")
    video_name = (Path("/sys/class/video4linux") / resolved.name / "name").read_text().strip()
    sysdev = (Path("/sys/class/video4linux") / resolved.name / "device").resolve()
    usb_ids = None
    for directory in (sysdev, *sysdev.parents):
        if (directory / "idVendor").is_file() and (directory / "idProduct").is_file():
            usb_ids = ((directory / "idVendor").read_text().strip().lower(),
                       (directory / "idProduct").read_text().strip().lower())
            break
    if usb_ids != (EXPECTED_VENDOR, EXPECTED_PRODUCT):
        raise RuntimeError(f"{resolved} 是 {video_name} ({usb_ids})，而不是 2bc5:0511；拒绝误用内置相机")
    if (Path("/sys/class/video4linux") / resolved.name / "index").read_text().strip() != "0":
        raise RuntimeError(f"{resolved} 不是采集节点 index=0，可能是 metadata 节点")
    print(f"相机: {resolved} ({video_name}, 2bc5:0511)")
    return resolved


def fourcc_text(raw):
    return "".join(chr((int(raw) >> (8 * shift)) & 0xff) for shift in range(4))


def open_camera(device, width, height, fps):
    cap = cv2.VideoCapture(str(device), cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"无法打开 {device}；关闭 Cheese 等占用该相机的程序后重试")
    try:
        # FOURCC first; OpenCV set() can fail silently, so verify every property.
        cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
        cap.set(cv2.CAP_PROP_FPS, fps)
        actual = {"fourcc": fourcc_text(cap.get(cv2.CAP_PROP_FOURCC)),
                  "width": round(cap.get(cv2.CAP_PROP_FRAME_WIDTH)),
                  "height": round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)),
                  "fps_requested_mode": cap.get(cv2.CAP_PROP_FPS)}
        print(f"模式回读: {actual}")
        if (actual["fourcc"] != "MJPG" or actual["width"] != width
                or actual["height"] != height or abs(actual["fps_requested_mode"] - fps) > 0.51):
            raise RuntimeError("相机实际模式与请求不一致；不会用 YUYV/错误分辨率继续采集")
        return cap, actual
    except BaseException:
        cap.release()
        raise


def lock_controls(fd, args):
    settings = [
        ("exposure_auto", 1),
        ("exposure", args.exposure),
        ("auto_white_balance", 0),
        ("white_balance_temperature", args.white_balance),
        ("gain", args.gain),
        ("brightness", args.brightness),
        ("contrast", args.contrast),
        ("sharpness", args.sharpness),
        ("power_line_frequency", args.power_line_frequency),
    ]
    previous = {name: get_ctrl(fd, CONTROLS[name]) for name, _ in settings}
    try:
        print("锁定控件 (曝光单位: 100 微秒):")
        for name, value in settings:
            set_ctrl_checked(fd, CONTROLS[name], value, name)
    except BaseException:
        restore_controls(fd, previous)
        raise
    return previous, {name: get_ctrl(fd, CONTROLS[name]) for name, _ in settings}


def restore_controls(fd, previous):
    if not previous:
        return {"status": "not_needed", "failed": []}
    # Restore dependent manual values before reenabling the automatic controls.
    order = ["exposure", "white_balance_temperature", "gain", "brightness",
             "contrast", "sharpness", "power_line_frequency",
             "exposure_auto", "auto_white_balance"]
    failed = []
    for name in order:
        try:
            c = V4L2Control(id=CONTROLS[name], value=previous[name])
            fcntl.ioctl(fd, VIDIOC_S_CTRL, c)
            if get_ctrl(fd, CONTROLS[name]) != previous[name]:
                raise OSError("恢复后回读值不一致")
        except OSError as exc:
            failed.append({"control": name, "error": str(exc)})
            print(f"警告: 未能恢复控件 {name}: {exc}", file=sys.stderr)
    return {"status": "partial_failure" if failed else "restored", "failed": failed}


def positive_int(value):
    n = int(value)
    if n < 1:
        raise argparse.ArgumentTypeError("必须为正整数")
    return n


def nonnegative_float(value):
    n = float(value)
    if not 0 <= n < float("inf"):
        raise argparse.ArgumentTypeError("必须为非负有限数")
    return n


def arguments(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    # All editable defaults are shared with the offline script in calibration_config.py.
    parser.add_argument("--device", type=Path, default=config.DEVICE)
    parser.add_argument("--output", type=Path, default=config.SESSION_DIR,
                        help="新采集目录（生成 cam0/*.png）；每次采集请换新目录")
    parser.add_argument("--width", type=positive_int, default=config.WIDTH)
    parser.add_argument("--height", type=positive_int, default=config.HEIGHT)
    parser.add_argument("--fps", type=positive_int, default=config.FPS)
    parser.add_argument("--exposure", type=positive_int, default=config.EXPOSURE, help="100 微秒单位")
    parser.add_argument("--white-balance", type=positive_int, default=config.WHITE_BALANCE)
    parser.add_argument("--gain", type=int, default=config.GAIN)
    parser.add_argument("--brightness", type=int, default=config.BRIGHTNESS)
    parser.add_argument("--contrast", type=int, default=config.CONTRAST)
    parser.add_argument("--sharpness", type=int, default=config.SHARPNESS)
    parser.add_argument("--power-line-frequency", type=int, choices=(0, 1, 2), default=config.POWER_LINE_FREQUENCY)
    parser.add_argument("--cols", type=positive_int, default=config.COLS, help="横向内角点数，必须和实物一致")
    parser.add_argument("--rows", type=positive_int, default=config.ROWS, help="纵向内角点数，必须和实物一致")
    parser.add_argument("--auto-interval", type=nonnegative_float, default=config.AUTO_INTERVAL,
                        help="自动存图间隔秒；0=只用空格键存图")
    parser.add_argument("--duration", type=nonnegative_float, default=config.DURATION,
                        help="运行秒数；0=按 Q/CTRL-C 退出")
    parser.add_argument("--max-images", type=int, default=config.MAX_IMAGES, help="保存张数上限；0=不限")
    parser.add_argument("--no-preview", action="store_true", default=config.NO_PREVIEW,
                        help="无图形界面时使用；配合 --duration")
    parser.add_argument("--preview", action="store_false", dest="no_preview", help="开启预览")
    parser.add_argument("--backend", choices=("auto", "v4l2", "opencv"), default=config.BACKEND)
    parser.add_argument("--read-retries", type=positive_int, default=config.READ_RETRIES)
    parser.add_argument("--retry-interval", type=nonnegative_float, default=config.RETRY_INTERVAL)
    parser.add_argument("--warmup-frames", type=int, default=config.WARMUP_FRAMES)
    args = parser.parse_args(argv)
    if (args.cols is None) != (args.rows is None):
        parser.error("--cols 与 --rows 必须同时提供")
    if args.no_preview and args.duration == 0 and (args.max_images == 0 or args.auto_interval == 0):
        parser.error("--no-preview 需要 --duration，或同时设置 --auto-interval 与 --max-images")
    if args.max_images < 0:
        parser.error("--max-images 不能为负数")
    if not isinstance(args.warmup_frames, int) or args.warmup_frames < 0:
        parser.error("--warmup-frames 必须为非负整数")
    for name in ("width", "height", "fps", "exposure", "white_balance", "read_retries", "cols", "rows"):
        value = getattr(args, name)
        if not isinstance(value, int) or value < (2 if name in ("cols", "rows") else 1):
            parser.error(f"{name} 默认值无效，请检查配置文件")
    for name in ("auto_interval", "duration", "retry_interval"):
        if not 0 <= getattr(args, name) < float("inf"):
            parser.error(f"{name} 必须非负且有限")
    if args.backend not in ("auto", "v4l2", "opencv") or args.power_line_frequency not in (0, 1, 2):
        parser.error("BACKEND 或 POWER_LINE_FREQUENCY 配置无效")
    return args


def detect_board(gray, cols, rows):
    """和离线标定 calibrate_intrinsics.find_corners 保持一致的检测策略。

    原实现只试 findChessboardCornersSB，失败即返回 None。SB 虽然精确，
    但在真实拍摄条件下（轻微模糊、反光、对比度低、板子偏远）比经典检测器
    更容易漏检，导致预览不显示绿点，而同一张图离线标定时其实能被经典
    检测器找到 —— 用户会误以为这张图不能用。

    这里补上经典检测器回退，使预览所见与离线标定结果一致。
    """
    pattern = (cols, rows)
    if hasattr(cv2, "findChessboardCornersSB"):
        # [2026-10-15 修复，见 vendor/PROVENANCE.md]
        # 原来写的是位置参数：cv2.findChessboardCornersSB(gray, pattern, cv2.CALIB_CB_NORMALIZE_IMAGE)
        # 但 Python 签名是 f(image, patternSize[, corners[, flags]]) —— 第 3 个位置参数是
        # **输出数组 corners**，不是 flags。OpenCV 不报错、静默接受，flags 实际用默认值 0，
        # 于是丢掉了 NORMALIZE_IMAGE（直方图归一化）。
        # 后果：暗光/低对比度下图全检不出，而离线标定程序（用 flags= 关键字，正确）能检出 ——
        # 预览和 board_detected 会一直报 no，把人误导成"板子有问题"。
        # 实测：同一批图，flags=0 检出 0/12，flags=NORMALIZE_IMAGE 检出 10/12。
        found, corners = cv2.findChessboardCornersSB(
            gray, pattern, flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
        if found:
            return corners
    flags = cv2.CALIB_CB_ADAPTIVE_THRESH | cv2.CALIB_CB_NORMALIZE_IMAGE
    found, corners = cv2.findChessboardCorners(gray, pattern, flags)
    if not found:
        return None
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 40, 1e-3)
    return cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)


def preview_image(frame, corners, stats, covered):
    height, width = frame.shape[:2]
    scale = min(1.0, 960.0 / width)
    canvas = cv2.resize(frame, (round(width * scale), round(height * scale))) if scale < 1 else frame.copy()
    if corners is not None:
        for point in corners.reshape(-1, 2):
            x, y = point * scale
            cv2.circle(canvas, (round(x), round(y)), 2, (0, 255, 0), -1)
    if covered is not None:
        for row in range(3):
            for col in range(3):
                x = int((col + 0.5) * canvas.shape[1] / 3)
                y = int((row + 0.5) * canvas.shape[0] / 3)
                cv2.circle(canvas, (x, y), 5, (0, 255, 0) if (col, row) in covered else (0, 0, 255), 2)
    cv2.rectangle(canvas, (0, 0), (canvas.shape[1], 90), (0, 0, 0), -1)
    for i, line in enumerate(stats):
        cv2.putText(canvas, line, (10, 25 + i * 25), cv2.FONT_HERSHEY_SIMPLEX,
                    0.60, (255, 255, 255), 2, cv2.LINE_AA)
    return canvas


class PreviewWindow:
    """Tk/Pillow preview because this conda OpenCV build has GUI: NONE."""

    def __init__(self):
        import tkinter as tk
        from PIL import Image, ImageTk
        self.Image = Image
        self.ImageTk = ImageTk
        try:
            self.root = tk.Tk()
        except tk.TclError as exc:
            raise RuntimeError(f"无法打开图形窗口，请检查 DISPLAY 或使用 --no-preview: {exc}") from exc
        self.root.title("Orbbec calibration capture")
        self.label = tk.Label(self.root)
        self.label.pack()
        self.key = -1
        self.closed = False
        self._photo = None
        self.root.bind("<space>", lambda event: self._press(32))
        self.root.bind("<KeyPress-q>", lambda event: self._press(ord('q')))
        self.root.bind("<KeyPress-Q>", lambda event: self._press(ord('q')))
        self.root.protocol("WM_DELETE_WINDOW", self._close)
        self.root.focus_force()

    def _press(self, key):
        self.key = key

    def _close(self):
        self.closed = True

    def show(self, bgr):
        if self.closed:
            return ord('q')
        rgb = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
        self._photo = self.ImageTk.PhotoImage(self.Image.fromarray(rgb))
        self.label.configure(image=self._photo)
        try:
            self.root.update_idletasks()
            self.root.update()
        except Exception as exc:
            raise RuntimeError(f"预览窗口更新失败: {exc}") from exc
        key, self.key = self.key, -1
        return ord('q') if self.closed else key

    def close(self):
        try:
            self.root.destroy()
        except Exception:
            pass


def main(argv=None):
    args = arguments(argv)
    device = identify_camera(args.device)
    output = args.output.expanduser().resolve()
    camera_dir = output / "cam0"
    if (camera_dir.exists() and any(camera_dir.glob("*.png"))) or (output / "capture_manifest.json").exists():
        raise RuntimeError(f"{camera_dir} 已有图片；请为每次采集使用新目录，避免混用不同参数")
    cap = fd = preview = None
    mode = settings = original = None
    backend_name = None
    reconnects = []
    last_metadata = None
    termination = "completed"
    restoration = {"status": "not_attempted", "failed": []}
    frames = 0
    saved = []
    first_frame_time = last_frame_time = None
    recent_times = deque(maxlen=60)
    last_save_time = last_detect_time = 0.0
    corners = None
    covered = set()
    warmup_remaining = args.warmup_frames
    try:
        cap, backend_name = open_backend(device, args)
        mode = cap.mode
        fd = os.open(device, os.O_RDWR | os.O_NONBLOCK)
        original, settings = lock_controls(fd, args)
        if not args.no_preview:
            preview = PreviewWindow()
        camera_dir.mkdir(parents=True, exist_ok=True)
        start = time.monotonic()
        print("警告: 传感器曝光时间尚未验证，sensor_timestamp_ns=null。"
              "本次数据仅用于内参，不能据此完成相机-IMU 时间偏移标定。", file=sys.stderr)
        print("空格键保存，Q 退出。存图是未经缩放/标注的灰度 PNG。")
        while True:
            if args.duration and time.monotonic() - start >= args.duration:
                break
            try:
                frame, metadata = cap.read()
            except (OSError, RuntimeError, cv2.error) as first_error:
                print(f"警告: 读帧失败 ({first_error})，尝试恢复连接", file=sys.stderr)
                cap.release()
                cap = None
                if fd is not None:
                    os.close(fd)
                    fd = None
                for attempt in range(1, args.read_retries + 1):
                    if args.duration and time.monotonic() - start >= args.duration:
                        raise RuntimeError("恢复连接时已达到采集时限，停止采集")
                    time.sleep(args.retry_interval)
                    try:
                        device = identify_camera(args.device)
                        # Preserve backend provenance; do not switch timestamp source mid-run.
                        cap = (V4L2Camera(device, args.width, args.height, args.fps)
                               if backend_name == "v4l2" else
                               OpenCVCamera(device, args.width, args.height, args.fps))
                        fd = os.open(device, os.O_RDWR | os.O_NONBLOCK)
                        _, settings = lock_controls(fd, args)
                        frame, metadata = cap.read()
                        reconnects.append({"attempt": attempt, "host_time_ns": time.time_ns(),
                                           "initial_error": str(first_error)})
                        recent_times.clear()
                        warmup_remaining = args.warmup_frames
                        print(f"重连成功，第 {attempt} 次尝试；已保存图片保留")
                        break
                    except (OSError, RuntimeError, ValueError, cv2.error) as retry_error:
                        if cap is not None:
                            cap.release()
                            cap = None
                        if fd is not None:
                            restore_controls(fd, original)
                            os.close(fd)
                            fd = None
                        print(f"警告: 重连 {attempt}/{args.read_retries} 失败: {retry_error}", file=sys.stderr)
                else:
                    raise RuntimeError(f"{args.read_retries} 次重连失败；已有图片与清单会保留") from first_error
            if warmup_remaining:
                warmup_remaining -= 1
                continue
            timestamp_ns = metadata["host_receive_time_ns"]
            if last_metadata is None:
                print(f"首帧时间信息: {metadata}")
            last_metadata = metadata
            now = time.monotonic()
            if frame.shape[:2] != (args.height, args.width):
                raise RuntimeError(f"实际帧尺寸 {frame.shape[:2]} 与请求不一致")
            frames += 1
            first_frame_time = first_frame_time or now
            last_frame_time = now
            recent_times.append(now)
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            if args.cols and (frames == 1 or now - last_detect_time >= 0.5):
                corners = detect_board(gray, args.cols, args.rows)
                last_detect_time = now
            measured_fps = ((len(recent_times)-1) / (recent_times[-1]-recent_times[0])
                            if len(recent_times) > 1 and recent_times[-1] > recent_times[0] else 0)
            key = -1
            if not args.no_preview:
                board_state = ("N/A" if args.cols is None else
                               "YES" if corners is not None else "NO")
                lines = [f"{args.width}x{args.height} MJPG | read {measured_fps:.1f} fps | saved {len(saved)}",
                         f"Exposure {settings['exposure']} (100us) Gain {settings['gain']} chessboard {args.cols}x{args.rows}: {board_state}",
                         "SPACE: save   Q: quit" + ("   green dots: covered regions" if args.cols else "") ]
                key = preview.show(preview_image(frame, corners, lines, covered if args.cols else None))
            if key in (ord('q'), ord('Q')):
                break
            automatic = args.auto_interval > 0 and (not saved or now - last_save_time >= args.auto_interval)
            if key == 32 or automatic:
                # Re-detect on the exact frame being written, not a previous preview frame.
                # [2026-10-15 修改，见 vendor/PROVENANCE.md] 只在开着预览时做。
                # 这个检测是给预览窗的绿点和 3x3 覆盖度用的；连续录像（--no-preview）时
                # 它对每一帧都跑一次完整棋盘检测（1080p 上 180~330 ms），
                # 会把帧率从 ~19 fps 拖到 ~2.6 fps（实测 9.23 秒只存下 25 张）。
                # 外参标定不需要它 —— 角点由离线标定程序自己检测。
                if args.cols and not args.no_preview:
                    corners = detect_board(gray, args.cols, args.rows)
                    last_detect_time = now
                dest = camera_dir / f"{timestamp_ns}.png"
                if dest.exists():
                    raise RuntimeError("主机时间戳重复，拒绝覆盖已有图片")
                ok, encoded = cv2.imencode(".png", gray)
                if not ok:
                    raise OSError(f"PNG 编码失败: {dest}")
                encoded.tofile(str(dest))
                saved.append({"file": str(dest.relative_to(output)), **metadata,
                              "board_detected": corners is not None})
                last_save_time = now
                if corners is not None:
                    for x, y in corners.reshape(-1, 2):
                        covered.add((min(2, int(3*x/args.width)), min(2, int(3*y/args.height))))
                print(f"保存 {dest} (board={'yes' if corners is not None else 'no'})")
            if args.duration and now - start >= args.duration:
                break
            if args.max_images and len(saved) >= args.max_images:
                break
    except KeyboardInterrupt:
        termination = "user_interrupted"
        print("用户停止采集")
    except BaseException as exc:
        termination = f"failed: {exc}"
        raise
    finally:
        if cap is not None:
            cap.release()
        if preview is not None:
            preview.close()
        if fd is not None:
            restoration = restore_controls(fd, original)
            os.close(fd)
        elif original:
            restoration = {"status": "device_unavailable", "failed": []}
            print("警告: 退出时设备不可用，无法恢复采集前的相机控件", file=sys.stderr)
        if camera_dir.is_dir() and mode is not None and settings is not None:
            elapsed = (last_frame_time - first_frame_time) if frames > 1 else 0.0
            observed = (frames - 1) / elapsed if elapsed > 0 else 0.0
            summary = {"device": str(device), "camera_ids": "2bc5:0511",
                       "mode": mode, "locked_controls": settings, "frames_read": frames,
                       "original_controls": original, "control_restoration": restoration,
                       "backend": backend_name, "termination": termination, "reconnections": reconnects,
                       "timestamp_source": (last_metadata or {}).get("timestamp_source", "no_frames"),
                       "sensor_timestamp_ns": None,
                       "last_frame_timestamp_metadata": last_metadata,
                       "filename_timestamp_source": "host_receive_time_ns_unix",
                       "usable_for_camera_imu_time_calibration": False,
                       "board_inner_corners": [args.cols, args.rows],
                       "elapsed_between_first_last_frames_s": elapsed,
                       "observed_read_fps": observed, "requested_fps": args.fps,
                       "throughput_shortfall_vs_requested_percent": max(0.0, 100 * (1 - observed / args.fps)),
                       "note": "Throughput shortfall is not a true dropped-frame count. "
                               "MONOTONIC specifies a clock, not a verified sensor exposure timestamp. "
                               "Driver times are separate from host times; sensor timestamp is unknown.",
                       "saved": saved, "covered_nine_grid_cells": sorted(covered)}
            with (output / "capture_manifest.json").open("w", encoding="utf-8") as stream:
                json.dump(summary, stream, ensure_ascii=False, indent=2)
            print(f"读到 {frames} 帧，平均 {observed:.1f} fps；"
                  f"相对请求帧率吞吐差 {summary['throughput_shortfall_vs_requested_percent']:.1f}%"
                  "（无法据此精确计算掉帧）；保存 " + str(len(saved)) + " 张")
            print(f"Kalibr 图片目录: {camera_dir}")


if __name__ == "__main__":
    try:
        main()
    except (OSError, RuntimeError, ValueError, cv2.error) as exc:
        print(f"采集失败: {exc}", file=sys.stderr)
        sys.exit(1)
