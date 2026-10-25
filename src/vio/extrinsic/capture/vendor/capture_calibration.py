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
import math
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


class V4L2QueryCtrl(ctypes.Structure):
    """struct v4l2_queryctrl —— 用来问驱动"这个控件真实的范围是多少"。"""
    _fields_ = [("id", ctypes.c_uint32), ("type", ctypes.c_uint32),
                ("name", ctypes.c_char * 32),
                ("minimum", ctypes.c_int32), ("maximum", ctypes.c_int32),
                ("step", ctypes.c_int32), ("default_value", ctypes.c_int32),
                ("flags", ctypes.c_uint32), ("reserved", ctypes.c_uint32 * 2)]


def _iowr(number, size):
    return 0xC0000000 | (size << 16) | (ord("V") << 8) | number


VIDIOC_G_CTRL = _iowr(27, ctypes.sizeof(V4L2Control))
VIDIOC_S_CTRL = _iowr(28, ctypes.sizeof(V4L2Control))
VIDIOC_QUERYCTRL = _iowr(36, ctypes.sizeof(V4L2QueryCtrl))
V4L2_CTRL_FLAG_DISABLED = 0x0001


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
    "backlight_compensation": 0x0098091C,
    "gamma": 0x00980910,
}

# 少一个就拒绝采集：没有它们，"这次的亮度"就没有任何保证。
REQUIRED_SETTINGS = ("exposure_auto", "exposure", "auto_white_balance",
                     "white_balance_temperature", "gain")

# 相机输出是 gamma 编码的（实测 γ≈2.4：2 倍曝光在原始灰度域只有 1.330 ≈ 2**(1/2.4)）。
# 任何"亮度/光强"的比较都必须先换到线性域，否则结论全是错的。
GAMMA = 2.4


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
    """把影响亮度和成像的控件全部钉死。

    【2026-10-20 改】以前只锁 5 项，brightness/contrast/sharpness/逆光补偿/gamma
    全是裸默认值。逆光补偿默认是 1，AE 一旦漏网它会再改一次测光；gamma 则决定
    "同样的辐照度显示成多亮"，两个进程取值不同就会一个亮一个暗。现在：
      - 必选项（曝光/增益/白平衡）锁不上 → 抛异常，拒绝继续采集；
      - 可选项支持就写死，不支持/只读只记录（不同固件差别很大，别一刀切拦死）；
      - gamma 不猜数值：读出当前值再显式写回同一个值，把它钉住。
    """
    all_names = list(REQUIRED_SETTINGS) + [
        "brightness", "contrast", "sharpness", "backlight_compensation",
        "power_line_frequency", "gamma"]
    previous, unsupported = {}, []
    for name in all_names:
        try:
            previous[name] = get_ctrl(fd, CONTROLS[name])
        except OSError:
            unsupported.append(name)

    missing = [name for name in REQUIRED_SETTINGS if name not in previous]
    if missing:
        raise RuntimeError(f"相机不支持这些必需控件 {missing}，曝光无法保证可复现；拒绝采集")

    # 顺序很重要：先关自动、再写手动量（restore_controls 正好反过来）。
    wanted = [("exposure_auto", 1), ("exposure", args.exposure),
              ("auto_white_balance", 0),
              ("white_balance_temperature", args.white_balance),
              ("gain", args.gain),
              ("brightness", args.brightness),
              ("contrast", args.contrast),
              ("sharpness", args.sharpness),
              ("backlight_compensation", args.backlight_compensation)]
    wanted = [(name, value) for name, value in wanted if name in previous]
    if "power_line_frequency" in previous:
        wanted.append(("power_line_frequency", args.power_line_frequency))
    if "gamma" in previous:
        wanted.append(("gamma", previous["gamma"]))   # 钉住当前值，不猜

    # 有些 UVC 控件是只读的：G_CTRL 能读、S_CTRL 返回 EINVAL。
    # 可选项遇到这种情况只记录，不能让整台相机起不来；必选项失败才拒绝采集。
    write_failed = []
    pinned_to_current = ["gamma"] if "gamma" in previous else []
    try:
        print("锁定控件 (曝光单位: 100 微秒):")
        for name, value in wanted:
            if name in REQUIRED_SETTINGS:
                set_ctrl_checked(fd, CONTROLS[name], value, name)
                continue
            try:
                set_ctrl_checked(fd, CONTROLS[name], value, name)
            except (OSError, RuntimeError) as exc:
                write_failed.append({"control": name, "requested": value, "error": str(exc)})
                print(f"  ⚠️ {name} 写不进去（只读或越界），跳过：{exc}", file=sys.stderr)
        if unsupported:
            print(f"  本固件不支持、已跳过: {', '.join(unsupported)}（只记录，不影响采集）")
    except BaseException:
        restore_controls(fd, previous)
        raise
    locked = {name: get_ctrl(fd, CONTROLS[name]) for name in previous}
    notes = {"unsupported": unsupported, "write_failed": write_failed,
             "pinned_to_current_value": pinned_to_current}
    return previous, locked, notes


def restore_controls(fd, previous):
    if not previous:
        return {"status": "not_needed", "failed": []}
    # Restore dependent manual values before reenabling the automatic controls.
    order = ["exposure", "white_balance_temperature", "gain", "brightness",
             "contrast", "sharpness", "backlight_compensation", "gamma",
             "power_line_frequency", "exposure_auto", "auto_white_balance"]
    failed = []
    for name in order:
        if name not in previous:
            continue
        try:
            c = V4L2Control(id=CONTROLS[name], value=previous[name])
            fcntl.ioctl(fd, VIDIOC_S_CTRL, c)
            if get_ctrl(fd, CONTROLS[name]) != previous[name]:
                raise OSError("恢复后回读值不一致")
        except OSError as exc:
            failed.append({"control": name, "error": str(exc)})
            print(f"警告: 未能恢复控件 {name}: {exc}", file=sys.stderr)
    return {"status": "partial_failure" if failed else "restored", "failed": failed}


def _frame_light(frame):
    """一帧的 (线性光平均, 饱和像素比例)。"""
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
    linear = (gray.astype(np.float32) / 255.0) ** GAMMA
    return float(linear.mean()), float(np.mean(gray >= 250))


def _discard(cap, count):
    for _ in range(count):
        cap.read()


def _measure_light(cap, count):
    light = saturation = 0.0
    for _ in range(count):
        frame, _ = cap.read()
        one, two = _frame_light(frame)
        light += one
        saturation += two
    return light / count, saturation / count


def _write_exposure(fd, value):
    """写曝光并返回**实际生效**的值（固件可能把值夹到自己的范围里）。"""
    fcntl.ioctl(fd, VIDIOC_S_CTRL, V4L2Control(id=CONTROLS["exposure"], value=int(value)))
    return get_ctrl(fd, CONTROLS["exposure"])


def query_ctrl(fd, cid):
    """问驱动这个控件的真实范围（min/max/step/default）。

    【2026-10-20 加】以前所有曝光/增益都是"猜一个数字写死"。
    没有这个范围，就没法知道"还能不能再加增益"—— 也就没法判断
    "拍不亮"到底是现场太暗，还是我们把相机限制死了。
    """
    q = V4L2QueryCtrl(id=cid)
    fcntl.ioctl(fd, VIDIOC_QUERYCTRL, q)
    if int(q.flags) & V4L2_CTRL_FLAG_DISABLED:
        raise OSError("驱动标记该控件为不可用")
    return {"name": q.name.decode("utf-8", "replace").strip() or hex(cid),
            "min": int(q.minimum), "max": int(q.maximum),
            "step": max(1, int(q.step)), "default": int(q.default_value),
            "flags": int(q.flags)}


def _set_raw(fd, name, value):
    """写控件并返回实际生效值；不抛异常（固件越界会自己夹）。"""
    fcntl.ioctl(fd, VIDIOC_S_CTRL, V4L2Control(id=CONTROLS[name], value=int(value)))
    return get_ctrl(fd, CONTROLS[name])


def _gray_mean(cap, settle=4, frames=5):
    """丢几帧过渡、再取几帧的平均灰度。"""
    for _ in range(settle):
        cap.read()
    total = 0.0
    for _ in range(frames):
        frame, _ = cap.read()
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) if frame.ndim == 3 else frame
        total += float(gray.mean())
    return total / frames


def auto_expose(cap, fd, args, log=print):
    """进场先测光，找到这段采集该用的曝光/增益，然后锁死。

    【2026-10-20 加】以前 EXPOSURE=100 / GAIN=30 是写死的，这组数字只是照
    某一次**有日光**的 session 定的。换到只有室内灯、或者晚上，同样的数字
    直接不够 —— 手机能拍是因为它先测光，不是因为它更灵敏。
    相机不需要"永远同一个曝光"，只需要**一段采集内**固定（这决定了
    cam-IMU 时移是否可复现）。所以先搜到合适的值，再锁。

    搜索策略（优先用不增噪的杠杆）：
      1) 曝光先顶到「模糊预算」和「帧周期」两者较小值 —— 曝光不增噪，只增模糊；
      2) 还不够深，就在增益范围里爬梯子 —— 增益不增模糊，只增噪；
      3) 两个都到顶还低于下限 → 报告能到的最好值并拒绝采集。

    返回 (是否可用, 证据字典)。
    """
    evidence = {"status": "disabled", "ladder": []}
    try:
        exp_range = query_ctrl(fd, CONTROLS["exposure"])
        gain_range = query_ctrl(fd, CONTROLS["gain"])
    except OSError as exc:
        log(f"⚠️ 问不到控件范围（{exc}），沿用命令行的 曝光={args.exposure} 增益={args.gain}")
        evidence.update(status="unknown_ranges", exposure=args.exposure, gain=args.gain)
        return True, evidence

    # 曝光上限：帧周期 0.9 倍 与 用户给的模糊预算，取小。单位都是 100µs。
    frame_cap = int(0.9 * 1e6 / (args.fps * 100))
    # 【2026-10-21】曝光上限 = min(模糊预算反推, 硬上限)。
    # 以前 blur_cap 直接等于 auto_expose_max_exposure_ms（18 ms），等于没有约束。
    _omega = args.auto_expose_expected_rate_dps * math.pi / 180.0
    _focal = args.focal_px
    _budget_px = args.auto_expose_blur_budget_px
    if _omega > 1e-6 and _focal > 1.0:
        _blur_ms = 1000.0 * _budget_px / (_focal * _omega)
    else:
        _blur_ms = args.auto_expose_max_exposure_ms
    _eff_ms = min(_blur_ms, args.auto_expose_max_exposure_ms)
    blur_cap = int(round(_eff_ms * 10))
    exp_max = max(exp_range["min"] + exp_range["step"],
                  min(exp_range["max"], frame_cap, blur_cap))
    target = float(args.auto_expose_target)
    gain_max = min(gain_range["max"], args.auto_expose_max_gain)
    log(f"自动测光：目标灰度 {target:.0f}｜曝光上限 {exp_max * 0.1:.2f} ms"
        f"（{_budget_px:g} px 模糊预算 @ {args.auto_expose_expected_rate_dps:g}°/s → {_blur_ms:.2f} ms；"
        f"帧周期限 {frame_cap * 0.1:.1f} ms；硬上限 {args.auto_expose_max_exposure_ms:.1f} ms）"
        f"｜增益上限 {gain_max}（相机 {gain_range['min']}..{gain_range['max']}）"
        f"｜曝光范围 {exp_range['min']}..{exp_range['max']}")
    if exp_max * 0.1 > 6.0:
        log(f"⚠️ 曝光上限 {exp_max * 0.1:.1f} ms 偏大：角速度一上去角点就被糊没了")

    # 增益梯子按"范围的百分比"走，不假设单位是 dB 还是线性倍数
    best, reached = None, None
    for frac in (0.0, 0.03, 0.07, 0.12, 0.2, 0.3, 0.45, 0.65, 1.0):
        want_gain = int(round(gain_range["min"] + frac * (gain_max - gain_range["min"])))
        actual_exp = _set_raw(fd, "exposure", exp_max)
        actual_gain = _set_raw(fd, "gain", want_gain)
        mean = _gray_mean(cap)
        row = {"gain": actual_gain, "exposure": actual_exp, "mean": round(mean, 1)}
        evidence["ladder"].append(row)
        log(f"  曝光 {actual_exp}（{actual_exp * 0.1:.1f} ms）增益 {actual_gain}"
            f" → 灰度 {mean:.1f}")
        if best is None or mean > best["mean"]:
            best = row
        if mean >= target:
            reached = row
            break
    if reached is not None:
        best = reached

    # 光很足时按比例收曝光：同样达到目标亮度，但运动模糊更小。
    # 迭代两次 —— 第一次可能因为高光截断（255 封顶）而收得不够。
    for _ in range(2):
        if best is None or best["mean"] <= target * 1.25 or best["exposure"] <= exp_range["min"]:
            break
        shrink = target / best["mean"]
        want = max(exp_range["min"] + exp_range["step"],
                   int(round(best["exposure"] * shrink)))
        if want >= best["exposure"]:
            break
        actual_exp = _set_raw(fd, "exposure", want)
        _set_raw(fd, "gain", best["gain"])
        mean = _gray_mean(cap)
        best = {"gain": best["gain"], "exposure": actual_exp, "mean": round(mean, 1)}
        evidence["ladder"].append({**best, "note": "按目标收缩曝光以减少运动模糊"})
        log(f"  收缩曝光：{actual_exp * 0.1:.1f} ms 增益 {best['gain']}"
            f" → 灰度 {mean:.1f}")

    _set_raw(fd, "exposure", best["exposure"])
    _set_raw(fd, "gain", best["gain"])
    evidence.update(status="ok", exposure=best["exposure"], gain=best["gain"],
                    exposure_ms=round(best["exposure"] * 0.1, 1),
                    achieved_mean=best["mean"], target=target,
                    gain_range=[gain_range["min"], gain_range["max"]],
                    exposure_range=[exp_range["min"], exp_range["max"]])
    log(f"  锁定：曝光 {best['exposure']}（{best['exposure'] * 0.1:.1f} ms）"
        f"增益 {best['gain']} → 灰度 {best['mean']:.1f}")

    # 【2026-10-21】曝光/增益都顶满才勉强够亮 = 场地光不够。session29 就是这样过的关
    # （曝光 18 ms、增益 100、灰度只有 49.5，而门槛 45），结果尺度偏 5 倍。
    exp_pinned = exp_max >= int(round(min(_blur_ms, args.auto_expose_max_exposure_ms) * 10)) - exp_range["step"]
    gain_pinned = best["gain"] >= gain_max
    if (exp_pinned and gain_pinned and best["mean"] < target
            and not args.allow_blur and not args.allow_dark):
        log(f"❌ 曝光和增益都顶到上限（{best['exposure'] * 0.1:.2f} ms / 增益 {best['gain']}）"
            f"才到灰度 {best['mean']:.1f}，离目标 {target:.0f} 差得远 —— 光不够。")
        log(f"   这样录下去运动模糊会超过 {_budget_px:g} px，KLT 跟踪会崩"
            f"（实测存活率会从 80% 掉到 40% 以下）。")
        log("   加灯，或者把 AUTO_EXPOSE_EXPECTED_RATE_DPS 调低到你的真实角速度。")
        evidence["status"] = "insufficient_light"
        evidence["exp_pinned"] = True
        evidence["gain_pinned"] = True
        return False, evidence

    if best["mean"] < args.auto_expose_min_mean:
        if args.allow_dark:
            log(f"⚠️ 只能到灰度 {best['mean']:.1f}（低于 {args.auto_expose_min_mean}），"
                "是 --allow-dark 才继续的 —— 这种亮度的数据大概率发散")
            evidence["status"] = "too_dark_allowed"
            return True, evidence
        evidence["status"] = "too_dark"
        return False, evidence
    return True, evidence


def verify_manual_exposure(cap, fd, args, attempts=3):
    """用图像本身验证手动曝光真的接管了 —— G_CTRL 读回只证明驱动缓存。

    【2026-10-20 加】set_ctrl_checked 比对的是 uvcvideo 的驱动缓存，不是固件。
    session05 恢复成 exposure_auto=3 且回读一致，session06 一开相机读到的却还是
    锁定的 1 —— 写入根本没落地。唯一可信的验证是看图像：目标曝光测线性亮度 L1，
    把曝光调成 k 倍测 L2，期望 L2/L1 ≈ k。比值 ≈ 1 就是 AE 还在掌权（它会把
    画面拉回原来的亮度）。

    必须在 (v/255)**2.4 的线性域比，不能拿原始灰度比：实测 2 倍曝光在原始域
    只有 1.330（正好是 2**(1/2.4)=1.33），线性域 1.983 ≈ 理论 2.00。

    返回 (是否通过, 证据字典)。太暗导致无法验证时按通过处理并注明 ——
    光不够是另一码事，不该被这里拦成"曝光没锁上"。
    """
    target = int(args.exposure)
    flush = max(6, int(args.warmup_frames))
    samples = 8
    multiplier = 2.0          # 第一次用 2 倍；过曝就退到 1.5 倍重试
    evidence = {"status": "skipped", "gamma_used": GAMMA, "attempts": []}

    def finish(passed):
        # 每次尝试要读 2*(丢弃+采样) 帧，最后一次恢复曝光再丢 flush 帧。
        # 不把这个数记下来，外面用 v4l2_sequence 算"丢帧"会把这 40 多帧
        # 当成相机丢帧 —— 实际是验证自己消耗掉的。
        evidence["frames_consumed"] = len(evidence["attempts"]) * 2 * (flush + samples) + flush
        return passed, evidence
    for attempt in range(1, attempts + 1):
        actual_high = real_k = 0.0
        try:
            actual_target = _write_exposure(fd, target)
            if actual_target != target:
                raise RuntimeError(f"目标曝光 {target} 写不进去（实际 {actual_target}）")
            _discard(cap, flush)
            light_low, sat_low = _measure_light(cap, samples)
            # 画面已经偏亮时别用 2 倍，否则高光截断会把比值压低、造成假失败。
            k = min(multiplier, 2.0 if light_low < 0.25 else 1.5)
            actual_high = _write_exposure(fd, int(round(target * k)))
            real_k = actual_high / target if target else 0.0
            _discard(cap, flush)
            light_high, sat_high = _measure_light(cap, samples)
        finally:
            try:
                _write_exposure(fd, target)
                _discard(cap, flush)
            except (OSError, RuntimeError, cv2.error):
                pass

        ratio = light_high / light_low if light_low > 0 else 0.0
        one = {"attempt": attempt, "target_exposure": target, "high_exposure": actual_high,
               "actual_multiplier": round(real_k, 3),
               "linear_before": round(light_low, 5), "linear_after": round(light_high, 5),
               "linear_ratio": round(ratio, 4),
               "saturation_before": round(sat_low, 4), "saturation_after": round(sat_high, 4)}
        evidence["attempts"].append(one)

        if real_k < 1.25:
            one["verdict"] = f"跳过：曝光上限只有 {real_k:.2f} 倍，测不出比值"
            evidence.update(status="inconclusive", verdict=one["verdict"])
            return finish(True)
        if light_low < 0.0015:
            one["verdict"] = f"跳过：太暗（线性亮度 {light_low:.5f}），比值不可信"
            evidence.update(status="too_dark", verdict=one["verdict"])
            return finish(True)
        if sat_high > 0.25:
            one["saturated"] = True
            one["verdict"] = f"重试：高光过曝 {100 * sat_high:.0f}%，比值不可信"
            multiplier = 1.5
            continue
        need = 1.0 + 0.5 * (real_k - 1.0)
        if ratio >= need:
            one["verdict"] = f"通过：线性比值 {ratio:.2f} ≥ 门槛 {need:.2f}"
            evidence.update(status="passed", verdict=one["verdict"], need=round(need, 3))
            return finish(True)
        one["verdict"] = f"不通过：线性比值 {ratio:.2f} < 门槛 {need:.2f}（像 AE 还在接管）"

    last = evidence["attempts"][-1]
    if last.get("saturated"):
        # 过曝是"测不出来"，不是"锁定失败"：别把亮场景判成相机坏了。
        evidence["status"] = "inconclusive"
        evidence["verdict"] = ("跳过：多次高光过曝，比值不可信"
                               "（把灯移开或降低曝光后重测）")
        return finish(True)
    evidence["status"] = "failed"
    evidence["verdict"] = last["verdict"]
    return finish(False)


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
    parser.add_argument("--backlight-compensation", type=int,
                        default=config.BACKLIGHT_COMPENSATION,
                        help="逆光补偿；默认 0（关）。开着会在 AE 漏网时二次改测光")
    parser.add_argument("--skip-exposure-check", action="store_true",
                        default=config.SKIP_EXPOSURE_CHECK,
                        help="跳过锁定后的曝光步进验证（仅用于确认是误判时）")
    parser.add_argument("--auto-expose", action="store_true", default=config.AUTO_EXPOSE,
                        help="先测光再锁定（默认开）；关掉就用 --exposure/--gain 写死")
    parser.add_argument("--no-auto-expose", action="store_false", dest="auto_expose")
    parser.add_argument("--auto-expose-target", type=float, default=config.AUTO_EXPOSE_TARGET,
                        help="测光目标：灰度均值")
    parser.add_argument("--auto-expose-min-mean", type=float,
                        default=config.AUTO_EXPOSE_MIN_MEAN,
                        help="测光最好也只能到这个灰度以下时，拒绝采集")
    parser.add_argument("--auto-expose-max-exposure-ms", type=float,
                        default=config.AUTO_EXPOSE_MAX_EXPOSURE_MS,
                        help="曝光绝对上限（毫秒）。真正的模糊约束请看下面三项")
    parser.add_argument("--auto-expose-blur-budget-px", type=float,
                        default=getattr(config, "AUTO_EXPOSE_BLUR_BUDGET_PX", 3.0),
                        help="允许的运动模糊（像素）。实测 >12 px 时 KLT 存活率掉到 55%%")
    parser.add_argument("--auto-expose-expected-rate-dps", type=float,
                        default=getattr(config, "AUTO_EXPOSE_EXPECTED_RATE_DPS", 100.0),
                        help="预期最大角速度（度/秒）。曝光上限 = blur / (f * ω)")
    parser.add_argument("--focal-px", type=float,
                        default=getattr(config, "FOCAL_PX", 1367.5),
                        help="相机焦距（像素），与 kalibr_imucam_chain.yaml 的 fx 一致")
    parser.add_argument("--auto-expose-max-gain", type=int,
                        default=getattr(config, "AUTO_EXPOSE_MAX_GAIN", 40),
                        help="增益上限；100 的噪声会把 KLT 拖死（实测 σ≥8 时存活率腰斩）")
    parser.add_argument("--allow-blur", action="store_true", default=False,
                        help="明知光不够、会糊也硬录（只用于排查）")
    parser.add_argument("--allow-dark", action="store_true", default=config.ALLOW_DARK,
                        help="测光不达标也硬录（数据大概率发散，只用于排查）")
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
    control_notes = exposure_check = None
    start = None
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
        original, settings, control_notes = lock_controls(fd, args)
        auto = {"status": "disabled"}
        if args.auto_expose:
            ok, auto = auto_expose(cap, fd, args)
            if not ok:
                raise RuntimeError(
                    f"测光失败：曝光和增益都到顶，也只能到灰度 {auto.get('achieved_mean')}"
                    f"（门槛 {args.auto_expose_min_mean:g}）。这个亮度下特征会断崖式消失"
                    "（session25/26/27 就是），录了也是白录。换更亮的时段/位置，或者加灯；"
                    "确认要硬录可加 --allow-dark。")
            # 后续验证和 manifest 都以测光选出的值为准
            args.exposure = int(auto.get("exposure", args.exposure))
            args.gain = int(auto.get("gain", args.gain))
            # 【2026-10-20 修】lock_controls 返回的 settings 是**测光之前**的快照。
            # 测光会改曝光/增益，不重新读一遍，manifest 的 locked_controls 就会
            # 写着 gain=30 而实际跑的是 100（session29 就是这样，差点误判成"测光没生效"）。
            settings = {name: get_ctrl(fd, CONTROLS[name]) for name in settings}
        else:
            print("⚠️ 已关闭自动测光（--no-auto-expose）：曝光/增益是写死的，"
                  "换场景亮度就会不匹配")
        exposure_check = {"status": "skipped", "verdict": "已用 --skip-exposure-check 跳过"}
        if not args.skip_exposure_check:
            # 控件回读只证明驱动缓存；这里从图像本身确认手动曝光真的接管了。
            passed, exposure_check = verify_manual_exposure(cap, fd, args)
            print(f"曝光验证: {exposure_check['verdict']}")
            last = (exposure_check.get("attempts") or [{}])[-1]
            if last:
                print(f"  曝光 {last['target_exposure']} → {last['high_exposure']}"
                      f"（×{last['actual_multiplier']}），线性亮度 "
                      f"{last['linear_before']} → {last['linear_after']}，"
                      f"比值 {last['linear_ratio']}（γ={GAMMA}）")
            if not passed:
                raise RuntimeError(
                    "曝光步进测试不通过：控件回读说明写进去了，但图像亮度不随曝光变化，"
                    "手动曝光没有真正接管（相机还在自动曝光）。先跑 `v4l2-ctl -d <节点> "
                    "--list-ctrls-menus | grep -A2 auto_exposure` 确认是不是 1(Manual)，"
                    "拔插一次相机再试；确认是误判可加 --skip-exposure-check。")
        else:
            print("⚠️ 已跳过曝光验证：本次亮度是否可复现没有证据", file=sys.stderr)
        if not args.no_preview:
            preview = PreviewWindow()
        camera_dir.mkdir(parents=True, exist_ok=True)
        # 【2026-10-20 加】锁定+验证要花几秒，倒计时不能从进程启动算起，
        # 否则"剩 25 秒"其实只剩 21 秒，闭环的时长窗口整个错位。
        # record_session.py 见到这一行才把计时器归零。
        print("@记录开始@", flush=True)
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
                        _orig, settings, _notes = lock_controls(fd, args)
                        # 重连后相机状态未知，重新从图像确认一次（只试 1 次，别拖太久）
                        if not args.skip_exposure_check:
                            passed, check = verify_manual_exposure(cap, fd, args, attempts=1)
                            if not passed:
                                raise RuntimeError(f"重连后曝光验证不通过：{check['verdict']}")
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
            # [2026-10-18 修改] 和下面"保存时检测"同一个毛病：连续录像（--no-preview）时
            # 这个每 0.5 秒一次的棋盘检测在 1080p 上要 470~530 ms，把 29 fps 拖成 11 fps，
            # 而且制造 330~400 ms 的驱动级空洞（直接触发标定侧 max_camera_gap_s=0.35）。
            # 它只给预览窗绿点和 3x3 覆盖度用，录像时不需要。
            if args.cols and not args.no_preview and (frames == 1 or now - last_detect_time >= 0.5):
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
                termination = "user_stopped"
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
                termination = "max_images_reached"
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
            requested = float(args.duration) if args.duration else None
            delivered = (last_frame_time - start
                         if requested and start is not None and last_frame_time else None)
            # 【2026-10-20 加】静默截断不再记成 completed。
            # 实测有一次请求 60 秒只交付 39.84 秒：v4l2_sequence 连续、无重连、
            # termination 写着 completed —— 翻 manifest 完全看不出问题，
            # 而闭环 VIO 的时长窗口会直接错掉。宁可在采集端就判失败。
            if (requested and delivered is not None and not args.max_images
                    and termination == "completed" and delivered < 0.9 * requested):
                termination = (f"failed: 只交付 {delivered:.2f}s / 请求 {requested:g}s"
                               f"（相机在 {delivered:.1f}s 处停止出流）")
                print(f"❌ {termination}", file=sys.stderr)
            summary = {"device": str(device), "camera_ids": "2bc5:0511",
                       "mode": mode, "locked_controls": settings, "frames_read": frames,
                       "original_controls": original, "control_restoration": restoration,
                       "control_notes": control_notes or {},
                       "auto_exposure": auto,
                       "unsupported_controls": (control_notes or {}).get("unsupported", []),
                       "exposure_verification": exposure_check,
                       "requested_duration_s": requested,
                       "delivered_duration_s": None if delivered is None else round(delivered, 3),
                       "backend": backend_name, "termination": termination, "reconnections": reconnects,
                       "timestamp_source": (last_metadata or {}).get("timestamp_source", "no_frames"),
                       "sensor_timestamp_ns": None,
                       "last_frame_timestamp_metadata": last_metadata,
                       "filename_timestamp_source": "host_receive_time_ns_unix",
                       "usable_for_camera_imu_time_calibration": False,
                       "board_inner_corners": [args.cols, args.rows],
                       # 【2026-10-20】不记这个数，外面用
                       # (末尾 v4l2_sequence + 1 - frames_read) 算"丢帧"会
                       # 恰好多出 warmup-1，看起来像相机丢帧。
                       "warmup_frames": args.warmup_frames,
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
