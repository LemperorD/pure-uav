#!/usr/bin/env python3
"""跨平台串口抽象 —— 让录制器在 Windows 和 Linux 上都能跑。

为什么需要这层：
    Linux 上有内核自带的 termios，不用装任何包就能开串口；
    Windows 上没有 termios，必须用 pyserial。
    录制器本身的逻辑（MAVLink 解析、CSV 落盘）两边完全一样，
    所以只把"开串口/读/写/找口"这四件事抽出来。

两条后端自动选择：
    Windows  → pyserial（需要 pip install pyserial）
    Linux/macOS → termios（内核自带，无需装包；装了 pyserial 也不用）

用法:
    from serial_port import list_candidates, find_mavlink_port, open_port
    port = open_port("/dev/ttyACM0")   # 或 "COM3"
    data = port.read(8192)             # 没数据返回 b""，最多等 50 ms
    port.write(b"...")
    port.close()
"""
from __future__ import annotations

import os
import sys
import time
from pathlib import Path

IS_WINDOWS = sys.platform.startswith("win")
DEFAULT_BAUD = 115200
READ_TIMEOUT_S = 0.05


# --------------------------------------------------------------------------
# 端口枚举
# --------------------------------------------------------------------------

def list_candidates():
    """列出可能的飞控串口。

    Linux : /dev/ttyACM* + /dev/ttyUSB*（CUAV 7-Nano 会同时枚举出 ttyACM0 和 ttyACM1）
    Windows: COM* —— 优先挑 USB CDC 设备（VID 0x1209 = pid.codes，CUAV 用的就是它）
    """
    if IS_WINDOWS:
        try:
            from serial.tools import list_ports
        except ImportError:
            return []
        ports = list(list_ports.comports())
        # CUAV 7-Nano 的 USB ID 是 1209:5740
        preferred = [p.device for p in ports
                     if (p.vid == 0x1209) or "CUAV" in (p.description or "").upper()]
        others = [p.device for p in ports if p.device not in preferred]
        return preferred + others

    out = []
    for pattern in ("ttyACM*", "ttyUSB*"):
        out += sorted(str(p) for p in Path("/dev").glob(pattern))
    return out


def describe_candidates():
    """给用户看的端口清单（带描述）。"""
    if IS_WINDOWS:
        try:
            from serial.tools import list_ports
        except ImportError:
            return ["（没装 pyserial —— 见 WINDOWS.md）"]
        return [f"{p.device:8s} {p.description}" for p in list_ports.comports()]
    return list_candidates()


# --------------------------------------------------------------------------
# 串口对象
# --------------------------------------------------------------------------

class Port:
    """统一接口：read(n) / write(data) / close()。"""

    def __init__(self, name: str, baud: int = DEFAULT_BAUD):
        self.name = name
        self.baud = baud

    def read(self, n: int = 8192) -> bytes:
        raise NotImplementedError

    def write(self, data: bytes) -> int:
        raise NotImplementedError

    def close(self) -> None:
        raise NotImplementedError

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()


class _TermiosPort(Port):
    """Linux / macOS：内核自带，不依赖任何第三方包。"""

    def __init__(self, name, baud=DEFAULT_BAUD):
        super().__init__(name, baud)
        import termios
        self._termios = termios
        self.fd = os.open(str(name), os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
        try:
            a = termios.tcgetattr(self.fd)
            a[0] = 0                                        # iflag：关掉所有输入处理
            a[1] = 0                                        # oflag
            a[2] = termios.CLOCAL | termios.CREAD | termios.CS8
            a[3] = 0                                        # lflag：原始模式
            a[4] = a[5] = _baud_constant(termios, baud)
            termios.tcsetattr(self.fd, termios.TCSANOW, a)
            termios.tcflush(self.fd, termios.TCIOFLUSH)
        except Exception:
            os.close(self.fd)
            raise

    def read(self, n=8192) -> bytes:
        import select
        try:
            r, _, _ = select.select([self.fd], [], [], READ_TIMEOUT_S)
            if not r:
                return b""
            return os.read(self.fd, n)
        except (BlockingIOError, InterruptedError):
            return b""

    def write(self, data: bytes) -> int:
        return os.write(self.fd, data)

    def close(self) -> None:
        try:
            os.close(self.fd)
        except OSError:
            pass


class _PyserialPort(Port):
    """Windows：用 pyserial。

    注意：USB CDC（ttyACM 类设备）在 Windows 上波特率是名义值，
    写多少都不影响实际吞吐 —— 和 Linux 一样。
    """

    def __init__(self, name, baud=DEFAULT_BAUD):
        super().__init__(name, baud)
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError(
                "Windows 上需要 pyserial： pip install pyserial") from exc
        self._serial = serial.Serial(
            port=name, baudrate=baud,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=READ_TIMEOUT_S, write_timeout=2.0,
        )
        self._serial.reset_input_buffer()

    def read(self, n=8192) -> bytes:
        ser = self._serial
        waiting = ser.in_waiting
        return ser.read(waiting if waiting else 1)      # timeout 到了返回 b""

    def write(self, data: bytes) -> int:
        return self._serial.write(data)

    def close(self) -> None:
        try:
            self._serial.close()
        except Exception:                                      # noqa: BLE001
            pass


def _baud_constant(termios, baud: int) -> int:
    name = f"B{int(baud)}"
    if not hasattr(termios, name):
        raise ValueError(f"termios 不支持波特率 {baud}；USB CDC 下随便填 115200 即可")
    return getattr(termios, name)


def open_port(name: str, baud: int = DEFAULT_BAUD) -> Port:
    if IS_WINDOWS:
        return _PyserialPort(name, baud)
    return _TermiosPort(name, baud)


# --------------------------------------------------------------------------
# 找一个真的在讲 MAVLink 的口
# --------------------------------------------------------------------------

def find_mavlink_port(candidates=None, timeout: float = 1.5, baud: int = DEFAULT_BAUD,
                      log=print):
    """逐个监听候选串口，返回第一个真的在讲 MAVLink 的口。

    只看不发（纯粹听 1.5 秒），对飞控没有任何副作用。
    """
    import sys as _sys
    _here = os.path.dirname(os.path.abspath(__file__))
    for cand in (_here, os.path.join(_here, "vendor")):
        if cand not in _sys.path:
            _sys.path.insert(0, cand)
    from mavlink_min import MavlinkParser

    candidates = list(candidates) if candidates else list_candidates()
    if not candidates:
        log("  ✗ 没有找到任何串口设备")
        if IS_WINDOWS:
            log("    → 设备管理器里看看有没有「USB 串行设备 (COMx)」")
            log("    → 飞控插好了吗？换根数据线（不是只充电的）？")
        else:
            log("    → 飞控插好了吗？换个 USB 口？（避开 hub）")
        return None

    log(f"  串口候选 {candidates}，逐个监听 {timeout:g}s……")
    for name in candidates:
        try:
            port = open_port(name, baud)
        except Exception as exc:                                        # noqa: BLE001
            log(f"    {name}: 打不开（{exc}）")
            if not IS_WINDOWS and ("ermission" in str(exc) or "13" in str(exc)):
                log("      → 权限问题：sudo usermod -aG dialout $USER 然后重新登录")
            if IS_WINDOWS and "拒绝访问" in str(exc):
                log("      → 被别的程序占着（QGroundControl？Mission Planner？）先关掉")
            continue
        parser = MavlinkParser()
        seen = {}
        deadline = time.monotonic() + timeout
        try:
            while time.monotonic() < deadline:
                data = port.read(4096)
                for frame in parser.feed(data or b""):
                    seen[frame.name] = seen.get(frame.name, 0) + 1
        finally:
            port.close()
        if seen:
            log(f"    {name}: ✅ 收到 {sum(seen.values())} 帧 {sorted(seen)}")
            return name
        log(f"    {name}: 静默（没有有效 MAVLink 帧）")

    log("  ✗ 没有任何串口在讲 MAVLink")
    log("    → 飞控供电了吗？USB 线是数据线吗？别的程序占着串口吗？")
    return None
