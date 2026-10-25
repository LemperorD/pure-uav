"""CUAV 7-Nano IMU → imu.csv（外参标定用）

不依赖 pyserial：用内核自带的 termios + os.read，和你 `imu/src` 里那几个脚本一致。
MAVLink 编解码用 `vendor/mavlink_min.py`（已在真实串口抓包上验证：98 帧、0 CRC 错）。

时间戳策略 —— 这一段决定标定能不能做，别改：
  相机侧记的是宿主机 CLOCK_REALTIME（`time.time_ns()`）。这里必须记同一个钟。
  若直接改用飞控的 `time_usec`（开机微秒），两路时间轴差了"开机时长 + 1970 年"这么大
  的量级，标定程序 ±0.1 s 的搜索窗根本够不着，会直接报"时间重叠不足"。

  但串口是批量读的：一次 read 可能拿到好几帧。若给同批帧打同一个时间戳，
  200 Hz 下批内就有 5~15 ms 的人为抖动 —— 而标定要估的时移精度是 1 ms 量级。
  所以分两层：
    1) 批内：用飞控自己的 `time_usec` 算相对间隔，抖动压到亚毫秒级；
    2) 全段：录完后用 (宿主收到时间 − time_usec) 的**下包络**拟合一条直线，
       把每批 read 的锚点换成"全段统一映射"（`imu_timebase.py`）。
       这样"这一批在 OS 缓冲里躺了多久"不再进入时间戳 —— 2026-10-18 实测，
       旧方案下同一台设备的相机-IMU 时移在 −27…+9 ms 之间跳，变的就是这个积压。
  剩下的固定传输延迟（线缆 + 飞控输出）是常数，会被标定的 offset 搜索吸收；
  下包络拟合只保证它**跨录制稳定**，并且把积压量写进统计（`timebase`）。
"""
from __future__ import annotations

import csv
import json
import math
import os
import select
import sys
import termios
import threading
import time
from dataclasses import dataclass, field
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]  # extrinsic/：唯一需要上 sys.path 的路径
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from capture import imu_timebase  # noqa: E402  同包，纯运算模块
from capture.vendor.mavlink_min import MavlinkParser, pack  # noqa: E402

MAV_CMD_SET_MESSAGE_INTERVAL = 511
MSG_HIGHRES_IMU = 105
# HIGHRES_IMU.fields_updated 里代表 IMU 六个量的位：xacc..zgyro
IMU_FIELD_BITS = 0x3F
CSV_HEADER = ["timestamp", "ax", "ay", "az", "gx", "gy", "gz"]


# --------------------------------------------------------------------------
# 串口
# --------------------------------------------------------------------------

def _baud_constant(baud: int) -> int:
    name = f"B{int(baud)}"
    if not hasattr(termios, name):
        raise ValueError(f"termios 不支持波特率 {baud}；USB CDC 下随便填 115200 即可")
    return getattr(termios, name)


def open_serial(path, baud=115200):
    """打开串口并配成 8N1 原始模式。返回 fd。"""
    fd = os.open(str(path), os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK)
    try:
        a = termios.tcgetattr(fd)
        a[0] = 0            # iflag：关掉所有输入处理
        a[1] = 0            # oflag
        a[2] = termios.CLOCAL | termios.CREAD | termios.CS8
        a[3] = 0            # lflag：原始模式
        a[4] = a[5] = _baud_constant(baud)
        termios.tcsetattr(fd, termios.TCSANOW, a)
        termios.tcflush(fd, termios.TCIOFLUSH)
    except Exception:
        os.close(fd)
        raise
    return fd


def list_serial_candidates():
    """列出可能的飞控串口。CUAV 7-Nano 会同时枚举出 ttyACM0 和 ttyACM1。"""
    out = []
    for pattern in ("ttyACM*", "ttyUSB*"):
        out += sorted(Path("/dev").glob(pattern))
    return out


def find_mavlink_port(candidates=None, timeout=1.5, baud=115200, log=print):
    """在候选串口里逐个监听，返回第一个真的在讲 MAVLink 的口。

    只看不发：不发任何请求，纯粹听 1.5 秒。所以对飞控没有任何副作用。
    7-Nano 的两个 ACM 口里，通常只有一个在跑 MAVLink。
    """
    candidates = list(candidates) if candidates else list_serial_candidates()
    if not candidates:
        return None, {}
    log(f"串口候选 {[str(c) for c in candidates]}，逐个监听 {timeout:g}s 找 MAVLink……")
    report = {}
    for path in candidates:
        try:
            fd = open_serial(path, baud)
        except Exception as exc:                       # noqa: BLE001
            report[str(path)] = f"打不开: {exc}"
            log(f"  {path}: 打不开（{exc}）")
            continue
        parser = MavlinkParser()
        seen = {}
        deadline = time.monotonic() + timeout
        try:
            while time.monotonic() < deadline:
                r, _, _ = select.select([fd], [], [], 0.1)
                if not r:
                    continue
                try:
                    data = os.read(fd, 4096)
                except BlockingIOError:
                    continue
                for frame in parser.feed(data or b""):
                    seen[frame.name] = seen.get(frame.name, 0) + 1
        finally:
            os.close(fd)
        if seen:
            report[str(path)] = f"{sum(seen.values())} 帧 {sorted(seen)}"
            log(f"  {path}: ✅ 收到 {sum(seen.values())} 帧 {sorted(seen)}")
            return str(path), report
        report[str(path)] = "没有有效 MAVLink 帧"
        log(f"  {path}: 静默（没有有效 MAVLink 帧）")
    return None, report


# --------------------------------------------------------------------------
# 记录器
# --------------------------------------------------------------------------

@dataclass
class _Sample:
    timestamp_ns: int
    ax: float
    ay: float
    az: float
    gx: float
    gy: float
    gz: float
    # 原始证据：飞控 time_usec（µs）和这一批 read 的宿主时间（ns）。
    # timestamp_ns 先按"本批 read"打，录完后再用全段下包络重算（见 imu_timebase）。
    fc_usec: int = 0
    host_ns: int = 0


@dataclass
class ImuRecorder:
    """后台线程持续读串口 → 内存里的样本列表 → 一次性写成 CSV。"""

    port: str
    request_hz: float = 200.0
    baud: int = 115200
    log: object = print

    samples: list = field(default_factory=list)
    stats: dict = field(default_factory=dict)

    _fd: int = field(default=0, init=False, repr=False)
    _thread: threading.Thread = field(default=None, init=False, repr=False)
    _stop: threading.Event = field(default_factory=threading.Event, init=False, repr=False)
    _first: threading.Event = field(default_factory=threading.Event, init=False, repr=False)
    _error: str = field(default="", init=False)
    _timebase: dict = field(default_factory=dict, init=False, repr=False)
    _anchored: bool = field(default=False, init=False, repr=False)

    # ---- 生命周期 ----
    def start(self, wait_first_s=3.0):
        self._fd = open_serial(self.port, self.baud)
        self._thread = threading.Thread(target=self._loop, name="imu-recorder", daemon=True)
        self._thread.start()
        if wait_first_s and not self._first.wait(wait_first_s):
            self.stop()
            raise RuntimeError(
                f"{self.port} 打开了但 {wait_first_s:g}s 内没收到 HIGHRES_IMU。"
                "飞控在跑吗？USB 口对吗？换另一个 ttyACM 试试。")
        return self

    def stop(self):
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=3.0)
        if self._fd:
            try:
                os.close(self._fd)
            except OSError:
                pass
            self._fd = 0
        self._anchor_timebase()
        self._finish_stats()
        return self.stats

    def _anchor_timebase(self):
        """录完后把时间戳从"每批 read 锚点"改成"全段飞控时间锚点"（幂等）。"""
        if self._anchored:
            return self._timebase
        self._anchored = True
        if not self.samples:
            self._timebase = {"ok": False, "reason": "没有样本"}
            return self._timebase
        stamps, info = imu_timebase.anchor_timestamps(
            [s.fc_usec for s in self.samples],
            [s.host_ns for s in self.samples],
            [s.timestamp_ns for s in self.samples])
        for s, t in zip(self.samples, stamps):
            s.timestamp_ns = int(t)
        self._timebase = info
        return info

    # ---- 线程体 ----
    def _loop(self):
        parser = MavlinkParser()
        target = (1, 1)         # 飞控 sysid/compid，收到 HEARTBEAT 后修正
        requested = False
        asked_at = time.monotonic()
        last_time_usec = None
        last_ts = None
        instances = set()
        n_frames = 0
        n_dup = 0
        n_fields_updated_zero = 0
        n_interpolated = 0
        n_clamped = 0
        decode_fail = 0

        try:
            while not self._stop.is_set():
                # 每 2 秒重发一次 SET_MESSAGE_INTERVAL：飞控重启/换口后能自愈
                if not requested or time.monotonic() - asked_at > 2.0:
                    try:
                        os.write(self._fd, pack(76, {
                            "param1": float(MSG_HIGHRES_IMU),
                            "param2": 1e6 / self.request_hz if self.request_hz > 0 else -1.0,
                            "param3": 0.0, "param4": 0.0, "param5": 0.0,
                            "param6": 0.0, "param7": 0.0,
                            "command": MAV_CMD_SET_MESSAGE_INTERVAL,
                            "target_system": target[0], "target_component": target[1],
                            "confirmation": 0, "unused1": 0, "unused2": 0, "unused3": 0,
                        }, sysid=255, compid=190, seq=n_frames & 0xFF))
                        requested = True
                        asked_at = time.monotonic()
                    except OSError as exc:
                        self._error = f"写串口失败: {exc}"
                        return

                r, _, _ = select.select([self._fd], [], [], 0.05)
                if not r:
                    continue
                try:
                    data = os.read(self._fd, 4096)
                except BlockingIOError:
                    continue
                except OSError as exc:
                    self._error = f"读串口失败: {exc}"
                    return
                if not data:
                    continue

                t_host = time.time_ns()
                frames = parser.feed(data)
                if not frames:
                    continue
                for frame in frames:
                    if frame.name == "HEARTBEAT" and frame.compid == 1:
                        target = (frame.sysid, frame.compid)

                hi = [f for f in frames if f.name == "HIGHRES_IMU"]
                if not hi:
                    continue
                n_frames += len(hi)

                # 批次内插值：以本批最后一帧为锚，用飞控 time_usec 反推每帧的宿主时间
                anchor_usec = int(hi[-1].fields.get("time_usec", 0) or 0)
                for frame in hi:
                    f = frame.fields
                    try:
                        t_usec = int(f.get("time_usec", 0) or 0)
                        if t_usec and last_time_usec is not None and t_usec == last_time_usec:
                            n_dup += 1          # 飞控发得比传感器快：同一样本重复
                            continue
                        if t_usec:
                            last_time_usec = t_usec
                        # 注意括号：`&` 的优先级低于 `==`，写错就恒为假
                        if (int(f.get("fields_updated", 0) or 0) & IMU_FIELD_BITS) == 0:
                            n_fields_updated_zero += 1
                        if t_usec and anchor_usec and anchor_usec >= t_usec:
                            ts = t_host - (anchor_usec - t_usec) * 1000
                            n_interpolated += 1
                        else:
                            ts = t_host
                        # 强制严格递增。串口是批量读的：飞控被 SET_MESSAGE_INTERVAL 唤醒时
                        # 会先把请求之前积压的历史样本一次性倒出来，读线程连续两次 read 的
                        # 宿主时间只差一两毫秒，而每块的 time_usec 跨度有几百毫秒 ——
                        # 插出来的时间戳就会退到上一批最后一帧之前。
                        # 标定程序要求严格递增，会直接拒绝整份数据。
                        # （2026-10-15 实际踩到过：2744 个样本里 8 个倒退 4.7~9.3 ms。）
                        #
                        # 代价要说清楚：被兜住的样本会被压到上一帧之后 1 ns，批内相对间隔
                        # 失真。对"开头一小段积压"这种真实情况无所谓（那段本来就会被标定
                        # 程序按 max_offset_s 的边距丢掉）；但如果占比很高，说明串口读得断续，
                        # 那时该修的是链路，不是继续兜 —— verdict() 会对高占比报警。
                        if last_ts is not None and ts <= last_ts:
                            ts = last_ts + 1
                            n_clamped += 1
                        last_ts = ts
                        inst = int(f.get("id", 0) or 0)
                        instances.add(inst)
                        self.samples.append(_Sample(
                            ts,
                            float(f["xacc"]), float(f["yacc"]), float(f["zacc"]),
                            float(f["xgyro"]), float(f["ygyro"]), float(f["zgyro"]),
                            fc_usec=t_usec or 0, host_ns=t_host))
                    except (KeyError, TypeError, ValueError):
                        decode_fail += 1
                    if not self._first.is_set():
                        self._first.set()
        finally:
            self._finish_extra = dict(
                frames=n_frames, duplicates=n_dup,
                fields_updated_zero=n_fields_updated_zero,
                interpolated=n_interpolated, decode_fail=decode_fail,
                clamped=n_clamped,
                instances=sorted(instances), parser_stats=dict(parser.stats),
                target=target)

    # ---- 统计 ----
    def _finish_stats(self):
        extra = getattr(self, "_finish_extra", {}) or {}
        ts = [s.timestamp_ns for s in self.samples]
        gaps_ms = [(b - a) / 1e6 for a, b in zip(ts, ts[1:])]
        gaps_sorted = sorted(gaps_ms)
        span = (ts[-1] - ts[0]) / 1e9 if len(ts) > 1 else 0.0
        instances = extra.get("instances", [])
        # 录制期间到底转了多少 —— 由陀螺仪客观回答，不靠手感。
        # 标定要求 min_rotation_std_rad_s=0.05 rad/s ≈ 2.9°/s 的多轴激励；
        # 而运动模糊的门槛在几十 °/s 的量级。中位数落在这两者之间才算"转得合适"。
        if self.samples:
            mags = sorted(math.degrees(math.sqrt(s.gx ** 2 + s.gy ** 2 + s.gz ** 2))
                          for s in self.samples)
            n = len(mags)
            gyro_med = mags[n // 2]
            gyro_p95 = mags[min(n - 1, int(0.95 * n))]
            def _std(values):
                mean = sum(values) / len(values)
                return math.sqrt(sum((v - mean) ** 2 for v in values) / len(values))
            gyro_std = math.degrees(max(_std([s.gx for s in self.samples]),
                                        _std([s.gy for s in self.samples]),
                                        _std([s.gz for s in self.samples])))
        else:
            gyro_med = gyro_p95 = gyro_std = 0.0
        stats = {
            "port": self.port,
            "requested_hz": self.request_hz,
            "samples": len(self.samples),
            "frames_received": extra.get("frames", 0),
            "duplicates_skipped": extra.get("duplicates", 0),
            "span_s": span,
            "observed_hz": (len(ts) - 1) / span if span > 0 else 0.0,
            "median_gap_ms": gaps_sorted[len(gaps_sorted) // 2] if gaps_sorted else 0.0,
            "max_gap_ms": max(gaps_ms) if gaps_ms else 0.0,
            "gaps_over_20ms": sum(1 for g in gaps_ms if g > 20.0),
            "gaps_over_50ms": sum(1 for g in gaps_ms if g > 50.0),
            "gaps_over_150ms": sum(1 for g in gaps_ms if g > 150.0),
            "instances": instances,
            "mixed_instances": len(instances) > 1,
            "gyro_med_deg_s": gyro_med,
            "gyro_p95_deg_s": gyro_p95,
            "gyro_axis_std_deg_s": gyro_std,
            "fields_updated_zero": extra.get("fields_updated_zero", 0),
            "timestamps_interpolated": extra.get("interpolated", 0),
            "timestamps_clamped": extra.get("clamped", 0),
            "timestamp_anchor": "fc_usec" if self._timebase.get("ok") else "read_time",
            "timebase": dict(self._timebase),
            "decode_failures": extra.get("decode_fail", 0),
            "parser": extra.get("parser_stats", {}),
            "error": self._error,
        }
        self.stats = stats
        return stats

    # ---- 输出 ----
    def write_csv(self, path):
        path = Path(path)
        path.parent.mkdir(parents=True, exist_ok=True)
        with path.open("w", newline="", encoding="utf-8") as fh:
            writer = csv.writer(fh)
            writer.writerow(CSV_HEADER)
            for s in self.samples:
                writer.writerow([s.timestamp_ns, s.ax, s.ay, s.az, s.gx, s.gy, s.gz])
        # 时间基准的取证记录：这一份 imu.csv 的时间戳到底怎么来的、读线程积压多大。
        # 标定侧只读 imu.csv，这个文件是给人排查"跨录制时移漂"用的。
        if self._timebase:
            info = dict(self._timebase)
            info["csv"] = str(path)
            info["samples"] = len(self.samples)
            if self.stats:
                info["span_s"] = self.stats.get("span_s")
                info["timestamp_anchor"] = self.stats.get("timestamp_anchor")
            (path.parent / "imu_timebase.json").write_text(
                json.dumps(info, indent=2, ensure_ascii=False), encoding="utf-8")
        return path

    # ---- 体检结论 ----
    def verdict(self):
        """把统计翻译成"这份数据能不能拿去标定"。"""
        return verdict(self.stats or self._finish_stats())


def verdict(s):
    """把统计 dict 翻译成 (level, 说明)。level ∈ {pass, warn, fail}。"""
    if s.get("error"):
        return "fail", f"串口出错：{s['error']}"
    if s["samples"] < 100:
        return "fail", f"只收到 {s['samples']} 个样本，太少了 —— 飞控没在发 HIGHRES_IMU？"
    # 结构性缺陷优先报（它们是硬伤，转动量是运动建议）
    if s["mixed_instances"]:
        return "fail", (f"记录中途 IMU 实例变了 {s['instances']} —— 标定程序会拒绝，"
                        "请固定同一个实例后再采")
    clamped = s.get("timestamps_clamped", 0)
    if clamped > max(10, 0.01 * s["samples"]):
        return "warn", (f"有 {clamped}/{s['samples']} 个样本的时间戳因乱序被强制递增"
                        f"（占 {100*clamped/s['samples']:.1f}%）。"
                        "集中在开头是正常的积压突发；占比高说明串口读得断续，"
                        "换 USB 口/线，或把 IMU_REQUEST_HZ 降下来")
    if s.get("timestamp_anchor") == "read_time":
        reason = (s.get("timebase") or {}).get("reason") or "飞控没发 HIGHRES_IMU.time_usec"
        return "warn", (f"时间戳只能按串口读时间打（{reason}）：跨录制的相机-IMU 时移"
                        "会漂 10 ms 量级，标定时会被当成真实时移。查一下飞控固件是否在"
                        "HIGHRES_IMU 里发 time_usec")
    if s["gaps_over_150ms"]:
        return "warn", (f"有 {s['gaps_over_150ms']} 处超过 150 ms 的空洞。"
                        "用 configs/probe.yaml（max_imu_gap_s=0.15）仍可能被拒，"
                        "建议降低请求速率或换 USB 口/线")
    if s["observed_hz"] < 40:
        return "warn", (f"实测只有 {s['observed_hz']:.1f} Hz。50 Hz 能标，但平移外参精度"
                        "会从 ~3 mm 掉到 ~5 mm（实测）。想更好就提高 IMU_REQUEST_HZ "
                        "或把 SCHED_LOOP_RATE 调高")
    if s["gaps_over_50ms"]:
        return "warn", (f"实测 {s['observed_hz']:.1f} Hz，有 {s['gaps_over_50ms']} 处 >50 ms 空洞。"
                        "必须用 configs/probe.yaml（max_imu_gap_s=0.15），"
                        "否则默认的 0.05 会让标定直接拒绝")
    # 转动量：这是"你到底动了没有"的客观答案，不靠手感。
    # 标定要 min_rotation_std_rad_s=0.05 rad/s ≈ 2.9°/s 的多轴激励；
    # 运动模糊的门槛在几十 °/s 量级 —— 中位数落在两者之间才算"转得合适"。
    med = s.get("gyro_med_deg_s", 0.0)
    if med < 2.0:
        return "fail", (f"录制期间陀螺中位只有 {med:.1f}°/s —— 你基本上没动。"
                        "标定需要多轴转动（门槛是约 2.9°/s 的标准差），"
                        "静静举着是估不出外参的")
    if med > 60.0:
        return "warn", (f"录制期间陀螺中位 {med:.1f}°/s、P95 {s.get('gyro_p95_deg_s',0):.0f}°/s "
                        "—— 转得偏快，容易运动模糊。建议压低到 10~20°/s")
    if s.get("gyro_p95_deg_s", 0) > 120.0:
        return "warn", (f"陀螺峰值 P95 = {s['gyro_p95_deg_s']:.0f}°/s —— 有猛甩动作，"
                        "那几帧会糊掉并在窗口里凿出空洞。慢一点、匀一点")
    return "pass", f"实测 {s['observed_hz']:.1f} Hz，最大空洞 {s['max_gap_ms']:.1f} ms，干净"


def _format_timebase(stats) -> str:
    tb = stats.get("timebase") or {}
    if not tb:
        return "无锚定信息（旧数据）"
    if stats.get("timestamp_anchor") == "fc_usec" and tb.get("ok"):
        return (f"飞控 time_usec 下包络锚定（{tb.get('bins', 0)} 个包络箱，"
                f"漂移 {tb.get('drift_ppm', 0):+.1f} ppm；"
                f"读延迟中位 {tb.get('delay_median_ms', 0):.1f} ms / P95 {tb.get('delay_p95_ms', 0):.1f} ms；"
                f"读时间戳被回拨中位 {tb.get('shift_median_ms', 0):.1f} ms、最大 {tb.get('shift_max_ms', 0):.1f} ms）")
    reason = tb.get("reason") or "无飞控时间"
    return f"⚠️ 回退到串口读时间（{reason}）—— 跨录制的相机-IMU 时移会漂"


def format_stats(stats) -> str:
    return (
        f"  串口            {stats['port']}\n"
        f"  请求速率        {stats['requested_hz']:g} Hz\n"
        f"  实测速率        {stats['observed_hz']:.1f} Hz（中位间隔 {stats['median_gap_ms']:.2f} ms）\n"
        f"  样本 / 时长     {stats['samples']} 个 / {stats['span_s']:.2f} s\n"
        f"  HIGHRES_IMU    收到 {stats['frames_received']} 帧，"
        f"重复丢弃 {stats['duplicates_skipped']}，解码失败 {stats['decode_failures']}\n"
        f"  时间基准        {_format_timebase(stats)}\n"
        f"  时间戳插值      {stats['timestamps_interpolated']} 个样本用了批内插值"
        f"（其中 {stats['timestamps_clamped']} 个因乱序被强制递增）\n"
        f"  最大空洞        {stats['max_gap_ms']:.1f} ms"
        f"（>50ms: {stats['gaps_over_50ms']}, >150ms: {stats['gaps_over_150ms']}）\n"
        f"  IMU 实例        {stats['instances']}"
        f"{'  ⚠️ 混用了多个实例' if stats['mixed_instances'] else ''}\n"
        f"  转动量 |ω|      中位 {stats.get('gyro_med_deg_s',0):.1f} °/s"
        f"，P95 {stats.get('gyro_p95_deg_s',0):.0f} °/s"
        f"，各轴标准差 {stats.get('gyro_axis_std_deg_s',0):.1f} °/s  ← 你到底动了没有\n"
        f"  fields_updated  全零的帧 {stats['fields_updated_zero']}\n"
        f"  MAVLink         {stats['parser']}"
    )


# --------------------------------------------------------------------------
# 单独运行 = IMU 体检（不采图，只验证飞控这条链路）
# --------------------------------------------------------------------------
def _main():
    from capture import capture_config as cfg

    seconds = float(getattr(cfg, "IMU_CHECK_SECONDS", 10.0))
    print("=" * 74)
    print(f"IMU 单独体检 —— 录 {seconds:g} 秒，不采图")
    print("=" * 74)

    port = cfg.IMU_PORT
    if port == "auto":
        port, report = find_mavlink_port(baud=cfg.IMU_BAUD)
        if port is None:
            print("\n❌ 没找到在讲 MAVLink 的串口。看到的候选：")
            for k, v in report.items():
                print(f"     {k}: {v}")
            print("\n  检查：飞控通电了吗？USB 线是数据线不是充电线吗？"
                  "换一个 ttyACM 试试（在 capture_config.py 里写死 IMU_PORT）。")
            return 2
    rec = ImuRecorder(port, request_hz=cfg.IMU_REQUEST_HZ, baud=cfg.IMU_BAUD)
    print(f"打开 {port} ……")
    rec.start()
    try:
        time.sleep(seconds)
    except KeyboardInterrupt:
        print("\n提前结束")
    stats = rec.stop()

    print("\n采集统计")
    print(format_stats(stats))
    level, text = rec.verdict()
    icon = {"pass": "✅", "warn": "⚠️", "fail": "❌"}[level]
    print(f"\n{icon} 结论：{text}")
    if level != "fail":
        out = rec.write_csv(here / "imu_check.csv")
        print(f"样本已写出：{out}")
    return 0 if level != "fail" else 2


if __name__ == "__main__":
    sys.exit(_main())
