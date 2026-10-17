#!/usr/bin/env python3
"""静置录 IMU —— 给 Allan 方差噪声标定采长时数据。

为什么录这条流（而不是飞控 SD 卡里的原始日志）：
    OpenVINS 的 `/imu0` 就是飞控的 `HIGHRES_IMU` @ 200 Hz —— 已经过飞控滤波、
    降采样、USB 传输。Kalibr/OpenVINS 的 `*_noise_density` 描述的正是【喂给估计器
    的那条流】。录它，测出来的参数就是能直接填进配置的；
    而且不用 SD 卡、不用管日志下载（那个之前反复失败过）。

怎么用（PyCharm 里直接点绿三角，不需要任何参数）:
    python record_static_imu.py             # 按下面 DURATION_H 录
    python record_static_imu.py 0.1         # 覆盖成 6 分钟（第一次先跑这个试通）
    python record_static_imu.py --probe     # 只探 5 秒，确认链路通不通

输出:
    imu/data/static_YYYYmmdd_HHMMSS.csv     host_ns,fc_time_us,ax,ay,az,gx,gy,gz
    imu/data/static_YYYYmmdd_HHMMSS.meta.json

设计要点（每条都是踩过坑换来的）:
  * 【流式写盘】旧版把样本堆在内存 list 里，8 小时 = 576 万个对象 ≈ 2 GB，会炸。
    这里每收一帧就写一行，每秒 flush、每分钟 fsync —— 中途崩了也能保住已有数据。
  * 【两个时间轴都记】`fc_time_us` 是飞控自己的微秒时钟（均匀、无 USB 抖动），
    分析时用它当时间基准；`host_ns` 保留下来用于和相机数据对齐。
  * 【批次内插值】串口一次 read 可能拿到好几帧。若给同批帧打同一个宿主时间戳，
    200 Hz 下批内就有 5~15 ms 的人为抖动。所以用飞控 time_usec 反推每帧的宿主时间。
  * 【严格递增】飞控被 SET_MESSAGE_INTERVAL 唤醒时会先倒出积压的历史样本，
    插出来的时间戳会倒退。这里强制递增并计数（占比高会报警）。
  * 【断线重连】8 小时里 USB 抽一下是常事。断开后自动重连，缝隙记在报告里。
"""
from __future__ import annotations

import csv
import json
import os
import shutil
import signal
import sys
import time
from datetime import datetime, timedelta
from pathlib import Path

# ==========================================================================
#  你要改的只有这一段
# ==========================================================================
DURATION_H = 8.0        # 录多久（小时）。★ 第一次先改成 0.1（6 分钟）试通
IMU_PORT = "auto"       # "auto" 自动找；或写死 "/dev/ttyACM0"（Linux）/ "COM3"（Windows）
REQUEST_HZ = 200.0      # 向飞控请求 HIGHRES_IMU 的速率
# ==========================================================================

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent                       # ~/calib/imu
OUT_DIR = ROOT / "data"
VENDOR = HERE / "vendor"

for _p in (str(HERE), str(VENDOR)):
    if _p not in sys.path:
        sys.path.insert(0, _p)
from mavlink_min import MavlinkParser, pack          # noqa: E402
from serial_port import open_port, find_mavlink_port, describe_candidates  # noqa: E402

MAV_CMD_SET_MESSAGE_INTERVAL = 511
MSG_HIGHRES_IMU = 105
IMU_FIELD_BITS = 0x3F                    # HIGHRES_IMU.fields_updated 的 xacc..zgyro 位
CSV_HEADER = ["host_ns", "fc_time_us", "ax", "ay", "az", "gx", "gy", "gz"]

FLUSH_EVERY_S = 1.0
FSYNC_EVERY_S = 60.0
PROGRESS_EVERY_S = 10.0
MIN_FREE_MB = 1024                       # 8 小时约 450 MB，留足余量
RECONNECT_WAIT_S = 2.0


# 串口相关的全部逻辑在 serial_port.py 里（Windows 用 pyserial，Linux 用 termios）。
# 本文件不直接碰 termios / select / /dev，所以两边都能跑。


# --------------------------------------------------------------------------
# 录制
# --------------------------------------------------------------------------

class Recorder:
    def __init__(self, port: str, out_csv: Path, request_hz: float, baud: int = 115200):
        self.port = port
        self.out_csv = out_csv
        self.request_hz = request_hz
        self.baud = baud
        self.stop = False
        self.stats = dict(
            samples=0, duplicates=0, clamped=0, decode_fail=0,
            backward_time_usec=0, fields_updated_zero=0,
            reconnects=0, read_errors=0, frames_other=0,
        )
        self.instances = set()
        self._parser = MavlinkParser()
        self._target = (1, 1)
        self._last_time_usec = None
        self._last_host_ns = None
        self._fc_first_us = None

    # ---- 主循环 ----
    def run(self, duration_h: float) -> None:
        deadline = time.monotonic() + duration_h * 3600.0
        t_start = time.time()

        with open(self.out_csv, "w", newline="") as fh:
            writer = csv.writer(fh)
            writer.writerow(CSV_HEADER)

            port = open_port(self.port, self.baud)
            print(f"\n  ✅ 串口已打开 {self.port}（USB CDC，波特率设置无实际作用）")
            print(f"  正在请求 HIGHRES_IMU @ {self.request_hz:g} Hz ……")
            n_seq = 0
            t_next_flush = time.monotonic() + FLUSH_EVERY_S
            t_next_fsync = time.monotonic() + FSYNC_EVERY_S
            t_next_progress = time.monotonic() + PROGRESS_EVERY_S
            t_next_req = 0.0

            try:
                while not self.stop:
                    now = time.monotonic()
                    if now >= deadline:
                        print("\n  ⏰ 到达设定时长，正常结束")
                        break

                    # 每 2 秒重发一次 SET_MESSAGE_INTERVAL：飞控重启/换口后能自愈
                    if now >= t_next_req:
                        n_seq = self._request_interval(port, n_seq)
                        t_next_req = now + 2.0

                    try:
                        data = port.read(8192)
                    except Exception as exc:                            # noqa: BLE001
                        self.stats["read_errors"] += 1
                        print(f"\n  ⚠️ 读串口失败：{exc} —— 尝试重连……")
                        port.close()
                        port = self._reconnect()
                        if port is None:
                            print("  ✗ 重连失败，停止录制（已录数据已保存）")
                            break
                        t_next_req = 0.0
                        data = b""
                    if data:
                        self._consume(data, writer)

                    now = time.monotonic()
                    if now >= t_next_flush:
                        fh.flush()
                        t_next_flush = now + FLUSH_EVERY_S
                    if now >= t_next_fsync:
                        fh.flush()
                        os.fsync(fh.fileno())
                        t_next_fsync = now + FSYNC_EVERY_S
                    if now >= t_next_progress:
                        self._progress(t_start, duration_h)
                        t_next_progress = now + PROGRESS_EVERY_S
            finally:
                port.close()
                fh.flush()
                os.fsync(fh.fileno())

        self.stats["wall_seconds"] = time.time() - t_start

    # ---- 内部 ----
    def _request_interval(self, port, n_seq: int) -> int:
        try:
            port.write(pack(76, {
                "param1": float(MSG_HIGHRES_IMU),
                "param2": 1e6 / self.request_hz if self.request_hz > 0 else -1.0,
                "param3": 0.0, "param4": 0.0, "param5": 0.0,
                "param6": 0.0, "param7": 0.0,
                "command": MAV_CMD_SET_MESSAGE_INTERVAL,
                "target_system": self._target[0], "target_component": self._target[1],
                "confirmation": 0, "unused1": 0, "unused2": 0, "unused3": 0,
            }, sysid=255, compid=190, seq=n_seq & 0xFF))
            return n_seq + 1
        except Exception:                                                # noqa: BLE001
            return n_seq

    def _reconnect(self):
        for attempt in range(1, 21):
            time.sleep(RECONNECT_WAIT_S)
            if self.stop:
                return None
            try:
                new_port = open_port(self.port, self.baud)
                self._parser = MavlinkParser()      # 残帧状态清掉
                self.stats["reconnects"] += 1
                print(f"  ✅ 第 {attempt} 次重连成功（缝隙已记录）")
                return new_port
            except Exception:                                        # noqa: BLE001
                if attempt % 5 == 0:
                    print(f"  …… 重连 {attempt}/20 仍失败")
        return None

    def _consume(self, data: bytes, writer) -> None:
        t_host = time.time_ns()
        frames = self._parser.feed(data)
        if not frames:
            return
        for frame in frames:
            if frame.name == "HEARTBEAT" and frame.compid == 1:
                self._target = (frame.sysid, frame.compid)

        hi = [f for f in frames if f.name == "HIGHRES_IMU"]
        self.stats["frames_other"] += len(frames) - len(hi)
        if not hi:
            return

        anchor_usec = int(hi[-1].fields.get("time_usec", 0) or 0)
        for frame in hi:
            f = frame.fields
            try:
                t_usec = int(f.get("time_usec", 0) or 0)
                if t_usec and t_usec == self._last_time_usec:
                    self.stats["duplicates"] += 1
                    continue
                if t_usec and self._last_time_usec is not None and t_usec < self._last_time_usec:
                    self.stats["backward_time_usec"] += 1
                if t_usec:
                    self._last_time_usec = t_usec

                if (int(f.get("fields_updated", 0) or 0) & IMU_FIELD_BITS) == 0:
                    self.stats["fields_updated_zero"] += 1

                if t_usec and anchor_usec and anchor_usec >= t_usec:
                    ts = t_host - (anchor_usec - t_usec) * 1000
                else:
                    ts = t_host
                if self._last_host_ns is not None and ts <= self._last_host_ns:
                    ts = self._last_host_ns + 1
                    self.stats["clamped"] += 1
                self._last_host_ns = ts

                if self._fc_first_us is None and t_usec:
                    self._fc_first_us = t_usec
                self.instances.add(int(f.get("id", 0) or 0))

                writer.writerow([
                    ts, t_usec,
                    f"{float(f['xacc']):.9g}", f"{float(f['yacc']):.9g}", f"{float(f['zacc']):.9g}",
                    f"{float(f['xgyro']):.9g}", f"{float(f['ygyro']):.9g}", f"{float(f['zgyro']):.9g}",
                ])
                self.stats["samples"] += 1
            except (KeyError, TypeError, ValueError):
                self.stats["decode_fail"] += 1

    def _progress(self, t_start: float, duration_h: float) -> None:
        el = time.time() - t_start
        n = self.stats["samples"]
        hz = n / el if el > 0 else 0.0
        remain = max(0.0, duration_h * 3600.0 - el)
        pct = 100.0 * el / (duration_h * 3600.0) if duration_h > 0 else 100.0
        flag = ""
        if hz < 0.9 * self.request_hz:
            flag = f"  ⚠️ 实际速率低于请求值 {self.request_hz:g} Hz"
        print(f"  [{_hms(el)} / {_hms(duration_h * 3600.0)}] {pct:5.1f}%  "
              f"{n:,} 样本  {hz:6.1f} Hz  剩 {_hms(remain)}"
              f"{'  重连=' + str(self.stats['reconnects']) if self.stats['reconnects'] else ''}"
              f"{flag}")

    # ---- 收尾 ----
    def meta(self) -> dict:
        m = dict(self.stats)
        m["port"] = self.port
        m["request_hz"] = self.request_hz
        m["instances"] = sorted(self.instances)
        m["parser_stats"] = dict(self._parser.stats)
        return m


def _hms(sec: float) -> str:
    sec = int(max(0, sec))
    return f"{sec // 3600:02d}:{(sec % 3600) // 60:02d}:{sec % 60:02d}"


# --------------------------------------------------------------------------
# 入口
# --------------------------------------------------------------------------

def _parse_args(argv):
    hours = DURATION_H
    probe = "--probe" in argv
    rest = [a for a in argv if not a.startswith("--")]
    if rest:
        try:
            hours = float(rest[0])
        except ValueError:
            print(f"⚠️ 看不懂的时长参数 {rest[0]!r}，用默认 {DURATION_H} 小时")
    return hours, probe


def main(argv=None) -> int:
    argv = list(sys.argv[1:] if argv is None else argv)
    hours, probe = _parse_args(argv)

    print("=" * 74)
    print("  静置录 IMU —— 给 Allan 方差噪声标定采数据")
    print("=" * 74)
    print(f"  时长      {hours:g} 小时 ({_hms(hours * 3600)})")
    print(f"  请求速率  {REQUEST_HZ:g} Hz")
    print(f"  输出目录  {OUT_DIR}")
    print()

    # --- 1. 物理条件提醒 ---
    print("【开始前请确认】（这些条件直接影响测量结果）")
    print("  □ 飞控平放在桌上，USB 供电，不接电机、不装桨")
    print("  □ 桌面稳固；人不要碰桌子；附近不要有风扇/空调直吹")
    print("  □ 笔记本接着电源，电源管理里【关掉自动休眠/息屏】")
    print("  □ 室温稳定（MEMS 零偏随温度漂）")
    print()

    # --- 2. 磁盘空间 ---
    OUT_DIR.mkdir(parents=True, exist_ok=True)
    free_mb = shutil.disk_usage(OUT_DIR).free / 1e6
    need_mb = hours * 3600 * REQUEST_HZ * 90 / 1e6      # 每行约 90 字节
    print(f"  磁盘可用 {free_mb:,.0f} MB，预计需要约 {need_mb:,.0f} MB")
    if free_mb < max(MIN_FREE_MB, need_mb * 2):
        print("  ✗ 空间不够，先清一下磁盘")
        return 2
    print()

    # --- 3. 找串口 ---
    port = IMU_PORT
    if port == "auto":
        port = find_mavlink_port()
        if not port:
            return 3
    print(f"  使用串口  {port}")

    # --- 4. 试录 5 秒，确认链路 ---
    probe_csv = OUT_DIR / "_probe.csv"
    print("\n  先试录 5 秒，确认链路真的通……")
    rc = Recorder(port, probe_csv, REQUEST_HZ)
    _install_signals(rc)
    rc.run(5.0 / 3600.0)
    n = rc.stats["samples"]
    hz = n / max(1e-9, rc.stats["wall_seconds"])
    print(f"  试录结果：{n} 样本 / {hz:.1f} Hz")
    if n < 50:
        print("  ✗ 基本收不到 HIGHRES_IMU。检查：飞控在跑吗？串口对吗？")
        print(f"     （试录数据留在 {probe_csv}，可以直接看）")
        return 4
    if hz < 0.9 * REQUEST_HZ:
        print(f"  ⚠️ 只有 {hz:.1f} Hz，低于请求的 {REQUEST_HZ:g} Hz。")
        print("     可能是别的进程在抢串口，或飞控的 SRx_ 流速率参数偏低。")
        print("     继续录也可以，但要在报告里说明实际速率。")
    probe_csv.unlink(missing_ok=True)

    if probe:
        print("\n  --probe 模式：链路确认完毕，不正式录制。")
        return 0

    # --- 5. 正式录制 ---
    stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    out_csv = OUT_DIR / f"static_{stamp}.csv"
    meta_path = OUT_DIR / f"static_{stamp}.meta.json"

    print("\n" + "=" * 74)
    print(f"  开始正式录制 → {out_csv}")
    print(f"  预计结束 {datetime.now() + timedelta(hours=hours):%Y-%m-%d %H:%M:%S}")
    print("  中途按 Ctrl-C 可以安全停止（已录数据会保留）")
    print("=" * 74)

    rec = Recorder(port, out_csv, REQUEST_HZ)
    _install_signals(rec)
    rec.run(hours)

    meta = rec.meta()
    meta.update(
        started=datetime.now().isoformat(timespec="seconds"),
        requested_hours=hours, csv=str(out_csv),
        csv_header=CSV_HEADER,
        tool="record_static_imu.py", interrupted=bool(rec.stop),
    )
    meta_path.write_text(json.dumps(meta, indent=2, ensure_ascii=False))

    # --- 6. 报告 ---
    st = rec.stats
    el = max(1e-9, st["wall_seconds"])
    print("\n" + "=" * 74)
    print("  录制结束")
    print("=" * 74)
    print(f"  CSV       {out_csv}")
    print(f"  元数据    {meta_path}")
    print(f"  样本      {st['samples']:,}")
    print(f"  时长      {_hms(el)}   实际速率 {st['samples'] / el:.2f} Hz")
    print(f"  重复帧    {st['duplicates']}      时间戳兜底 {st['clamped']}")
    print(f"  解码失败  {st['decode_fail']}      time_usec 倒退 {st['backward_time_usec']}")
    print(f"  重连      {st['reconnects']}       读错误 {st['read_errors']}")
    print(f"  IMU 实例  {sorted(rec.instances)}")
    print()

    warn = []
    if st["samples"] / el < 0.9 * REQUEST_HZ:
        warn.append(f"实际速率 {st['samples'] / el:.1f} Hz 明显低于请求值")
    if st["backward_time_usec"] > 0:
        warn.append(f"有 {st['backward_time_usec']} 次 time_usec 倒退（飞控时钟跳变？）")
    if st["decode_fail"] > 0.001 * max(1, st["samples"]):
        warn.append(f"解码失败 {st['decode_fail']} 次，占比偏高")
    if st["reconnects"]:
        warn.append(f"中途断线重连 {st['reconnects']} 次（数据里有缝隙，分析时会自动切段）")
    if warn:
        print("  ⚠️ 需要注意：")
        for w in warn:
            print(f"     · {w}")
    else:
        print("  ✅ 没有发现异常")

    print()
    print("  下一步：跑分析")
    print("     python analyze_static_noise.py")
    return 0


def _install_signals(rec: Recorder) -> None:
    def handler(signum, frame):                                        # noqa: ARG001
        if not rec.stop:
            print("\n  ⏹  收到停止信号，正在安全收尾……（再按一次强制退出）")
            rec.stop = True
        else:
            raise KeyboardInterrupt
    for sig in (signal.SIGINT, signal.SIGTERM):
        try:
            signal.signal(sig, handler)
        except (ValueError, OSError):
            pass


if __name__ == "__main__":
    raise SystemExit(main())
