"""★ 采集链路的离线自检 —— 不插相机、不插飞控也能跑。PyCharm 点绿三角即可。

为什么需要它：
  采集这条链路要等你插上硬件才能真跑。但其中真正容易写错的部分（MAVLink 解码、
  批次内时间戳插值、去重、CSV 格式）全都可以离线验证。先在这里把它们钉死，
  等真机插上时，出问题的地方就只剩"线插没插对"。

四组测试：
  1. MAVLink 解析器 —— 喂真实串口抓包（testdata/raw_ttyACM0.bin）
  2. IMU 记录器全链路 —— 用伪终端(pty)造一个假飞控，走完 open/termios/select/
     read/parse/去重/插值/落盘 的完整路径；并验证时间戳是用飞控 time_usec 的
     下包络锚定到宿主时钟（不是"读到了才打戳"）
  3. manifest → images.csv —— 含错误路径（缺文件、时间戳乱序）
  4. 格式交接 —— 用标定程序自己的 loader 把生成的两份 CSV 读回来
"""
from __future__ import annotations

import os
import pty
import shutil
import statistics
import sys
import termios
import threading
import time
from pathlib import Path

import math

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(HERE / "vendor"))

import imu_recorder                                 # noqa: E402
import make_images_csv                              # noqa: E402
from mavlink_min import MavlinkParser, pack         # noqa: E402

WORK = HERE / "_selftest_tmp"
PASS, FAIL = [], []


def _fake_gyro(i, hz):
    """假飞控的陀螺读数（rad/s）：多轴正弦，中位约 13°/s —— 落在
    『够激励（≥2.9°/s 标准差）又不会糊（几十 °/s 以下）』的真实区间里。
    真实录制就是这种量级，所以夹具也该是这个量级。"""
    t = i / hz
    w = 2 * math.pi * 0.4
    return (0.22 * math.sin(w * t),
            0.18 * math.sin(1.3 * w * t + 1.0),
            0.14 * math.sin(0.7 * w * t + 2.0))


def check(name, ok, detail=""):
    print(f"  [{'PASS' if ok else 'FAIL'}] {name}{('  ' + detail) if detail else ''}")
    (PASS if ok else FAIL).append(name)
    return ok


def hr(title):
    print()
    print("=" * 76)
    print(title)
    print("=" * 76)


# --------------------------------------------------------------------------
# 1. 解析器 —— 真实抓包
# --------------------------------------------------------------------------

def test_parser():
    hr("[1/4] MAVLink 解析器 · 真实串口抓包")
    raw = (HERE / "testdata" / "raw_ttyACM0.bin").read_bytes()
    parser = MavlinkParser()
    frames = parser.feed(raw)
    names = sorted({f.name for f in frames})
    check("解析出帧", len(frames) > 50, f"{len(frames)} 帧，消息类型 {names}")
    check("无 CRC 错误", parser.stats["crc_bad"] == 0, str(parser.stats))
    check("无未知消息", parser.stats["unknown_msgid"] == 0)
    check("含 HEARTBEAT", "HEARTBEAT" in names)

    # 自己造一条 HIGHRES_IMU 再解回来，验证 IMU 那条路径的字段名没写错
    payload = {"time_usec": 123456789, "xacc": 0.5, "yacc": -1.25, "zacc": 9.81,
               "xgyro": 0.01, "ygyro": -0.02, "zgyro": 0.03,
               "fields_updated": 0x3F, "id": 0}
    back = MavlinkParser().feed(pack(105, payload, sysid=1, compid=1, seq=0))
    check("HIGHRES_IMU 往返", len(back) == 1 and back[0].name == "HIGHRES_IMU")
    if back:
        f = back[0].fields
        ok = (abs(f["zacc"] - 9.81) < 1e-4 and f["fields_updated"] == 0x3F
              and int(f["time_usec"]) == 123456789)
        check("字段名与数值正确", ok,
              f"zacc={f['zacc']} fields_updated={f['fields_updated']} time_usec={f['time_usec']}")


# --------------------------------------------------------------------------
# 2. 记录器全链路 —— 伪终端当假飞控
# --------------------------------------------------------------------------

def _configure_raw(fd):
    a = termios.tcgetattr(fd)
    a[0] = 0
    a[1] = 0
    a[2] = termios.CLOCAL | termios.CREAD | termios.CS8
    a[3] = 0
    a[4] = a[5] = termios.B115200
    termios.tcsetattr(fd, termios.TCSANOW, a)


def _fake_fc(master_fd, stop, *, hz, instance, repeat_every):
    """假飞控：周期发 HIGHRES_IMU，每隔 repeat_every 个样本重发一次同样的 time_usec
    （模拟"飞控发得比传感器快"），用来验证去重逻辑。

    注意：纯 Python 里每帧要 pack + 算 CRC + 写 pty，实际发不到设定的 hz。
    所以下面断言用的是"自洽性"（实测速率 == 去重后帧数 / 时长），
    而不是"实测速率 == 设定速率" —— 后者测的是这个夹具，不是被测代码。
    """
    time.sleep(0.3)                      # 等记录器把 termios 设好
    period = 1.0 / hz
    t0 = 1_000_000_000
    i = 0
    seq = 0
    started = time.monotonic()
    sent_usec = t0
    try:
        os.write(master_fd, pack(0, {"type": 2, "autopilot": 3, "mavlink_version": 3},
                                 sysid=1, compid=1, seq=0))
    except BlockingIOError:
        pass
    while not stop.is_set():
        try:
            if repeat_every and i % repeat_every == repeat_every - 1:
                pass                      # 重发上一个样本的 time_usec
            else:
                # time_usec 来自"墙上时间"，和真实飞控一致。不能写 t0 + i*period：
                # 纯 Python 的 pack+write 跑不到 200 Hz，那样飞控时钟会比真实时间慢
                # 得多，下包络拟合会把它当成几百 ppm 的晶振漂移而拒绝锚定。
                sent_usec = t0 + int((time.monotonic() - started) * 1e6)
            os.write(master_fd, pack(105, {
                "time_usec": sent_usec,
                "xacc": 0.01 * i, "yacc": -0.02 * i, "zacc": 9.80665,
                "xgyro": _fake_gyro(i, hz)[0], "ygyro": _fake_gyro(i, hz)[1],
                "zgyro": _fake_gyro(i, hz)[2],
                "fields_updated": 0x3F, "id": instance}, sysid=1, compid=1, seq=seq & 0xFF))
            seq += 1
        except BlockingIOError:
            pass                          # pty 缓冲满了就丢这一帧，等价于真链路的丢包
        i += 1
        time.sleep(period)


def test_recorder():
    hr("[2/4] IMU 记录器全链路 · 伪终端假飞控")
    hz, seconds, instance, repeat_every = 200.0, 2.5, 0, 5

    master, slave = pty.openpty()
    slave_path = os.ttyname(slave)
    _configure_raw(slave)                 # 必须在写入任何字节之前设成 raw
    os.set_blocking(master, False)

    stop = threading.Event()
    writer = threading.Thread(target=_fake_fc, args=(master, stop),
                              kwargs={"hz": hz, "instance": instance,
                                      "repeat_every": repeat_every}, daemon=True)
    writer.start()
    try:
        rec = imu_recorder.ImuRecorder(slave_path, request_hz=hz)
        rec.start()
        time.sleep(seconds)
        stats = rec.stop()
    finally:
        stop.set()
        writer.join(timeout=2)
        os.close(master)
        os.close(slave)

    print("  " + imu_recorder.format_stats(stats).replace("\n", "\n  "))
    # 自洽性检验：实测速率必须等于"去重后帧数 / 时长"。这条不依赖夹具跑多快。
    unique = stats["frames_received"] - stats["duplicates_skipped"]
    expected_hz = unique / stats["span_s"] if stats["span_s"] else 0.0
    check("收到了样本", stats["samples"] > 100, f"{stats['samples']} 个")
    check("样本数 == 收到帧数 - 重复数", stats["samples"] == unique,
          f"{stats['samples']} == {stats['frames_received']} - {stats['duplicates_skipped']}")
    check("实测速率与帧数自洽", abs(stats["observed_hz"] - expected_hz) < expected_hz * 0.05,
          f"{stats['observed_hz']:.1f} Hz vs 推算 {expected_hz:.1f} Hz")
    check("夹具本身跑得动（>100 Hz 消息率）",
          stats["frames_received"] / stats["span_s"] > 100,
          f"{stats['frames_received'] / stats['span_s']:.0f} Hz 消息率")
    check("重复样本被丢掉", stats["duplicates_skipped"] > 0,
          f"丢弃 {stats['duplicates_skipped']} 个")
    check("时间戳严格递增",
          all(b > a for a, b in zip([s.timestamp_ns for s in rec.samples],
                                    [s.timestamp_ns for s in rec.samples][1:])))
    check("没有大空洞", stats["max_gap_ms"] < 50.0, f"最大 {stats['max_gap_ms']:.1f} ms")
    check("批次内插值生效", stats["timestamps_interpolated"] > 0,
          f"{stats['timestamps_interpolated']} 个样本")
    check("实例识别正确", stats["instances"] == [instance] and not stats["mixed_instances"],
          str(stats["instances"]))
    check("没有因乱序被强制递增的样本（正常流不该出现）",
          stats["timestamps_clamped"] == 0, f"{stats['timestamps_clamped']} 个")

    # 落盘 → 用标定程序自己的 loader 读回来
    csv_path = rec.write_csv(WORK / "imu.csv")
    from calib.data import read_imu_csv
    stamps, values = read_imu_csv(csv_path, "ns")
    check("calib.data.read_imu_csv 能读", len(stamps) == len(rec.samples),
          f"{len(stamps)} 行")
    check("加速度是比力（含重力）",
          abs(statistics.median(abs(v[2]) for v in values) - 9.8) < 0.1,
          f"|az| 中位 {statistics.median(abs(v[2]) for v in values):.3f}")
    level, text = imu_recorder.verdict(stats)
    check("体检结论不是 fail", level != "fail", f"{level}: {text}")
    tb = stats.get("timebase") or {}
    check("时间基准锚定到飞控 time_usec（不是读时间）",
          stats.get("timestamp_anchor") == "fc_usec" and tb.get("ok"),
          f"{stats.get('timestamp_anchor')} / {tb.get('reason') or 'ok'}")
    check("下包络漂移在合理范围", abs(tb.get("drift_ppm", 0.0)) < 200.0,
          f"{tb.get('drift_ppm', 0.0):+.1f} ppm")
    check("锚定没有破坏严格递增",
          all(b > a for a, b in zip([x.timestamp_ns for x in rec.samples],
                                    [x.timestamp_ns for x in rec.samples][1:])))


def test_recorder_rejects_mixed_instances():
    """实例中途变化必须被判 fail —— 标定程序会拒绝这种数据，越早发现越好。"""
    master, slave = pty.openpty()
    slave_path = os.ttyname(slave)
    _configure_raw(slave)
    os.set_blocking(master, False)
    stop = threading.Event()

    def two_instances(master_fd, stop_evt):
        time.sleep(0.3)
        i = 0
        started = time.monotonic()
        while not stop_evt.is_set():
            os.write(master_fd, pack(105, {
                "time_usec": 1_000_000_000 + int((time.monotonic() - started) * 1e6),
                "xacc": 0.0, "yacc": 0.0, "zacc": 9.8,
                "xgyro": _fake_gyro(i, 200.0)[0], "ygyro": _fake_gyro(i, 200.0)[1],
                "zgyro": _fake_gyro(i, 200.0)[2],
                "fields_updated": 0x3F,
                "id": 1 if i > 200 else 0}, sysid=1, compid=1, seq=i & 0xFF))
            i += 1
            time.sleep(0.005)

    writer = threading.Thread(target=two_instances, args=(master, stop), daemon=True)
    writer.start()
    try:
        rec = imu_recorder.ImuRecorder(slave_path, request_hz=200.0)
        rec.start()
        time.sleep(1.8)
        stats = rec.stop()
    finally:
        stop.set()
        writer.join(timeout=2)
        os.close(master)
        os.close(slave)
    level, text = imu_recorder.verdict(stats)
    check("混用实例被判 fail", level == "fail" and stats["mixed_instances"],
          f"实例 {stats['instances']} → {level}: {text}")


# --------------------------------------------------------------------------
# 2b. 积压突发 → 时间戳倒退（2026-10-15 现场踩到的真实故障）
# --------------------------------------------------------------------------

def _fake_fc_backlog(master_fd, stop, *, backlog, period_us, stream_seconds=2.0):
    """假飞控，复现 2026-10-15 现场那次真实故障的成因：

      飞控在被 SET_MESSAGE_INTERVAL 请求之前就一直在采样，只是不发。
      一旦开始发，它先把这堆【历史积压】一次性倒出来（这段时间里
      它的 time_usec 推进得比墙上时间快得多），倒完才回到实时节奏。

    读线程于是会：先一次 read 拿到一大块，紧接着又一次 read 又拿到一大块，
    两次 read 的宿主时间只差一两毫秒，而每块的 time_usec 跨度有几百毫秒
    —— 按"以本批最后一帧为锚"插值，第二块的头几帧就退到了第一块之前。

    注意：这个夹具故意让 time_usec 跑得比墙上时间快得多（真飞控倒历史积压时
    不会这样）。所以它同时是"飞控时钟不可信"的负例：下包络拟合应当拒绝锚定、
    回退到读时间 —— 这正是我们要的行为。

    阶段 2 严格按真实时间发（time_usec 与墙上时间 1:1），这才是稳态。
    """
    time.sleep(0.3)
    usec = 1_000_000_000
    seq = 0

    # ---- 阶段 1：一次性倒出积压 ----
    blob = b""
    for _ in range(backlog):
        blob += pack(105, {
            "time_usec": usec,
            "xacc": 0.0, "yacc": 0.0, "zacc": 9.80665,
            "xgyro": _fake_gyro(seq, 200.0)[0], "ygyro": _fake_gyro(seq, 200.0)[1],
            "zgyro": _fake_gyro(seq, 200.0)[2],
            "fields_updated": 0x3F, "id": 0}, sysid=1, compid=1, seq=seq & 0xFF)
        usec += period_us
        seq += 1
    while blob and not stop.is_set():
        try:
            blob = blob[os.write(master_fd, blob):]
        except BlockingIOError:
            time.sleep(0.001)

    # ---- 阶段 2：实时节奏 ----
    deadline = time.monotonic() + stream_seconds
    while not stop.is_set() and time.monotonic() < deadline:
        try:
            os.write(master_fd, pack(105, {
                "time_usec": usec,
                "xacc": 0.0, "yacc": 0.0, "zacc": 9.80665,
                "xgyro": _fake_gyro(seq, 200.0)[0], "ygyro": _fake_gyro(seq, 200.0)[1],
                "zgyro": _fake_gyro(seq, 200.0)[2],
                "fields_updated": 0x3F, "id": 0}, sysid=1, compid=1, seq=seq & 0xFF))
            seq += 1
        except BlockingIOError:
            pass
        usec += period_us
        time.sleep(period_us / 1e6)


def test_recorder_burst_monotonic():
    hr("[2b/4] 飞控时钟异常 · 必须安全回退且时间戳仍严格递增")
    backlog, period_us = 60, 5000
    span_ms = backlog * period_us / 1e3
    print(f"  夹具：飞控先一次性倒出 {backlog} 个历史样本"
          f"（time_usec 跨度 {span_ms:.0f} ms），然后回到 200 Hz 实时流")

    master, slave = pty.openpty()
    slave_path = os.ttyname(slave)
    _configure_raw(slave)
    os.set_blocking(master, False)
    stop = threading.Event()
    writer = threading.Thread(target=_fake_fc_backlog, args=(master, stop),
                              kwargs={"backlog": backlog, "period_us": period_us},
                              daemon=True)
    writer.start()
    try:
        rec = imu_recorder.ImuRecorder(slave_path, request_hz=200.0)
        rec.start()
        time.sleep(2.6)
        stats = rec.stop()
    finally:
        stop.set()
        writer.join(timeout=2)
        os.close(master)
        os.close(slave)

    ts = [s.timestamp_ns for s in rec.samples]
    back = sum(1 for a, b in zip(ts, ts[1:]) if b <= a)
    print(f"  样本 {len(ts)} 个，被强制递增 {stats['timestamps_clamped']} 个，"
          f"兜住之后仍然倒退的 {back} 个")
    check("确实触发了乱序场景", stats["timestamps_clamped"] > 0,
          f"强制递增 {stats['timestamps_clamped']} 个（为 0 说明夹具没造出这个场景）")
    check("飞控时钟与宿主不一致时，时间基准安全回退到读时间",
          stats.get("timestamp_anchor") == "read_time",
          f"{stats.get('timestamp_anchor')} / {(stats.get('timebase') or {}).get('reason', '')}")
    check("最终时间戳严格递增", back == 0, f"倒退 {back} 个")
    check("时间戳落在合理范围内",
          all(1_600_000_000_000_000_000 < t < time.time_ns() + 10**9 for t in ts))
    check("整段跨度接近真实录制时长", 2.0 < stats["span_s"] < 4.0,
          f"{stats['span_s']:.2f} s")
    level, text = imu_recorder.verdict(stats)
    check("这种数据没被判成不可用", level != "fail", f"{level}: {text}")


# --------------------------------------------------------------------------
# 3. manifest → images.csv
# --------------------------------------------------------------------------

def _tiny_png(path, value):
    import cv2
    import numpy as np
    path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imencode(".png", np.full((8, 8), value, dtype="uint8"))[1].tofile(str(path))


# --------------------------------------------------------------------------
# 2c. 「你到底动了没有」—— 陀螺转动量判据
# --------------------------------------------------------------------------

def test_gyro_motion_verdict():
    hr("[2c/4] 转动量判据 · 陀螺客观回答『你到底动了没有』")
    base = {"error": "", "samples": 1000, "mixed_instances": False, "instances": [0],
            "timestamps_clamped": 0, "gaps_over_150ms": 0, "gaps_over_50ms": 0,
            "observed_hz": 200.0, "max_gap_ms": 5.0, "gyro_p95_deg_s": 25.0}
    cases = [("几乎没动（1°/s）", 1.0, 25.0, "fail"),
             ("转得合适（15°/s）", 15.0, 25.0, "pass"),
             ("转太快（90°/s）", 90.0, 110.0, "warn"),
             ("中位正常但峰值猛甩", 15.0, 200.0, "warn")]
    for name, med, p95, want in cases:
        got = imu_recorder.verdict(dict(base, gyro_med_deg_s=med, gyro_p95_deg_s=p95))[0]
        check(f"{name} → {want}", got == want, f"实得 {got}")


def test_manifest():
    hr("[3/4] capture_manifest.json → images.csv")
    import json
    session = WORK / "session"
    if session.exists():
        shutil.rmtree(session)
    (session / "cam0").mkdir(parents=True)

    stamps = [1_700_000_000_000_000_000 + i * 33_333_333 for i in range(6)]
    saved = []
    for i, stamp in enumerate(stamps):
        rel = f"cam0/{stamp}.png"
        _tiny_png(session / rel, i * 30)
        saved.append({"file": rel, "host_receive_time_ns": stamp, "board_detected": i % 2 == 0})
    (session / "capture_manifest.json").write_text(json.dumps(
        {"mode": {"width": 1920, "height": 1080}, "termination": "duration_reached",
         "saved": saved}), encoding="utf-8")

    stats = make_images_csv.build(session, log=lambda *a: None)
    check("生成了 images.csv", Path(stats["csv"]).is_file())
    check("张数正确", stats["images"] == 6, str(stats["images"]))
    check("实测帧率正确", abs(stats["observed_fps"] - 30.0) < 0.5,
          f"{stats['observed_fps']:.2f} fps")
    check("检出统计正确", stats["board_detected"] == 3, str(stats["board_detected"]))

    from calib.data import image_manifest
    pairs = image_manifest(stats["csv"], "ns")
    check("calib.data.image_manifest 能读", len(pairs) == 6)
    check("文件都能解析到", all(p.is_file() for _, p in pairs))
    check("时间戳原样保留", [s for s, _ in pairs] == stamps)

    # 错误路径
    bad = WORK / "bad_session"
    if bad.exists():
        shutil.rmtree(bad)
    (bad / "cam0").mkdir(parents=True)
    (bad / "capture_manifest.json").write_text(json.dumps(
        {"saved": [{"file": "cam0/nothere.png", "host_receive_time_ns": 1}]}), encoding="utf-8")
    try:
        make_images_csv.build(bad, log=lambda *a: None)
        check("缺失图像会被拒绝", False, "居然通过了")
    except make_images_csv.ManifestError as exc:
        check("缺失图像会被拒绝", "不存在" in str(exc), str(exc)[:60])

    (bad / "cam0" / "nothere.png").parent.mkdir(parents=True, exist_ok=True)
    _tiny_png(bad / "cam0" / "nothere.png", 0)
    (bad / "capture_manifest.json").write_text(json.dumps(
        {"saved": [{"file": "cam0/nothere.png", "host_receive_time_ns": 500},
                   {"file": "cam0/nothere.png", "host_receive_time_ns": 100}]}), encoding="utf-8")
    try:
        make_images_csv.build(bad, log=lambda *a: None)
        check("乱序时间戳会被拒绝", False, "居然通过了")
    except make_images_csv.ManifestError as exc:
        check("乱序时间戳会被拒绝", "递增" in str(exc), str(exc)[:60])


# --------------------------------------------------------------------------
# 4. 端到端：这两份 CSV 能不能被标定程序真正吃下去
# --------------------------------------------------------------------------

def test_end_to_end_formats():
    hr("[4/4] 格式交接 · 标定程序的原生 loader")
    from calib.data import Camera, read_imu_csv, image_manifest, read_yaml
    cam = Camera.load(ROOT / "configs" / "gemini_pro_rgb.yaml")
    check("内参可读", tuple(cam.size) == (1920, 1080), str(cam.size))
    target = read_yaml(ROOT / "configs" / "checkerboard.yaml")
    check("标定板可读", target.get("target_type") == "checkerboard",
          f"{target.get('cols')}x{target.get('rows')} @ {target.get('square_size')} m")
    probe = read_yaml(ROOT / "configs" / "probe.yaml")
    check("probe.yaml 可读", set(probe) == {"camera_rate_hz", "max_imu_gap_s"}, str(probe))

    imu_csv = WORK / "imu.csv"
    if imu_csv.is_file():
        stamps, values = read_imu_csv(imu_csv, "ns")
        check("自检产出的 imu.csv 能被标定程序读", len(stamps) == len(values) > 100,
              f"{len(stamps)} 行")


# --------------------------------------------------------------------------
# 5. 画面体检的判据
# --------------------------------------------------------------------------

def test_image_health():
    hr("[5/5] 画面体检 · 判据与阈值")
    import numpy as np
    import image_health as ih

    def bright(p90):
        return {"p50": p90 * 0.4, "p90": p90, "p99": p90 * 1.5, "mean": p90 * 0.5, "n": 8}

    check("p90=41 判为太暗（这次真实数据的值）",
          ih.brightness_verdict(bright(41))[0] == "dark")
    check("p90=90 判为可用", ih.brightness_verdict(bright(90))[0] == "ok")
    check("p90=250 判为可能过曝", ih.brightness_verdict(bright(250))[0] == "bright")
    check("读不到图时不炸", ih.brightness_verdict(None)[0] == "unknown")

    # 边缘余量：板子角点贴到/超出画面时必须是负数或接近 0
    inside = np.array([[100., 100.], [300., 100.], [300., 200.], [100., 200.]])
    check("完全在画面内的余量正确", ih._edge_margin(inside, (400, 600)) == 100.0,
          str(ih._edge_margin(inside, (400, 600))))
    over = np.array([[100., 100.], [300., 100.], [300., 260.], [100., 260.]])
    check("越出画面下边缘 → 负余量", ih._edge_margin(over, (200, 400)) < 0,
          str(ih._edge_margin(over, (200, 400))))

    # 采样函数
    check("_pick 抽样不重复且覆盖首尾",
          ih._pick(list(range(10)), 4) == [0, 3, 6, 9], str(ih._pick(list(range(10)), 4)))

    # 回归：2026-10-15 真实踩到 —— p90=43 被判 dark，但检出 10/12=83%，
    # 工具给了 warn 把人拦下来，其实是可用的。检出率必须是主判据。
    silent = lambda *a, **k: None                                    # noqa: E731
    det_ok = {"tried": 12, "detected": 10, "rate": 10 / 12, "kinds": ["SB"],
              "min_edge_margin_px": 152.0, "shape": (1080, 1920), "subgrid": {},
              "sharp_median": 20.0, "sharp_hit": 21.0, "sharp_miss": 15.0}
    real = {"images": 30, "board": "7x7",
            "brightness": {"p50": 20, "p90": 43, "p99": 64, "mean": 22.5, "n": 8},
            "detection": det_ok}
    check("暗但检出 83% → 判 ok（不再误拦）", ih.report(real, log=silent)[0] == "ok")

    ok_bright = dict(real, brightness={"p50": 60, "p90": 110, "p99": 200, "mean": 65, "n": 8})
    check("亮且检出 83% → 判 ok", ih.report(ok_bright, log=silent)[0] == "ok")

    zero_hit = dict(real, detection=dict(det_ok, tried=12, detected=0, rate=0.0, kinds=[],
                                         min_edge_margin_px=None,
                                         subgrid={"6x6": 1, "5x5": 4}))
    check("零检出 + 小网格能检出 → fail（提示切边）",
          ih.report(zero_hit, log=silent)[0] == "fail")

    # 回归：2026-10-15 真机那次 —— 检出 82%、亮度合格、静态清晰度也正常，
    # 但 17 帧 PnP 残差 3~4 px 被剔，留出 5 处 >0.35s 空洞，整份数据作废。
    # 只有 PnP 抽检 + 清晰度对比能提前发现。
    blurry = dict(real, quality={"window": 24, "detected": 16, "accepted": 14, "rejected": 2,
                                 "effective_rate": 14 / 24, "pnp_bad_median": 2.95,
                                 "sharp_ok": 27.0, "sharp_bad": 14.0})
    lvl, acts = ih.report(blurry, log=silent)
    check("检出高但被 PnP 剔的帧糊 → warn", lvl == "warn", lvl)
    check("并且给出'运动模糊/转太快'的动作",
          any("糊" in a for a in acts), str(acts)[:70])

    clean = dict(real, quality={"window": 24, "detected": 23, "accepted": 23, "rejected": 0,
                                "effective_rate": 23 / 24, "pnp_bad_median": None,
                                "sharp_ok": 27.0, "sharp_bad": None})
    check("PnP 抽检干净 → ok", ih.report(clean, log=silent)[0] == "ok")


# --------------------------------------------------------------------------

def main():
    print("=" * 76)
    print("采集链路离线自检 —— 不需要插相机、不需要插飞控")
    print("=" * 76)
    WORK.mkdir(parents=True, exist_ok=True)
    try:
        test_parser()
        test_recorder()
        test_recorder_rejects_mixed_instances()
        test_recorder_burst_monotonic()
        test_gyro_motion_verdict()
        test_manifest()
        test_end_to_end_formats()
        test_image_health()
    finally:
        pass

    hr("汇总")
    print(f"通过 {len(PASS)} 项，失败 {len(FAIL)} 项")
    if FAIL:
        print("失败项：")
        for name in FAIL:
            print(f"  - {name}")
    else:
        print("✅ 采集链路在没有硬件的情况下验证通过")
        print(f"   （临时文件留在 {WORK}，可以整个删掉）")
    return 1 if FAIL else 0


if __name__ == "__main__":
    sys.exit(main())
