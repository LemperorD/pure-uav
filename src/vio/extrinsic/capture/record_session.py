"""★ 外参标定采集 —— PyCharm 右上角绿三角直接运行这个文件，不需要任何命令行参数。

它做五件事：
  1) 体检：相机在不在、飞控串口在不在、标定板配置和内参分辨率对不对
  2) 先起 IMU 录制线程，再拉起相机采集子进x`程（两个都用宿主机 CLOCK_REALTIME → 天然同源）
  3) 相机跑完 → 收 IMU → 写 imu.csv
  4) capture_manifest.json → images.csv
  5) 打印这次数据的体检报告 + 下一步该点哪个文件

相机那半段用的是 vendor/capture_calibration.py（原样复制的、已经跑过好几轮内参的脚本），
不重写、不魔改。所有可调参数都在 capture_config.py。
"""
from __future__ import annotations

import json
import subprocess
import sys
import time
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(ROOT))

import capture_config as cfg                       # noqa: E402
import image_health                                # noqa: E402
import imu_recorder                                # noqa: E402
import make_images_csv                              # noqa: E402

VENDOR = HERE / "vendor"
CAMERA_SCRIPT = VENDOR / "capture_calibration.py"


def hr(title=""):
    print("=" * 76)
    if title:
        print(title)
        print("=" * 76)


# --------------------------------------------------------------------------
# 体检
# --------------------------------------------------------------------------

def scan_cameras(expected="2bc5:0511"):
    """扫描 /dev/video*，返回 USB 身份匹配且是采集节点（index=0）的设备。"""
    vendor, product = expected.lower().split(":")
    found, seen = [], []
    for node in sorted(Path("/dev").glob("video*")):
        info = Path("/sys/class/video4linux") / node.name
        try:
            name = (info / "name").read_text().strip()
            index = (info / "index").read_text().strip()
            sysdev = (info / "device").resolve()
        except OSError:
            continue
        ids = None
        for directory in (sysdev, *sysdev.parents):
            if (directory / "idVendor").is_file():
                ids = ((directory / "idVendor").read_text().strip().lower(),
                       (directory / "idProduct").read_text().strip().lower())
                break
        seen.append(f"{node}  {name}  {ids}  index={index}")
        if ids == (vendor, product) and index == "0":
            found.append((node, name))
    return found, seen


def usb_devices(ids="2bc5:0511"):
    """按 USB 身份在 sysfs 里找设备（不看 /dev/video*）。

    用来区分两种完全不同的故障：
      - 设备在总线上、只是没有 /dev/video 节点  → uvcvideo 没绑上（被 libuvc 抢走后没还）
      - 设备根本不在总线上                      → 线没插好 / 口不供电
    返回 sysfs 子目录名，例如 ['3-6.1']。
    """
    vendor, product = ids.lower().split(":")
    hits = []
    for d in sorted(Path("/sys/bus/usb/devices").glob("*")):
        if ":" in d.name:          # 接口项（3-6.1:1.0）也带 idVendor，跳过
            continue
        try:
            v = (d / "idVendor").read_text().strip().lower()
            p = (d / "idProduct").read_text().strip().lower()
        except OSError:
            continue
        if (v, p) == (vendor, product):
            hits.append(d.name)
    return hits


def read_target(path):
    """读标定板配置。两种板子的字段语义不同，所以返回 dict 而不是裸元组。"""
    from calib.data import read_yaml
    entry = read_yaml(path)
    kind = entry.get("target_type")
    if kind in ("checkerboard", "chessboard"):
        cols, rows = int(entry.get("cols", 0)), int(entry.get("rows", 0))
        size = float(entry.get("square_size", 0))
        span = ((cols - 1) * size, (rows - 1) * size)     # 最外圈内角点之间的跨度
        label = (f"棋盘 {cols}x{rows} 内角点，格边 {size * 100:g} cm，"
                 f"角点跨度 {span[0] * 100:.1f}x{span[1] * 100:.1f} cm")
    else:
        cols, rows = int(entry.get("tagCols", 0)), int(entry.get("tagRows", 0))
        size = float(entry.get("tagSize", 0))
        spacing = float(entry.get("tagSpacing", 0.0))
        pitch = size * (1 + spacing)                      # 相邻 tag 的中心距
        span = ((cols - 1) * pitch + size, (rows - 1) * pitch + size)   # 整板外沿
        label = (f"AprilGrid {cols}x{rows} tag，tagSize {size * 100:g} cm，"
                 f"整板 {span[0] * 100:.1f}x{span[1] * 100:.1f} cm")
    return {"kind": kind, "cols": cols, "rows": rows, "size": size,
            "board_m": span, "label": label, "path": str(path)}


def preflight():
    """体检。通过则返回 (相机设备, 串口, 标定板信息)；不通过抛异常。"""
    print(f"会话           {cfg.SESSION_NAME}")
    print(f"会话目录       {cfg.SESSION_DIR}")
    print(f"结果目录       {cfg.RESULTS_DIR}")
    print(f"图像           {cfg.RESOLUTION}（{cfg.WIDTH}x{cfg.HEIGHT} @ {cfg.FPS} fps）")
    print(f"相机内参       {cfg.CAMERA_YAML}")
    print(f"标定板         {cfg.TARGET_YAML}")
    print(f"采集时长       {cfg.DURATION_S:g} s"
          f"（IMU 前后各多录 {cfg.IMU_PRIME_S:g} / {cfg.IMU_TAIL_S:g} s）")
    print()

    print("[1/4] 找相机……")
    if not CAMERA_SCRIPT.is_file():
        raise RuntimeError(f"缺少相机采集脚本 {CAMERA_SCRIPT}")
    cameras, seen = scan_cameras(cfg.CAMERA_USB_ID)
    if not cameras:
        print(f"  ❌ 没找到 USB 身份为 {cfg.CAMERA_USB_ID} 的相机；"
              f"/dev/video* 上现在挂着的是：")
        for line in (seen or ["     （一个都没有）"]):
            print(f"     {line}")
        on_bus = usb_devices(cfg.CAMERA_USB_ID)
        if on_bus:
            node = on_bus[0]
            print()
            print(f"  ⚠️  但它在 USB 总线上是在的：{', '.join(on_bus)}")
            print("     => 设备没掉线，是内核没给它建 /dev/video 节点。")
            print("     最常见原因：刚跑过 vio_online 或 orbbec_probe —— 它们用 libuvc")
            print("     直接抢走了这颗相机的 UVC 接口，退出时 uvcvideo 没能重新绑回去。")
            print()
            print("     恢复（任选一个，a 最稳）：")
            print("       a) 把相机的 USB 线拔掉、等 3 秒、插回去")
            print(f"       b) sudo sh -c 'echo 0 > /sys/bus/usb/devices/{node}/authorized;"
                  f" sleep 1; echo 1 > /sys/bus/usb/devices/{node}/authorized'")
            print("       c) sudo modprobe -r uvcvideo && sudo modprobe uvcvideo")
            print("     恢复后 v4l2-ctl --list-devices 里应该重新出现 2bc5:0511。")
            raise RuntimeError("相机在总线上但没有 /dev/video 节点（uvcvideo 没绑定回去）。")
        raise RuntimeError("相机没插或没被识别：USB 数据线插好了吗？"
                           "（笔记本内置摄像头是 04f2:b829，不是它）")
    device, name = cameras[0]
    print(f"  ✅ {device}  ({name})")

    print("[2/4] 找飞控串口……")
    port = cfg.IMU_PORT
    if port == "auto":
        port, report = imu_recorder.find_mavlink_port(baud=cfg.IMU_BAUD)
        if port is None:
            for k, v in report.items():
                print(f"     {k}: {v}")
            raise RuntimeError("没找到在讲 MAVLink 的串口：飞控通电了吗？USB 线是数据线吗？"
                               "或在 capture_config.py 里把 IMU_PORT 写死成 /dev/ttyACM0 试试。")
    else:
        print(f"  使用配置里写死的 {port}")
    print(f"  ✅ {port}")

    print("[3/4] 检查内参与标定板……")
    if not Path(cfg.CAMERA_YAML).is_file():
        raise RuntimeError(f"内参文件不存在：{cfg.CAMERA_YAML}")
    from calib.data import Camera
    cam = Camera.load(cfg.CAMERA_YAML)
    if tuple(cam.size) != (cfg.WIDTH, cfg.HEIGHT):
        raise RuntimeError(
            f"内参写的分辨率 {cam.size} 与本次采集的 {cfg.WIDTH}x{cfg.HEIGHT} 不一致。"
            "改 capture_config.py 的 RESOLUTION，或换一份匹配的内参 —— 工具不会自动缩放。")
    print(f"  ✅ 内参 {cam.size}，fx={cam.params[0]:.1f}")

    if not Path(cfg.TARGET_YAML).is_file():
        raise RuntimeError(f"标定板文件不存在：{cfg.TARGET_YAML}")
    board = read_target(cfg.TARGET_YAML)
    print(f"  ✅ 标定板 {board['label']}")
    if board["kind"] in ("checkerboard", "chessboard"):
        print("     ⚠️ 行列数必须和实物完全一致，否则检测出 0 帧。"
              "数的是【内角点】，不是格子数。")
    else:
        print("     ⚠️ tagRows/tagCols 是【tag 个数】不是内角点；tagSize 是黑框外沿，卡尺实测。")
        print(f"        按当前配置，整板 {board['board_m'][0] * 100:.1f}x"
              f"{board['board_m'][1] * 100:.1f} cm —— 和实物对不上就是 0 帧，"
              "而且平移会按比例错（tagSize 填错 10 倍 → 平移错 10 倍）。")

    print("[4/4] 检查目录……")
    session_dir = Path(cfg.SESSION_DIR)
    if session_dir.exists() and any(session_dir.iterdir()):
        if not cfg.REUSE_EXISTING_SESSION:
            raise RuntimeError(
                f"{session_dir} 里已经有东西了。把 capture_config.py 的 SESSION_NAME 改成 "
                "session02/03…，或把 REUSE_EXISTING_SESSION 设成 True 才允许复用。")
        print("  ⚠️ 复用已有目录（REUSE_EXISTING_SESSION=True）")
    session_dir.mkdir(parents=True, exist_ok=True)
    results_dir = Path(cfg.RESULTS_DIR)
    if results_dir.exists() and any(results_dir.iterdir()):
        print(f"  ⚠️ 结果目录 {results_dir} 非空 —— 标定程序会拒绝往里写，"
              "换个 SESSION_NAME 或把旧结果移走。")
    print(f"  ✅ {session_dir}")

    return device, port, board


# --------------------------------------------------------------------------
# 采集
# --------------------------------------------------------------------------

def camera_command(device, target_info):
    cols, rows = target_info["cols"], target_info["rows"]
    return [
        sys.executable, str(CAMERA_SCRIPT),
        "--device", str(device),
        "--output", str(cfg.SESSION_DIR),
        "--width", str(cfg.WIDTH), "--height", str(cfg.HEIGHT), "--fps", str(cfg.FPS),
        "--exposure", str(cfg.EXPOSURE),
        "--white-balance", str(cfg.WHITE_BALANCE),
        "--gain", str(cfg.GAIN),
        # 【2026-10-20】影响亮度的其余控件也从 capture_config 显式传下去，
        # 不再让采集脚本自己取 vendor/calibration_config.py 的默认值。
        "--brightness", str(cfg.BRIGHTNESS),
        "--contrast", str(cfg.CONTRAST),
        "--sharpness", str(cfg.SHARPNESS),
        "--power-line-frequency", str(cfg.POWER_LINE_FREQUENCY),
        "--backlight-compensation", str(cfg.BACKLIGHT_COMPENSATION),
        "--cols", str(cols), "--rows", str(rows),
        # 每帧都存：给一个远小于帧间隔的自动间隔，避免浮点比较漏掉某些帧
        "--auto-interval", f"{0.5 / cfg.FPS:.6f}",
        "--max-images", "0",
        "--duration", str(cfg.DURATION_S),
        "--auto-expose-max-exposure-ms", str(cfg.AUTO_EXPOSE_MAX_EXPOSURE_MS),
        "--auto-expose-target", str(cfg.AUTO_EXPOSE_TARGET),
        "--auto-expose-min-mean", str(cfg.AUTO_EXPOSE_MIN_MEAN),
    ] + ([] if cfg.AUTO_EXPOSE else ["--no-auto-expose"]
         ) + (["--allow-dark"] if cfg.ALLOW_DARK else []
    ) + (["--skip-exposure-check"] if cfg.SKIP_EXPOSURE_CHECK else []
         ) + ([] if cfg.SHOW_PREVIEW else ["--no-preview"])


def _pump_with_timer(proc):
    """边采集边报时：每秒一行"已过几秒 / 还剩几秒"。

    相机子进程每存一张图就打一行 "保存 ..."，30 fps 会把控制台刷爆，所以这里把它
    吞掉只计数；其它行（警告/错误/模式回读）照常转发。

    最后 10 秒开始每秒响一次终端提示音，这样不用盯着屏幕也能知道快结束了。
    """
    import queue
    import threading

    lines = queue.Queue()

    def _reader():
        try:
            for ln in proc.stdout:
                lines.put(ln.rstrip("\n"))
        finally:
            lines.put(None)

    threading.Thread(target=_reader, daemon=True).start()

    t0 = time.monotonic()
    saved = 0
    last_beat = -1.0
    last_qual = -99.0
    beeped = set()
    duration = float(cfg.DURATION_S)
    width = 78
    latest = None
    qual = None
    try:
        import cv2
        _fast = cv2.FastFeatureDetector.create(20, True)
    except Exception:                                          # noqa: BLE001
        cv2, _fast = None, None

    def _clear():
        print("\r" + " " * width + "\r", end="")

    while True:
        drained = False
        try:
            ln = lines.get(timeout=0.1)
            if ln is None:
                drained = True
            elif ln.startswith("保存 "):
                saved += 1
                parts = ln.split()
                if len(parts) >= 2:
                    latest = parts[1]
            elif ln.strip() == "@记录开始@":
                # 【2026-10-20 加】相机子进程锁定控件 + 曝光步进验证要花几秒，
                # 倒计时从前面对齐到真正开始存图的那一刻，否则"还剩 25s"其实只剩 21s，
                # 闭环的时长窗口整个错位。
                t0 = time.monotonic()
                last_beat = -1.0
                beeped.clear()
                _clear()
                print("   ▶ 曝光已锁定并验证，计时从现在开始")
            elif ln.strip():
                _clear()
                print(f"   {ln}")
        except queue.Empty:
            pass

        now = time.monotonic()
        elapsed = now - t0
        remain = duration - elapsed
        # 每 2 秒读一张刚存的图算亮度和特征数 —— 太暗就当场看得出，不用走完才发现
        if cv2 is not None and latest and now - last_qual >= 2.0:
            last_qual = now
            img = cv2.imread(latest, cv2.IMREAD_GRAYSCALE)
            if img is not None:
                nf = len(_fast.detect(img, None))
                qual = (float(img.mean()), nf)

        if now - last_beat >= 1.0:
            last_beat = now
            tail = f" | 还剩 {remain:4.1f}s" if remain >= 0 else " | 超时了"
            if qual is not None:
                mean_v, nf = qual
                mark = "OK" if (mean_v >= 40 and nf >= 400) else "!!"
                tail += f" | 亮度 {mean_v:3.0f} 特征 {nf:4d} {mark}"
            print(f"\r  [{elapsed:5.1f}s / {duration:g}s]  存图 {saved:4d}{tail}",
                  end="", flush=True)

        # 最后 10 秒：每秒一声；1 秒时连响三声
        sec = int(round(remain))
        if 0 <= sec <= 10 and sec not in beeped:
            beeped.add(sec)
            n = 3 if sec == 0 else 1
            sys.stdout.write("\a" * n)
            sys.stdout.flush()

        if drained:
            break

    try:
        proc.wait(timeout=5)
    except Exception:                                          # noqa: BLE001
        pass
    _clear()
    print(f"  采集结束：{saved} 张图，用时 {time.monotonic() - t0:.1f} s")
    print("  请确认你此刻正站在起点地贴上 —— 闭环要靠这个")


def record(device, port, target_info):
    """录一段。返回 (imu_stats, 相机返回码, 是否被中断)。imu.csv 在这里写。"""
    print()
    hr("开始采集")
    print(f"  相机 {device}   |   IMU {port}   |   {cfg.DURATION_S:g} 秒")
    if cfg.SHOW_PREVIEW:
        print("  预览窗口：空格手动存一张，Q 提前结束")
    print("  接下来拿着整套设备，在标定板前做连续、平滑、多轴转动 —— 要转得够多轴，别只平移")
    if target_info.get("kind") not in ("checkerboard", "chessboard"):
        print("  注意：下行每句末尾的 board=no 是采图脚本自带的【棋盘】检测器打的，"
              "对 AprilGrid 恒为 no，可以忽略")
    print()

    recorder = imu_recorder.ImuRecorder(port, request_hz=cfg.IMU_REQUEST_HZ, baud=cfg.IMU_BAUD)
    recorder.start()
    print(f"  IMU 已在录，先跑 {cfg.IMU_PRIME_S:g} 秒保证两路有重叠……")
    time.sleep(cfg.IMU_PRIME_S)

    proc, interrupted = None, False
    try:
        proc = subprocess.Popen(camera_command(device, target_info),
                                stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, bufsize=1)
        _pump_with_timer(proc)
    except KeyboardInterrupt:
        interrupted = True
        print("\n  收到中断，正在收尾……")
        if proc is not None and proc.poll() is None:
            proc.terminate()
            try:
                proc.wait(timeout=5)
            except subprocess.TimeoutExpired:
                proc.kill()
    finally:
        if not interrupted:
            time.sleep(cfg.IMU_TAIL_S)
        imu_stats = recorder.stop()
        csv_path = recorder.write_csv(Path(cfg.SESSION_DIR) / "imu.csv")
        imu_stats["csv"] = str(csv_path)

    return imu_stats, (proc.returncode if proc is not None else None), interrupted


# --------------------------------------------------------------------------
# 报告
# --------------------------------------------------------------------------

def report(imu_stats, cam_rc, img_stats, interrupted):
    hr("采集结果")
    print("IMU")
    print(imu_recorder.format_stats(imu_stats))
    imu_level, imu_text = imu_recorder.verdict(imu_stats)
    icon = {"pass": "✅", "warn": "⚠️", "fail": "❌"}[imu_level]
    print(f"  {icon} {imu_text}")

    print()
    print("图像")
    if img_stats is None:
        print("  ❌ 没有生成 images.csv（相机那一步没成功）")
    else:
        print(f"  张数 {img_stats['images']}，跨度 {img_stats['span_s']:.2f} s，"
              f"实测 {img_stats['observed_fps']:.1f} fps")
        print(f"  最大帧间隔 {img_stats['max_gap_ms']:.1f} ms"
              f"（>100ms: {img_stats['gaps_over_100ms']}, >350ms: {img_stats['gaps_over_350ms']}）")
        from calib.data import read_yaml
        if read_yaml(cfg.TARGET_YAML).get("target_type") in ("checkerboard", "chessboard"):
            print(f"  采集脚本自带的棋盘检出 {img_stats['board_detected']}/{img_stats['images']}"
                  "（仅供参考；标定程序用的是更鲁棒的检测器）")
        else:
            print("  采集脚本自带的棋盘检出  N/A（采图脚本只会找棋盘；AprilGrid 看下面的画面体检）")
        print(f"  结束状态 {img_stats['manifest_termination']}")
        manifest_path = Path(cfg.SESSION_DIR) / "capture_manifest.json"
        try:
            mm = json.loads(manifest_path.read_text(encoding="utf-8"))
            ae = mm.get("auto_exposure") or {}
            if ae.get("achieved_mean") is not None:
                print(f"  自动测光 {ae.get('status')}：曝光 {ae.get('exposure_ms')} ms，"
                      f"增益 {ae.get('gain')} → 灰度 {ae.get('achieved_mean')}"
                      f"（目标 {ae.get('target')}）")
                if ae.get("gain_range"):
                    print(f"           增益可用范围 {ae['gain_range']}，"
                          f"曝光范围 {ae['exposure_range']}")
                for row in (ae.get("ladder") or [])[-4:]:
                    note = f"  {row['note']}" if row.get("note") else ""
                    print(f"           {row.get('exposure', 0) * 0.1:5.1f} ms  "
                          f"gain {row.get('gain'):>4}  → 灰度 {row.get('mean')}{note}")
        except Exception:                                          # noqa: BLE001
            pass
        # 【2026-10-20 加】termination 里带 failed 就是采集脚本自己判失败了
        # （曝光没锁上 / 出流中断提前结束）。以前这种情况只打印不判死，
        # 于是会带着一份时长不足的数据往下跑标定。
        if str(img_stats["manifest_termination"] or "").startswith("failed"):
            print("  ❌ 相机采集没正常完成，这次数据不要用")

    slow = (img_stats is not None and img_stats["observed_fps"] < cfg.FPS * 0.6
            and img_stats["span_s"] > 2.0)
    if slow:
        print()
        print(f"  ⚠️ 实测 {img_stats['observed_fps']:.1f} fps，远低于请求的 {cfg.FPS} fps。")
        print("     这不是相机的问题，是保存循环太慢。按可能性排：")
        print("       1) SHOW_PREVIEW 是不是 True？那个 Tk/PIL 预览每帧要 ~300 ms")
        print("       2) RESOLUTION 是不是 1080p？改成 720p 能省一半")
        print("       3) 关掉 PyCharm 里别的吃 CPU 的东西")
        print("     数据本身仍可用于标定（下限是 35 帧有效检出），但帧率越低越难转得平滑。")

    # 亮度 + 检出：预览关掉之后，这里是唯一的"眼睛"。
    # 2026-10-15 那次 155 张 0 检出的教训：跑完标定才知道板子根本没进画面，
    # 白等几分钟。所以采完就当场判。
    health = None
    health_level = None
    session_dir = Path(cfg.SESSION_DIR)
    images = sorted((session_dir / "cam0").glob("*.png"))
    board = read_target(cfg.TARGET_YAML) if Path(cfg.TARGET_YAML).is_file() else None
    # 标定侧真正用的门槛要从 extrinsics_config 读，别在采集侧写死 ——
    # 2026-10-16 踩过：这边 2.0、标定侧 3.5，同一份数据算出不同的"有效帧"。
    try:
        import extrinsics_config as ec
        pnp_gate = float(getattr(ec.SETTINGS, "max_pnp_rms_px", 2.0))
        min_tags = int(getattr(ec.SETTINGS, "min_tags", 4))
    except Exception:                                          # noqa: BLE001
        pnp_gate, min_tags = 2.0, 4
    purpose = str(getattr(cfg, "CAPTURE_PURPOSE", "calibration")).lower()
    if images and board is not None and purpose == "vio":
        # 【2026-10-20 加】VIO 采集是"拿着相机走场景"，画面里没有 AprilGrid，
        # 检测器必然 0 检出。以前照样报 "0/20 检出" + 一整页"为什么一个 tag 都解不出"
        # + 一轮 1~2 分钟的逐帧复算 —— 全是误导，还白等。这里只报亮度。
        b = image_health.brightness_stats(images, sample=20)
        print()
        print("画面体检（VIO 用途：画面里没有标定板，不做 tag 检出）")
        if b:
            print(f"  亮度            p50={b['p50']:.0f}  p90={b['p90']:.0f}"
                  f"  p99={b['p99']:.0f}  均值={b['mean']:.0f}")
            level, text = image_health.brightness_verdict(b)
            icon = {"pass": "✅", "warn": "⚠️", "fail": "❌"}.get(level, "•")
            print(f"  {icon} {text}")
        print("  标定板检出      跳过（这份数据是给 VIO 用的，不是内参/外参标定）")
    elif images and board is not None:
        print()
        print("画面体检")
        health = image_health.analyze(images, cfg.TARGET_YAML, sample=20)
        # 再加一步"标定程序实际会用多少帧"。这一步才能在采完当场发现运动模糊 ——
        # 静态的检出率和亮度都看不出糊，只有 PnP 残差看得出（2026-10-15 的教训：
        # 检出 82%，但 17 帧 PnP 3~4 px 被剔，留出 5 处空洞，整份数据作废）。
        try:
            health["quality"] = image_health.calib_quality_stats(
                images, cfg.CAMERA_YAML, cfg.TARGET_YAML, window=24,
                min_tags=min_tags, max_pnp_rms=pnp_gate)
        except Exception as exc:                               # noqa: BLE001
            print(f"  （标定质量抽检跳过：{exc}）")
        health_level, health_actions = image_health.report(health)
        for action in health_actions:
            print(f"     → {action}")
        # ★ 逐帧复算：把标定程序真正会接受哪些帧、以及【空洞结构】算出来。
        # 采样 12 张的"检出率"预测不了 max_camera_gap_s —— 86% 检出率照样
        # 可能被 4 处 0.5~1.9 秒的盲区毁掉（2026-10-15 白跑了三次标定，每次 5 分钟）。
        print()
        print("逐帧复算（跑标定程序自己的检测+PnP，约 1~2 分钟）……")
        try:
            print(f"（按 extrinsics_config 的 max_pnp_rms_px={pnp_gate:g}、"
                  f"min_tags={min_tags} 复算）")
            tl = image_health.full_timeline(
                images, cfg.CAMERA_YAML, cfg.TARGET_YAML,
                max_pnp_rms=pnp_gate, min_tags=min_tags, log=lambda *_a: None)
            window_ok, window_note = image_health.report_timeline(tl)
            health["timeline"] = tl
            health["window_ok"] = window_ok
            if window_ok and tl.get("best"):
                a, span, n = tl["best"]
                print()
                print("  ★ 把 extrinsics_config.py 最下面那行 SETTINGS 换成：")
                print(f"      SETTINGS = Settings(start_s={a:.2f}, max_duration_s={span:.2f},"
                      " camera_rate_hz=60, max_pnp_rms_px=3.5)")
                print(f"    （用 {a:.2f}s 起、{span:.2f} 秒、{n} 帧那一段）")
        except Exception as exc:                               # noqa: BLE001
            print(f"  （逐帧复算跳过：{exc}）")

    print()
    hr("结论")
    fatal = False
    if interrupted:
        print("⚠️  这次是被中断的，数据可能不完整")
    if cam_rc not in (0, None):
        print(f"❌ 相机采集返回码 {cam_rc} —— 看上面相机子进程打印的报错")
        fatal = True
    if imu_level == "fail":
        print(f"❌ IMU：{imu_text}")
        fatal = True
    if img_stats is None:
        fatal = True
    elif str(img_stats.get("manifest_termination") or "").startswith("failed"):
        print(f"❌ 相机采集失败：{img_stats['manifest_termination']}")
        fatal = True
    elif img_stats["images"] < 35:
        print(f"❌ 图像只有 {img_stats['images']} 张，低于标定下限（35 张有效检出）")
        fatal = True

    window_ok = bool(health and health.get("window_ok"))
    if health is not None and not window_ok:
        print("❌ 逐帧复算：没有一段【连续】≥5 秒 且 ≥35 帧的可用窗口 ——")
        print("   带着这份数据去跑标定，只会得到「成功检测标定板帧之间存在大空洞」")
        print("   别浪费时间，直接重录。转动时让棋盘全程保持在画面中央、慢一点。")
        fatal = True
    if health is not None and health_level == "fail":
        d = health["detection"]
        print(f"❌ 画面：{d['detected']}/{d['tried']} 检出（{100*d['rate']:.0f}%）"
              " —— 带着这个去跑标定，只会得到「有效标定板帧只有 0」")
        print("   先点 capture/framing_check.py 把取景和曝光调对，别浪费一轮采集")
        fatal = True
    if not fatal:
        print("✅ 两份 CSV 就绪、画面过关、也有连续可用窗口 —— 可以往下标定了")
    return fatal


def pat_hint():
    import extrinsics_config as ec
    want = {"images": Path(cfg.SESSION_DIR) / "images.csv",
            "imu": Path(cfg.SESSION_DIR) / "imu.csv",
            "output": Path(cfg.RESULTS_DIR)}
    hr("标定那一步的路径")
    if all(Path(ec.PATHS[k]) == v for k, v in want.items()):
        print("✅ extrinsics_config.py 的 PATHS 已指向本次会话 —— "
              "直接点 calibrate_extrinsics.py 的绿三角即可。")
    else:
        print("⚠️  extrinsics_config.py 的 PATHS 还没指向本次会话。"
              "把它最下面那几行改成：")
        print()
        print(f'    "images":  ROOT / "data/{cfg.SESSION_NAME}/images.csv",')
        print(f'    "imu":     ROOT / "data/{cfg.SESSION_NAME}/imu.csv",')
        print(f'    "output":  ROOT / "results/{cfg.SESSION_NAME}",')
        print()
        print(f"  （当前 images 指向 {ec.PATHS['images']}）")


def next_steps():
    hr("接下来点哪个")
    print(f"  1) 打开 {ROOT / 'calibrate_extrinsics.py'}，点右上角绿三角")
    print("     —— 它读的是 extrinsics_config.py 里的 PATHS 和 SETTINGS。")
    print()
    print("     冒烟测试建议先把 extrinsics_config.py 最下面这一行：")
    print("         SETTINGS = Settings()")
    print("     改成（一行搞定，就是 configs/probe.yaml 里那两项）：")
    print("         SETTINGS = Settings(camera_rate_hz=30, max_imu_gap_s=0.15)")
    print("     不用的时候改回来即可。")
    print()
    print(f"  2) 然后看 {Path(cfg.RESULTS_DIR) / 'extraction.json'}")
    print("     里面逐帧有 detected 标记 —— 这就是冒烟测试的判据：")
    print("     true 占比 >80% → 链路通了；<50% → 板子/距离/光照有问题；")
    print("     0% → 标定板行列数或分辨率不对（iPad 那张 PNG 是 8x6，别用 8x11 去测）")


# --------------------------------------------------------------------------

def main():
    hr("相机–IMU 外参标定 · 采集")
    try:
        device, port, target_info = preflight()
    except Exception as exc:                                   # noqa: BLE001
        print()
        hr("❌ 体检没通过，没有开始采集")
        print(f"{exc}")
        return 2

    imu_stats, cam_rc, interrupted = record(device, port, target_info)

    session_dir = Path(cfg.SESSION_DIR)
    img_stats = None
    if (session_dir / "capture_manifest.json").is_file():
        try:
            img_stats = make_images_csv.build(session_dir)
        except make_images_csv.ManifestError as exc:
            print(f"❌ images.csv 生成失败：{exc}")
    else:
        print(f"⚠️  {session_dir}/capture_manifest.json 不存在 —— 相机一张都没存下。")

    fatal = report(imu_stats, cam_rc, img_stats, interrupted)
    pat_hint()
    next_steps()
    print()
    print(f"数据在 {session_dir}")
    return 1 if fatal else 0


if __name__ == "__main__":
    sys.exit(main())
