#!/usr/bin/env python3
"""AprilGrid 实时预览 —— 举着板子调距离/角度/曝光，屏幕上直接画框报数。

采图脚本自带的那个预览是**棋盘**检测器，对着 AprilGrid 永远只显示
`chessboard 6x6: no`，一个绿点都不会有，却每帧要 ~300 ms。所以正式采集时它是关的。
这个脚本补上"能看见"这件事：用标定程序自己的 Target（aruco 36h11）实时检测，
绿框 = 认出来的 tag，HUD 告诉你三个数够不够：

    tags   这一帧认出几个 tag（要 ≥ min_tags，默认 8）
    tag px 每个 tag 在**全分辨率**下多少像素（≥45 舒服，<30 解不出）
    PnP    平面 PnP 残差（>3 px 说明 yaml 的 tagSpacing / tag_rotation_deg 填错了）

操作：Q / Esc 退出，S 存一张 1920x1080 原图（不画框）到 data/_preview/

自检（不插相机）：python capture/preview_aprilgrid.py --self-test
"""
from __future__ import annotations

import argparse
import fcntl
import struct
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent                # extrinsic/：唯一需要上 sys.path 的路径
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from capture import capture_config as cfg                 # noqa: E402
from capture import record_session as rs                  # noqa: E402

SAVE_DIR = cfg.DATA_ROOT / "_preview"
DISPLAY_WIDTH = 880          # 预览窗宽度（只影响显示，检测始终在全分辨率上跑）

# ---- 最小 V4L2 控件封装（和 vendor/capture_calibration.py 同一套 cid） -----------
VIDIOC_G_CTRL = 0xC008561B   # _IOWR('V', 27, struct v4l2_control)
VIDIOC_S_CTRL = 0xC008561C
V4L2_CTRL_FMT = "Ii"
CIDS = {
    "exposure_auto": 0x009A0901,
    "exposure": 0x009A0902,
    "auto_white_balance": 0x0098090C,
    "white_balance_temperature": 0x0098091A,
    "gain": 0x00980913,
}


def set_ctrl(fd, cid, value):
    fcntl.ioctl(fd, VIDIOC_S_CTRL, struct.pack(V4L2_CTRL_FMT, cid, int(value)))


def get_ctrl(fd, cid):
    raw = fcntl.ioctl(fd, VIDIOC_G_CTRL, struct.pack(V4L2_CTRL_FMT, cid, 0))
    return struct.unpack(V4L2_CTRL_FMT, raw)[1]


def lock_controls(device):
    """把曝光/白平衡/增益钉死 —— 让预览看到的亮度和正式采集一致。"""
    try:
        import os
        fd = os.open(str(device), os.O_RDWR | os.O_NONBLOCK)   # 和 vendor 脚本同一套打开方式
    except OSError as exc:
        print(f"  ⚠️ 打不开 {device} 调 V4L2 控件：{exc}")
        print("     预览亮度可能和正式采集不一致（正式采集那边仍然会锁）")
        return
    try:
        wanted = [("exposure_auto", 1), ("exposure", int(cfg.EXPOSURE)),
                  ("auto_white_balance", 0),
                  ("white_balance_temperature", int(cfg.WHITE_BALANCE)),
                  ("gain", int(cfg.GAIN))]
        failed = 0
        for name, value in wanted:
            try:
                set_ctrl(fd, CIDS[name], value)
                actual = get_ctrl(fd, CIDS[name])
                if actual != value:
                    failed += 1
                    print(f"  ⚠️ {name} 想要 {value}，实际 {actual}")
            except OSError as exc:
                failed += 1
                print(f"  ⚠️ {name}={value} 设置失败：{exc}")
        if failed:
            print("  ⚠️ 有控件没锁上 —— 预览亮度和正式采集可能不一致（正式采集那边会再锁一次）")
        else:
            print(f"  控件已锁：曝光 {cfg.EXPOSURE}（100µs）, 白平衡 {cfg.WHITE_BALANCE}K, 增益 {cfg.GAIN}")
    finally:
        import os
        os.close(fd)


def open_camera(device, width, height, fps):
    cap = cv2.VideoCapture(device, cv2.CAP_V4L2)
    if not cap.isOpened():
        raise RuntimeError(f"无法打开 {device}；关掉 Cheese / 别的占用相机的程序再试")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, height)
    cap.set(cv2.CAP_PROP_FPS, fps)
    got = (round(cap.get(cv2.CAP_PROP_FRAME_WIDTH)), round(cap.get(cv2.CAP_PROP_FRAME_HEIGHT)))
    if got != (width, height):
        cap.release()
        raise RuntimeError(f"相机实际 {got}，不是请求的 {width}x{height}")
    return cap


def analyse(target, camera, gray, max_points=0):
    """在全分辨率灰度图上检测 + 解平面位姿。返回 (uv|None, 每tag边长px, PnP残差|None)。"""
    from calib.data import make_observation
    result = target.detect(gray)
    if result is None:
        return None, None, None
    uv = np.asarray(result[1], float)
    quads = uv.reshape(-1, 4, 2)
    tag_px = float(np.median(np.linalg.norm(quads[:, 1] - quads[:, 0], axis=1)))
    obs = make_observation(0, result[0], uv, camera, None, checker=False,
                           max_points=max_points)
    return uv, tag_px, (None if obs is None else float(obs.pnp_rms))


def render(gray, uv, min_tags, tag_px, rms, fps, width=DISPLAY_WIDTH):
    """把全分辨率灰度图缩到 width 宽，叠上检测框和 HUD。返回 (BGR图, n_tag)。"""
    h, w = gray.shape[:2]
    k = width / w
    small = cv2.resize(gray, (width, round(h * k)), interpolation=cv2.INTER_AREA)
    canvas = cv2.cvtColor(small, cv2.COLOR_GRAY2BGR)
    n_tag = 0 if uv is None else len(uv) // 4
    if uv is not None:
        colour = (0, 230, 0) if n_tag >= min_tags else (0, 200, 255)
        for quad in np.asarray(uv, float).reshape(-1, 4, 2) * k:
            pts = np.rint(quad).astype(int)
            cv2.polylines(canvas, [pts], True, colour, 2)
            cv2.circle(canvas, tuple(pts[0]), 3, (0, 0, 255), -1)
    hud = [
        (f"tags {n_tag:2d} / need >={min_tags}", (0, 230, 0) if n_tag >= min_tags else (0, 200, 255)),
        (f"tag size {tag_px:5.1f} px  (want >=45)" if tag_px else "tag size    -  ", 
         (0, 230, 0) if (tag_px and tag_px >= 45) else ((0, 0, 255) if tag_px else (200, 200, 200))),
        (f"PnP {rms:5.2f} px  (want <3)" if rms else "PnP    -   ",
         (0, 230, 0) if (rms and rms < 3.0) else ((0, 0, 255) if rms else (200, 200, 200))),
        (f"{fps:4.1f} fps   Q quit   S save", (200, 200, 200)),
    ]
    overlay = canvas.copy()
    cv2.rectangle(overlay, (0, 0), (420, 24 + 27 * len(hud)), (0, 0, 0), -1)
    canvas = cv2.addWeighted(overlay, 0.45, canvas, 0.55, 0)
    for i, (text, colour) in enumerate(hud):
        y = 28 + 27 * i
        cv2.putText(canvas, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.68, (0, 0, 0), 4)
        cv2.putText(canvas, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.68, colour, 2)
    return canvas, n_tag


def synthetic_frame(cols=6, rows=6, tag_px=100, spacing=0.56):
    """按 yaml 的行列数/间隙比例画一块 AprilGrid（tag 用 generateImageMarker，不旋转）。"""
    d = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    gap = max(1, int(round(tag_px * spacing)))
    pitch = tag_px + gap
    board = np.full((rows * pitch - gap, cols * pitch - gap), 225, np.uint8)
    for y in range(rows):
        for x in range(cols):
            marker = cv2.aruco.generateImageMarker(d, y * cols + x, tag_px)
            board[(rows - 1 - y) * pitch:(rows - 1 - y) * pitch + tag_px,
                  x * pitch:x * pitch + tag_px] = marker
    frame = np.full((1080, 1920), 170, np.uint8)
    y0, x0 = (1080 - board.shape[0]) // 2, (1920 - board.shape[1]) // 2
    frame[y0:y0 + board.shape[0], x0:x0 + board.shape[1]] = board
    return frame


def self_test(args):
    """不开相机：合成一块 6x6 AprilGrid，走一遍检测+叠加+存图。"""
    from calib.data import Camera, Target, read_yaml
    entry = read_yaml(args.target)
    # 合成的板子用 cv2.aruco.generateImageMarker 画，每个 tag 是"正"的，
    # 所以这里必须把 Kalibr PDF 的 180° 设成 0，否则 PnP 会报一百多像素。
    entry["tag_rotation_deg"] = 0
    target = Target(entry, min_tags=1)
    camera = Camera.load(args.camera)
    gray = synthetic_frame(int(entry["tagCols"]), int(entry["tagRows"]),
                           spacing=float(entry["tagSpacing"]))
    uv, tag_px, rms = analyse(target, camera, gray)
    canvas, n_tag = render(gray, uv, 8, tag_px, rms, 0.0)
    out = SAVE_DIR / "selftest_preview.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(out), canvas)
    print(f"  检出 {n_tag} 个 tag，tag {tag_px and round(tag_px, 1)} px，"
          f"PnP {rms and round(rms, 2)} px")
    print(f"  叠加图已存 {out}")
    return 0 if n_tag >= 8 and (rms or 99) < 3 else 1


def main(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--target", type=Path, default=ROOT / "configs" / "aprilgrid.yaml")
    ap.add_argument("--camera", type=Path, default=cfg.CAMERA_YAML)
    ap.add_argument("--headless", action="store_true", help="不开窗口，只在终端打印统计")
    ap.add_argument("--self-test", action="store_true", help="不开相机，用合成板验证叠加")
    args = ap.parse_args(argv)
    if args.self_test:
        return self_test(args)

    from calib.data import Camera, Target, read_yaml
    entry = read_yaml(args.target)
    if entry.get("target_type") != "aprilgrid":
        print(f"⚠️ {args.target} 不是 AprilGrid；这个预览只对 AprilGrid 有意义")
    target = Target(entry, min_tags=1)                # 预览要能看到"半块板"
    min_tags = 8
    try:
        import extrinsics_config as ec
        min_tags = int(getattr(ec.SETTINGS, "min_tags", 8))
    except Exception:                                  # noqa: BLE001
        pass
    camera = Camera.load(args.camera)

    cameras, seen = rs.scan_cameras(cfg.CAMERA_USB_ID)
    if not cameras:
        print(f"❌ 没找到 USB {cfg.CAMERA_USB_ID} 的相机。当前 /dev/video*：")
        for line in (seen or ["     （一个都没有）"]):
            print(f"     {line}")
        return 2
    device, name = cameras[0]
    print("=" * 74)
    print(f"AprilGrid 实时预览   {device} ({name})")
    print(f"  板子    {int(entry['tagCols'])}x{int(entry['tagRows'])} tag，"
          f"tagSize {float(entry['tagSize']) * 100:g} cm，"
          f"tagSpacing {entry['tagSpacing']}，tag_rotation_deg {entry.get('tag_rotation_deg', 180)}")
    print(f"  画面    {cfg.WIDTH}x{cfg.HEIGHT} @ {cfg.FPS} fps，预览缩到 {DISPLAY_WIDTH} 宽")
    print("  操作    Q / Esc 退出，S 存一张原图")
    print("=" * 74)
    lock_controls(device)
    cap = open_camera(device, cfg.WIDTH, cfg.HEIGHT, cfg.FPS)

    root = label = None
    if not args.headless:
        try:
            import tkinter as tk
            from PIL import Image, ImageTk
            root = tk.Tk()
            root.title("AprilGrid preview   (Q quit / S save)")
            label = tk.Label(root, bd=0)
            label.pack()
        except Exception as exc:                       # noqa: BLE001
            print(f"⚠️ 起不了窗口（{exc}），退回终端统计模式")
            root = label = None
    SAVE_DIR.mkdir(parents=True, exist_ok=True)
    state = {"quit": False, "save": False}

    if root is not None:
        def on_key(event):
            char = (event.char or "").lower()
            if char in ("q",) or event.keysym == "Escape":
                state["quit"] = True
            elif char == "s":
                state["save"] = True
        root.bind("<KeyPress>", on_key)

    frames, t0, last_print = 0, time.time(), 0.0
    try:
        while not state["quit"]:
            ok, frame = cap.read()
            if not ok:
                print("⚠️ 读帧失败，退出")
                return 1
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            uv, tag_px, rms = analyse(target, camera, gray)
            fps = frames / max(1e-6, time.time() - t0)
            canvas, n_tag = render(gray, uv, min_tags, tag_px, rms, fps)
            frames += 1
            if state["save"]:
                state["save"] = False
                out = SAVE_DIR / f"{time.time_ns()}.png"
                cv2.imwrite(str(out), frame)
                print(f"  已存 {out}")
            now = time.time()
            if now - last_print > 1.0:
                print(f"  {n_tag:2d} tag | tag {tag_px and round(tag_px)} px | "
                      f"PnP {rms and round(rms, 2)} px | {fps:.1f} fps", flush=True)
                last_print = now
            if root is not None:
                photo = ImageTk.PhotoImage(Image.fromarray(cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)))
                label.configure(image=photo)
                label.image = photo                    # 保住引用，否则图像会被回收
                root.update_idletasks()
                root.update()
            elif frames % 30 == 0:
                continue
    except KeyboardInterrupt:
        pass
    finally:
        cap.release()
        if root is not None:
            root.destroy()
    print("退出。")
    return 0


if __name__ == "__main__":
    sys.exit(main())
