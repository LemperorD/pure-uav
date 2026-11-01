#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""kalibr/run_kalibr.py —— 一键用 Docker 里的 Kalibr 标定 camera-IMU 外参 + 时间偏移

数据流
------
    data/calib/<session>/      images.csv + imu.csv + cam0/*.png
        │  capture/make_rosbag.py     （复用你已验证的打包脚本，不重写）
        ▼
    kalibr/bags/<session>.bag   ROS1 bag，/cam0/image_raw + /imu0
        │  docker run kalibr:latest
        ▼
    kalibr/results/<session>/   *-camchain-imucam.yaml / *-results-imucam.txt / *-report-imucam.pdf

用法
----
    conda activate vins
    cd src/vio/extrinsic
    python kalibr/run_kalibr.py                       # 用最新的 session
    python kalibr/run_kalibr.py session17             # 指定 session
    python kalibr/run_kalibr.py session17 --rebuild   # 强制重打 bag
    python kalibr/run_kalibr.py --target-type checkerboard --cols 8 --rows 11 --square-size 0.03

容器里看到的路径
----------------
    宿主  <repo>/src/vio/                 →   容器  /data/
    于是  data/calib/session17/               →   /data/data/calib/session17/
          extrinsic/kalibr/bags/session17.bag →   /data/extrinsic/kalibr/bags/session17.bag
          extrinsic/kalibr/configs/           →   /data/extrinsic/kalibr/configs/

本脚本只在 kalibr/ 下新增文件；项目里既有的脚本和配置一个都不动。
运行环境必须是 `vins`（ROS1 Noetic），因为打 bag 需要 rosbag 模块。
"""
from __future__ import annotations

import argparse
import os
import re
import shutil
import subprocess
import sys
from pathlib import Path

# ---------------------------------------------------------------- 路径常量
ROOT = Path(__file__).resolve().parent.parent          # extrinsic/
DATA = ROOT.parent / "data" / "calib"   # 标定板数据（src/vio/data/calib/）
KALIB = ROOT / "kalibr"
CONF = KALIB / "configs"
BAGS = KALIB / "bags"
RES = KALIB / "results"
LOGS = KALIB / "logs"

MOUNT = "/data"          # 容器内的挂载点
IMAGE = "kalibr:latest"  # 由 Dockerfile_ros1_20_04_j4 构建出来的镜像

# ------------------------------------------------- 默认参数（都可命令行覆盖）
# IMU 噪声：2026-10-17 八小时静置实测（~/calib/imu/results/20261017_085103）
IMU_DEFAULTS = {
    "update_rate": 200.3,
    "gyroscope_noise_density": 6.0816e-05,
    "accelerometer_noise_density": 4.2627e-04,
    "gyroscope_random_walk": 7.1351e-06,
    "accelerometer_random_walk": 1.4861e-05,   # 三轴离散 7.8×，当量级参考
}

# 默认目标板：Kalibr PDF 版 tag36h11 AprilGrid 6×6（你卡尺量的值）
TARGET_APRILGRID = dict(rows=6, cols=6, tag_size=0.088, tag_spacing=0.56)


# ---------------------------------------------------------------- 小工具
def info(msg: str = "") -> None:
    print(msg, flush=True)


def fail(msg: str, code: int = 1):
    print("\n❌ " + msg, file=sys.stderr, flush=True)
    sys.exit(code)


def container_path(p: Path) -> str:
    """宿主路径 → 容器内 /data 下的路径（挂载的是 src/vio/）。"""
    return MOUNT + "/" + p.relative_to(ROOT.parent).as_posix()


def have_module(name: str) -> bool:
    import importlib.util
    return importlib.util.find_spec(name) is not None


# ---------------------------------------------------------------- session
def find_sessions() -> list[str]:
    """data/ 下同时有 images.csv 和 imu.csv 的目录，按修改时间从新到旧。"""
    if not DATA.is_dir():
        return []
    rows = [(p.stat().st_mtime, p.name) for p in DATA.iterdir()
            if p.is_dir() and (p / "images.csv").is_file() and (p / "imu.csv").is_file()]
    return [name for _, name in sorted(rows, reverse=True)]


def resolve_session(arg: str | None) -> str:
    sessions = find_sessions()
    if not sessions:
        fail(f"{DATA} 下没有同时含 images.csv 和 imu.csv 的 session")
    if arg is None:
        return sessions[0]
    if arg not in sessions:
        fail(f"没有这个 session：{arg}\n   可用的：{'  '.join(sessions[:8])}")
    return arg


# ---------------------------------------------------------------- 配置生成
def write_if_missing(path: Path, text: str) -> None:
    if path.exists():
        info(f"  保留已有  kalibr/configs/{path.name}")
    else:
        path.write_text(text, encoding="utf-8")
        info(f"  生成      kalibr/configs/{path.name}")


def resolve_board_size(args) -> tuple[int, int]:
    """没显式给 --rows/--cols 时按板型取默认：AprilGrid 6x6，棋盘格 8x11。"""
    if args.rows is not None and args.cols is not None:
        return args.rows, args.cols
    if args.target_type == "aprilgrid":
        r, c = TARGET_APRILGRID["rows"], TARGET_APRILGRID["cols"]
    else:
        r, c = 11, 8
    return (args.rows if args.rows is not None else r,
            args.cols if args.cols is not None else c)


def make_target_yaml(args) -> str:
    rows, cols = resolve_board_size(args)
    if args.target_type == "aprilgrid":
        return (
            "# Kalibr AprilGrid（Kalibr PDF 版 tag36h11）\n"
            f"target_type: 'aprilgrid'\n"
            f"tagRows: {rows}\n"
            f"tagCols: {cols}\n"
            f"tagSize: {args.tag_size}\n"
            f"tagSpacing: {args.tag_spacing}\n"
        )
    return (
        "# Kalibr checkerboard：cols/rows 是【内角点】数，不是黑白格数\n"
        "target_type: 'checkerboard'\n"
        f"targetCols: {cols}\n"
        f"targetRows: {rows}\n"
        f"rowSpacingMeters: {args.square_size}\n"
        f"colSpacingMeters: {args.square_size}\n"
    )


def make_imu_yaml() -> str:
    d = IMU_DEFAULTS
    return (
        "# Kalibr IMU 噪声（连续时间），数值 = 2026-10-17 八小时静置实测\n"
        "# 约定：离散噪声 std = noise_density × sqrt(fs)\n"
        "rostopic: /imu0\n"
        f"update_rate: {d['update_rate']}\n\n"
        f"accelerometer_noise_density: {d['accelerometer_noise_density']:.4e}\n"
        f"accelerometer_random_walk:   {d['accelerometer_random_walk']:.4e}\n"
        f"gyroscope_noise_density:     {d['gyroscope_noise_density']:.4e}\n"
        f"gyroscope_random_walk:       {d['gyroscope_random_walk']:.4e}\n"
    )


def make_camchain_yaml(args) -> str | None:
    """相机链：默认复制项目里已经标好的内参文件，可用 --camchain 覆盖。"""
    src = Path(args.camchain) if args.camchain else (ROOT / "configs" / "gemini_pro_rgb.yaml")
    if not src.is_absolute():
        src = (ROOT / src).resolve()
    if not src.is_file():
        return None
    return src.read_text(encoding="utf-8")


def ensure_configs(args) -> None:
    info("配置：")
    write_if_missing(CONF / "target.yaml", make_target_yaml(args))

    imu_text = make_imu_yaml()
    write_if_missing(CONF / "imu.yaml", imu_text)

    cam = make_camchain_yaml(args)
    if cam is None:
        fail("找不到 configs/gemini_pro_rgb.yaml（相机内参），请先用标定脚本拿到 camchain，"
             "或用 --camchain 指定")
    write_if_missing(CONF / "camchain.yaml", cam)


# ---------------------------------------------------------------- docker
def _docker_ok(prefix: list[str]) -> bool:
    return subprocess.run(prefix + ["info"], stdout=subprocess.DEVNULL,
                          stderr=subprocess.DEVNULL).returncode == 0


def docker_prefix() -> list[str]:
    if shutil.which("docker") is None:
        fail("没有找到 docker 命令")
    if _docker_ok(["docker"]):
        return ["docker"]
    info("  当前环境没有 docker 组权限，改用 sudo docker（会提示输入密码）")
    if _docker_ok(["sudo", "docker"]):
        return ["sudo", "docker"]
    fail("docker 不可用：直接连和 sudo 连都失败，先排查 docker 服务/权限")


def run_tee(cmd: list[str], log_path: Path | None = None) -> int:
    """把子进程输出同时打到终端和日志文件。"""
    info("  $ " + " ".join(cmd))
    fh = log_path.open("w", encoding="utf-8") if log_path else None
    try:
        proc = subprocess.Popen(cmd, stdout=subprocess.PIPE, stderr=subprocess.STDOUT,
                                text=True, bufsize=1, errors="replace")
        assert proc.stdout is not None
        for line in proc.stdout:
            sys.stdout.write(line)
            sys.stdout.flush()
            if fh:
                fh.write(line)
        return proc.wait()
    finally:
        if fh:
            fh.close()


# ---------------------------------------------------------------- bag
def build_bag(session: str, rebuild: bool) -> Path:
    bag = BAGS / f"{session}.bag"
    if bag.exists() and not rebuild:
        info(f"bag：复用已有 kalibr/bags/{session}.bag（{bag.stat().st_size/1e6:.0f} MB）")
        return bag
    if not have_module("rosbag"):
        fail("当前 Python 里没有 rosbag 模块 —— 先 `conda activate vins` 再运行本脚本")
    script = ROOT / "capture" / "make_rosbag.py"
    if not script.is_file():
        fail(f"找不到 {script}")
    info(f"bag：从 data/calib/{session}/ 打包（复用 capture/make_rosbag.py）")
    rc = run_tee([sys.executable, str(script), "--session", session,
                  "--output", str(bag)])
    if rc != 0 or not bag.exists():
        fail("打 bag 失败，先把上面的报错解决")
    info(f"       → kalibr/bags/{session}.bag（{bag.stat().st_size/1e6:.0f} MB）")
    return bag


def bag_topic_check(bag: Path) -> tuple[str, str, int]:
    """用宿主 vins 环境里的 rosbag 读话题（不依赖 docker）。

    读不到不报错，返回空串 —— 调用方会退回默认话题名继续跑，
    真正的错误交给 Kalibr 自己报（它的报错比这里准）。
    """
    try:
        import rosbag
    except ImportError:
        info("  （当前 Python 没有 rosbag，跳过话题校验）")
        return "", "", 0
    try:
        with rosbag.Bag(str(bag), "r") as b:
            meta = b.get_type_and_topic_info()
        topics = meta.topics
        image = [t for t, v in topics.items() if v.msg_type.endswith("/Image")]
        imu = [t for t, v in topics.items() if v.msg_type.endswith("/Imu")]
        if image and imu:
            return image[0], imu[0], len(topics)
        info(f"  （bag 里没找到 Image/Imu，现有 {len(topics)} 个话题）")
        return "", "", len(topics)
    except Exception as exc:                                   # noqa: BLE001
        info(f"  （读取 bag 话题失败：{exc}，跳过校验）")
        return "", "", 0


def align_topics(image_topic: str, imu_topic: str) -> None:
    """让配置里的 rostopic 和 bag 里的话题对齐（只改 kalibr/configs 下的副本）。"""
    imu_yaml = CONF / "imu.yaml"
    text = imu_yaml.read_text(encoding="utf-8")
    new = re.sub(r"^rostopic:.*$", f"rostopic: {imu_topic}", text, count=1, flags=re.MULTILINE)
    if new != text:
        imu_yaml.write_text(new, encoding="utf-8")
        info(f"  已把 imu.yaml 的 rostopic 对齐为 {imu_topic}")

    cam_yaml = CONF / "camchain.yaml"
    text = cam_yaml.read_text(encoding="utf-8")
    new, n = re.subn(r"^(\s+)rostopic:.*$", rf"\g<1>rostopic: {image_topic}", text,
                     count=1, flags=re.MULTILINE)
    if n and new != text:
        cam_yaml.write_text(new, encoding="utf-8")
        info(f"  已把 camchain.yaml 的 rostopic 对齐为 {image_topic}")


# ---------------------------------------------------------------- 跑 Kalibr
def run_kalibr(args, docker: list[str], session: str, bag: Path) -> Path:
    out_dir = RES / session
    out_dir.mkdir(parents=True, exist_ok=True)
    log = LOGS / f"{session}_{time_stamp()}.log"

    inner = (
        "source /catkin_ws/devel/setup.bash && "
        "rosrun kalibr kalibr_calibrate_imu_camera "
        f"--bag {container_path(bag)} "
        f"--cams {container_path(CONF / 'camchain.yaml')} "
        f"--imu {container_path(CONF / 'imu.yaml')} "
        f"--target {container_path(CONF / 'target.yaml')}"
    )
    if args.from_to:
        inner += f" --bag-from-to {args.from_to[0]} {args.from_to[1]}"

    cmd = docker + ["run", "--rm",
                    "-v", f"{ROOT.parent}:{MOUNT}",
                    "-w", container_path(out_dir),
                    "--user", f"{os.getuid()}:{os.getgid()}",
                    "-e", "HOME=/tmp", "-e", "MPLCONFIGDIR=/tmp/mpl",
                    "-e", "MPLBACKEND=Agg", "-e", "QT_QPA_PLATFORM=offscreen",
                    IMAGE, "/bin/bash", "-c", inner]
    info(f"\n开始标定：docker（日志 kalibr/logs/{log.name}）\n")
    rc = run_tee(cmd, log)

    # Kalibr 把输出写在 bag 同目录（文件名由 bag 派生），不是 cwd：
    #   kalibr/bags/session17.bag -> kalibr/bags/session17-camchain-imucam.yaml ...
    stem = str(bag.with_suffix(""))
    produced = []
    for suffix in ("-camchain-imucam.yaml", "-imu.yaml",
                   "-results-imucam.txt", "-report-imucam.pdf"):
        src = Path(stem + suffix)
        if src.exists():
            produced.append(src)
            shutil.copy2(src, out_dir / src.name)

    got_camchain = any(p.name.endswith("-camchain-imucam.yaml") for p in produced)
    if not got_camchain:
        info(f"\n--- kalibr/logs/{log.name} 最后 30 行 ---")
        tail = log.read_text(encoding="utf-8", errors="replace").splitlines()[-30:]
        for line in tail:
            info("  " + line)
        fail(f"Kalibr 没有产出 camchain-imucam.yaml（退出码 {rc}）"
             f" —— 完整日志见 kalibr/logs/{log.name}")
    if rc != 0:
        info(f"\n⚠️ Kalibr 退出码 {rc}，但结果文件已生成（通常是收尾画图/报告那步的环境问题，"
             f"不影响 yaml）。缺哪个文件就看 kalibr/logs/{log.name}")
    info(f"结果已复制到 kalibr/results/{session}/（原文件在 kalibr/bags/）：")
    for f in produced:
        info(f"    {f.name}")
    return log


def time_stamp() -> str:
    import datetime
    return datetime.datetime.now().strftime("%Y%m%d_%H%M%S")


# ---------------------------------------------------------------- 结果整理
def report(session: str) -> None:
    out_dir = RES / session
    files = sorted(p for p in out_dir.glob("*") if p.is_file())
    cam_imu = next((p for p in files if p.name.endswith("-camchain-imucam.yaml")), None)
    results = next((p for p in files if p.name.endswith("-results-imucam.txt")), None)
    pdf = next((p for p in files if p.name.endswith("-report-imucam.pdf")), None)

    info("\n" + "=" * 72)
    info(f"  结果目录：kalibr/results/{session}/")
    for p in files:
        info(f"    {p.name}")
    info("=" * 72)

    if results is not None:
        info("\n--- results-imucam.txt ---")
        info(results.read_text(encoding="utf-8", errors="replace").strip()[:4000])

    if cam_imu is None:
        info("\n⚠️ 没有生成 camchain-imucam.yaml —— 标定没收敛。")
        info("   常见原因：板子没检出（确认板型/尺寸）、运动激励不足、图像与 IMU 时间不对齐。")
        info("   看一眼 kalibr/results 里的日志和 report-imucam.pdf。")
        return

    try:
        import yaml
        data = yaml.safe_load(cam_imu.read_text(encoding="utf-8"))
        cam = data["cam0"]
        T = cam["T_cam_imu"]
        ts = cam.get("timeshift_cam_imu", 0.0)
    except Exception as exc:                                   # noqa: BLE001
        info(f"\n（解析 {cam_imu.name} 失败：{exc}）")
        return

    info("\nT_cam_imu（IMU → 相机，Kalibr 约定）：")
    for row in T:
        info("    " + "  ".join(f"{v: .6f}" for v in row))
    info(f"\ntimeshift_cam_imu = {ts:+.6f} s   （相机时间戳相对 IMU 的偏移）")

    try:
        import numpy as np
        Ti = np.linalg.inv(np.asarray(T, dtype=float))
        info("\nT_imu_cam = inv(T_cam_imu)（OpenVINS 的 T_imu_cam 用这个）：")
        for row in Ti:
            info("    " + "  ".join(f"{v: .6f}" for v in row))
    except Exception:                                          # noqa: BLE001
        pass

    info("\n下一步：把上面的 T_imu_cam 填进 OpenVINS 配置，或先备份再替换 imu 参数文件。")


# ---------------------------------------------------------------- main
def parse_args() -> argparse.Namespace:
    ap = argparse.ArgumentParser(
        description="用 Docker 里的 Kalibr 标定 data/<session> 的 camera-IMU",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="例：python kalibr/run_kalibr.py session17 --rebuild")
    ap.add_argument("session", nargs="?", default=None,
                    help="session 名（默认用 data/ 下最新一个）")
    ap.add_argument("--rebuild", action="store_true", help="强制重新打 bag")
    ap.add_argument("--no-verify", action="store_true", help="跳过 bag 话题校验")
    ap.add_argument("--target-type", choices=["aprilgrid", "checkerboard"],
                    default="aprilgrid", help="标定板类型（默认 aprilgrid）")
    ap.add_argument("--rows", type=int, default=None,
                    help="AprilGrid tag 行数 / 棋盘格内角点行数（默认 6 / 11）")
    ap.add_argument("--cols", type=int, default=None,
                    help="AprilGrid tag 列数 / 棋盘格内角点列数（默认 6 / 8）")
    ap.add_argument("--tag-size", type=float, default=TARGET_APRILGRID["tag_size"],
                    help=f"AprilGrid tag 外框边长 [m]（默认 {TARGET_APRILGRID['tag_size']}）")
    ap.add_argument("--tag-spacing", type=float, default=TARGET_APRILGRID["tag_spacing"],
                    help=f"白隙/tagSize（默认 {TARGET_APRILGRID['tag_spacing']}）")
    ap.add_argument("--square-size", type=float, default=0.03,
                    help="棋盘格边长 [m]（默认 0.03）")
    ap.add_argument("--camchain", type=Path, default=None,
                    help="相机内参 yaml（默认 configs/gemini_pro_rgb.yaml）")
    ap.add_argument("--from-to", nargs=2, type=float, metavar=("T0", "T1"),
                    help="只用 bag 的 [T0, T1] 秒（可选）")
    return ap.parse_args()


def main() -> int:
    args = parse_args()
    for d in (CONF, BAGS, RES, LOGS):
        d.mkdir(parents=True, exist_ok=True)

    session = resolve_session(args.session)
    info(f"session：{session}" + ("（最新）" if args.session is None else ""))
    info(f"项目根：{ROOT}")

    ensure_configs(args)
    bag = build_bag(session, args.rebuild)
    docker = docker_prefix()

    if not args.no_verify:
        image_topic, imu_topic, n_topics = bag_topic_check(bag)
        if image_topic and imu_topic:
            info(f"bag 话题：image={image_topic}  imu={imu_topic}（共 {n_topics} 个）")
            align_topics(image_topic, imu_topic)
        else:
            info("  ⚠️ 没能自动校验 bag 话题，继续用默认 /cam0/image_raw + /imu0")

    run_kalibr(args, docker, session, bag)
    report(session)
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        info("\n已中断")
        sys.exit(130)
