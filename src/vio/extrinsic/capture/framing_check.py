"""★ 取景 / 曝光体检 —— PyCharm 点绿三角即可，不需要 IMU。

为什么需要它：为了把帧率从 2.6 拉回 21.6，采集时的预览窗被关了（那个 Tk/PIL 预览每帧要
306 ms）。代价是**采集变成盲拍** —— 你只能等采完、跑完标定，才知道板子压根没进画面。

这个脚本把预览加回来，而且只录 10 秒，录完立刻自动判读：
  · 亮度够不够（p90 是"白格有多亮"，判据看它）
  · 板子认不认得出
  · 如果认不出，是"板子被切边"还是"太暗"还是"行列数不对" —— 分别给不同的动作

数据写进 data/_framing_check/（每次运行前自动清空），跑完留着让你自己看，可以随时删。
**这个脚本不用来出标定结果，只用来把取景和曝光调对。**
"""
from __future__ import annotations

import shutil
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent                # extrinsic/：唯一需要上 sys.path 的路径
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

from capture import capture_config as cfg         # noqa: E402
from capture import image_health                  # noqa: E402
from capture import record_session as rs          # noqa: E402

CHECK_DIR = cfg.DATA_ROOT / "_framing_check"
SECONDS = 10.0


def main():
    print("=" * 76)
    print("取景 / 曝光体检 —— 不开采集正式流程，只把画面调对")
    print("=" * 76)

    cameras, seen = rs.scan_cameras(cfg.CAMERA_USB_ID)
    if not cameras:
        print(f"❌ 没找到 USB 身份为 {cfg.CAMERA_USB_ID} 的相机。当前 /dev/video* 上是：")
        for line in (seen or ["     （一个都没有）"]):
            print(f"     {line}")
        return 2
    device, name = cameras[0]

    board = rs.read_target(cfg.TARGET_YAML)
    checker = board["kind"] in ("checkerboard", "chessboard")
    print(f"  相机      {device}  ({name})")
    print(f"  分辨率    {cfg.WIDTH}x{cfg.HEIGHT} @ {cfg.FPS} fps，曝光 {cfg.EXPOSURE}，增益 {cfg.GAIN}")
    print(f"  标定板    {cfg.TARGET_YAML.name} → {board['label']}")
    if checker:
        print("  提示      预览窗里的【绿点】= 棋盘被认出来了（采图脚本自带的那套检测）")
    else:
        print("  提示      AprilGrid 下预览窗不会出绿点（采图脚本只会找棋盘），"
              "靠这 10 秒跑完的自动判读来调")
        print("  注意      下面每行末尾的 board=no 是采图脚本自带的【棋盘】检测器打的，"
              "对 AprilGrid 恒为 no，\n"
              "            和 AprilGrid 认不认得出来【没有关系】—— 以本脚本最后的判读为准")

    print()
    print("─" * 76)
    print(f"  接下来 {SECONDS:.0f} 秒，把相机对准标定板慢慢挪：")
    if checker:
        print("    · 让【整块】板子连同四周留白都进画面 —— 占画面 1/3~1/2 就够")
        print("    · 预览窗里出现【绿点】= 板子被认出来了；没有绿点就是这个距离/角度/亮度不行")
    else:
        print("    · 板子占画面 1/3~2/3 就行，出一部分画面也没关系（AprilGrid 部分可见也能用）")
        print("    · 关键是 tag 够不够大：每个 tag 边长 ≥45 px 最舒服，<30 px 解不出 ID")
        print("    · 别为了「整块进画面」往后退 —— 退远只会让 tag 变小，反而更糟")
    print("    · 试几个不同的距离和角度，把能对上的那种记下来")
    print("─" * 76)
    print("  想看实时画面 + 实时检测框：另开一个终端跑 "
          "python capture/preview_aprilgrid.py（这个脚本本身不开预览）")
    print()

    shutil.rmtree(CHECK_DIR, ignore_errors=True)
    CHECK_DIR.mkdir(parents=True, exist_ok=True)

    base = [
        sys.executable, str(rs.CAMERA_SCRIPT),
        "--device", str(device),
        "--output", str(CHECK_DIR),
        "--width", str(cfg.WIDTH), "--height", str(cfg.HEIGHT), "--fps", str(cfg.FPS),
        "--exposure", str(cfg.EXPOSURE),
        "--white-balance", str(cfg.WHITE_BALANCE), "--gain", str(cfg.GAIN),
        "--cols", str(board["cols"]), "--rows", str(board["rows"]),
        "--auto-interval", "0.2", "--max-images", "0",
        "--duration", str(SECONDS),
    ]
    # 【2026-10-18】AprilGrid 下预览窗一点用都没有（采图脚本只会找棋盘，永远不出绿点），
    # 但那个 Tk/PIL 预览每帧要 ~300 ms，开着只有 1.3 fps、10 秒只能存 13 张。
    # 所以非棋盘格时直接不开预览，帧数能翻好几倍。
    if not checker:
        base.append("--no-preview")

    rc = subprocess.Popen(base).wait()
    images = sorted((CHECK_DIR / "cam0").glob("*.png"))
    if rc != 0 and not images:
        print()
        print("⚠️ 采集子进程起不来（多半是没有图形界面 / DISPLAY 不可用）。")
        print("   改成不带预览再跑一次，只做自动判读……")
        rc = subprocess.Popen(base + ["--no-preview"]).wait()
        images = sorted((CHECK_DIR / "cam0").glob("*.png"))

    print()
    print("=" * 76)
    print("判读")
    print("=" * 76)
    if not images:
        print("❌ 一张图都没存下来 —— 相机子进程失败了吗？看上面它打印的报错。")
        return 2

    res = image_health.analyze(images, cfg.TARGET_YAML, sample=20)
    level, actions = image_health.report(res)

    print()
    icon = {"ok": "✅", "warn": "⚠️", "fail": "❌", "unknown": "❔"}[level]
    print(f"{icon} 判定：{level}")
    for a in actions:
        print(f"   → {a}")

    print()
    if level == "ok":
        print("取景和亮度都可用。现在可以去点 capture/record_session.py 采正式数据了。")
    else:
        print("按上面的动作调完，再点一次这个脚本。**不要带着这个问题去采正式数据** ——")
        print("零检出的 10 秒窗口，标定程序只会回你一句「有效标定板帧只有 0」。")
    print(f"\n这次拍的图留在 {CHECK_DIR}（下次运行会先清空），可以自己打开看。")
    return 0 if level == "ok" else 1


if __name__ == "__main__":
    sys.exit(main())
