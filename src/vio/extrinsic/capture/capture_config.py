"""相机–IMU 外参标定 · 现场采集配置

PyCharm 用法：只改这个文件 → 右键 `record_session.py` → Run（右上角绿三角）。
不用命令行，不用记路径。

改完 `record_session.py` 会自动：
  1) 检查相机（USB 2bc5:0511）和飞控串口到底在不在
  2) 一边采图、一边录 IMU —— 两路都用宿主机的 CLOCK_REALTIME，
     所以天然落在同一个时钟域，不需要额外同步
  3) 收尾生成 `images.csv` / `imu.csv`，正好是 `calibrate_extrinsics.py` 要的格式
  4) 打印这次采集的体检报告，以及下一步该点哪个文件

本文件只影响采集。标定那一步的参数在 `extrinsics_config.py`。
"""
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent      # extrinsic/

# ==========================================================================
# 1. 这次录到哪儿
# ==========================================================================
# 每换一轮就往上加一：session02、session03……
# 改完这里，还要把 extrinsics_config.py 最下面 PATHS 的 images / imu / output
# 一起改掉 —— record_session.py 跑完会把该复制的三行直接打印出来。
SESSION_NAME = "session01"

SESSION_DIR = ROOT / "data" / SESSION_NAME
RESULTS_DIR = ROOT / "results" / SESSION_NAME

# 目标目录里已经有上一次的采集结果时，是否允许覆盖。
# False = 直接报错停下（防止把上一轮的图和 IMU 混在一起）。
REUSE_EXISTING_SESSION = False

# ==========================================================================
# 2. 相机
# ==========================================================================
CAMERA_USB_ID = "2bc5:0511"        # Orbbec Gemini Pro 的 RGB（UVC）。别的值会拒绝运行

# 分辨率决定用哪份内参 —— 两者必须一致，工具会比对并报错，不会自动缩放。
#   "1080p" → 用 configs/gemini_pro_rgb.yaml（1920x1080）
#   "720p"  → 用 720p 内参（1280x720）
#
# 【2026-10-15 实测结论：用 720p】
# 第一次 1080p 试录只存到 1.8 fps（9.15 秒 17 张），原因不是相机，是保存循环太慢：
#     预览窗口 ~306 ms + 棋盘检测 169 ms + PNG 编码 18 ms ≈ 538 ms/帧
# 720p 把检测和编码各砍一半，配合下面的 SHOW_PREVIEW=False，可回到 ~30 fps。
# 正式标定想用 1080p 就把这里改回 "1080p"（记得同步改 --camera 内参），
# 但 1080p 上不去 30 fps，大约 12 fps —— 对 10~30 秒的窗口仍然够用。
RESOLUTION = "1080p"

_RESOLUTION_TABLE = {
    "1080p": (1920, 1080, ROOT / "configs" / "gemini_pro_rgb.yaml"),
    "720p": (1280, 720, ROOT / "configs" / "gemini_pro_rgb_720p.yaml"),
}

if RESOLUTION not in _RESOLUTION_TABLE:
    raise ValueError(f"RESOLUTION 只能是 {sorted(_RESOLUTION_TABLE)} 之一，现在是 {RESOLUTION!r}")

# 这三项由 RESOLUTION 推导出来，不要在别处手改
WIDTH, HEIGHT, CAMERA_YAML = _RESOLUTION_TABLE[RESOLUTION]

FPS = 30                # 请求帧率。真机 30 fps 时，标定侧的 camera_rate_hz 要设成 30
EXPOSURE = 100          # 单位 100 微秒；100 = 10 ms。对着 iPad 屏幕如果发白就往下调（如 40）
# 【2026-10-18】方向别搞反：房间暗的时候往下调只会更黑（实测 175 + 关灯 → 白格 p90=31，
# 前 20 帧检出 0）。暗就先开灯/加补光，实在不能加光才往上调 250~300 + GAIN 30；
# 往下调只用于"过曝/白格糊成一片"或"光够但转动太糊、确认要缩短曝光"。
WHITE_BALANCE = 4600    # 固定白平衡（K）
GAIN = 10                # 增益，大了噪声大
# 【必须 False，2026-10-15 实测】
# 这个 conda 的 OpenCV 没有 GUI（cv2.imshow 不可用），脚本改用 Tkinter + PIL 做预览。
# ImageTk.PhotoImage 每刷一帧要 ~306 ms，而且预览图一律缩到 960 宽
# —— 开销和采集分辨率无关，1080p/720p 一样慢。
# 开着它：整条循环 1.8 fps，10 秒只能存 17 张，数据基本废掉。
# 关掉它：720p 下回到 ~30 fps。
#
# 想先看看取景对不对？把它临时改成 True、DURATION_S 改成 3 跑一次，
# 看完再改回来（那次的数据不要用，删掉目录）。
SHOW_PREVIEW = False

# ==========================================================================
# 3. 标定板 —— 必须和标定那一步用同一份，行列数要和你实际摆的板子完全一致
# ==========================================================================
# 注意：aprilgrid.yaml 是 Kalibr 版式（tag 个数不是内角点！）
#   tagCols/tagRows = tag 个数；tagSize = 黑框外沿边长，米；tagSpacing = 白隙 / tagSize
# 换回打印棋盘格就把下面改成 checkerboard.yaml。
# 【打印板比 iPad 屏幕暗 5~8 倍】换打印板后 EXPOSURE 一般要从 150 提到 250~300，
# 再加一盏正对板子的灯；aruco 解 ID 对对比度很敏感，欠曝会直接 0 检出。
TARGET_YAML = ROOT / "configs" / "aprilgrid.yaml"

# ==========================================================================
# 4. 飞控 IMU
# ==========================================================================
IMU_PORT = "auto"       # "auto" = 在 ttyACM*/ttyUSB* 里自动找会讲 MAVLink 的那个
                        # 也可以写死，例如 "/dev/ttyACM0"
IMU_REQUEST_HZ = 200    # 向飞控请求 HIGHRES_IMU 的速率。ArduPilot 上限约 0.8×SCHED_LOOP_RATE
                        # 请求 200 实际可能只有 50 —— 以采集后打印的实测速率为准
IMU_BAUD = 115200       # USB CDC 下这个值不生效（走 USB 全速），真实串口链路才需要

# 相机开始前后各多录一点 IMU，保证两路时间轴有重叠（标定工具要按重叠区取窗口）
IMU_PRIME_S = 2.0
IMU_TAIL_S = 0.5
IMU_CHECK_SECONDS = 10.0   # 单独点 imu_recorder.py 做体检时录多久

# ==========================================================================
# 5. 采集时长
# ==========================================================================
# 标定侧硬门槛：至少 35 帧有效检出、且时间跨度 >= 5 秒，所以别低于 8 秒。
# 第一次冒烟测试建议 8~10 秒；正式采集 15~30 秒，并且另录 2~3 个独立窗口做复核。
DURATION_S = 10.0
