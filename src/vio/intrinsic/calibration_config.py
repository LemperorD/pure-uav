"""两个程序的默认参数；现场只修改这个文件。命令行可临时覆盖。"""
from pathlib import Path

# ==================== 现场必改：棋盘和保存位置 ====================
COLS = 8  # 横向内角点数，不是格子数；当前仅是 iPad 测试板的值。
ROWS = 11  # 纵向内角点数；采集和标定共用，现场按实物修改。
SQUARE_SIZE_M = 0.03  # 相邻角点距离，米；25 mm 填 0.025，请实测。
SESSION_DIR = Path(__file__).resolve().parent / "data" / "session16"
# 每轮改成新的 session02、session03 等，不混用不同相机设置。
IMAGES_DIR = SESSION_DIR / "cam0"  # 离线标定读取的原始图片目录。
RESULT_DIR = SESSION_DIR / "intrinsics"  # 内参、报告、角点检查图的输出目录。

# ==================== 相机设置：与后续 VIO 模式一致 ====================
DEVICE = Path("/dev/video4")  # RGB 采集节点；重插可能变化，会核对 USB 身份。
WIDTH = 1920  # 原始图像宽度，像素；不会缩放存图。
HEIGHT = 1080  # 原始图像高度，像素。
FPS = 30  # 请求帧率；会回读验证，传输格式固定 MJPG。
MODEL = "pinhole"  # 普通镜头用 pinhole；确认鱼眼投影模型后才用 fisheye。
BACKEND = "auto"  # auto 优先直接 V4L2，失败退回 OpenCV；v4l2 禁止回退。
EXPOSURE = 100  # 单位 100 微秒，100 = 10 ms；调到清晰、不饱和。
WHITE_BALANCE = 4600  # 固定白平衡，K；影响彩色转灰度后的亮度。
GAIN = 0  # 增益；过大增加噪声。
BRIGHTNESS = 0  # 亮度偏置；一般不改。
CONTRAST = 32  # 对比度；避免黑白细节截断。
SHARPNESS = 3  # 锐化；过强会影响角点定位。
POWER_LINE_FREQUENCY = 1  # 0=关闭，1=50 Hz，2=60 Hz；不保证消除屏幕条纹。

# ==================== 采集方式 ====================
AUTO_INTERVAL = 2.0  # 自动存图间隔，秒；0 表示只按空格存图。
DURATION = 90.0  # 最长运行时间，秒；0 表示手动退出。
MAX_IMAGES = 40  # 最多保存张数；0 不限；建议得到 25~40 个有效不同视角。
NO_PREVIEW = False  # True 不开预览；现场桌面采集通常用 False。
READ_RETRIES = 10  # 读帧失败后的重连次数；耗尽后停止并保留已有数据。
RETRY_INTERVAL = 0.5  # 重连尝试间隔，秒。
WARMUP_FRAMES = 10  # 设置曝光/重连后丢弃前几帧，让新控件生效；不保存这些帧。

# ==================== 质量检查：通常不用改 ====================
MIN_VIEWS = 20  # 过滤后最少有效视角；正式拍摄建议 25~40 张。
SAVE_DETECTIONS = True  # 保存带角点的检查图，不修改原始图片。
OUTLIER_FACTOR = 2.0  # 阈值 = max(本项 * 逐视角误差中位数, 下项)。
OUTLIER_FLOOR_PX = 1.0  # 异常误差下限，像素；每轮仅剔除最差一张再重标。
MAX_RMS_PX = 0.5  # 总 RMS 门槛；超限写失败报告，不生成新内参 YAML。
MIN_COVERED_CELLS = 7  # 九宫格最少覆盖格数，同时应覆盖四角；不足会警告。
STRICT_COVERAGE = False  # True 时覆盖不足拒绝输出；False 保留警告便于诊断。
FOCAL_MIN_RATIO = 0.2  # 焦距下限 / max(宽,高)，当前 RGB 相机的经验护栏。
FOCAL_MAX_RATIO = 5.0  # 焦距上限 / max(宽,高)，不是通用物理定律。
MAX_FOCAL_ASPECT_ERROR = 0.15  # fx/fy 相对差上限，假定方形像素、无非等比拉伸。
RADTAN_LIMITS = (1.5, 1.5, 0.1, 0.1)  # |k1|,|k2|,|p1|,|p2| 的防发散护栏。
# 系数量级依赖归一化半径及镜头；超限应调查，不表示这种镜头物理上不存在。
FISHEYE_FOCAL_RATIO = 0.5  # 鱼眼初始焦距 / max(宽,高)，没有通用正确初值。
CALIBRATION_ITERATIONS = 200  # 优化器最大迭代次数。
TOPIC = "/cam0/image_raw"  # 仅写入 YAML 的 ROS 原始图像话题名。