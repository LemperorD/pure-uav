"""相机–IMU 标定默认设置。所有噪声值是权重初值，不是 CUAV 出厂参数。
在 PyCharm 修改最下面 PATHS 并直接运行 calibrate_extrinsics.py；也支持命令行。
"""
from dataclasses import dataclass, fields
from pathlib import Path
import math
import sys

ROOT = Path(__file__).resolve().parent

@dataclass
class Settings:
    knot_spacing_s: float = 0.04  # 均匀三次样条结点间距；更小能拟合更快的运动，耗时增加
    max_duration_s: float = 30.0  # 每次标定一个连续窗口；多次用不同 start_s 独立验证
    start_s: float = 0.0  # 相对于第一张输入图像
    camera_rate_hz: float = 15.0  # 检测/优化最高图像率，不插值图像
    imu_rate_hz: float = 100.0  # 优化最高 IMU 率，只取原始样本，不做插值/滤波
    max_offset_s: float = 0.10  # t_imu = t_cam + offset 的搜索边界
    max_translation_m: float = 0.50  # T_cam_imu 各平移分量边界，按实际安装尺寸调整
    image_sigma_px: float = 0.5  # 单坐标像素标准差，用于白化残差
    gyro_sigma_rad_s: float = 0.01  # 保留的单个 IMU 样本标准差，非连续时间噪声密度
    accel_sigma_m_s2: float = 0.10
    gravity_m_s2: float = 9.80665  # 固定重力模长；优化重力方向
    gravity_norm_sigma: float = 0.005  # 重力模长约束的标准差 m/s²
    max_gyro_bias_rad_s: float = 0.5
    max_accel_bias_m_s2: float = 2.0
    min_frames: int = 35
    min_duration_s: float = 5.0
    max_camera_gap_s: float = 0.35  # 成功检测帧之间的最大空洞；超限拒绝跨洞拟合
    max_imu_gap_s: float = 0.05
    min_rotation_std_rad_s: float = 0.05  # 角速度协方差第2特征方向的最低标准差
    min_lever_singular: float = 0.15  # 杠杆臂回归矩阵最小归一化奇异值，单位 s^-2
    min_target_depth_m: float = 0.01  # 数值保护下界；最终所有角点必须至少在相机前 1cm
    depth_sigma_m: float = 0.001  # 优化试探步的可见性惩罚尺度，不是传感器噪声
    max_pnp_rms_px: float = 2.0
    max_reprojection_rms_px: float = 1.0  # 最终二维角点欧氏距离 RMS
    max_gyro_rms_rad_s: float = 0.04  # 最终各轴合并 RMS
    max_accel_rms_m_s2: float = 0.40
    min_tags: int = 4
    april_corner_window_px: int = 2  # AprilTag 亚像素细化半窗，像素
    max_points_per_frame: int = 100  # 均匀选取角点，0 表示全部
    init_max_nfev: int = 80
    joint_max_nfev: int = 300
    optimizer_ftol: float = 1e-5
    optimizer_xtol: float = 1e-7
    optimizer_gtol: float = 1e-6
    robust_scale: float = 2.5  # soft_l1 的白化残差尺度
    seed: int = 7
    save_detection_every: int = 20  # 每 N 张有效帧保存角点检查图，0=不保存
    verbose: int = 1

    def __post_init__(self):
        """Normalize numeric YAML strings; reject bools, NaN and fractional counts."""
        for field in fields(self):
            value = getattr(self, field.name)
            if isinstance(value, bool) or not isinstance(value, (int, float, str)):
                raise ValueError(f"配置项 {field.name}: 需要数值，收到 {value!r}")
            try:
                number = float(value)
            except (ValueError, OverflowError) as exc:
                raise ValueError(f"配置项 {field.name}: 无效数值 {value!r}") from exc
            if not math.isfinite(number):
                raise ValueError(f"配置项 {field.name}: 必须为有限数值")
            if field.type is int:
                if not number.is_integer():
                    raise ValueError(f"配置项 {field.name}: 必须为整数")
                number = int(number)
            setattr(self, field.name, number)
        for field in fields(self):
            value = getattr(self, field.name)
            if field.type is float and (value < 0 if field.name == "start_s" else value <= 0):
                raise ValueError(f"配置项 {field.name}: 必须 {'>=0' if field.name == 'start_s' else '>0'}")
        minima = {"min_frames": 3, "min_tags": 1, "april_corner_window_px": 1,
                  "max_points_per_frame": 0, "init_max_nfev": 1, "joint_max_nfev": 1,
                  "seed": 0, "save_detection_every": 0, "verbose": 0}
        for key, minimum in minima.items():
            if getattr(self, key) < minimum:
                raise ValueError(f"配置项 {key}: 必须 >= {minimum}")
        if self.verbose > 2 or self.seed >= 2**32:
            raise ValueError("配置项 verbose 必须为 0/1/2；seed 必须小于 2**32")
        for key in ("optimizer_ftol", "optimizer_xtol", "optimizer_gtol"):
            if getattr(self, key) <= sys.float_info.epsilon:
                raise ValueError(f"配置项 {key}: 必须大于机器精度")

# PyCharm 入口相对路径
PATHS = {
    "camera": ROOT / "configs/gemini_pro_rgb.yaml",  # 建议换成自己的内参文件
    "target": ROOT / "configs/checkerboard.yaml",  # 尺寸必须实测！
    "images": ROOT / "data/session08/images.csv",  # timestamp,filename
    "imu": ROOT / "data/session08/imu.csv",  # timestamp,ax,ay,az,gx,gy,gz
    "bag": None,  # ROS1 .bag / ROS2 bag 目录；设置后替代 images / imu
    "output": ROOT / "results/session08",
}
CAM_TOPIC = "/cam0/image_raw"
IMU_TOPIC = "/imu0"
IMAGE_TIMESTAMP_UNIT = "ns"  # ns / us / ms / s，必须明确
IMU_TIMESTAMP_UNIT = "ns"
SETTINGS = Settings(camera_rate_hz=60,max_pnp_rms_px=3.5)
