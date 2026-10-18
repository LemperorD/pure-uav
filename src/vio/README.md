# vio —— 相机内参 / IMU 噪声 / 相机–IMU 外参标定

## 目录结构

```
vio/
├── intrinsic/                      ① 相机内参（棋盘格 → cv2.calibrateCamera）
│   ├── capture_calibration.py            采图：V4L2 直采，锁曝光/白平衡，棋盘格预览
│   ├── calibrate_intrinsics.py           求解：角点检测 + 标定 + 逐轮剔除 + RMS 门槛
│   └── calibration_config.py             配置：板子内角点数 / 格长 / 分辨率 / 曝光
│
├── imu_noise/                      ② IMU 噪声（Allan 方差，静置数小时）
│   ├── record_static_imu.py              录制：MAVLink HIGHRES_IMU @200 Hz → CSV
│   ├── analyze_static_noise.py           分析：Allan → 噪声密度 + 随机游走
│   ├── serial_port.py                    串口抽象（Linux 用 termios，Windows 用 pyserial）
│   └── vendor/                           最小 MAVLink 解析 + Allan 核心 + DataFlash 日志解析
│
└── extrinsic/                      ③ 相机–IMU 外参与时间偏移
    ├── calibrate_extrinsics.py           主入口：连续时间 B 样条联合优化
    ├── extrinsics_config.py              配置：输入路径 / 求解参数 / 残差门槛
    ├── prepare_openvins.py               把结果导出成 OpenVINS 配置
    ├── remap_imu_clock.py                飞控时钟 ↔ 宿主时钟映射
    ├── calib/                            数学模型与求解
    │   ├── spline.py                         连续时间 B 样条
    │   ├── solver.py                         联合优化与导出
    │   ├── data.py                           输入读取 / 板检测 / PnP
    │   └── acquisition.py                    采集数据解析
    ├── configs/                          相机内参、标定板、动作模板
    └── capture/                          采集链
        ├── record_session.py                 录一次完整会话（图像 + IMU）
        ├── imu_recorder.py                   MAVLink IMU 录制
        ├── run_calibration.py                一键：自动挑连续窗口 → 标定
        ├── image_health.py                   画面体检（亮度 / 清晰度 / 检出率）
        ├── framing_check.py                  取景试拍
        ├── make_images_csv.py                manifest → images.csv
        ├── make_rosbag.py                    images.csv + imu.csv → ROS1 bag
        ├── capture_config.py                 采集参数（会话名 / 分辨率 / 曝光）
        └── vendor/                           内参采集器的本地副本（保证本目录自包含）
```

## 环境

**Python 3.11**，conda 环境vins：

```bash
conda create -n vins python=3.11 -y
conda activate vins
pip install -r requirements.txt
conda install -c conda-forge tk        # 采集预览窗口需要（标准库 tkinter）
```

| 包 | 本机实测版本 | 谁在用 |
|---|---|---|
| numpy | 1.26.4 | 全部 |
| scipy | 1.17.1 | 外参联合优化的最小二乘 |
| **opencv-contrib-python** | 4.10.0 | 角点检测 / PnP / AprilGrid（**必须 contrib 版**，要有 `cv2.aruco`） |
| PyYAML | 6.0.3 | 配置与结果读写 |
| matplotlib | 3.4.3 | IMU 的 Allan 曲线 |
| rosbags | 0.11.5 | 读写 ROS1 bag（纯 Python，**不需要 ROS**） |
| Pillow | 12.3.0 | 采集预览 |
| tkinter（Tk 8.6.13） | 标准库 | 采集预览窗口 |


### 设备权限

```bash
sudo usermod -aG video,dialout $USER     # 相机 /dev/video*，飞控 /dev/ttyACM*
# 重新登录生效
```

## 运行

**内参**

```bash
cd intrinsic
python capture_calibration.py --output data/session01
python calibrate_intrinsics.py --images data/session01/cam0 --output data/session01/intrinsics
```

板子参数在 `calibration_config.py` 的 `COLS` / `ROWS` / `SQUARE_SIZE_M`（**内角点数，不是格子数**）。

**IMU 噪声**

```bash
cd imu_noise
python record_static_imu.py       # 时长改 DURATION_H（单位：小时）
python analyze_static_noise.py    # 自动取 ../data/static_*.csv，或把 CSV 路径当参数传
```

噪声密度 10 分钟够；随机游走要 4~8 小时。

**外参**

```bash
cd extrinsic
python capture/record_session.py                         # 采集
python capture/run_calibration.py --session session01    # 标定
```

输入路径与求解参数在 `extrinsics_config.py` 的 `PATHS` / `SETTINGS`。
