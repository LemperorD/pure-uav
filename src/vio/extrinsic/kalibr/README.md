# kalib/ —— 用 Docker 里的 Kalibr 标定 camera–IMU

这个目录是**新增**的，不修改项目里任何既有文件（因为既有文件在 `MANIFEST.sha256` 里）。

## 它做什么

```
data/<session>/                     images.csv + imu.csv + cam0/*.png
      │  capture/make_rosbag.py     （复用你已经验证过的打包脚本）
      ▼
kalib/bags/<session>.bag            ROS1 bag，/cam0/image_raw + /imu0
      │  docker run kalibr:latest
      ▼
kalib/results/<session>/            *-camchain-imucam.yaml   ← 外参 T_cam_imu + timeshift
                                    *-results-imucam.txt    ← 残差/收敛信息
                                    *-report-imucam.pdf     ← 可视化报告
```

容器里看到的路径：宿主 `camera_imu_calibration/` 挂到 **`/data/`**，
所以 `data/session17/` 在容器里是 `/data/data/session17/`，
`kalib/bags/session17.bag` 在容器里是 `/data/kalib/bags/session17.bag`。

## 前置条件

| 项 | 要求 |
|---|---|
| Docker 镜像 | `kalibr:latest` 已构建（`docker images | grep kalibr` 能看到） |
| Python 环境 | `conda activate vins`（打 bag 需要 ROS1 的 `rosbag` 模块） |
| Docker 权限 | 有 docker 组权限，或者脚本会自动改用 `sudo docker` |
| 相机内参 | `configs/gemini_pro_rgb.yaml`（默认读这个，可用 `--camchain` 覆盖） |

## 用法

```bash
conda activate vins
cd ~/calib/camera_imu_calibration

python kalib/run_kalibr.py                     # 用 data/ 下最新的 session
python kalib/run_kalibr.py session17           # 指定 session
python kalib/run_kalibr.py session17 --rebuild # 强制重新打 bag（数据改过时用）
```

跑完结果在 `kalib/results/<session>/`，脚本会顺便打印：

- `T_cam_imu`（Kalibr 约定：IMU → 相机）
- `timeshift_cam_imu`（相机时间戳相对 IMU 的偏移，秒）
- `T_imu_cam = inv(T_cam_imu)`（**OpenVINS 的 `T_imu_cam` 用这个**）

## 换板型

默认是 **AprilGrid 6×6**（`tagSize=0.088`、`tagSpacing=0.56`，你卡尺量的值）：

```bash
python kalib/run_kalibr.py session17
```

如果这一轮用的是**打印棋盘格**（8×11 内角点、格边 0.03 m）：

```bash
python kalib/run_kalibr.py session17 \
    --target-type checkerboard --rows 11 --cols 8 --square-size 0.03
```

> `--rows/--cols` 是**内角点**数，不是黑白格数。数错一个，检测直接全灭。

想把某几秒单独拿出来（比如避开开头静止段）：

```bash
python kalib/run_kalibr.py session17 --from-to 2.0 18.0
```

## 配置在哪

首次运行会在 `kalib/configs/` 生成三个文件，之后**不会覆盖**（想改就直接编辑）：

| 文件 | 内容 | 来源 |
|---|---|---|
| `target.yaml` | 标定板几何 | 命令行参数（默认 AprilGrid 6×6） |
| `imu.yaml` | IMU 噪声（连续时间） | 2026-10-17 八小时静置实测 |
| `camchain.yaml` | 相机内参 | 复制 `configs/gemini_pro_rgb.yaml` |

`imu.yaml` 里现在写的是实测值：

```yaml
gyroscope_noise_density:     6.0816e-05    # 旧值 1.57e-3 → 小 25.8 倍
accelerometer_noise_density: 4.2627e-04
gyroscope_random_walk:       7.1351e-06
accelerometer_random_walk:   1.4861e-05
update_rate: 200.3
```

> Kalibr 的 IMU 残差权重 ∝ `1/(noise_density·√fs)²`，所以这份实测值会显著提高
> IMU 的话语权 —— 这是正确的设置（旧值是典型值，偏大两个数量级）。

## 出问题怎么查

| 现象 | 原因 / 处理 |
|---|---|
| `Kalibr 退出码 != 0` | 看 `kalib/logs/<session>_*.log`，最后 30 行通常有直接原因 |
| 检测不到板子 | 板型/尺寸参数不对，或这一轮没拍全板；换 `--target-type` 再试 |
| `肉眼看得到板，但检出 0 帧` | 先确认是 AprilGrid 还是棋盘格；AprilGrid 的 `tagSize` 是**黑色外框边长** |
| 结果里没有 `camchain-imucam.yaml` | 没收敛：激励不足 / 时间对不齐 / 板子太小（每格 < 80 px） |
| `current shell 没有 docker 组权限` | 脚本自动改用 `sudo docker`；或在**新终端**里跑（`usermod -aG docker` 后要重新登录） |
| `当前 Python 里没有 rosbag 模块` | 忘了 `conda activate vins` |
| 结果文件属主是 root | 脚本已用 `--user $(id -u):$(id -g)` 跑容器，正常不会出现 |

## 关于数据长度

Kalibr 官方建议 **40–60 秒**、多轴充分旋转。你现在的 session 大约 20 秒
（500+ 帧 @ ~25 fps）。能跑，但残差可能偏大；如果 `results-imucam.txt` 里
重投影 RMS 明显偏大，建议重录一段更长的再标。
