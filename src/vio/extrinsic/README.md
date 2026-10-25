# Gemini Pro RGB + CUAV 7 Nano 相机–IMU 外参标定

这是一套独立 Python 实现的**目标板式、单目相机–IMU 外参及时间偏移标定**程序。相机不需要自带 IMU；相机与外置 IMU 必须刚性固定在一起，并在同一次运动中连续采集图像和 IMU。已有静态内参照片不能替代这次动态采集。

程序借鉴 Kalibr 的连续时间联合优化思路，没有克隆、编译或调用 Kalibr。当前支持 `pinhole + radtan`、固定内参、棋盘格与 tag36h11 AprilGrid。优化旋转、平移、时间偏移、恒定陀螺仪/加计零偏、重力方向和整段样条轨迹。

**开发验证与设备标定分开看**：本目录的 `validation/` 是合成数据和软件测试证据，里面的外参不是你的 Gemini Pro 与 CUAV 的实际安装外参。真实设备外参必须用实际同步采集的数据重新求解。详见 `VALIDATION.md`。

## 1. 先运行软件自检

在 PyCharm 把本目录作为项目打开。你当前可使用的解释器为 `/home/kxy/miniforge3/envs/vins/bin/python`。从终端运行也可以：

```bash
conda activate vins
cd /path/to/camera_imu_calibration
python -m unittest discover -s tests -v
# 含高频运动与因果滤波负对照的较长回归（最多两个求解进程）
# python verify_review.py --output validation/my_review_run
OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 python verify_synthetic.py --output validation/my_synthetic_test
```

`/path/to/...` 是占位路径，替换成本目录的实际位置。验证生成带畸变、模糊和像素噪声的**合成图片**以及带零偏、噪声的**合成 IMU**，调用真正的检测、PnP 与联合标定流程，然后和保存在独立真值文件中的外参比较。首次命令执行包括渲染图片；每次请使用新的输出目录。

新建环境时安装 `requirements.txt`：

```bash
python -m pip install -r requirements.txt
```

如果环境已有 `cv2.aruco`，不要为运行本工程额外混装多个 OpenCV wheel。标定本身不需要 ROS、roscore 或 Ceres；读 bag 只需 `rosbags`。ROS 只涉及现场采集或后续 OpenVINS 运行。

## 2. 你这套设备怎样采数据

1. 把 Gemini Pro 与 CUAV 7 Nano 固定在同一块坚固支架上，标定后用于 VIO 时保持安装关系。机内 IMU 到飞控外壳的精确位置可以未知，程序估计有效 IMU 测量原点到相机的变换。
2. 相机使用做内参时的 RGB 流、分辨率与镜头设置。示例 `configs/gemini_pro_rgb.yaml` 的数字来自提供的参考文档，应优先替换为你最终的内参 YAML。不要混用深度流、IR 流、缩放图像或者其他分辨率的内参。
3. 用一块静止、平整、尺寸实测的标定板。`configs/checkerboard.yaml` 默认 8×11 **内角点**、格边长 0.03 m，仅是示例。AprilGrid 的 `tagSize` 是黑色外框边长，`tagSpacing` 是白间隙与该边长之比；使用官方 tag36h11 排列和方向。屏幕显示尺寸、刷新与摩尔纹会增加误差，最终建议使用实体板。
4. 固定标定板，手持整套设备，在板前做连续、平滑、多轴转动及适量平移；保持整块棋盘或足够 AprilTags 持续可见。默认优化一个约 30 秒窗口，另录几个独立窗口做复核。仅静止、纯平移或只绕一根轴转动，不足以可靠估计全部外参。
5. 图像建议连续 20–30 Hz，IMU 建议至少 100–200 Hz；这些是采集建议，不是本次已经实测的设备性能。默认优化最多保留 15 Hz 图像和 100 Hz 原始 IMU 样本，可在配置中调节。运动应相对这些频率足够平滑；本程序的抽样不是抗混叠滤波。
6. 飞控输出必须是同一个 IMU/同一组固定轴定义的 `gx,gy,gz` 和 `ax,ay,az`。角速度单位 rad/s，加速度单位 m/s²；加速度是**比力，静止时模长约为当地重力值**，不能使用去重力后的线加速度、姿态角或飞控估计位置。不要把多个 IMU 实例混在一起，也不要让飞控在记录中途切换实例。
7. 核实固件数据链：应获取额外因果滤波前、同一固定 IMU 逻辑实例的陀螺和加计，并核对两路的采样时间定义。消息名含 `raw` 不足以证明这一点。硬件滤波/积分窗口仍需核实；当前模型不估计两路频率响应。不要为标定擅自关闭飞控运行中的安全滤波。
8. 保存采样时间。相机与飞控必须先映射到同一时钟域；相机主机时间与飞控开机微秒不能直接混用。程序的时间偏移搜索默认只有 ±0.1 s，用来估计映射后剩余的固定偏差。USB/串口到包时间的抖动、曝光时间定义、时钟漂移仍会影响精度，不能靠“第一帧和第一条 IMU 对齐”解决。本项目的采集器把 IMU 时间戳锚定到飞控 `time_usec` 的全段下包络（`capture/imu_timebase.py`），读线程积压不再进入时间戳；`imu.csv` 旁的 `imu_timebase.json` 记录了这次锚定的漂移/延迟/回拨量。即便如此，**时移仍应在同一段录制上现场估计**，不要跨 session 搬（见 3.1）。
9. **先看亮度，再谈模糊。** 检出掉到 0 有两个完全相反的原因，处理方式也相反：
   - **太暗**（白格 p90 < 45，`capture/framing_check.py` 会直接判 dark）：板子没被光照到，任何曝光参数都救不回来，**先开灯/加一盏补光灯正对板子**。房间暗时把 `EXPOSURE` 往下调只会更黑（2026-10-18 实测：房间关灯后 p90 从 205 掉到 31，检出 0/20）。
   - **运动模糊**（亮度正常、被拒帧清晰度只有合格帧的一半左右）：曝光 `200`（20 ms）在快速转动时会把 tag 边缘拖成一片。这时才考虑把 `EXPOSURE` 调到 150→100（越暗越往上加，别反过来），`GAIN` 10→30 补亮度但会带噪声。

   两条路的共同目标都是白格 p90 落在 **60~150**；录制时让整块板子全程留在画面里、慢一点更稳 —— 标定只需要约 3°/s（`min_rotation_std_rad_s=0.05`）的多轴激励。

本项目不包含已经在这两件设备上验证过的实时采集驱动。CUAV 的固件、连接方式和现有驱动输出需要现场确认；本次没有访问硬件或配置飞控。你可以用现有驱动录制 image/Imu 话题的 ROS1 `.bag` 或 ROS2 bag 目录，也可以提供下述文件格式。**同一个 bag 和 `header.stamp` 都不保证同一时钟域**：时间来自消息发布者，而非 bag 自动同步。严格递增也不能证明没有混用 IMU 实例。

## 3. 输入方案 A：图像清单 + IMU CSV

目录示例：

```text
data/session01/
  images.csv
  imu.csv
  images/
    000000.png
    000001.png
    ...
```

`images.csv` 必须有明确表头，文件名相对于该 CSV 所在目录：

```csv
timestamp,filename
1700000000000000000,images/000000.png
1700000000033333333,images/000001.png
```

`imu.csv` 必须有以下字段，可有其他列：

```csv
timestamp,ax,ay,az,gx,gy,gz
1700000000000000000,0.03,0.02,9.80,0.01,-0.02,0.01
1700000000005000000,0.04,0.02,9.79,0.01,-0.01,0.02
```

以上是**格式示例**，不能作为标定数据。这里的时间单位是整数 ns；也支持显式指定 `s / ms / us`。十进制秒由 Decimal 转为纳秒，先减去整数共同起点后才转浮点，避免大 epoch 丢失精度。不从文件修改时间推算拍摄时间，不自动猜时间单位，不自动重排重复/逆序样本。

```bash
python calibrate_extrinsics.py \
  --camera configs/gemini_pro_rgb.yaml \
  --target configs/checkerboard.yaml \
  --images data/session01/images.csv \
  --imu data/session01/imu.csv \
  --image-unit ns --imu-unit ns \
  --output results/session01
```

PyCharm 直接运行：修改 `extrinsics_config.py` 最下方的 `PATHS`、话题和单位，再运行 `calibrate_extrinsics.py`，不用填写命令行。初次运行前必须换好实际数据路径与标定板尺寸。

### 3.1 检出空洞：自动选窗与诊断

整段录像里只要有一处「连续超过 `max_camera_gap_s`（默认 0.35 s）一帧都认不出标定板」——手转太快糊了、板子出画、被遮挡——过去是整段拒绝，只报一句「成功检测标定板帧之间存在大空洞」。现在默认 `auto_window: 1`：

- 先按 `detect_rate_hz`（默认 0 = 每帧）逐帧检测，再按 `camera_rate_hz` 把**已接受**的帧抽稀。抽样只省时间，不决定窗口边界：30 fps 原流按 15 Hz 预抽帧会把 5.36 s 的可用段切成 4.73 s，低于 `min_duration_s` 就误判成「整段没有可用窗口」（session11 实测）。
- 接受帧按 `max_camera_gap_s` 切成连续段，自动取最长且满足 `min_duration_s` / `min_frames` 的一段来拟合，终端打印选窗、丢弃帧数、空洞位置。
- 空洞列表、最长连续段、被拒帧与合格帧的清晰度对比写进 `report.json` 的 `window` / `visual_diagnostics`，失败报告里也带这些证据。
- 一段都不够长时直接失败，但会说清「最长连续段只有 4.73 s / 50 帧，低于下限 5 s」「被拒帧清晰度只有合格帧的 64% → 运动模糊」，而不是让你猜。

`--no-auto-window` 恢复「整段必须连续」的老行为。只想先出个结果时可以 `--min-duration 4` 放宽时长下限：那是在门槛外拟合，报告里的 `limitations` 会记一笔，不能当正式结果。

## 4. 输入方案 B：ROS1 / ROS2 bag

```bash
python calibrate_extrinsics.py \
  --camera configs/gemini_pro_rgb.yaml \
  --target configs/aprilgrid.yaml \
  --bag data/session01.bag \
  --cam-topic /cam0/image_raw --imu-topic /imu0 \
  --start 2 --duration 25 \
  --output results/session01_bag
```

ROS2 传 `--bag data/session01_ros2/`（含 metadata.yaml 的 bag 目录）；已用原生 ROS2 Humble bag 做读取回归，不需要安装 ROS 运行时。话题名是示例，改为 bag 中的实际名称。图像支持原始 `sensor_msgs/Image` 的 mono8/rgb8/bgr8/rgba8/bgra8，或 `sensor_msgs/CompressedImage`。IMU 为 `sensor_msgs/Imu`；姿态四元数不参与标定，使用消息的比力和角速度。

始终使用 `header.stamp`，不使用 bag 记录时间。`--start` 相对于第一张输入图像的 header 时间；标定程序只优化选中的窗口，仍会顺序读取 bag 的相关消息。普通 EuRoC 飞行序列的无板场景不能替代目标板标定序列。

## 5. 飞控时钟与相机时钟不同怎么办

如果现有驱动已正确同步/映射到共同时间基准，不需要重复映射。只有在确认时钟不同后，`remap_imu_clock.py` 接受**已经实测的同步锚点**，拟合时钟频率比与起点偏差：

```csv
imu_timestamp,camera_timestamp
1000000,1700000000000000000
...
```

其中每一对必须对应同一个物理时刻。锚点至少 5 个，覆盖全部数据以及不少于 5 秒；不能拿两条无关联消息凑成锚点。若使用协议时钟同步，需先正确换算出与图像时间一致的主机时钟锚点；本脚本不实现 MAVLink TIMESYNC。

```bash
python remap_imu_clock.py \
  --input data/session01/imu_fcu_clock.csv \
  --anchors data/session01/clock_anchors.csv \
  --imu-unit us --camera-unit ns \
  --output data/session01/imu.csv
```

输出时间单位固定为 ns，输出旁边的 `.clock.json` 保存拟合漂移和残差。默认拟合 RMS 超过 2 ms 拒绝转换。这是对锚点一致性的检查，不是对真实同步误差的认证；带有未知系统偏差的到包时间仍可能得到看似很好的拟合。

## 6. 输出与坐标约定

程序成功后生成：

- **4 系数 radtan**：`camchain-kalibr.yaml` 包含 `T_cam_imu` 和 `timeshift_cam_imu`；`camchain-openvins.yaml` 包含互逆的 `T_imu_cam`，使用 OpenCV 可解析的 YAML。
- **5 系数 OpenCV 畸变输入**：仍可求解，但只生成 `camchain-opencv.yaml` 保留 k3、`T_cam_imu` 和时移；不生成 Kalibr/OpenVINS 兼容文件。`report.json` 明确说明跳过原因。不能直接截掉 k3；需要目标模型一致的内参再导出。
- `report.json`：残差、激励指标、每阶段优化器终止状态、零偏、重力、时间偏移、变换矩阵和设置。
- `residuals.npz`、`trajectory.npz`：实际残差与样条参数，便于追查。
- `extraction.json`、`detections/`：逐帧检测信息和抽样角点检查图。

约定明确如下，不能按文件名猜方向：

```text
p_C = R_CI p_I + p_CI = T_cam_imu * p_I
p_I = T_imu_cam * p_C
T_imu_cam = inverse(T_cam_imu)
t_imu = t_cam + timeshift_cam_imu
```

`p_CI` 是 IMU 原点在相机坐标系里的位置。它**不是**相机原点在 IMU 中的位置，后者为 `-R_CI.T @ p_CI`。时间偏移为正时，对应物理时刻的 IMU 时间戳比相机时间戳大。参考文档里的 `T_cam_imu` 示例注释把变换方向写反了，本实现按 Kalibr 官方定义处理。

每次使用新的输出目录。目录非空时拒绝运行，避免失败后把上一次 YAML 当作这次结果。标定不收敛、超出残差门槛、时间偏移/平移/零偏贴近边界时只保留诊断，不生成新的有效外参 YAML。

## 7. 算法与适用边界

相机位姿 `R_WC(t),p_WC(t)` 由 PnP 初始化；世界坐标系是静止标定板。平移采用均匀三次 B-spline，旋转采用 SO(3) 累积三次 B-spline。解析计算平移二阶导数、相机体坐标角速度和角加速度。先固定轨迹估计全局参数，再用稀疏最小二乘联合优化角点投影和 IMU 残差，采用 soft-L1 降低异常值影响。

IMU 的世界位置为 `p_WI=p_WC+R_WC p_CI`，因此加速度同时包含杠杆臂的切向项和向心项。IMU 样本时间 `t_i` 在相机轨迹上的查询时间为 `t_i-shift`。重力模长以强约束固定，方向随数据估计；零偏按整段常数处理。

`extrinsics_config.py` 集中设置结点间距、窗口、抽样率、噪声权重、求解预算和验收门槛；`--settings settings.yaml` 可覆盖其中任意字段，未知字段报错。命令行还可覆盖 `--start / --duration / --knot-spacing / --max-nfev`。默认结点间距 **0.04 s**、联合求解预算 **300**；这是针对本次高频合成运动更合适的起点，不是任意手持数据的精度保证。慢且平滑的运动可试 `--settings configs/motion_slow.yaml`（0.12 s / 120）。修订版慢运动回归有误差变大、接近 5mm 门槛的样例，见 `VALIDATION.md`；不要通过放宽门槛把未收敛包装为通过。

`gyro_sigma_rad_s` 和 `accel_sigma_m_s2` 是**保留的单个样本标准差**，不是 Kalibr/OpenVINS 的连续时间噪声密度，也不是 bias random walk。二者不能数值直接抄用；采样、积分、滤波和样本相关性都会影响白化权重。实测噪声只有在统计定义/测量数据链匹配时才有意义。默认权重是工程初值，**不是 CUAV 噪声实测值**。

可选采集说明：复制 `configs/acquisition.example.yaml` 到本次数据目录，据实填写并传 `--acquisition data/session01/acquisition.yaml`。未知项允许诊断运行，但在 stderr 和报告中警告；已声明不同域/混用实例/额外因果滤波则拒绝按当前理想模型标定。`gyro_instance`/`accel_instance` 指同一逻辑实例，不要求硬件 gyro/accel device_id 数值相同。声明是操作员提供的溯源信息，**不是工具认证**；不要为了通过而填未经验证的 `raw/shared/single`。

静止数据和多轴旋转不足会拒绝。平移激励检查会先从杠杆臂线性回归中消去重力与加计零偏，再看剩余矩阵的最小奇异值。但这些是必要的工程筛查，不能代替完整可观测性证明或参数置信区间。请检查残差、观察不同窗口是否给出一致结果，并在实机上做后续轨迹对比。不一致还可能源于激励、模型不足或局部收敛，不能仅归因为滤波或时钟。**因果滤波可能在低残差下污染时移和平移，单一时移/在线估计不能一般性补偿两路频率响应。** 深度保护在优化试探步中使用有限延拓加独立可见性惩罚；最终所有角点须在相机前至少 `min_target_depth_m`，后方点不被当作物理投影。

当前不估计滚动快门行延迟、IMU 比例因子/非正交性、温漂、时变偏移或多 IMU 切换。若使用的 RGB 流存在明显滚动快门效应，运动要平滑，并检查模型残差；这里没有声称已确认你的相机快门类型。降低分辨率/提高帧率不自动消除行延迟；若改变相机采集模式，需要复核该模式内参。也没有实现双目相机之间的外参标定。优化器终止只说明数值终止条件满足，结果精度需要另外验证。

## 8. 给 OpenVINS 使用

`prepare_openvins.py` 基于你提供的 OpenVINS 配置模板生成一个单独目录，保持离线外参可被后续评估：

```bash
python prepare_openvins.py \
  --calibration results/session01 \
  --template-config /path/to/open_vins/config/your_template \
  --imu-topic /imu0 --cam-topic /cam0/image_raw \
  --timeoffset-mode frozen \
  --output results/openvins_gemini_cuav
```

生成配置设置单目、关闭外参和内参在线标定。默认 `frozen` 关闭在线时移估计，用于隔离评估离线结果；另用新的输出目录传 `--timeoffset-mode online` 可只开启时间偏移在线估计，作单独对照。这不承诺修复滤波、时钟漂移或滚动快门。

脚本现会同时更新 estimator 的话题和相机/IMU 链中的 `rostopic`，不需要按旧报告再手改两处。优先级：显式 CLI > 标定报告的实际 bag 话题 > 模板/相机链话题。模板里的 IMU 噪声、变换及扩展字段保留并写入新目录，原模板不改；噪声数值不会由此变为 CUAV 实测值，必须换成你的 CUAV 实测/确认过的噪声。不能把 EuRoC 的 IMU 噪声当作 CUAV 的噪声，也不能把合成标定外参拿来验证 EuRoC 的真实轨迹。

这次交付没有运行 Gemini Pro + CUAV 的 OpenVINS，也没有报告该设备的 ATE。完整设备性能测试需要这套设备的实际 VIO 数据和独立参考轨迹。这是实现完成之后的实验工作。

## 9. 参考与源码定位

- Kalibr 官方 YAML 方向与时间定义：https://github.com/ethz-asl/kalibr/wiki/yaml-formats
- Kalibr 官方 AprilGrid 点序：https://github.com/ethz-asl/kalibr/blob/master/aslam_cv/aslam_cameras_april/src/GridCalibrationTargetAprilgrid.cpp
- OpenVINS 配置解析：https://github.com/rpng/open_vins/blob/master/ov_core/src/utils/opencv_yaml_parse.h
- OpenVINS 外参与时间参数加载：https://github.com/rpng/open_vins/blob/master/ov_msckf/src/core/VioManagerOptions.h

`calib/spline.py` 是数学模型，`calib/solver.py` 是联合优化与导出，`calib/data.py` 是输入/检测/PnP。主入口、合成验证、时钟映射和 OpenVINS 配置导出均位于项目根目录。交付目录及压缩包不包含第三方仓库或 `.git` 文件夹。

## 10. 本次复核修订

2026-10-15 修订、采纳/限定/纠正理由见 `REVIEW_RESPONSE.md`；27 项自动测试和七组数值对照见 `VALIDATION.md`。原 2026-10-13 证据保留为历史，不混作新默认配置结果。最终文件完整性可在本目录运行 `sha256sum -c MANIFEST.sha256` 检查。
