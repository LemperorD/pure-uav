# 相机深度、检测测距与制导

更新：2026-11-11。

`camera_server` 独占相机。VIO 和检测使用同一次采集的原生深度，分别处理，
检测测距不需要等待 VIO 特征初始化，也不在制导主循环内执行。

```text
camera_server ── VIO 通道：gray | depth_mm ──→ main / vio_live → VIO
              └─ 彩色通道：depth_mm | color ─→ detect_rt → 主目标 + 距离
                                                         ↓
                                           MissionTargetPacket → guidance
```

## 深度与帧对应

共享内存协议为 **V2**。`StreamMeta` 增加原生深度/彩色畸变、彩色内参和完整测距标定有效位。
两个通道保留同一个帧集的 `seq / host_ns / device_us / depth_device_us`。
深度为 `CV_16UC1`、毫米、原生深度相机像素域，未对齐到彩色。
不能用彩色检测框中心直接索引原生深度。

彩色通道负载按协议顺序排列为 `depth | color`，无深度时仍能检测彩色。
检测入队时一起复制彩色与对应深度；队列替换、异步推理、会话变化不会把新深度配给旧检测框。
相机 epoch 变化会清除待处理帧和旧检测结果，允许帧序号重新从 1 开始。

`pure_depth_range` 只依赖 OpenCV，不链接 OpenVINS、飞控或 IMU。
它通过真实 D2C 外参将原生深度正向投影到原始彩色图，邻域查询取中位深度；
稀疏支持不足、前后景深度不连续、超量程、无效像素和两路设备时间差超过 50 ms 时不输出距离。
默认距离范围为 0.15–10 m，这是软件过滤范围，实际可用距离仍取决于相机和场景。

`range_m` 是**彩色相机中心到目标表面的直线距离**，单位米，包含离轴射线长度；
不是原生深度 Z 值。无目标、无深度、标定缺失或投影失败时，每帧均清零：
`has_range=false, range_m=0`。目标框可以继续发布，任务入口仍要求有效测距。

采集时间 `host_ns` 为 CLOCK_MONOTONIC，不能直接当成飞控 boot 时间或 Unix 时间。
测距利用同源设备时间差，把深度时间映射到彩色帧的单调时钟域。
主程序继续通过 VIO 的时间映射把目标采集时间换为飞控 boot 时间。

## 标定配置

- `config/camera.yaml` 的 `camera.range_color_calibration_path` 指定原始彩色 Kalibr `cam0`
  内参/畸变。默认指向 `gemini_1920x1080/kalibr_imucam_chain.yaml`；分辨率必须匹配。
  空路径使用 SDK 彩色标定。
- 深度内参/畸变与 D2C 外参来自 SDK，通过共享内存传递；外参无法读取时不假定两相机重合，
  测距保持不可用。SDK 的畸变字段转换为 OpenCV 顺序 `k1,k2,p1,p2,k3,k4,k5,k6`。
- `config/guidance-camera.yaml` 保存制导的彩色内参/畸变与光学系到机体 FRD 的旋转。
  guidance 会先对原始检测像素去畸变，再反投影为视线。
- `main --mission` 在配置阶段校验制导标定分辨率，连接相机后再核对实际发布的彩色内参与畸变。
  测距与制导的彩色标定不一致时启动失败。
- 当前相机安装旋转未确认，`guidance-camera.yaml` **保持 `calibrated: false`**；
  不自动填入假外参。测距可单独验收，制导须待真实安装标定完成后启用。

VIO 继续由 `config/vio.yaml` 的 `use_depth / use_depth_init` 控制。
驱动统一将 SDK 深度缩放为毫米，缩放前的 `0 / 65535` 无效码转换为 0，非法单位拒绝使用。
VIO 的估计器投影与制导目标测距分别运行。

## 重建与无硬件验证

V2 不兼容旧版进程。先停止旧进程，再一起重建、重启 `camera_server / main / vio_live / detect_rt`。
VIO 与彩色通道默认各 3×8 MiB；`/dev/shm` 建议至少预留 64 MiB。

```bash
cmake -S . -B build
cmake --build build -j2
ctest --test-dir build --output-on-failure \
  -R '^(depth_range|rgbd_pipeline|detection_runtime|target_bridge|guidance_camera|camera_shm|shm_camera_source|app_config|mission_wire)$'
./bin/camera_server --check-config
./bin/main --mission --no-send --check-config
```

`rgbd_pipeline` 覆盖真实发布器/共享内存、深度与彩色偏移、外参求逆、米制距离及目标 socket 传递；
`detection_runtime` 使用合成推理后端测试实际线程、深度所有权、队列替换、会话重置和无效深度清零；
`guidance_camera` 验证配置加载和去畸变射线。

实机验收时先运行 `camera_server`，再观察 `detect_rt --json` 的 `has_range / range_m`。
确认已知距离与画面中心/边缘测量，再验证转动、遮挡、目标边缘和相机重启。
独立 `depth_range_probe --pixel U V` 会直接打开设备，只能在相机服务停止后使用。
无硬件测试不代表已经验证实际深度精度、USB 时序或飞行行为。
