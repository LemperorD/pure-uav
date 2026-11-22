# 单独启动 VIO

本文说明如何在**单个终端**内单独启动实时 VIO。`vio_live` 默认会自动启停同目录下的 `camera_server`，只运行相机采集、飞控 IMU 接收与 VIO 位姿估计，**不会启动目标检测、制导或任务控制**。

所有命令默认在**仓库根目录**执行。

---

## 1. 编译（如 `bin/` 下无产物）

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2 --target vio_live
```

（构建 `vio_live` 时会自动一并构建 `camera_server`。）

---

## 2. 启动前检查

运行前确认两份配置无误：

- `config/camera.yaml`：相机分辨率、帧率、深度开关、曝光/增益及 `vio_config_dir`（默认 `1920x1080@30`、开深度）。
- `config/driver.yaml`：飞控 IMU 请求间隔 `highres_imu_us`（默认 `5000` μs = 200 Hz）。

无硬件校验：

```bash
./bin/vio_live --check-config
```

---

## 3. 启动命令

### 推荐：使用一键脚本

自动启停 `camera_server`，并按时间戳把日志、轨迹和 IMU 保存到 `src/vio/out/live_<时间戳>/`：

```bash
# 本地测试（不向飞控发送位姿，开实时窗口）
bash src/vio/scripts/run_vio_live.sh

# 正式运行（向飞控发送 MAVLink ODOMETRY，不开窗口）
bash src/vio/scripts/run_vio_live.sh --fly
```

可按需叠加脚本选项（支持自由组合，例如 `--fly --static`）：

| 选项 | 作用 |
|---|---|
| `--static` | 改为静态初始化（先完全静止 ≥2 秒，再果断移动触发；重力方向更准），默认动态初始化 |
| `--fly` | 向飞控发送 MAVLink `ODOMETRY` 且不开预览窗口，默认不发送、开窗口 |
| `--duration S` | 运行 `S` 秒后自动退出，默认一直运行到手动停止 |
| `--out DIR` | 指定日志输出目录（相对 `src/vio/`），默认 `out/live_<时间戳>` |
| `-- <参数>` | `--` 之后的参数原样透传给 `vio_live`，如指定串口 `-- --serial /dev/ttyUSB0 --baud 921600`、录制数据集 `-- --record datasets/run1`、复用外部相机服务 `-- --external-camera` |

### 或直接运行 `./bin/vio_live`

不走脚本、直接拉起程序（同样自动启停 `camera_server`，完整参数见 `./bin/vio_live --help`）：

```bash
./bin/vio_live --no-send --view   # 本地测试（不发飞控，开窗口）
./bin/vio_live                    # 正式运行（向飞控发送 ODOMETRY，不开窗口）
```

> 两者差别：`run_vio_live.sh` 是 `vio_live` 的封装脚本，默认不发飞控、开窗口并按时间戳目录保存全套日志和轨迹；直接跑 `./bin/vio_live` 默认直接发飞控、不开窗口且只写一份会被下次覆盖的轨迹文件。

---

## 4. 退出与结果查看

- **退出**：在预览窗口按 `q` / `Esc`，或在终端按 `Ctrl-C`，`vio_live` 会自动停止并回收其启动的 `camera_server`。
- **结果分析**：脚本运行后会在 `src/vio/out/<目录>/` 生成 `pose.csv`、`traj.tum`、`traj_safe.tum`、`imu.csv` 和 `console.log`，可用以下命令分析：
  ```bash
  python3 src/vio/scripts/analyze_live.py src/vio/out/<目录>
  ```

---

## 5. 注意事项

1. **串口独占**：`vio_live` 与 `bin/main` 或地面站不能同时占用同一个飞控串口；设备名不是 `/dev/ttyACM0` 时请通过 `--serial` 指定。
2. **复用外部相机服务**：如果已手动启动 `camera_server`，请给 `vio_live` 传 `--external-camera`（脚本用 `-- --external-camera`），仅连接而不重复启停。
3. **相机参数位置**：曝光、增益、测光和深度开关统一在 `config/camera.yaml` 中修改。
4. **更多文档**：详见 [src/vio/README.md](../../src/vio/README.md) 与 [camera-ipc.md](app/camera-ipc.md)。
