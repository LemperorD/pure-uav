# 单独启动 VIO（`camera_server` + `vio_live`）

本文说明如何用独立入口 `vio_live` 单独运行实时 VIO。该方式只运行相机采集、飞控 IMU 接收与 VIO 位姿估计，**不会启动目标检测、制导或任务控制**。

所有命令默认在**仓库根目录**执行。

---

## 1. 编译（如 `bin/` 下无产物）

```bash
cmake -S . -B build -DCMAKE_BUILD_TYPE=Release
cmake --build build -j2 --target camera_server vio_live
```

---

## 2. 启动前检查

运行前确认两份配置无误：

- `config/camera.yaml`：相机分辨率、帧率、深度开关、曝光/增益及 `vio_config_dir`（默认 `1920x1080@30`、开深度）。
- `config/driver.yaml`：飞控 IMU 请求间隔 `highres_imu_us`（默认 `5000` μs = 200 Hz）。

可先做无硬件校验：

```bash
./bin/camera_server --check-config
./bin/vio_live --check-config
```

---

## 3. 启动步骤

`vio_live` 通过 POSIX 共享内存（默认 `/pure_uav_camera_vio`）从 `camera_server` 读图，必须分两步启动。

### 第 1 步：启动相机服务（终端 1）

```bash
./bin/camera_server
```

等待终端打印相机就绪、开始发布帧后，再执行第 2 步。

### 第 2 步：启动 VIO（终端 2）

**推荐：使用一键脚本**（自动保存日志、轨迹和 IMU 数据到 `src/vio/out/live_<时间戳>/`）：

```bash
# 本地测试（不向飞控发送位姿，开实时窗口）
bash src/vio/scripts/run_vio_live.sh

# 正式运行（向飞控发送 MAVLink ODOMETRY，不开窗口）
bash src/vio/scripts/run_vio_live.sh --fly
```

以上命令还可按需叠加以下可选参数（可自由组合，如 `--fly --static`）：

| 选项 | 作用 |
|---|---|
| `--static` | 改为静态初始化（先完全静止 ≥2 秒，再果断移动触发；重力方向更准），默认动态初始化 |
| `--fly` | 正式飞行模式：向飞控发送 MAVLink `ODOMETRY` 且不开预览窗口，默认不发送、开窗口 |
| `--duration S` | 运行 `S` 秒后自动退出，默认一直运行到手动停止 |
| `--out DIR` | 指定日志输出目录（相对 `src/vio/`），默认 `out/live_<时间戳>` |
| `-- <参数>` | `--` 之后的参数原样透传给 `vio_live`，如 `-- --record datasets/run1`（录制离线数据集）或 `-- --serial /dev/ttyUSB0 --baud 921600`（指定串口） |

---

## 4. 退出与结果查看

- **退出**：在预览窗口按 `q` / `Esc`，或在终端 2 按 `Ctrl-C` 停止 `vio_live`，再在终端 1 按 `Ctrl-C` 停止 `camera_server`。
- **日志输出**：脚本运行后会在 `src/vio/out/<目录>/` 生成 `pose.csv`、`traj.tum`、`traj_safe.tum`、`imu.csv` 和 `console.log`，可用以下命令做快速分析：
  ```bash
  python3 src/vio/scripts/analyze_live.py src/vio/out/<目录>
  ```

---

## 5. 注意事项

1. **串口独占**：`vio_live` 与 `bin/main` 或地面站不能同时占用同一个飞控串口。
2. **相机独占**：`camera_server` 独占 Orbbec 相机，运行期间不要同时启动其他直采工具。
3. **参数位置**：曝光、增益、测光和深度采集开关统一在 `config/camera.yaml` 中修改，修改后需重启 `camera_server`。
4. **更多文档**：详见 [src/vio/README.md](../../src/vio/README.md) 与 [camera-ipc.md](app/camera-ipc.md)。
