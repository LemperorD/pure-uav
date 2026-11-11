# 板端测距验收清单

- 更新：2026-11-11
- 范围：目标测距链路 `camera_server → detect_rt → MissionTargetPacket → main`
- 不在本清单内：撞板/制导闭环（需 `guidance-camera.yaml` 完成相机安装标定）
- 背景：[vio-depth-range-contract.md](vio-depth-range-contract.md)、[depth-guidance.md](depth-guidance.md)

---

## 0 软件前置（任意机器，无硬件）

```bash
cmake -S . -B build && cmake --build build -j2
ctest --test-dir build --output-on-failure \
  -R '^(depth_range|rgbd_pipeline|detection_runtime|target_bridge|guidance_camera|mission|mission_wire|target_adapter)$'
./bin/camera_server --check-config
./bin/detect_rt --check-config
./bin/main --mission --no-send --check-config
```

预期输出：

```text
[camera_server] 配置有效: 1920x1080@30 depth=1 vio=/pure_uav_camera_vio color=/pure_uav_camera_color color_hz=10
[detect_rt] 配置有效（未加载模型、未连接相机服务）
Configuration OK
```

任一失败都先在本机解决，不要带上飞机。

---

## 1 上板前置检查

- [ ] `config/camera.yaml`：`camera.use_depth: true`
- [ ] `camera.range_color_calibration_path` 指向真实彩色 Kalibr cam0，分辨率 1920×1080
- [ ] `/dev/shm` 剩余 ≥ 64 MiB
- [ ] 旧版进程已全部停止（V2 共享内存协议不兼容旧进程）
- [ ] `bin/camera_server`、`bin/detect_rt`、`bin/main` 为同一提交构建

```bash
df -h /dev/shm
pkill -f camera_server; pkill -f detect_rt; pkill -f 'bin/main'
```

---

## 2 启动 camera_server（终端 A）

```bash
./bin/camera_server
```

通过判据：日志中三条都出现

```text
[Orbbec] 深度内参: fx=... fy=... cx=... cy=... (WxH)
[Orbbec] 彩色内参: fx=... fy=... cx=... cy=...
[Orbbec] 深度→彩色 平移: [x y z] mm
```

直接判失败：

- `[Orbbec] 深度外参查不到，目标测距不可用` —— 没有 D2C 外参，测距恒不可用；
- `[Orbbec] 深度流开启失败` —— 只用彩色，测距不可用；
- `[Orbbec] 深度内参` / `彩色内参` 缺失。

---

## 3 检测测距（终端 B）

```bash
./bin/detect_rt --json
```

通过判据：

- 有目标时输出包含 `"present":true,"has_range":true,"range_m":<米>`；
- `range_m` 与卷尺/地面真值同量级，且随距离单调变化；
- 目标移出画面或遮挡后，下一帧变为 `"has_range":false,"range_m":0`，**不能保留旧值**；
- 相机服务重启后，序号/会话变化，旧距离立即失效。

### 3.1 静态精度表（每个距离测 5 次）

| 真值 (m) | 位置 | 5 次 `range_m` | 均值 | 最大误差 (m) | 有效率 |
| --- | --- | --- | --- | --- | --- |
| 1.0 | 画面中心 |  |  |  |  |
| 2.0 | 画面中心 |  |  |  |  |
| 3.0 | 画面中心 |  |  |  |  |
| 5.0 | 画面中心 |  |  |  |  |
| 2.0 | 画面左/右边缘 |  |  |  |  |
| 2.0 | 画面上下边缘 |  |  |  |  |

通过标准（建议值，按实际标定可调）：

- 1–5 m 静态误差 ≤ 0.2 m；
- 8 m 内仍能给出有效距离；
- 中心与边缘误差差异 ≤ 0.1 m；
- 越过 0.15 m / 10 m 软限幅时输出无效，不 clamp。

### 3.2 失效场景

- [ ] 遮挡目标：`has_range=false`；
- [ ] 背景/地面强反光：不出现明显跳变到错误距离；
- [ ] 目标贴边、框中心落在背景：拒绝输出，而不是给背景距离；
- [ ] 拔相机/停 `camera_server`：`detect_rt` 报错退出，不刷旧值。

---

## 4 任务侧启动（终端 C）

```bash
./bin/main --mission --no-send --check-config
./bin/main --mission --no-send
```

通过判据：

- `--check-config` 输出 `Configuration OK`；
- `--no-send` 启动时**不出现** `[CAM] 目标测距不可用`；
- 若 `guidance-camera.yaml` 仍为 `calibrated: false`，允许出现
  `[MAIN] 彩色相机未标定…任务只搜索，不进入撞板`，这是预期；
- `mission_guidance_ready` 只有在相机安装标定完成后才应为 `true`。

观察任务日志：

```text
[MISSION] state=preflight ...
[MISSION] state=offboard ...
[MISSION] state=arming ...
[MISSION] state=takeoff ...
[MISSION] state=search ...
```

`--no-send` 下不会真正解锁/起飞，只验证状态机与测距门控不报错；实飞撞板需等标定完成。

---

## 5 失败排查

| 现象 | 原因 | 处理 |
| --- | --- | --- |
| `目标测距不可用：camera.yaml 的 use_depth=false 或深度流未开启` | 深度未开 | 改 `config/camera.yaml` 的 `use_depth: true` 后重启服务 |
| `目标测距不可用：相机服务未提供完整的深度内参/D2C 外参/彩色标定` | SDK 缺 D2C 或标定文件 | 看 `camera_server` 日志；换相机固件或补标定 |
| `guidance 彩色标定分辨率与 camera_server/VIO 期望不一致` | `guidance-camera.yaml` 分辨率错 | 与 `config/vio.yaml` 的 1920×1080 对齐 |
| `guidance-camera.yaml 与 camera_server 的彩色内参/畸变/分辨率不一致` | 两份彩色标定不同源 | 统一用同一份 Kalibr cam0 |
| `detect_rt` 一直 `has_range=false` | 深度稀疏/超量程/时间差 > 50 ms | 拉近距离、避免纯色/反光面，检查两路时间戳 |
| 旧进程连不上共享内存 | V2 协议不兼容 | 停掉所有旧进程后一起重启 |

---

## 6 验收记录

| 项 | 结果 |
| --- | --- |
| 提交 / 分支 |  |
| 板端型号 / 相机 |  |
| `camera_server` D2C 日志 | 通过 / 失败 |
| 1–5 m 误差 |  |
| 边缘误差 |  |
| 失效场景 |  |
| `main --mission --no-send` 启动检查 |  |
| 操作者 / 日期 |  |
