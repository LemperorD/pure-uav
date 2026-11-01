# VIO 测试 + RViz 接入

日期 2026-10-19。目标：跑 `vio_online` 的完整链路，**原地绕圈**看漂移，并在 RViz 里实时/回放看轨迹。

---

## 0. 这次改了什么（都已生效）

| 文件 | 改动 |
|---|---|
| `config/gemini_1920x1080/kalibr_imucam_chain.yaml` | **换成新的最终外参**（`T_imu_cam` + `timeshift_cam_imu = −15.844 ms`）；旧文件备份在 `config/_backup_before_final_extrinsics/` |
| `config/gemini_1280x720/kalibr_imucam_chain.yaml` | 同上（外参同一套机械，内参仍用 1280 的） |
| `config/*/kalibr_imu_chain.yaml` | IMU 噪声换成实测值（gyro 6.08e−5、accel 4.26e−4、RW 7.14e−6 / 1.49e−5、200.3 Hz） |
| `src/pose_logger.cpp` | 每写一行就 flush（原文件备份为 `.bak`），否则 RViz 实时看会晚 ~8 s |
| `tools/tum_to_rviz.py` | 新：TUM → `/vio/path`、`/vio/pose`、`/vio/markers`、TF |
| `tools/drift_report.py` | 新：首尾漂移 / 最大偏离 / 累计转角 / 末段漂移速率 |
| `tools/session_to_dataset.py` | 新：采集目录（ns、`cam0/`、imu 列序 `ax..gx..`）→ VIO 数据集（µs、`gx..ax..`） |
| `rviz/vio.rviz` + `scripts/vio_rviz.sh` | RViz2 配置与一键启动 |
| `scripts/test_inplace_offline.sh` | 采集目录 → 数据集 → 跑 VIO → 漂移报告，一条命令 |

> 两个构建产物已重编（`cmake --build build -j`），`build/bin/vio_offline` 用的是新配置加载逻辑，无需再编。

---

## 1. 先说一个坏消息：**标定 session 不能当 VIO 测试数据**

我拿 session16（新安装、19.5 s、1920×1080）试跑：无论是
「新外参 + 新噪声」「新外参 + 旧噪声」还是「旧外参 + 旧噪声」，轨迹都在 17 s 内漂到 **46–63 m**。
而同一套程序跑 `datasets/session05`（你之前验证过的那段）仍然是 **217 帧 / 3.25 m**，和 README 完全一致。

原因不是程序：session 12–17 是**对着平面标定板快速转动**的数据，单目 VIO 在这种
「整屏一个平面、几乎只转不平移」的场景里本来就退化。**必须重新录一段有纹理的房间**（桌子、书架、墙上的画都行）。

---

## 2. 环境

| 项 | 状态 |
|---|---|
| RViz | 系统 ROS2 Humble 的 `rviz2`（`/opt/ros/humble`），**不要用 conda 的 Python** |
| 显示 | `DISPLAY=:1`，可正常开窗 |
| 相机 | Orbbec `2bc5:0511` 已连（live 走 Orbbec SDK；离线采集走 `/dev/video4`） |
| 飞控 | 录/跑之前插上，通常是 `/dev/ttyACM0` |
| ROS 日志 | 脚本自动设 `ROS_HOME=out/ros_home`（本机 `~/.ros` 在受限环境里不可写） |

---

## 3. 方案 A：实时跑（最接近“整体性能”）

**终端 1**（VIO + 相机 + 飞控 IMU）：

```bash
cd ~/calib/vio_online
./build/bin/vio_live \
  --serial /dev/ttyACM0 --baud 921600 \
  --config-dir config/gemini_1920x1080 \
  --no-send \
  --tum-out out/live_traj.tum
```

**终端 2**（RViz 实时看，边写边读）：

```bash
cd ~/calib/vio_online
./scripts/vio_rviz.sh out/live_traj.tum live
```

**测试动作**（建议 40–60 s）：

1. 静置 **5 s**（等 OpenVINS 初始化；终端 1 出现 `init=1` 后再动）；
2. **原地慢转 360°**（约 20–30 s，30°/s 左右，别甩）；
3. 回到起始朝向，**静置 5 s**；
4. 终端 1 按 `Ctrl-C`。

然后出漂移数字：

```bash
python3 tools/drift_report.py out/live_traj.tum --plot out/live_drift.png
```

> `--no-send` 只是不往飞控发 ODOMETRY，方便纯测试；要测完整链路就去掉它。
> 实时 IMU 默认请求 **200 Hz**（`src/mavlink_link.hpp:31` 的 `highres_imu_interval_us = 5000`），与标定/噪声模型的 200 Hz 对齐。
> 想退回 100 Hz 就改成 `10000`。注意单位是**微秒**：数值越大频率越低（20000 = 50 Hz，别看错）。
> 启动时看终端那行 `已请求 HIGHRES_IMU @ 200 Hz` 确认。

---

## 4. 方案 B：离线（更可复现，推荐用来对比改参）

### 4.1 录一段

编辑 `~/calib/camera_imu_calibration/capture/capture_config.py`：

```python
SESSION_NAME = "session18"   # ⚠️ 千万别用 session17，会覆盖标定数据
DURATION_S = 60.0            # 原地转圈够用；默认 20 s 偏短
```

```bash
conda activate vins
cd ~/calib/camera_imu_calibration
python capture/record_session.py
```

> 标定板体检那一步不需要板子：对着有纹理的房间转即可。检出率低只会让报告难看，
> **不影响图像/IMU 保存**。录完确认 `data/session18/{images.csv,imu.csv,cam0/}` 都在。

### 4.2 一条命令跑完

```bash
cd ~/calib/vio_online
./scripts/test_inplace_offline.sh session18
```

它做三件事：转数据集 → `vio_offline` → `drift_report`（并画 `out/session18_drift.png`）。

### 4.3 RViz 回放

```bash
./scripts/vio_rviz.sh out/session18_traj.tum replay --speed 3 --loop
```

---

## 5. RViz 里看什么

| 显示 | 内容 |
|---|---|
| Fixed Frame `world` | VIO 的 `LOCAL_FRD`（x 前、y 右、**z 向下**）；Grid 在水平面，不影响 |
| 绿色 Path | 累积轨迹（`/vio/path`） |
| 红色 Axes | 当前机体朝向（`/vio/pose`）；原地转圈时应“原地旋转” |
| 左上文字 | `drift ±x cm  yaw ±y°`（相对第一帧；`/vio/markers`） |
| TF | `world → base_link` |

觉得 z 向下别扭可以加 `--flu`（转成 z 向上显示，y 会镜像，只影响显示）。

---

## 6. 漂移怎么算、期望多少

`drift_report.py` 输出：

- **位置漂移** `|p_end − p_start|`
- **最大偏离**：过程中离起点最远多少
- **累计转角** Σ|Δyaw|：确认你真的转了 ~360°
- **首尾航向差**：转满一圈回到原朝向后应 ≈0
- **末段漂移速率**：判断是收敛了还是还在发散

### 6.1 ⚠️ 原地绕圈只能看 yaw，不能看位置

2026-10-19 实测（session16/17/20）：**纯旋转时，单目 VIO 的位置/速度是弱可观甚至不可观的**
（相机没有平移 → 没有视差 → 特征深度和相机位移都约束不住），位置基本靠 IMU 死推。
一个 0.1–0.5 m/s² 的加计残差，30 s 就能积分出几十米——RViz 里就是“自己飘到天上去了”。
这不是外参错，也不一定是算法坏，是这类运动的固有退化。

证据（session17，19.5 s）：VIO 的**姿态**变化 +1.1°，陀螺积分 +2.7°（差 1.6°，即旋转是对的）；
但位置跑到 46 m，而且是**从第 1 秒开始的平滑二次发散**，不是中途跟丢的跳变。

所以：

| 你测什么 | 该看的指标 | 怎么测 |
|---|---|---|
| 原地旋转 | **yaw 漂移**（位置忽略） | 静置 5 s → 慢转 360° → 停；画条地线/用转台，比较首尾朝向 |
| 位置漂移 | **闭环误差** | 走一个小矩形/圆圈（边长 0.5–1 m），回到起点，比较 `|p_end−p_start|` |

真正飞行时相机有平移，位置是可观的；纯旋转台架测试看着吓人，但不代表飞起来会这样。
位置精度请用**闭环轨迹**测（这也是 evo 等工具的标准做法）。

---

## 7. 漂移大时按这个顺序查

1. **初始化**：是否静置了 5 s？终端有没有 `init=1`？初始化时晃动会让初速度/零偏估错；
2. **图像**：曝光 200（20 ms）在转动时容易糊 —— 看 `p90` 亮度、降曝光到 150/100 或补光，转慢一点；
3. **时间戳**：live 走 Orbbec SDK 的到达时间，和标定用的 V4L2 到达时间可能差几 ms。加 `--online-timeoffset` 让 OpenVINS 在线估时移；
4. **外参**：确认用的是 `config/gemini_1920x1080`（新外参），不是 `config/session05_1920x1080`（旧安装）；
5. **IMU 频率/噪声**：live 默认已与标定对齐到 200 Hz；若改回 100 Hz 会牺牲积分精度。噪声用的是 200 Hz 静置实测。

---

## 8. 快速自检（不接硬件）

```bash
cd ~/calib/vio_online
# 已知能跑的 session05（旧安装 + 它的配置），应得 217 帧 / 3.25 m
./build/bin/vio_offline --dataset ~/calib/datasets/session05 \
  --config-dir config/session05_1920x1080 \
  --tum-out out/_selfcheck.tum >/dev/null
python3 tools/drift_report.py out/_selfcheck.tum | head -12

# RViz 桥（不起 rviz 也能测话题）
/usr/bin/python3 tools/tum_to_rviz.py --tum out/_selfcheck.tum --mode replay --speed 5 --loop &
source /opt/ros/humble/setup.bash
ROS_HOME=$PWD/out/ros_home ros2 topic hz /vio/pose
```
