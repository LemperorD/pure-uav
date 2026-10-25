# 把你的标定结果喂给 OpenVINS —— 手把手

从「采集数据」到「OpenVINS 出一条轨迹」，一共三步。

---

## 0. 为什么是 ROS 1（不是 ROS 2）

**结论：ROS 1 Noetic + serial 模式。**

理由是两条硬证据：

```
ROS2 版  ~/vio_ws/install/ov_msckf/lib/ov_msckf/
   run_simulation   run_subscribe_msckf   test_sim_meas   test_sim_repeat
   ↑ 没有 serial

ROS1 版  ~/vio_ws_ros1/devel/lib/ov_msckf/
   ros1_serial_msckf   ← ★ 只有 ROS1 有
```

**serial = 节点直接读 bag 文件**，绕开 ROS 消息传输 / QoS 队列 / 线程调度。
官方评估 EuRoC 用的就是它（`ov_msckf/scripts/run_ros_eth.sh` 里写着
`subscribe=live pub, serial=read from bag (fast)`）。

而 subscribe 模式（ROS 2）在 `~/vio_exp/run_openvins_serial.sh`
的注释里已经记录过实测发散：ROS2 版 IMU 订阅队列深度只有 5，
bag 每 50ms 突发投递 10 条正好撑爆，60 秒后发散。

**推论：bag 必须是 ROS 1 格式。** `make_rosbag.py` 打的就是 ROS1 bag。

---

## 1. 起环境（每次开新终端都要做）

**ROS 1 装在 `vins` 这个 conda 环境里**，不在 `/opt/ros`（那里只有 humble）。
所以必须**先激活 conda，再 source 工作区** —— 顺序反了会报
`_setup_util.py: return code 127`。

```bash
source ~/miniforge3/etc/profile.d/conda.sh
conda activate vins
source ~/vio_ws_ros1/devel/setup.bash
export LD_LIBRARY_PATH="$CONDA_PREFIX/lib:${LD_LIBRARY_PATH:-}"
```

第 4 行是必须的：conda 的 libstdc++（CXXABI 1.3.17）要优先于系统的（1.3.13），
否则 OpenVINS 起来就崩。

**验证（逐条敲）：**

```bash
rosversion -d
```
输出应该是 `noetic`。

```bash
rospack find ov_msckf
```
输出应该是 `/home/kxy/vio_ws_ros1/src/open_vins/ov_msckf`。

---

## 2. 第一步：把 session 打成 ROS 1 bag

### 方法 A：PyCharm 里点运行（推荐）

打开 `capture/make_rosbag.py`，点右上角绿色三角。它会自动挑**最新的 session**。

### 方法 B：终端

```bash
cd ~/calib/camera_imu_calibration
python capture/make_rosbag.py
```

指定某个 session：

```bash
python capture/make_rosbag.py --session session05
```

看有哪些 session：

```bash
python capture/make_rosbag.py --list
```

### 输出在哪

默认写到 `/home/kxy/calib/rosbags/<session>.bag`。

### 你应该看到

```
✅ 写完：479 张图 + 4715 条 IMU，1000 MB
   回读校验：{'/cam0/image_raw': 479, '/imu0': 4715}
```

**两个数字必须和你的数据对得上。** 对不上说明有图读不出来（脚本会打印警告）。

### 自己再验一遍（可选，但很值）

```bash
rosbag info ~/calib/rosbags/session05.bag
```

应该看到 `/cam0/image_raw` 和 `/imu0` 两个话题，以及官方 MD5：

```
types:       sensor_msgs/Image [060021388200f6f0f447d0fcd9c64743]
             sensor_msgs/Imu   [6a62c6daae103f4ff57a132d6f95cec2]
```

---

## 3. 第二步：从标定结果生成 OpenVINS 配置

用标定项目自带的 `prepare_openvins.py`。它要求标定报告的
`status` 是 `passed_residual_and_excitation_checks`，否则拒绝导出
（这是防呆，别绕过）。

```bash
cd ~/calib/camera_imu_calibration
python prepare_openvins.py \
    --calibration results/session05_window \
    --template-config ~/vio_ws/final_config \
    --output ~/calib/ov_config_session05 \
    --imu-topic /imu0 \
    --cam-topic /cam0/image_raw
```

（`session05_window` 是标定通过的那次；`--calibration` 要指向**通过的**目录。）

生成 4 个文件：

| 文件 | 内容 |
|---|---|
| `estimator_config.yaml` | 滤波器参数（从模板复制） |
| `kalibr_imucam_chain.yaml` | **你的外参 T_imu_cam + timeshift_cam_imu** |
| `kalibr_imu_chain.yaml` | IMU 噪声（从模板复制，**不是你的 CUAV**） |
| `PROVENANCE.json` | 来源记录 + 警告 |

> ⚠️ `PROVENANCE.json` 里有一句要读懂：
> `"IMU noise copied from supplied template. A EuRoC/other sensor template
> does NOT establish CUAV IMU noise parameters."`
> **IMU 噪声是从 EuRoC 模板抄的，不是你的 CUAV 7-Nano 的。**
> 要真做评估，得先标 IMU 的 Allan 方差。

---

## 4. 第三步：跑 OpenVINS

**`config_path` 和 `bag` 都传绝对路径，所以不用改 `~/vio_ws_ros1` 里任何东西。**

```bash
roslaunch ov_msckf serial.launch \
    config_path:=/home/kxy/calib/ov_config_session05/estimator_config.yaml \
    bag:=/home/kxy/calib/rosbags/session05.bag \
    bag_start:=0 \
    max_cameras:=1 \
    use_stereo:=false \
    dosave:=true \
    path_est:=/home/kxy/calib/vio_out/session05.txt
```

| 参数 | 含义 |
|---|---|
| `config_path` | 指向你生成的 `estimator_config.yaml`（它内部用相对路径找另外两个 yaml） |
| `bag` | 上一步打的 bag |
| `bag_start` | 从第几秒开始读。**0 = 从头**（你的 bag 就是这个 session 的全部） |
| `max_cameras` / `use_stereo` | 单目，必须和配置一致 |
| `dosave` / `path_est` | 把估计轨迹写到文件 |

`roslaunch` 如果发现没有 master 会自己起一个 roscore，不用单独开。

**要停就按 `Ctrl-C`。**

---

## 5. 怎么读结果

### 屏幕上要看这些

```
[INIT]: ...         初始化。要看到它成功，不是一直重试
number of features  特征数。掉到个位数说明跟丢了
```

**初始化失败是常见结果**，别慌 —— 见下面。

### 轨迹文件

```
/home/kxy/calib/vio_out/session05.txt
```

格式（TUM 风格）：`timestamp tx ty tz qx qy qz qw`

**注意：这是 VIO 自己估的轨迹，没有真值可比。** 你这个 session 是
拿着相机对着一块棋盘格慢慢转 —— 它验证的是"整条链路通不通"，
不是"精度好不好"。

### 快速看一眼轨迹有没有形状

```bash
python3 -c "
import numpy as np
d = np.loadtxt('/home/kxy/calib/vio_out/session05.txt')
p = d[:, 1:4]
print('采样点', len(p))
print('起点', p[0].round(3))
print('终点', p[-1].round(3))
print('走了多远', np.linalg.norm(np.diff(p, axis=0), axis=1).sum().round(3), 'm')
print('包围盒', (p.max(0) - p.min(0)).round(3))
"
```

---

## 6. 常见报错

| 报错 | 原因 | 怎么办 |
|---|---|---|
| `_setup_util.py: return code 127` | 没先 `conda activate vins` | 按第 1 节顺序重来 |
| `Failed to contact master at [localhost:11311]` | 跑了 `roslaunch` 但 roscore 没起 | 正常 `roslaunch` 会自己起；起不来就用 `~/vio_exp/run_openvins_serial.sh` 里那段 `setsid roscore` |
| `unable to open the configuration file!` | `config_path` 路径写错 | 用绝对路径，`ls` 确认文件存在 |
| `[SERIAL]: unable to parse all parameters` | 配置目录不完整 | `ov_config_xxx/` 里应该有 3 个 yaml |
| 起来就崩 / `CXXABI` 报错 | 忘了 `export LD_LIBRARY_PATH` | 补上第 1 节第 4 行 |
| 一直 `[INIT]` 不成功 | 运动不够 / 特征太少 | 见下 |
| 跑完打印 `process has died ... exit code -11` | 收尾时段错误（SIGSEGV） | **无害**，见第 8 节；轨迹已经写完了 |

### 初始化一直不成功怎么办

OpenVINS 动态初始化需要（`estimator_config.yaml` 里能看到）：

- `init_window_time: 2.0` —— 攒够 2 秒
- `init_dyn_min_deg: 10.0` —— **姿态变化至少 10°**
- `init_imu_thresh: 1.5` —— 加速度方差要有一次"顿挫"

你的采集运动很慢（中位 ~7°/s），2 秒才转 14° —— 勉强够。
如果真的起不来，试着把 `bag_start` 往后挪到运动明显的那一段：

```bash
bag_start:=5.0
```

---

## 7. 一句话总结这条链

```
data/<session>/cam0/*.png  ─┐
data/<session>/imu.csv     ─┴─→ make_rosbag.py ──→ ~/calib/rosbags/<session>.bag
                                                              │
results/<session>_window/  ──→ prepare_openvins.py ──→ ~/calib/ov_config_<session>/
   (status: passed)                                           │
                                                              ▼
                                    roslaunch ov_msckf serial.launch
                                        config_path:=.../estimator_config.yaml
                                        bag:=.../<session>.bag
```

---

## 8. 第一次实跑的结果（2026-10-16，session05）

**环境**：ROS 1 Noetic（`vins` conda 环境）+ `ros1_serial_msckf`
**配置**：`~/calib/ov_config_session05/`（外参来自 `results/session05_window/`）
**bag**：`~/calib/rosbags/session05.bag`（479 图 + 4715 IMU）

### 初始化成功了

```
[init]: disparity is 18.852,23.171 (10.00 thresh)
[init-d]: gravity in I0 was 7.371,-0.118,-6.472 and |g| = 9.8100
[init]: successful initialization in 0.0312 seconds
[init]: orientation = -0.8785, 0.0028, -0.4773, 0.0180
[init]: bias gyro   = -0.0060, -0.0034, 0.0023
```

### 结果

```
轨迹文件    ~/calib/vio_out/session05.txt
位姿        217 个 / 17.13 s / 12.7 Hz
图像覆盖率  99.8%          ← 全部图像都处理完了
处理耗时    3.37 s          ← serial 模式直接读 bag，比实时快 ~6 倍
```

### 交叉验证（这是最有价值的一项）

把 VIO 估出来的姿态速率，和**标定时陀螺直测的速率**画在一起：

| | 中位 |
|---|---|
| 陀螺直测（200 Hz） | **6.2 °/s** |
| VIO 从图像推的 | **6.9 °/s** |

**两条完全独立的通路对上了，而且尖峰一一对应。** 这是外参 + 时移正确的最直接证据。

### 但轨迹在 12 秒之后开始跳

```
相邻位姿位移：中位 8.2 mm，最大 188.1 mm（等效 2.74 m/s）

 7~10s    3.0 mm/步   ← 稳
12~13s   27.3 mm/步   ← 劣化
14~16s   19~40 mm/步  ← 明显劣化
```

**和标定时发现的那两个空洞是同一段数据**（5.13~5.53s、16.06~17.76s 盲区）。
那段图像质量差 / 运动快 → 特征跟丢 → 位姿跳。
**不是软件问题，是那段录制的质量问题。**

### 收尾的 segfault 是无害的

```
process has died [pid 15003, exit code -11, cmd .../ros1_serial_msckf]
```

`-11` = SIGSEGV。但它发生在打印完 `TIME: 3.368 seconds` **之后**，
而轨迹终点只比最后一帧早 0.05 秒 —— **全部数据都处理完了，轨迹文件完整**。

判断方法（不用猜，直接量）：

```bash
python3 - <<'PY'
import numpy as np
tr  = np.loadtxt("/home/kxy/calib/vio_out/session05.txt", comments="#")
img = np.loadtxt("/home/kxy/calib/camera_imu_calibration/data/session05/images.csv",
                 delimiter=",", skiprows=1, usecols=0, dtype=np.int64)/1e9
print(f"图像覆盖率 {(tr[-1,0]-img[0])/(img[-1]-img[0])*100:.1f}%")
PY
```

**覆盖率接近 100% 就说明没丢数据**，segfault 只是退出路径的问题。

### 一句话总结

**链路通了、外参和时间偏移是对的、轨迹形状合理。**
但这个 session 是「拿着相机对着一块棋盘格慢慢转」——
它验证的是**流程**，不是**精度**（没有真值可比）。

要评估精度，得换一个真正的 VIO 场景：有丰富纹理、有真实平移、有回环，
并且有真值轨迹（比如 EuRoC，或用动捕/全站仪）。
