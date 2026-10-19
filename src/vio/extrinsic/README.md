# extrinsic/ —— 相机–IMU 外参 + 时间偏移

两条路线，**结果以 Kalibr 为准**：

| 路线 | 入口 | 依赖 | 什么时候用 |
|---|---|---|---|
| **Kalibr**（基准） | `python kalibr/run_kalibr.py sessionNN` | Docker + `kalibr:latest`（8 GB） | 要交数值时 |
| 自写（无 Docker） | `python calibrate_extrinsics.py` | numpy / scipy / opencv | 机房没 Docker；或做交叉验证 |

同一段数据两条路线都跑，旋转差应 **< 1°**、平移差 **1–2 cm** 以内。
差得更多说明数据有问题（空洞 / 运动模糊 / 板面参数错 / 安装动过），
**不要在两个结果里挑一个看起来顺眼的**。

为什么不以自写版为准：同一安装的 5 段数据做重复性，Kalibr 两两差
**0.64° / 10.8 mm / 3.5 ms**，自写版 **6.05° / 22.3 mm / 14.4 ms**（旋转差 10 倍）。
自写版的问题不是算法错，而是它只挑最长的连续窗口（19.5 s 里 5–15 s）、
把 IMU 权重压低约 12 倍，等价于"以相机为主"的拟合。

---

## 目录

```
extrinsic/
├── calibrate_extrinsics.py    ① 自写路线：连续时间 B 样条联合优化（主入口）
├── extrinsics_config.py          自写路线的参数（Paths / Settings），现场只改这个
├── kalibr/
│   ├── run_kalibr.py          ② Kalibr 路线：打 bag → docker run → 收 yaml
│   ├── compare.py                两条路线逐 session 对比
│   └── configs/                  target.yaml（板子）/ imu.yaml（噪声）/ camchain.yaml（内参）
├── calib/                     自写路线的数学模型
│   ├── data.py                   读数据 / AprilGrid + 棋盘检测 / PnP
│   ├── spline.py                 均匀三次 B 样条
│   ├── solver.py                 初始化 + 联合优化 + 报告
│   ├── acquisition.py            采集说明解析（时钟域 / 滤波链 / IMU 实例声明）
│   └── windows.py                检出空洞诊断 + 自动挑连续窗口
├── capture/                   采集链（相机 + 飞控 IMU），详见 capture/README.md
├── configs/                   板子与相机的几何参数
├── results/final/             ★ 当前这套安装的最终外参（见下）
├── tests/                     48 个单元测试（unittest）
├── prepare_openvins.py        把标定结果导出成 OpenVINS 配置
├── remap_imu_clock.py         用测得的同步锚点把 IMU 时钟映射到图像时钟
└── verify_synthetic.py        合成数据端到端自检（真值是独立生成的）
```

---

## 一次完整流程

```bash
cd src/vio/extrinsic
conda activate vins                 # 必须是 vins：系统 python 的 OpenCV 4.5 检不出 AprilGrid

# 1) 取景/曝光体检（插相机，不用飞控）：录 10 秒，当场给判读
python capture/framing_check.py

# 2) 验板子：反解 tagSpacing / tag_rotation，报 PnP 残差（默认读上一步存的图）
python capture/check_board.py

# 3) 采集（相机 + 飞控通电）：先改 capture/capture_config.py 的 SESSION_NAME
python capture/record_session.py    # 20~30 s 多轴慢转，板子全程在画面里

# 4A) 标定 —— Kalibr（推荐）
python kalibr/run_kalibr.py session01

# 4B) 标定 —— 自写（无 Docker，或做交叉验证）
python capture/run_calibration.py --session session01   # 自动挑连续窗口再调 calibrate_extrinsics
#     也可以直接改 extrinsics_config.py 的 PATHS，然后 python calibrate_extrinsics.py

# 5) 两条路线对比（可选，但强烈建议）
python kalibr/compare.py session01

# 6) 导出给 OpenVINS
python prepare_openvins.py --calibration results/session01 \
    --template-config <你现有的 OpenVINS 配置目录> --output <输出目录>
```

采集动作的要求（决定能不能标出来）：

- 开头**真静止 5 s**（放桌上、手离开），再开始动；
- **三个轴都要转**，中位角速度 15~30 °/s，别超过 0.5 rad/s；
- 板子全程留在画面里，**别贴画面边**；每个 tag 边长 ≥ 45 px；
- 录 20~30 s，另录 1~2 段做重复性复核。

---

## 结果在哪

`results/final/` 是**当前这套安装**的最终外参，来自 Kalibr 在 5 段同安装数据
（s13–s17）上的平均：

| 文件 | 给谁用 |
|---|---|
| `extrinsics-final-camchain.yaml` | Kalibr 格式：`T_cam_imu` + `timeshift_cam_imu` |
| `camchain-openvins-final.yaml` | OpenVINS 格式：`T_imu_cam` + `timeshift_cam_imu` |
| `kalibr_imucam_chain.yaml` | 同上，文件名就是 OpenVINS 认的那个，直接替换 |
| `imu-final.yaml` | IMU 噪声（Allan 实测，连续时间） |
| `final_extrinsics.json` | 原始数值 + 逐段统计 |

约定（三个 yaml 一致）：

```
p_C = T_cam_imu · p_I        T_imu_cam = inverse(T_cam_imu)
t_imu = t_cam + timeshift_cam_imu
```

**⚠️ 换过支架/相机就得重标**。这套数值对应 2026-10 那次安装，之前任何一次
（相机相对 IMU 转了 180° 的那几轮）都不能混用。

---

## 精度

| 量 | 能做到 | 依据 |
|---|---|---|
| 旋转 | **±1°** | 5 段两两差 0.64°；IMU 噪声放大 10 倍只动 0.25° |
| 平移 | **±3~4 cm**（只有几厘米，别写成安装尺寸） | 5 段两两差 10.8 mm，但**换一套 IMU 噪声就动 37 mm** |
| 时间偏移 | **每次录制现场标** | 跨 session 从 −27 ms 到 +9 ms 都出现过 |

时间偏移不能跨录制搬：相机时间戳是"宿主收到图"的时刻，比曝光起点晚约 28 ms，
读数还受 USB 传输影响。要么每段自己标，要么 OpenVINS 开 online 估计。

至于 Kalibr 报告里那个 **10–12 px 的重投影残差**：它不是靶标几何错，也不是权重问题
（IMU 噪声 ×10 后 11.40 → 11.17 px，基本不变）。它是"整段 19.5 s（含快速转动帧）
+ 样条/IMU 约束"下的联合残差；自写版报的 1.5–2 px 是"挑出来的慢速窗口 + 以相机为主"
的残差，**两者口径不同**。它主要削弱平移，不改变旋转和时偏。

---

## 改代码前先看这几条

| 坑 | 现象 | 原因 |
|---|---|---|
| `tag_rotation_deg` | 检测正常但 PnP 残差 100 px 以上 | Kalibr 的 `kalibr_create_target_pdf` 画 tag 时写死 180° 旋转，所以 Kalibr PDF 打的板子必须填 `180` |
| `tagSpacing` | PnP 残差 10 px 量级 | 它是**白隙 ÷ tagSize**，不是设计值；板子是别家生成器画的就完全不一样（本板实测 0.56） |
| aruco 阈值窗口 | 每帧只认出 0~1 个 tag | `calib/data.py` 已改成 `min 5 / step 4`（默认 `3,13,23` 对低对比度打印板太粗），别改回去 |
| `camera_rate_hz` | 初始化发散、跑几分钟后失败 | 必须 ≤ 相机帧率的一半；30 fps 采集配 `camera_rate_hz=60` 会让样条结点比图像还密 |
| 采集帧率 11 fps | 帧间隔 700 ms，标定报"大空洞" | 预览 + 每 0.5 s 一次 1080p 棋盘检测；`capture_config.py` 的 `SHOW_PREVIEW` 必须 False |
| 画面太暗 | 检出 0 / 白格 p90 < 45 | 先开灯或补光。房间暗时**降曝光只会更黑**，不是所有情况都该降 |
| 输出目录非空 | 拒绝写入 | `calibrate_extrinsics.py` 不覆盖旧结果，换目录名 |

自检（不需要硬件）：

```bash
python -m unittest discover -s tests          # 48 项
python verify_synthetic.py                    # 合成数据端到端，真值独立生成
python capture/selftest_capture.py            # 采集链（假飞控 + 假串口）
```
