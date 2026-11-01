# vendor/ —— 现成脚本的本地副本

这三个文件不是新写的，是从你已有的、已经跑过好几轮的脚本复制过来的。
复制的原因：让 `camera_imu_calibration/` 这个 PyCharm 工程自包含，
不依赖 `~/Documents/Codex/...` 那个临时目录（那个目录哪天被清理掉，采集就断了）。

| 文件 | 来源 | 复制时的 SHA256 | 现在还是原版吗 |
|---|---|---|---|
| `capture_calibration.py` | `/home/kxy/Documents/Codex/2026-10-13/vio-1-https-github-com-ethz-2/outputs/capture_calibration.py` | `6116dc0e5939be63b755f5a5dee08537964df689f01948307fb0a22296348151` | ❌ **已改一处，见下** |
| `calibration_config.py` | 同上目录 | `24c541221f646ae38c179fb49878b8a0506d1408c825a4e6516985b11fe0ac45` | ✅ 原版 |
| `mavlink_min.py` | `/home/kxy/calib/camera/imu/src/mavlink_min.py` | `f7009882022682d5c78b8b6c275bea79389e7b4fae22f50bf8a150610feff24a` | ✅ 原版 |

复制日期：2026-10-15。复制时逐个 `sha256sum` 比对过，与源文件逐位一致。

---

## ⚠️ 本地修改记录

### 2026-10-18 · `capture_calibration.py` 第 607 行附近 —— **每 0.5 秒的预览检测**在录像时也要关

**症状**：外参采集 19.15 秒只存下 166 张（**8.6 fps**），最大帧间隔 **746 ms**，
37 处 >350 ms。标定侧直接判「没有一段连续 ≥5 秒 ≥35 帧的窗口」，整份数据作废。
但同一批图里检出 159/166、PnP 中位 1.65 px —— 数据质量本身没问题，是时序被打碎了。

**原因**：上一次修改只关了「保存时」那次检测，**循环开头这次每 0.5 秒的检测漏了**：

```python
            if args.cols and (frames == 1 or now - last_detect_time >= 0.5):
                corners = detect_board(gray, args.cols, args.rows)   # 1080p 上 470~530 ms
```

实测 `detect_board(6x6)` 在 1080p 上 **472~530 ms**、720p 上 264~279 ms。
每 0.5 秒堵一次 → 5 帧（~100 ms）快跑 + 400 ms 卡住，循环被拖成 11 fps，
并在驱动层留下 330~400 ms 的空洞。

**改法**：同样加 `not args.no_preview`（它只给预览窗绿点和 3×3 覆盖度用）。

**验证**（同一台机器、同一路径、1080p30、`--no-preview`）：

| | 改前 | 改后 |
|---|---|---|
| 实测帧率 | 11.5 fps | **30.0 fps**（吞吐差 0.0%） |
| 保存张数 / 8 s | 84 | **224** |
| 最大帧间隔 | 736 ms | **51 ms** |
| `v4l2_sequence` 跳变 | 大量 10~12 | **全部为 1（零丢帧）** |

### 2026-10-15 · `capture_calibration.py` 第 615~621 行 —— 连续录像时不再逐帧做棋盘检测

**症状**：外参采集时 9.23 秒只存下 **25 张**（2.6 fps），而 manifest 里
驱动的 `v4l2_sequence` 已经到 254 —— 相机在以 ~27.5 fps 正常出图，是保存循环太慢。

**原因**：`record_session.py` 传 `--auto-interval 0.0167`（每帧都存），
于是这段「保存时重新检测」对**每一帧**都跑一次完整棋盘检测：

```python
            if key == 32 or automatic:
                if args.cols:                       # ← 每帧一次，1080p 上 180~330 ms
                    corners = detect_board(gray, args.cols, args.rows)
```

这个逻辑是**内参标定**时代的设计（保证 manifest 里 `board_detected` 对应被存的那一帧，
顺带更新 3×3 覆盖度）。外参是连续录像，角点由离线标定程序自己检测，这里纯属浪费。

**改法**（一行）：`if args.cols:` → `if args.cols and not args.no_preview:`

开着预览时行为完全不变；`--no-preview` 下循环从 ~370 ms/帧降到 ~55 ms/帧。

**效果**：预期 1080p 从 2.6 fps 回到 **15~20 fps**。（2026-10-15 真机复测：**20.4 fps**，
9.33 秒读到 191 帧、存下 154 张。对比修之前的 25 张 / 9.23 秒。）

### 2026-10-15 · `capture_calibration.py` `detect_board()` —— 修复一个静默失效的 flags

**症状**：`record_session.py` 的控制台**每一张都打印 `board=no`**，manifest 里
`board_detected` 是 **0/154**；但同一批图用离线标定程序的检测器跑，检出率是 **82%**。

**原因**：第 424 行把 `flags` 当成了位置参数：

```python
# 错的
found, corners = cv2.findChessboardCornersSB(gray, pattern, cv2.CALIB_CB_NORMALIZE_IMAGE)
```

Python 签名是 `retval, corners = f(image, patternSize[, corners[, flags]])` ——
**第 3 个位置参数是输出数组 `corners`，不是 `flags`**。OpenCV 不报错、静默接受，
`flags` 用默认值 0，于是丢掉了 `NORMALIZE_IMAGE`（直方图归一化）。
暗光/低对比度下图全部检不出，而离线标定程序写的是 `flags=...`（关键字，正确）。

**改法**（一行）：

```python
found, corners = cv2.findChessboardCornersSB(
    gray, pattern, flags=cv2.CALIB_CB_NORMALIZE_IMAGE)
```

**实测**（同一批 12 张真实采集图）：修复前 **0/12**，修复后 **9/12**。

**影响范围**：只影响预览窗的绿点、manifest 里的 `board_detected`、3×3 覆盖度提示。
**离线标定结果不受影响**（它用 `calib/data.py` 自己的检测器，一直是正确的）。
但这个 bug 会让人误判"板子有问题"，2026-10-15 就因此多查了一轮。

### 想拿原版回来

```bash
cp /home/kxy/Documents/Codex/2026-10-13/vio-1-https-github-com-ethz-2/outputs/capture_calibration.py \
   /home/kxy/calib/camera_imu_calibration/capture/vendor/capture_calibration.py
```
（那样帧率会退回 2.6 fps —— 除非改回 `SHOW_PREVIEW = True` 并且不在乎帧率。）

## 为什么要复制 `capture_calibration.py`

它做了 687 行很扎实的事，重写没有任何好处：

- 校验 USB 身份 `2bc5:0511`，**拒绝误用笔记本内置摄像头**
- 锁定曝光 / 白平衡 / 增益 / 锐化（标定期间必须固定，否则角点会漂）
- V4L2 直采（`mmap` + `VIDIOC_DQBUF`），不是 `cv2.VideoCapture` 那层封装
- 断线自动重连，重连后重新锁定控件
- 文件名就是宿主墙钟纳秒时间戳，另外写一份 `capture_manifest.json`
- 退出时恢复相机原始控件

`capture/` 里新写的代码只做三件事：录 IMU、把 manifest 转成 `images.csv`、把两者编排起来。

### 2026-10-20 · `capture_calibration.py` / `calibration_config.py` —— 曝光"锁不死就别跑"

**症状**：同一段代码，第二次跑画面莫名变暗；`capture_manifest.json` 里
`original_controls` 在 `{exposure_auto:3, auto_white_balance:1}` 和
`{exposure_auto:1, auto_white_balance:0}` 之间摇摆（session05→06 是现成例子）。

**根因**：`S_CTRL` 之后立刻 `G_CTRL` 比对，读到的是 **uvcvideo 的驱动缓存**，
不是固件。所以"控件已锁"这句打印证明不了任何事。加上退出恢复也不可靠，
下一次打开相机时的初始状态取决于上一次是谁、怎么退出的。

**改法**（四条）：

1. **步进验证代替读回验证**（新函数 `verify_manual_exposure`）：目标曝光测线性
   亮度 L1 → 曝光调成 k 倍测 L2 → 期望 `L2/L1 ≈ k`；比值 ≈1 就是 AE 还在掌权。
   必须在 `(v/255)**2.4` 的线性域比：实测 2 倍曝光在原始灰度域只有 **1.330**
   （正好是 `2**(1/2.4)`），线性域 **1.983** ≈ 理论 2.00。
   不过就整套重试 3 次，仍不过 → 抛异常，拒绝采集（`--skip-exposure-check` 可绕过）。
2. **钉死所有影响亮度的控件**：新增 `backlight_compensation=0`（默认是 1，
   AE 一旦漏网它会再改一次测光）。`gamma` 不猜数值——读出当前值再显式写回同一个值。
3. **不支持的控件不再一刀切**：必选项（曝光/增益/白平衡）缺失 → 拒绝启动；
   可选项（亮度/对比度/锐化/逆光补偿/工频/gamma）缺失 → 只记录进 manifest。
4. **静默截断不再记成 completed**：实测有一次请求 60 秒只交付 **39.84 秒**
   （`v4l2_sequence` 连续、无重连），旧代码照样写 `termination: completed`。
   现在交付时长 < 请求的 90% 就写 `failed: 只交付 X s / 请求 Y s`，
   `record_session.py` 据此直接判这份数据不可用。

另外加了 `@记录开始@` 哨兵行：锁定 + 验证要花几秒，倒计时要对齐到真正开始存图
的那一刻，否则"还剩 25s"其实只剩 21s，闭环 VIO 的时长窗口整个错位。

**验证**：`tools/selftest_exposure_verify.py` 用假相机跑 4 个用例
（手动 / AE 接管 / 太暗 / 过曝），全部判对；真机 60 秒亮度时间线
min 33.7 / 中位 33.9 / max 34.0，波动 **0.7%**。

### 2026-10-20（补）· 进场先测光，再锁定 —— 固定曝光/增益这个思路本身是错的

**症状**：同一间教室，下午 4 点录出来灰度 49.3（VIO 跑通），晚上 7 点只有 20.4
（VIO 发散）。用户质疑：手机在同一个位置拍得很亮，RoboMaster 现场也只有室内灯，
不可能指望大太阳。

**根因**：`EXPOSURE=100` / `GAIN=30` 是照某一次**有日光**的 session 写死的。
更关键的是**增益 30 几乎等于没加增益** —— session25(gain 10) 灰度 26.6、
session26(gain 30) 灰度 30.5，3 倍增益只换来 15% 亮度，说明这个控件的单位是
dB 一类的东西（30 ≈ +3 dB ≈ 1.4×），而相机通常支持到 +20~30 dB，
**整整一个数量级的增益从来没被用过**。

手机能拍不是因为它更灵敏，是因为它**先测光再曝光**。

**改法**：新增 `auto_expose()`，进场先测光、选定后整段锁死。
可复现靠的是"采集期间不动"，不是"永远用 2018 年那个数字"。

- `VIDIOC_QUERYCTRL` 问出曝光/增益的**真实范围**，不再猜；
- 曝光先顶到「模糊预算」和「帧周期 0.9 倍」的较小值（曝光只加模糊、不增噪）；
- 还不够就按范围的百分比爬增益梯子（增益只增噪、不增模糊）；
- 光很足时反过来收缩曝光，按目标亮度找**最小的模糊**；
- 两个都到顶还低于 `AUTO_EXPOSE_MIN_MEAN`（默认 45）→ 拒绝采集，
  报出"最好只能到多少"，不再让人白录一轮加白跑一次 VIO。

配置项：`AUTO_EXPOSE` / `AUTO_EXPOSE_TARGET`(100) / `AUTO_EXPOSE_MIN_MEAN`(45) /
`AUTO_EXPOSE_MAX_EXPOSURE_MS`(15，走 0.65 m/s 时约 21 px 模糊) / `ALLOW_DARK`。
梯子每一步、选定的值、增益范围全部写进 manifest 的 `auto_exposure` 字段。

**验证**：`tools/selftest_exposure_verify.py` 用假相机跑 3 组
（正常室内光 → 选定 15 ms/gain 76；很暗 → 爬到增益上限只到 39 → 正确拒绝；
光很足 → 收缩到 3.3 ms 命中目标），全部判对。

注意：`tools/exposure_step_test.py` 必须加 `--no-auto-expose`，
它的全部意义就是"我指定两个曝光比亮度"，不能让测光改掉。

### 2026-10-21 · `capture_calibration.py` / `../capture_config.py` —— 测光的判据从「亮度」改成「跟踪得住」

**症状**：`session28/29/30` 三段全部发散（尺度偏 5 倍）。上一版加的进场测光**确实跑了**，
`capture_manifest.json` 的 `auto_exposure.ladder` 完整记录了它做的每一个决策：

```
曝光 180（18.0 ms）增益 0   → 灰度 19.0
曝光 180（18.0 ms）增益 100 → 灰度 49.5     ← 它选了这一行
目标 100，下限门槛 45 → 49.5 > 45，判定"可用"，开录
```

**根因**：三个洞，缺一不可。

1. **测的时候是静止的。** 运动模糊只有转起来才有，静态测光在原理上看不见这次失败的原因。
   18 ms 曝光在静止画面上看起来完美无瑕。
2. **"模糊预算"形同虚设。** `AUTO_EXPOSE_MAX_EXPOSURE_MS = 18.0`，而帧周期上限 30 ms、
   相机上限 500 ms —— 18 ms 就是它允许的全部，等于没有约束。设计注释里写了
   "曝光不增噪，只增模糊"，但没把"模糊"量化成像素。
3. **验收门槛只看亮度。** 目标 100、下限 45，实测 49.5 —— 只有目标一半的亮度也算过关。

结果：它做了一个局部完全合理的决策（不够亮就拉曝光、拉增益），却在全局毁了数据。
实测后果：模糊中位 4.4 px、p90 14.4 px；KLT 两帧存活率 67%（>20 px 时 38%）；
成熟特征 62 个/帧（session24 是 217）；三角化失败 55%、MSCKF 更新率 26%；
速度以 +120 mm/s² 空积分 20 s，位置偏 5 倍。

**改动**：

- 曝光上限 = `blur_budget_px / (f · ω)` 反推，不再写死。默认 3 px @ 100°/s、f=1367.5
  → 上限 **1.3 ms**（原来是 18 ms）。
- 增益上限 `AUTO_EXPOSE_MAX_GAIN = 40`（原来顶到 100；实测噪声 σ≥8 时存活率从 78% 掉到 43%）。
- 曝光和增益**都顶满**仍达不到目标灰度 → 返回 `insufficient_light` 拒绝采集，
  并打印"加灯，或把 `AUTO_EXPOSE_EXPECTED_RATE_DPS` 调成真实角速度"。
- 新增 `--auto-expose-blur-budget-px` / `--auto-expose-expected-rate-dps` /
  `--focal-px` / `--auto-expose-max-gain` / `--allow-blur`。
- 静态检查看不见模糊，这是原理性的：配套 `tools/track_check.py --shake`
  让你按飞行速率转 6 秒，直接量转动中的 KLT 存活率。

**重新跑一遍会发生什么**：同样的场地会被**拒绝采集**（光不够）。这是正确结果 ——
比录一段必然发散的数据好。

## 改了源文件怎么办

源文件如果以后有 bug 修复，需要重新复制并更新上表的哈希。检查是否漂移：

```bash
cd /home/kxy/calib/camera_imu_calibration/capture/vendor && sha256sum -c <<'EOF'
d6146081077007fb929618e75990653da8d5c8c15d14a126f45c65f0af203936  capture_calibration.py
a88920c94d3b55f729bd0eef4943c5bec3e1e6a10ee19ce86abba87d10bc1732  calibration_config.py
f7009882022682d5c78b8b6c275bea79389e7b4fae22f50bf8a150610feff24a  mavlink_min.py
EOF
```

## 注意

`calibration_config.py` 是 `capture_calibration.py` 的默认参数文件，
里面的 `DEVICE` / `WIDTH` / `HEIGHT` / `SESSION_DIR` 等**不会生效** ——
`record_session.py` 会用 `--device/--output/--width/--height/...` 全部显式覆盖，
真正的参数在 `../capture_config.py`。
