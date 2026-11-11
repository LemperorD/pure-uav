# VIO 深度测距接口要求

- 日期：2026-11-11
- 状态：待 VIO 负责人确认
- 需求方：制导 / 任务侧
- 目标分支：`dev/main-loop`（当前为 `f803498`，需先同步最新 mission main）

---

## 1. 目的

任务状态机进入 `Strike` 前需要目标距离 `range_m`。当前分工：

- 相机深度由 `camera_server` 采集，深度图只发布在 VIO 共享内存通道；
- detect 只消费彩色通道，输出目标像素框；
- VIO 在 main 进程内，已经消费 VIO 通道的灰度 + 深度；
- **深度解算由 VIO 负责人实现**，制导/任务侧只调用结果。

本文只定义 VIO 需要提供的**深度测距接口与语义**，不定义任务状态机、不定义制导律。

---

## 2. 范围与非目标

### 2.1 本接口负责

- 给定彩色图像上的目标像素和采样时刻，返回该目标点的距离；
- 处理深度帧匹配、彩色→深度投影、深度有效性检查；
- 返回距离的有效性、时间戳、质量信息和失败原因；
- 保证 50 Hz 主循环调用时不阻塞、不死锁。

### 2.2 本接口不负责

- 目标检测、分类、跟踪；
- 任务状态机、Strike 判定；
- 制导指令生成或发送；
- 修改 `MissionTargetPacket` 协议（任务侧可以自己填 range 字段）；
- 修改 `camera_server` 的通道布局（除非另行协商）。

---

## 3. 当前数据链路（供 VIO 侧确认）

```text
Orbbec 彩色 + 深度
  └─ camera_server
       ├─ /pure_uav_camera_vio   ：gray(CV_8UC1) + depth_mm(CV_16UC1)
       └─ /pure_uav_camera_color ：彩色编码字节（MJPG/RGB/YUYV）

detect_rt（独立进程）
  └─ 读彩色通道 → 目标框 u_px/v_px/w_px/h_px/conf/class_id/captured_ns
       └─ socketpair → main 的 MissionService

main
  ├─ VioRuntime 消费 VIO 通道的 gray+depth
  └─ MissionService 消费目标包，调用 VIO 深度测距
```

关键事实：

- 深度图**只在 VIO 通道**，彩色通道没有深度像素；
- `StreamMeta` 里的 `fx/fy/cx/cy` 是**深度相机内参**，不是彩色内参；
- `StreamMeta` 同时给出 `R_c2d`、`t_mm`（彩色→深度）和 `depth_width/depth_height`；
- 彩色内参/畸变来自 `config/guidance-camera.yaml`；
- 当前深度是**原始深度相机坐标系**，不要假设已经对齐到彩色。

---

## 4. 距离定义

### 4.1 必须明确的量

`range_m` 定义为：

> 彩色相机光心到目标框中心对应三维点的**欧氏距离**，单位米。

如果 VIO 侧只能先给出深度相机 z 向距离，也可以，但必须在字段中明确标识：

- `range_kind = EUCLIDEAN`：欧氏距离（默认要求）；
- `range_kind = DEPTH_Z`：深度相机 z 向距离。

**不能在不同实现之间混用**，否则制导的 t_go / 冻结判断会不一致。

### 4.2 目标点

- 默认取目标框中心 `(u_px, v_px)`；
- 在深度图上取小窗口（建议 5×5 或 7×7）内有效深度的中值；
- 不允许只取单个像素；
- 如果中心落在无效深度/背景上，允许在窗口内做鲁棒筛选，但必须报告有效像素数。

### 4.3 距离范围

- 默认有效范围：`0.3 m ~ 10.0 m`；
- 超出范围返回无效，不要 clamp 成边界值；
- `0`、`NaN`、`Inf`、负值一律无效。

---

## 5. 时间语义

### 5.1 输入时间

调用方传入的是目标彩色帧的采集时刻：

```text
target_captured_ns：CLOCK_MONOTONIC，纳秒
```

来源是 detect 的 `TargetSnapshot.captured_host_ns` / 目标包 `captured_ns`。

### 5.2 深度帧匹配

- 彩色和 VIO 通道共用 `FrameMeta.seq`；
- 彩色通道 10 Hz，VIO 通道 30 Hz；
- VIO 侧应选择与 `target_captured_ns` 最近的深度帧；
- 最大匹配时间差必须可配置，建议默认 `≤ 50 ms`；
- 超过最大时间差返回 `kStale` 或 `kNoDepth`，不能拿旧深度配新目标。

### 5.3 输出时间

返回结果必须带：

```text
depth_captured_ns：实际深度帧的 host_ns
```

调用方会用目标时间做 LOS、用深度时间检查距离年龄。

### 5.4 时间基

- 接口内部只使用 `CLOCK_MONOTONIC`；
- 不要求 VIO 做飞控 boot 时间换算；
- main 会用 `FcuClock` 把目标时间和距离时间映射到飞控 boot 时钟。

---

## 6. 建议接口

接口名可以调整，但语义必须满足。建议使用**不抛异常、不阻塞**的返回值。

### 6.1 结果结构

```text
RangeStatus:
  kOk               成功
  kNotReady         深度流尚未就绪
  kNoDepth          当前没有可用深度帧
  kOutOfBounds      目标像素超出深度图/彩色图范围
  kStale            深度帧太旧或时间不匹配
  kLowQuality       有效深度像素太少或离散度过大
  kSessionChanged   相机会话/代数变化
  kError            其他内部错误

RangeSample:
  RangeStatus status
  RangeKind range_kind        // EUCLIDEAN / DEPTH_Z
  double range_m              // 仅 kOk 时有效
  double u_px, v_px           // 实际采样使用的彩色像素
  int64_t target_captured_ns  // 调用方传入
  int64_t depth_captured_ns   // 实际深度帧时间
  uint64_t depth_seq          // 深度帧序号
  uint64_t session, generation
  int valid_pixels            // 窗口内有效深度像素数
  double spread_m             // 可选：MAD/标准差，用于诊断
  const char* reason          // 可选：失败原因，生命周期由实现保证
```

### 6.2 查询接口

```text
RangeSample rangeAtPixel(
    double u_px,
    double v_px,
    int64_t target_captured_ns);
```

要求：

- 调用频率：50 Hz；
- 单次调用不得阻塞等待相机/深度帧；
- 典型耗时 < 1 ms，最坏不得超过一个主循环周期（20 ms）；
- 无异常跨越接口边界；
- 可在没有新深度帧时返回 `kNoDepth`/`kStale`，不要阻塞。

### 6.3 健康与统计

建议同时提供：

```text
bool depthReady() const;
bool healthy() const;
DepthRangeStats stats() const;
```

`stats()` 至少包含：

- 收到的深度帧数；
- 有效深度帧数；
- 查询次数；
- 成功/失败计数；
- 最近失败原因；
- 最近一次深度时间。

---

## 7. 线程安全与锁

这是最重要的约束。

`MissionService::tick()` 在 50 Hz 主循环里调用，并且**持有 `VioRuntime::PublicationGuard`**，该 guard 持有 `publication_mutex` 直到本拍 setpoint 发送完成。

因此：

1. `rangeAtPixel()` **禁止**再锁 `VioRuntime::publication_mutex`；
2. 必须使用独立的 depth/range 互斥锁，或无锁快照；
3. 不允许在 `rangeAtPixel()` 内等待相机线程、条件变量或 sleep；
4. 不允许在持锁期间做图像拷贝/大计算；
5. 如果深度帧正在更新，返回上一帧有效结果或 `kNotReady`，不要等。

建议：

```text
深度线程：更新 latest_depth_snapshot（独立锁）
查询线程：复制小快照 → 算 range → 返回
```

---

## 8. 彩色/深度几何

VIO 侧需要完成：

1. 彩色像素 `(u_c, v_c)` → 去畸变/归一化；
2. 彩色相机射线 `d_c = [(u_c-cx_c)/fx_c, (v_c-cy_c)/fy_c, 1]`；
3. 用 `R_c2d`、`t_c2d` 转到深度相机；
4. 用深度内参投影到 `(u_d, v_d)`；
5. 在深度图取中值，得到 `Z_d`；
6. 按 `range_kind` 返回：
   - `DEPTH_Z`：`range_m = Z_d`；
   - `EUCLIDEAN`：`range_m = Z_d * sqrt(1 + x_n² + y_n²)`（或等价三维点范数）。

注意：

- 彩色 K/D 来自 `config/guidance-camera.yaml`；
- 深度 K、`R_c2d`、`t_mm` 来自 VIO 通道 `StreamMeta`；
- 不要用 `StreamMeta.fx/fy/cx/cy` 当彩色内参；
- 当前深度是原始深度坐标系，不要假设已对齐到彩色；
- 如果后续确认深度已经对齐，必须在接口文档中显式声明，并更新本文。

---

## 9. 质量与失败策略

返回 `kOk` 必须同时满足：

- 深度帧存在且时间匹配；
- `depth_seq > 0`；
- `session/generation` 与当前相机会话一致；
- 目标像素在彩色图范围内；
- 投影后的深度像素在深度图范围内；
- 窗口内有效深度像素数 ≥ 最小阈值（建议 5）；
- 中值深度在 `[min_range_m, max_range_m]`；
- 离散度（MAD/标准差）不超过阈值；
- `range_m` 有限且 > 0。

任一不满足：

- 返回对应 `RangeStatus`；
- `range_m = 0`；
- 不允许返回“估算值”冒充深度测距；
- 不允许用目标框高度/像素宽度反推距离。

建议配置项（放 `config/vio.yaml` 或单独的 `config/range.yaml`）：

```text
range:
  min_range_m: 0.3
  max_range_m: 10.0
  max_depth_age_ms: 50
  window_px: 5
  min_valid_pixels: 5
  max_spread_m: 0.5
```

---

## 10. 会话与失效

- 相机服务重启、`session/generation` 变化时，旧深度和旧结果必须失效；
- 返回 `kSessionChanged` 或 `kNotReady`；
- 不允许把上一会话的 range 交给新会话；
- VIO 时间基重置、相机断流时同样清空结果。

---

## 11. 与任务侧的集成

任务侧计划：

1. detect 继续只发像素框；
2. `MissionService` 收到目标包后调用 `rangeAtPixel(u_px, v_px, captured_ns)`；
3. 只有 `status == kOk` 且时间/序号匹配时，填：
   ```text
   GuidanceTargetSample.has_range = true
   GuidanceTargetSample.range_m = range_m
   ```
4. 否则 `has_range = false`，任务不进入 Strike；
5. `MissionController` 仍会检查 `has_range && range_m > 0`，不会绕过。

任务侧不会：

- 修改 detect 的深度逻辑；
- 新增跨进程 range 通道；
- 在 main 里自己算深度；
- 用假距离让状态机进入 Strike。

---

## 12. 测试要求

### 12.1 无硬件单测

- 合成深度图 + 已知 K/R，给定像素返回已知 range；
- 中心像素、边缘像素、畸变边界；
- 深度为 0/NaN/越界；
- 有效像素不足；
- 时间差过大；
- session/generation 变化；
- 并发查询 + 深度更新，50 Hz 压测无死锁/无数据竞争。

### 12.2 板端实测

- 已知距离的平面/装甲板，1 m、3 m、5 m、8 m；
- 记录 range 误差、RMS、最大误差；
- 记录查询耗时、深度帧匹配率；
- 目标在画面中心/边缘的差异；
- 遮挡、反光、远距离失效情况。

### 12.3 验收指标（建议）

- 1~5 m 内静态目标误差 ≤ 0.2 m（或按实际标定调整）；
- 8 m 内可用；
- 50 Hz 查询不阻塞主循环；
- 无死锁；
- 失败时明确返回状态，不返回假值。

---

## 13. 交付方式

- VIO 侧代码提交到 `vio` 分支或 `dev/main-loop` 的功能分支；
- 接口头文件 + 实现 + 单测 + 本文档的最终版本；
- 任务侧只依赖接口，不依赖 VIO 内部实现；
- 合并前先在 `dev/main-loop` 上做一次 `--mission --no-send` 和 FakeRangeProvider 回归；
- 不直接推 `main/master`。

---

## 14. 待确认问题

1. `range_m` 用欧氏距离还是深度 z？
   本文默认要求欧氏距离；如果暂用 z，必须带 `range_kind`。
2. 彩色 K/D 是否统一从 `config/guidance-camera.yaml` 读取？
   建议是；不要和 `StreamMeta` 的深度内参混用。
3. 深度是否确认未对齐到彩色？
   当前代码看起来是原始深度；如果后续启用对齐，需要更新本文和接口。
4. range 是“按像素查询”还是“按目标 ID/序号返回快照”？
   本文建议按像素 + 时间查询，避免目标关联歧义。
5. 最大匹配时间差、有效距离范围、最小有效像素数由谁配置？
   建议 VIO 侧提供默认值，任务侧可在 `vio.yaml`/`range.yaml` 覆盖。
6. `rangeAtPixel()` 是否会在持有 `publication_mutex` 时被调用？
   会；因此接口必须使用独立锁，不能锁 `publication_mutex`。
