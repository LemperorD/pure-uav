# 制导模块 —— 王乙芃

比例导引（PN）制导律与 3D 拦截仿真。

## 目录结构

```text
src/guidance/
├── CMakeLists.txt                  # target 一览见下节（core / sim / state_adapter / 两个聚合名）
├── guidance_libs/                  # 制导算法库（源码目录）
│   ├── include/
│   │   ├── guidance_types.hpp      # 数据结构：输入 state / 输出 cmd      ← 核心
│   │   ├── guidance.hpp            # 主接口 guidance_step()              ← 核心
│   │   ├── pn_law.hpp              # PN 族：TPN / PPN / APN              ← 核心
│   │   ├── quat.hpp                # 四元数通用数学（2026-10-16 从 sensor 移出）← 核心
│   │   ├── sensor.hpp              # 传感器墙（M4）：三级逼真度           ← **仿真**
│   │   └── target.hpp              # 目标模型（M2）：飞镖 / 抛体 / 机动    ← **仿真**
│   └── src/
│       ├── guidance.cpp            # 主流程（有效性→几何→选律→冻结→限幅）← 核心
│       ├── pn_law.cpp              # 导引律实现                            ← 核心
│       ├── quat.cpp                # 四元数实现                            ← 核心
│       ├── sensor.cpp              # 投影 / 像素噪声 / size 测距 / 延迟 / 丢帧 ← **仿真**
│       └── target.cpp              # 目标弹道（解析式 + 预积分，含气动阻力）  ← **仿真**
├── adapter/                        # 数据对齐适配层（2026-10-20 由 src/interface 迁入）
│   ├── include/
│   │   ├── state_adapter.hpp       # 外部口径 → GuidanceState（"边界只转一次"）
│   │   └── state_check.hpp         # 输入自检：不变量体检（只报告不修改）
│   └── src/
│       ├── state_adapter.cpp
│       └── state_check.cpp
└── sim/
    ├── pn_sim.cpp                  # 3D 拦截数值仿真（输出 CSV）
    ├── plot_sim.py                 # 单场景画图（3D 轨迹 / R / λ̇ / |a|）
    ├── plot_compare.py             # 多律对比画图（展示 PPN 的 N_eff 判据）
    ├── plot_sensor.py              # 传感器墙对比画图（噪声 / 测距误差 / 丢帧）
    └── plot_target.py              # 目标模型画图（弹道 / 速率 / 加速度）
```

> `vec3.hpp`（轻量三维向量）已于 2026-10-20 迁到 `src/common/common_libs/include/`，
> 见下节末的说明。

## ★ CMake target 一览（2026-10-20 更新）

**保证机身不带仿真代码**，换真机时 `guidance_step` 一行不改。详见
《制导讲义/换真机替换清单.md》。

| target | 内容 | 谁链接 |
| --- | --- | --- |
| **`guidance_core`** | `guidance.cpp` + `pn_law.cpp` + `quat.cpp`（+ 纯头文件） | ★ 真机主循环的**算法**部分 |
| **`guidance_sim`** | `sensor.cpp` + `target.cpp`（依赖 core，单向） | 只在仿真 / 离线测试里 link |
| **`state_adapter`** | `state_adapter.cpp` + `state_check.cpp`（适配层） | ★ 真机的**数据入口**（不 link core） |
| **`guidance_flight`** | **机载聚合名**（= core + state_adapter + mavlink_units） | ★★ **飞机上只 link 它**（INTERFACE，不产生库文件） |
| `guidance_libs` | **聚合名**（= core + sim，INTERFACE 目标） | 仅为向后兼容旧脚本；⚠ **机载不要 link** |

**验证**（二进制符号级）：`libguidance_core` 里 `Sensor=0  TargetModel=0`，
而 `guidance_step` / `quat_rotate_vec` 齐全 ⇒ 核心库**不含任何仿真符号**。

> **真机最小足迹** = 聚合目标 **`guidance_flight`**（= `guidance_core` + `state_adapter` + `mavlink_units`）。
> 源码位置：`src/guidance/`（核心）+ `src/guidance/adapter/`（适配层）+ `src/common/common_libs/`（单位换算 / `Vec3`）。
> 真机的数据入口是 `state_adapter::to_guidance_state()` —— **"谁填 GuidanceState"就是唯一的替换点**。
> 真机必须改的参数：`GuidanceParams::los_rate_source` → `FROM_LOS_DIFFERENCE`
> （真机拿不到 `v_T`；ω 由核内用 `û` 历史差分算出）。

单元测试在 `src/test/test_guidance/test_guidance.cpp`（15 组 126 条断言，产物 `bin/test/`）。
> ⚠ 这个测试**确实需要仿真**（用 `TruthState` / `Sensor` / `TargetModel` 做端到端），
> 所以它链接 `guidance_core` + `guidance_sim`。而 `test_state_adapter`（真机路径）
> 只链接 `state_adapter` + `mavlink_units` + `pure_vec3` + `guidance_core` —— 依旧**不碰仿真**。

**`Vec3` 已迁出**（2026-10-20）：`vec3.hpp` 现在在 `src/common/common_libs/include/`，由
header-only 的 INTERFACE 目标 `pure_vec3` 提供 include 目录。因此 `src/guidance/` 不再自带
工具头，**制导模块不能再脱离仓库单独拷走编译**（依赖 common 的头文件）。

## 设计要点

- **制导律是纯函数**：`guidance_step(state, params) -> cmd`，不读全局、不碰 I/O。
  仿真阶段用真值喂它，接真机时用视觉喂它，本文件一行不改。
- **默认导引律 PPN**：`GuidanceParams::law` 默认 `PPN`（`a = N·(ω × v_M)`）。
  它不含距离项，对测距误差免疫，是无人机常用形式。
- **数据结构对齐接口文档**：`GuidanceState` / `GuidanceCmd` 字段严格对应
  《输入数据字典.md》《输出数据字典.md》。
- **矢量运算用叉积**：3D 制导的视线角速率 `ω = (r × v_rel) / R²`，分母是 **R²**。
  真机也可用 `ω = û × (dû/dt)`（只用视线方向时序差分，不需要 v_T / R）。

## ω 的两套算法（可切换）

视线角速率 ω 保留**两套实现**，用 `GuidanceParams::los_rate_source` 切换：

| 枚举 | 公式 | 需要 | 用在 |
| --- | --- | --- | --- |
| `FROM_RELATIVE_VELOCITY` | `ω = (r × v_rel) / R²` | 距离 R + 目标速度 v_T | 仿真（真值） |
| `FROM_LOS_DIFFERENCE` | `ω = û × (dû/dt)` | 只需 û 的历史（存 `GuidanceMemory`） | 真机 |

**怎么改**：把 `GuidanceParams::los_rate_source` 从 `FROM_RELATIVE_VELOCITY` 改成 `FROM_LOS_DIFFERENCE` 即可，制导律其余部分不动。
推导见《比例导引讲义.html》第 12 节。

## 目标模型（M2）：飞镖的弹道

`target.hpp/.cpp` 把"目标"从"匀速直线"升级成有物理的弹道。接口只有一个：
`TargetModel::state(t) -> {p, v, a}`，**只依赖 t**（不依赖无人机），因此可预生成、可回放。

| 类型 | 物理模型 | 求解方式 | 用途 |
| --- | --- | --- | --- |
| `STRAIGHT` | 匀速直线 | 解析 | 基准对照（旧模型） |
| `BALLISTIC` | 只受重力 g | 解析（含解析落地时刻） | 无阻力飞镖 |
| **`DART`** | **重力 + 二次气动阻力** | **RK4 预积分成表** | **飞镖（默认）** |
| `CIRCLE` | 定常盘旋（a = ω×v） | 数值 | 打地面机器人 |
| `SNAKE` | 侧向正弦机动 | 数值 | 鲁棒性压力测试 |

### 飞镖动力学

```text
a = g + a_drag + a_maneuver
a_drag = −k·|v_rel|·v_rel        （方向与相对速度反向，大小 ∝ v²）
k      = ½·ρ·Cd·A/m              （1/m，阻力/惯量系数）
v_rel  = v − wind
```

发射参数用 `launch_speed / launch_elev / launch_azim` 给出（比直接给 `v0` 好调）：
`v0 = speed·(cosθ·cosφ, cosθ·sinφ, sinθ)`。

**为什么"有加速度"重要**（相对旧的匀速直线模型）：

1. **重力使弹道变成抛物线** —— 拦截几何完全变了（这是主要项）；
2. **阻力 ∝ v²** —— 速度单调衰减、弹道比理想抛物线更早落地，且 `a_T` 沿速度反向、
   大小随速度变化 → **APN 的 `a_T⊥` 补偿项不再是常值**；
3. 最近点出现在弹道下降段，而非几何中点。

### 阻力到底有多大（本仓库默认参数）

```text
Cd=0.50, m=0.40 kg, A=2.83e-3 m²（≈φ60mm）, ρ=1.225  →  k = 0.0022 1/m
v = 18 m/s 时  a_drag ≈ 0.70 m/s² ≈ 7% g
```

同发射条件下的实测差异：

| 项 | 无阻力 | 有阻力 | 差异 |
| --- | --- | --- | --- |
| 飞行时长 | 2.04 s | 2.02 s | −0.8% |
| 水平射程 | 31.7 m | 30.4 m | **−4.2%** |
| 落点速度 | 19.1 m/s | 18.1 m/s | **−5.2%** |

**结论**：对这个小而轻的飞镖，**重力是一阶项，阻力是二阶修正**（约几个百分点）。
但阻力对 `a_T` 的"时变性"有贡献，APN 若要精确补偿就得把它算进去。

> ⚠ **气动参数（`mass` / `drag_cd` / `drag_area`）目前是估计值**，需要飞镖组给出真值后替换。
> 参数在 `TargetConfig` 里集中定义，改一处即可。已记入《待决问题.md》Q2。
> 阻力影响对参数敏感（∝ Cd·A/m）：若飞镖更轻或带较大尾翼，影响会成倍放大。

### 对制导的含义（实测）

飞镖是**受重力的抛物线目标**，垂直速度从 `+9 m/s` 一路变到 `−12 m/s`，
这正是 APN 的 `a_T⊥` 项要处理的情形。同场景三律对比（N=6）：

| 律 | 最近距离 | 峰值 \|a\| | 峰值 λ̇ | 说明 |
| --- | --- | --- | --- | --- |
| PPN | 0.012 m | 22.20 | 0.485 | 不含重力补偿，靠 N 硬压 |
| TPN | 0.002 m | 28.75 | 0.140 | `V_c·(ω×û)`，压得更准 |
| **APN** | 0.016 m | **1.40** | 0.149 | `a_T⊥` 项**直接抵消重力** → 几乎不用出力 |

**结论**：打"受恒定重力"的飞镖，APN 的控制代价比 PPN 低一个数量级
（1.40 vs 22.20 m/s²），因为重力是可预测的常值加速度，补偿它几乎是免费的。
弹道曲线见 `sim/target_model_plot.png`（`python sim/plot_target.py`）。

## 末端冻结判据（t_go / R）与"加速度的影响"

末端 `R→0` 时 `ω=(r×v_rel)/R²`（或 `û` 差分）里的 `1/R` 会把噪声放大成大指令，
所以最后一小段要**冻结**（HOLD，保持上一帧指令）。冻结判据用
`GuidanceParams::freeze_criterion` 三选一：

| 判据 | 条件 | 需要 | 特点 |
| --- | --- | --- | --- |
| `BY_TGO`（默认） | `t_go < t_go_freeze` | Vc（或 R 的导数） | 以**时间**为基准，随接近速度自动缩放 |
| `BY_RANGE` | `R < R_freeze` | 只要 R（可直接测） | 真机最简；固定 R 阈值在 Vc 变化时会早/晚冻结 |
| `BY_TGO_OR_RANGE` | 任一满足 | 两者 | 用 R 兜底，防 Vc→0 时 t_go 发散 |

**两者在 Vc 近似常数时等价**：`R_freeze ≈ Vc · t_go_freeze`。本场景
`Vc ≈ 34.3 m/s`，故 `0.30 s ↔ 10 m`、`0.15 s ↔ 5 m`。实测（PPN N=6）：

| 判据 | 最近距离 | 结果 | 峰值 \|a\| |
| --- | --- | --- | --- |
| `BY_TGO t_go=0.30 s` | 0.108 | 脱靶 | 18.49 |
| `BY_RANGE R=10.0 m` | 0.113 | 脱靶 | 18.38 |
| `BY_TGO t_go=0.15 s` | 0.012 | 命中 | 22.20 |
| `BY_RANGE R=5.0 m` | 0.013 | 命中 | 22.11 |

→ `R=10 m` 与 `t_go=0.30 s` 表现一致、`R=5 m` 与 `t_go=0.15 s` 一致，
**证实二者等价**。所以 R 基准**完全可用**，而且真机上更好实现（R 由 size 反推测得，
不必再估 Vc）。

**为什么仍然默认 t_go**：当系统存在**加速度**（目标机动 / 自身机动）时，`Vc` 不再恒定，
`R_freeze = Vc·t_go_freeze` 的等价关系会漂移，固定 R 阈值意味着**冻结时刻随接近
动力学变化**。而制导可修正的横向量是 `Δy ≈ ½·a·t_go²` —— **时间才是物理根本量**，
所以 t_go 基准更稳健；R 基准作为兜底（`BY_TGO_OR_RANGE`）兼顾真机实现便利。

### 加速度怎样进入（参考 APN）

带加速度的相对运动（视线法向）方程：

```text
λ̈ = (a_T⊥ − a_M⊥) / R  +  (2·Vc / R)·λ̇
```

- 目标加速度 `a_T⊥` 是 `λ̇` 的**驱动源**：不补偿它，λ̇ 会持续增长，
  必须靠末端大 `1/R` 增益硬压 → 指令大、易饱和。
- **APN** 在指令里**前馈**掉这一项：`a = N·Vc·(ω×û) + (N/2)·a_T⊥`，
  于是反馈回路只需处理残差 → λ̇ 峰值小、指令小。
- 对"受恒定重力"的飞镖，`a_T⊥` 基本就是重力的横向分量，APN 几乎**免费**抵消它
  （实测峰值指令 22.20 → 1.40 m/s²，低一个数量级）。

**对冻结判据的含义**：因为 APN 把 `a_T⊥` 提前消掉，末端残差 `λ̇` 更小，
同一个 `t_go_freeze` 在 APN 下更安全（更不容易因"来不及修正"而脱靶）；
反过来，未补偿的 `a_T⊥` 会在末端持续产生新的脱靶量，`t_go_freeze` 需要留更大余量。

> 注：APN 需要 `a_T⊥` 的估计。仿真里由目标模型给真值；真机需加 `a_T` 估计器
> （见《待决问题.md》）。PPN 不含该项，故对机动目标只能靠提高 N 硬压。

## 边界假设（重要）

制导**假定上游数据处理已完成**：

```text
目标检测  →  û / R        ┐
定位惯导  →  q / v_M      ├──►  GuidanceState  ──►  guidance_step()  ──►  GuidanceCmd
视觉      →  λ̇            ┘
```

制导只消费 `GuidanceState`。至于"从原始传感器到 `GuidanceState` 的预处理
（反投影、坐标变换、测距、λ̇ 估计、时间对齐）是否要单独写一个 cpp、归属谁管"，
目前**归属待定**，已记录在《待决问题.md》Q1，等后续处理。
> 仿真里这部分已由 `guidance_libs/src/sensor.cpp`（传感器墙，见下节）实现，
> 可作为真机实现的参考；真机由 detection / vio 组替换同一接口。


## 三条铁律（写实现时务必遵守）

1. `ω = (r × v_rel) / R²` —— 分母是 **R²** 不是 R。
2. 过载限幅要**矢量等比缩放** `a = a·(a_max/‖a‖)`，禁止对 x/y/z 分量各自 clip。
3. `t_go` 过小时**冻结制导**（HOLD），防末端 `1/R` 放大噪声导致发散。

## 构建

在仓库根目录：

```bash
./scripts/autoBuild.sh          # 增量构建
```

产物：`lib/libguidance_core.so` + `lib/libguidance_sim.so`、`bin/pn_sim`、`bin/test/test_guidance`。

## 测试

```bash
./bin/test/test_guidance        # 14 组 97 条断言；退出码 0 = 全过
```

无外部框架（纯 `main()` + 轻量断言，风格同仓库 `test_math`），覆盖：
Vec3 基础 · 讲义 §10 手算例逐项对照 · ω 两套算法一致性 · Vc 符号 ·
TPN/PPN/APN 端到端 + APN 补偿恒等式 · 三条铁律（R² 分母 / 矢量等比限幅 / HOLD）·
SEARCH · ω 来源切换 · 传感器投影往返 · 四元数姿态 · 逼真度语义 · 丢帧 · 延迟 ·
目标模型（发射参数 / 解析抛物线 / **DART(Cd=0) ≡ BALLISTIC 交叉验证** / 气动阻力 /
CIRCLE / SNAKE / 查表连续性）· 末端冻结判据（**t_go 基准 ≡ R 基准** / R 兜底防 Vc→0）。

## 运行仿真

```bash
./bin/pn_sim                                 # 默认：PPN，N=6，传感器 IDEAL，飞镖目标
./bin/pn_sim out.csv 4 PPN                   # 指定 CSV 名 / 导航比 N / 导引律
./bin/pn_sim out.csv 6 TPN SEMI              # 第 4 个参数切传感器逼真度
./bin/pn_sim out.csv 6 PPN IDEAL BALLISTIC   # 第 5 个参数换目标模型
```

参数：`pn_sim [csv] [N] [law] [fidelity] [target]`
- `law ∈ {PPN, TPN, APN}`（也接受 0/1/2）
- `fidelity ∈ {IDEAL, SEMI, FULL}`（默认 IDEAL；SEMI/FULL 见下节）
- `target ∈ {dart, ballistic, straight, circle, snake}`（默认 dart）

不带参数时打印：目标模型摘要 + **气动阻力影响**、主场景、**目标模型对比**、
**导引律对比**、**末端冻结判据对比**、**传感器墙对比**、**像素噪声敏感性**、N 扫描（2→8）。
同时导出 `target_*.csv`（各目标弹道，供画图；已被 `.gitignore` 忽略）。

画图（需要 matplotlib）：

```bash
python sim/plot_sim.py pn_sim_log.csv    # 单场景 4 子图：3D 轨迹 / R(t) / λ̇(t) / |a|(t)
python sim/plot_compare.py               # 多律对比：λ̇(t) 与 |a|(t)，展示 N_eff 判据
python sim/plot_sensor.py                # 传感器墙对比：λ̇ / |a| / 测距误差 / 制导模式
python sim/plot_target.py                # 目标模型：弹道 / 速率衰减 / 加速度
```

## 传感器墙（M4）：三级逼真度

仿真与制导之间隔一道**传感器墙**：仿真侧用 `Sensor` 把真值变成"看起来像视觉系统给的"
`GuidanceState`，制导律一行不改。切 `SensorConfig::fidelity` 即可：

| 逼真度 | 做了什么 | 用在 |
| --- | --- | --- |
| `IDEAL` | 真值透传（无噪、无延迟、无丢帧） | 基线 |
| `SEMI_REAL` | 相机投影 + 像素噪声 + size 反推测距（λ̇ 走差分） | 半逼真 |
| `FULL_REAL` | 半逼真 + 链路延迟 + 丢帧 | 全逼真 |

**关键配对**：非 IDEAL 时传感器拿不到目标速度 `v_T`，制导必须改用
`FROM_LOS_DIFFERENCE`（真机唯一可行的 ω 来源）。仿真里由 `pair_los_source()` 自动配对。

### 实测（PPN N=6，制导 100 Hz，飞镖目标）

| 逼真度 | 最近距离 | 结果 | 峰值 \|a\| | 丢帧 |
| --- | --- | --- | --- | --- |
| IDEAL | 0.016 m | 命中 | 22.44 | 0 |
| SEMI_REAL | 0.025 m | 命中 | 24.44 | 0 |
| FULL_REAL | 0.113 m | **脱靶** | 30.00（饱和） | 5 |

**结论**：像素噪声把峰值指令从 22.4 顶到饱和 30 m/s²；真正致命的是
**延迟 + 丢帧**（FULL_REAL）—— 50 ms 延迟在 34 m/s 接近速度下等于约 **1.7 m 的位置滞后**，
而命中球半径只有 0.1 m。

### 失效边界：像素噪声扫描（SEMI_REAL）

| 像素噪声(px) | 最近距离 | 结果 | 峰值 \|a\| |
| --- | --- | --- | --- |
| 0.0 | 0.021 | 命中 | 23.79 |
| 0.1 | 0.025 | 命中 | 24.44 |
| 0.3 | 0.029 | 命中 | 29.78 |
| 0.5 | 0.038 | 命中 | 30.00（饱和） |
| 1.0 | 0.046 | 命中 | 30.00（饱和） |
| 2.0 | 0.270 | **脱靶** | 30.00（饱和） |

失效模式是**过载饱和**（撞上 `a_max = 30 m/s²`），不是制导发散。拐点在 **~2 px**。
→ 下一步该做的是 **λ̇ 滤波 / 估计器**（M5），或放宽过载上限。

曲线见 `sim/sensor_compare_plot.png`（`python sim/plot_sensor.py`）。

## 仿真发现：PPN 的有效导航比

跑 N 扫描会看到一个**反直觉**的现象：同样迎头场景，TPN 在 `N ≥ 2.5` 就能命中，
而 PPN 要 `N ≥ 4` 才开始命中。原因是 PPN 指令里用的是 `v_M` 而不是 `V_c`：

```text
N_eff ≈ N · (V_M / V_c)        # 稳定判据仍是 N_eff > 2
```

本场景（飞镖迎头）`V_M = 18`、`V_c ≈ 34.3`，故 `V_M/V_c ≈ 0.52` → 理论要求 `N > 3.8`。
N 扫描实测（最近距离 m）：

| N | PPN | TPN |
| --- | --- | --- |
| 2.0 | 0.457 脱靶 | 0.125 脱靶 |
| 2.5 | 0.347 脱靶 | 0.047 命中 |
| 3.0 | 0.242 脱靶 | 0.019 命中 |
| 4.0 | 0.077 命中（**临界**） | 0.014 命中 |
| 6.0 | 0.012 命中 | 0.002 命中 |
| 8.0 | 0.002 命中 | 0.012 命中 |

`N=4` 的 PPN 恰好压在稳定边界上（`N_eff = 2.08`），λ̇ 几乎不衰减、指令全程饱和到
30 m/s² —— 命中纯属临界侥幸，加噪声即脱靶。**所以本仿真默认取 N=6。**

**结论**：PPN 的 N 不能照搬 TPN 的"2~4"，要按 `V_M/V_c` 折算；迎头（`V_c` 大）时
尤其要放大 N。对比曲线见 `sim/pn_compare_plot.png`（`python sim/plot_compare.py`）。
这条已同步进讲义。

## 参考文档

- 《比例导引讲义.html》 —— 公式推导
- 《仿真代码设计思路.md》 —— 仿真架构
- 《输入数据字典.md》/《输出数据字典.md》 —— 接口契约
