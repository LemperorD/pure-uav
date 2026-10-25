# `src/app/` —— 机载主循环（外壳层）

> 面向对象的那一层：**取数 → 换算 → 调核 → 体检 → 发指令 → 统计**。
> 内核（`src/guidance/`）一行不改，仍是纯函数 `guidance_step()`。

---

## 1 分层与文件

```
src/app/
  sample.hpp           一次采样的数据载体（InsSample / TargetSample，★ 目标是像素）
  output.hpp           对外输出契约（GuidanceOutput / ControlMode / InputHealth / LoopStatus）
  sensor_source.hpp    数据源接口 IInsSource / ITargetSource（拉模型）
  command_sink.hpp     指令出口接口 ICommandSink（★ 契约 + 通信侧翻译对照表）
  guidance_loop.{hpp,cpp}   主循环本体（一拍做什么，顺序不可颠倒）
  synthetic_world.{hpp,cpp} 合成世界（零依赖闭环）+ 闭环驱动（测试复用）
  csv_io.{hpp,cpp}          录制 / 回放 / 指令落盘 / NullSink
  params.{hpp,cpp}          参数加载 / 校验（YAML 子集 + CLI 覆盖，零第三方依赖）
  main.cpp                  薄壳：命令行解析 + 打印
```

依赖方向：`app → guidance_flight`（= `guidance_core` + `state_adapter` + `mavlink_units`）。
⚠ **只有** `mavlink_ins_source.cpp` / `mavlink_sink.cpp` 这类实现文件才 include 通信模块头文件；
契约头文件（`output.hpp` / `command_sink.hpp`）**零外部依赖**。

---

## 2 三种跑法

```bash
# ① 合成世界闭环（零依赖：无 Gazebo / 无 PX4 / 无相机 / 无串口）
./bin/main --law=0 --n=4 --a-max=10 --ticks=300

# ② 边跑边录（黑匣子）：输入 + 指令各落一份 CSV
./bin/main --record=flight.csv --out=cmd.csv

# ③ 回放：只用录下来的输入重算一遍（不跑合成世界）
./bin/main --replay=flight.csv --out=replay.csv

# ④ 参数文件：默认值 < YAML < 显式命令行；先看最终参数
./bin/main --config=guidance.yaml --print-params
```

参数：`--hz=N --law=0|1|2 --n=X --a-max=X --ticks=N --realtime --quiet --record=F --out=F --replay=F --config=F --print-params`。

**默认全速跑**（不睡眠）—— 只验链路。降级计时用墙钟，全速下走不完，
要看降级行为加 `--realtime`（或直接用 `test_guidance` §16 的桩）。

### 基线（2026-10-25 复验，PPN N=4 a_max=10，50 Hz，300 拍）

| 量 | 值 |
| --- | --- |
| 最近距离 | **0.033 m**（第 250 拍，飞越瞬间） |
| 饱和 | 0 次 |
| 发送 | 300 拍全部发出（`sent == ticks`） |
| 目标帧 | 250（后 50 拍目标出画 ⇒ 只重发、不重算） |

### 2.1 参数加载（`params.{hpp,cpp}`）

- **优先级**：默认值 < `--config=FILE` < **显式给出的**命令行项
  （`--law/--n/--a-max/--hz`；没给的不会覆盖 YAML）。
- **YAML 子集**（故意不引入 yaml-cpp）：缩进嵌套或扁平点号键、`#` 注释、
  整数/浮点/布尔/枚举名（大小写不敏感）、`cam.R_body_cam` 9 个逗号分隔浮点。
- **校验**（`validate()`，失败直接退出码 2）：`N ∈ (0.5,20]`、`a_max ∈ (0,100]`、
  `loop_hz ∈ [1,1000]`、`sensor_timeout ≥ max_sample_age`、`hold_last ≤ sensor_timeout`、
  相机 `fx/fy` 必须成对……
- **支持的键**：
  `guidance.{law,N,a_max,freeze_criterion,t_go_freeze,R_freeze,los_rate_source}` ·
  `loop.{loop_hz,max_sample_age_s,sensor_timeout_s,on_unhealthy,hold_last_max_s,full_check_hz,send_on_every_tick}` ·
  `cam.{fx,fy,cx,cy,R_body_cam}`
- `--print-params` 打印最终值（两次配置可 diff）；`cam.*` 未配时**只有合成世界 demo**
  才回退 `demo_camera()`，真机必须在 YAML 里给相机。

### 2.2 端到端打点（`LoopStats::latency`）

- `run_once()` 每拍记录 **5 段耗时（ns）**：取数 / 反投影 / 制导 / 发送 / 总；
- 固定**环形缓冲 `kCap=128`**（≈2.5 s @50 Hz）：循环内零分配、零 STL、零 I/O，
  只有 4 次 `steady_clock::now()` + 5 次数组写入；
- `LatencyTrace` 是 RAII：**所有 return 路径**（超时 / 无帧 / 降级 / 不健康）都落一条样本；
- P50/P95/max 在**循环外**按需计算（复制到栈上排序）；`main` 汇总里打印一行；
- ⚠ Windows `steady_clock` 分辨率约 **100 ns** ⇒ 数值是 100 ns 的整数倍；
  WSL/Linux 上更细。真实机载时延（串口/调度）会比合成世界大得多，以实机为准。

---

## 3 CSV 格式（一行 = 一拍）

```text
tick, ins_new, ins_boot_us, has_pos, has_att, p_n,p_e,p_d, v_n,v_e,v_d, qw,qx,qy,qz,
tgt_new, tgt_boot_us, u_px,v_px, has_box,w_px,h_px, conf, class_id,
present, has_range, range_m, has_vT, vTn,vTe,vTd
```

- `#` 开头是注释（表头即注释，读取自动跳过）；浮点写 `%.17g` ⇒ 读回**逐位一致**。
- `ins_new / tgt_new`：该拍是否真有新帧。★ **必须原样保留** —— 回放时
  「没有新目标帧 ⇒ 只重发不重算」的行为全靠它，丢了 ω 会被系统性改变。
- `boot_us`：飞控 boot 时钟 µs（见 `sample.hpp` 的时间戳约定）。
- 指令落盘 CSV：`boot_us, ctrl, mode, valid, sat, a_n,a_e,a_d, anorm, t_go, Vc, R, lam_dot, health`。

### 用法（接线时一行不用改主循环）

```cpp
InputRecorder rec;
rec.open("flight.csv");
RecordingInsSource    ins(raw_ins, rec);   // 装饰器
RecordingTargetSource tgt(raw_tgt, rec);
GuidanceLoop loop(params, cfg, ins, tgt, sink);
```

复盘：`CsvReader` + `CsvReplaySource`（一个类同时喂两个接口）+ `CsvAccelSink`。

### ⚠ 回放的能力边界

回放验证的是**算得对不对**（数值 / 方向 / 模式），**不是"什么时候降级"** ——
超时与保持窗口用本机单调时钟，全速回放时它几乎不前进。

---

## 4 测试

| 组 | 内容 | 断言数 |
| --- | --- | --- |
| `test_guidance` §16 | 主循环外壳：像素反投影、PN 方向、SEARCH、三级降级、过期、重发、统计 | 37 |
| `test_guidance` §17 | **端到端闭环回归**：跑一整段交会并断言 min R / 饱和 / HOLD / 每拍发送 | 16 |
| `test_guidance` §18 | **CSV 录制 → 回放逐位一致**（黑匣子有效性）+ 句柄生命周期 + 句柄泄漏计数 | 59 |
| `test_guidance` §19 | **参数加载**：默认值 / 嵌套与扁平 YAML / 未知键 / 类型错 / 重复键 / 范围校验 / to_string | 33 |
| `test_guidance` §20 | **端到端打点**：五段 P50/P95/max、环形回绕、空统计 | 14 |

§17 的价值：§1~§16 都是分段验证，分段全绿 ≠ 整条链没坏
（2026-10-20 就踩过 HOLD 窗口起算点写错、分段测试全过的事）。
§18 的价值：拿一份历史数据就能复现当时算出的每一条指令。

---

## 4.1 资源所有权（内存体检结论，2026-10-23）

**本层没有任何手工堆分配**：全仓库 `src/app`、`src/guidance`、`src/common` 里
`new / delete / malloc / free / strdup` 出现次数 = **0**。所有容器（`std::vector` /
`std::string`）都是 RAII，随作用域释放。

唯一"手工持有"的资源是 **`std::FILE*`**，集中在 `csv_io.{hpp,cpp}` 三个类：

| 类 | 释放点 | 重复 `open()` | `close()` 幂等 | 拷贝 |
| --- | --- | --- | --- | --- |
| `InputRecorder` | 析构 / `close()` | ✅ 先关旧句柄 | ✅ | 禁止 |
| `CsvReader` | 析构 / `close()` | ✅ 先关旧句柄 | ✅ | 禁止 |
| `CsvAccelSink` | 析构 / `close()` | —（只构造时开一次） | ✅ | 禁止 |

`GuidanceLoop` / `TeeSink` / 两个装饰器**只持有引用、不拥有对象** ⇒
循环对象必须先于其数据源析构（`main` 与测试里的声明顺序都满足）。

§18 ⑤ 用**进程句柄计数**做真正的泄漏检测：200 轮 open/write/read/close
（每轮都故意重复 open 一次）前后句柄数必须**完全相同**。
2026-10-23 实测：修复前 76 → 276（每轮漏 1 个），修复后 76 → 76。

---

## 5 还没做（下一步）

| 项 | 依赖 |
| --- | --- |
| `mavlink_ins_source.cpp` / `mavlink_sink.cpp`（真机数据入口/出口） | communication 合进 main |
| `VisionTargetSource`（接检测） | 检测组接口（像素 + 距离 + 曝光时戳） |
| `SimTruthInsSource` / `SimTruthTargetSource`（XTDrone2 真值） | 仿真桥 |
| 端到端打点（t0 取数 → t3 发出，P50/P95/max） | 无 |
| 环形缓冲黑匣子（循环内零 I/O） | 无 |
| EKF 重置监控（`kEstimatorReset`） | 通信侧给标志 |

详见 `src/guidance/后续注意问题.md` §1（阻塞项）与 §9（缺口）。
