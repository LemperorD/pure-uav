# app CSV 格式（录制 / 回放 / 指令落盘）

- 状态：现行
- 相关代码：`src/app/sim/guidance_sim_tools.hpp`、`src/app/sim/guidance_sim_tools.cpp`
- 相关文档：[app/README.md](README.md)

CSV 是**地面工具**：用于离线复盘、调参 A/B、回归测试；机载二进制不需要链接它。

## 1 输入 CSV（一行 = 主循环一拍，31 列）

```text
tick, ins_new, ins_boot_us, has_pos, has_att,
p_n,p_e,p_d, v_n,v_e,v_d, qw,qx,qy,qz,
tgt_new, tgt_boot_us, u_px,v_px, has_box,w_px,h_px,
conf,class_id, present,has_range,range_m,
has_vT,vTn,vTe,vTd
```

- `ins_new / tgt_new`：这一拍是否真的有**新**帧（0 = 零阶保持）。
- `boot_us`：飞控 boot 时钟 µs（不是本机时钟）。
- `#` 开头是注释（表头即注释，读取自动跳过）。
- 浮点写 `%.17g` ⇒ 读回**逐位一致**。

### 为什么 `ins_new/tgt_new` 必须保留

回放时“没有新目标帧 ⇒ 只重发不重算”的行为全靠它。丢了它，就变成每秒 50 次重算，ω 会被系统性改变。

## 2 输出 CSV（一行 = 一拍，14 列）

```text
boot_us, ctrl, mode, valid, sat,
a_n,a_e,a_d, anorm, t_go, Vc, R, lam_dot, health
```

- `ctrl`：0 = 加速度，1 = 零速度；
- `mode`：0 = PN，1 = APN，2 = HOLD，3 = SEARCH；
- `sat`：这一拍是否撞到 `a_max`；
- `health`：输入体检结果。

## 3 录制 / 回放行为

- `GuidanceInputRecorder`：每拍一行，输入帧 + 目标帧一起写；没有新帧也写，保证行对齐。
- `GuidanceRecordingInsSource` / `GuidanceRecordingTargetSource`：装饰器，包在任意数据源外。
- `GuidanceCsvReader`：读 CSV，跳过注释/空行，解析 31 列。
- `GuidanceCsvReplaySource`：一行同时喂 `GuidanceInsSource` 和 `GuidanceTargetSource`；`ins_new=0` 时返回 false，让主循环沿用上一帧。
- `GuidanceCsvAccelSink`：输出 CSV；`GuidanceTeeSink` 可同时发执行器和文件。

## 4 用法

```bash
# 边跑边录
./bin/main --ticks=300 --record=flight.csv --out=cmd.csv

# 回放：同一份输入重算
./bin/main --replay=flight.csv --ticks=300 --out=replay_cmd.csv
```

## 5 回放的能力边界

回放验证的是“**算得对不对**”（数值、方向、模式），**不是“什么时候降级”**。

原因：`GuidanceLoop` 的超时/保持窗口用本机单调时钟；回放通常全速跑，本机时间几乎不前进，`sensor_timeout` / `hold_last_max` 不会触发。

要验降级时机：加 `--realtime`，或用 `test_guidance` §16 的桩。

## 6 实飞注意

**不要用 `GuidanceCsvAccelSink` 当实飞黑匣子。** stdio 落盘可能阻塞主循环（fsync 抖动几十 ms）。

实飞用：

- `MavlinkAccelSink` + 独立线程环形缓冲；
- 或 `GuidanceRingLog` 的内存黑匣子，出事再 dump。
