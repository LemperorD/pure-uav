# ADR 0005：统一时间基准

- 状态：接受
- 日期：2026-11-01（文档化）
- 相关代码：`src/guidance/adapter/include/guidance_state_adapter.hpp`
- 相关测试：`test_state_adapter` §2

## 背景

外部时间戳可能来自：

- 飞控 boot 时钟（`time_boot_ms`，uint32，约 49.7 天回绕）；
- PX4 uORB `timestamp`（uint64 µs，可能是 Unix epoch）；
- 相机自身时钟；
- 视觉/检测的 Unix 时间。

如果混用，差值就是垃圾，且**不会报错**。

## 决定

1. **制导统一时间基准 = 飞控 boot 时钟 µs**；
2. 所有进入制导的采样都带 `boot_us`；
3. **同一帧数据的所有时间戳必须同源**；
4. 跨源数据必须先换算到 boot 域，再参与差值；
5. `GuidanceStateTimeTracker` 用“累加每帧差值”而不是“先相减再转 double”。

## 为什么不用本机时钟

- 本机 `steady_clock` 只适合超时/降级计时，不能和飞控/相机数据对齐；
- 回放时本机时间几乎不前进，超时行为不可复现（见 `app/csv-format.md`）。

## 为什么用 `GuidanceStateTimeTracker`

调用方自己写 `(now − first) * 1e-6` 有两个坑：

1. uint32 ms 回绕（49.7 天）→ 差值变巨大负数；
2. uint64 µs 的绝对差在 uptime 很大时可能超过 `2^53`，double 丢精度。

`GuidanceStateTimeTracker` 累加小差值，天然容纳回绕，且精度恒定。

接口：

- `push_u64_us / push_u32_ms / push_s`；
- `t_s()` 相对参考点；
- `set_reference_*` 把当前原始值定为 0；
- `out_of_order()` / `last_jump_suspect()` 只提示。

## 相机/视觉时钟 → boot

用同帧 `SYSTEM_TIME`：

```text
offset_us = time_unix_usec − time_boot_ms × 1000
boot_us   = 视觉时间戳 − offset_us
```

- 单向换算（相机 → boot）；
- `offset_us` 持续维护（`SYSTEM_TIME` 约 1 Hz）；
- 视觉帧填**曝光时刻**，不是“收到时刻”。

## VIO / PX4

- 在线 VIO 的 `timestamp_us` 和 `ODOMETRY.time_usec` 自 2026-11-09 起使用主机 Unix µs；IMU、图像和深度在 VIO 入口统一换算，详见 `src/vio/estimator/README.md` 的时间戳说明；
- PX4 EKF2 融合 EV 时用 `EKF2_EV_DELAY` 补偿链路延迟；
- 制导只消费 PX4 `vehicle_local_position`，不直接消费 VIO。

## 维护

- 改单位/基准 → 同步 `interfaces.md` §4 和 `test_state_adapter` §2；
- 新增外部时钟源 → 先做“同源”证明或换算，再接线；
- 不要在业务代码里直接相减两个不同来源的时间戳。
