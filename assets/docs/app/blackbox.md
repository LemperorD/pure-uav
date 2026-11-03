# app 黑匣子（GuidanceRingLog）

- 状态：现行
- 相关代码：`src/guidance/flight/guidance_loop.hpp`、`guidance_loop.cpp`
- 相关文档：[README.md](README.md)、[csv-format.md](csv-format.md)

---

## 1 它是什么

`GuidanceRingLog` 是**内存里的“最后 10 秒”**：

- 固定环形缓冲 `kCap = 512` 拍；
- 50 Hz 下约 **10.2 s**，约 **49 KB**；
- 循环内只 memcpy，**零 I/O、零分配**；
- 只有 `dump_csv()` 才碰磁盘。

它和 CSV 的区别：

| | GuidanceRingLog | CSV (`guidance_sim_tools`) |
|---|---|---|
| 位置 | 内存 | 磁盘 |
| 时长 | 最近 512 拍 | 全程（可选） |
| I/O | 循环内零 | 每拍/限频写 |
| 用途 | 崩溃现场 | 复盘/回放 |

---

## 2 记录什么

`GuidanceRingRecord`：

```text
boot_us, ctrl, mode, health, flags,
a_ned[3], v_ned[3], a_norm,
t_go, Vc, R, lam_dot
```

`flags`：bit0 valid / bit1 sat / bit2 hold_yaw / bit3 velocity。

不记录：

- 输入（像素/四元数）→ CSV 负责；
- 图像 → 视频；
- 全流程 → `flight_logger`（未来）。

---

## 3 写入时机

- `GuidanceLoop::emit()`：**先 push 黑匣子，再发送**——发送失败也留下证据；
- `kStop` 分支：也会 push；
- 只有“产出指令”的拍才记录；传感器超时/首帧未齐的拍不记 → 可能有空档。

---

## 4 dump

```text
long dump_csv(const char *path, double seconds) const;
```

- `seconds <= 0`：全部；
- 按 `boot_us` 过滤最近 `seconds` 秒；
- 返回数据行数；打不开/写失败返回 -1；
- 输出表头：

```text
boot_us,ctrl,mode,health,valid,sat,hold_yaw,
a_n,a_e,a_d,v_n,v_e,v_d,anorm,t_go,Vc,R,lam_dot
```

- **唯一 I/O 入口，绝不能在循环里调用**。

命令行：

```bash
./bin/main --ticks=300 --blackbox=bb.csv --blackbox-sec=10
```

---

## 5 现在的限制

1. **没有自动 dump**：进程被 `SIGKILL`/断电就丢；目前靠 `main` 跑完或手动触发。
2. 只有 512 拍，不能配更长；要更长用 `flight_logger`。
3. 只有输出，没有输入。
4. dump 在调用线程里做 I/O，只能在循环停止后调用。
5. 没有 sequence 号，不支持增量 dump。

---

## 6 未来

- 信号处理/看门狗里自动 dump；
- 和 `flight_logger` 分工：GuidanceRingLog 负责“最后一口气”，logger 负责全程；
- 增加 `seq` 支持增量落盘。
