# patches/ 说明（2026-10-29 更新）

## 当前有效

| 文件 | 作用 | 适用分支 | 顺序 |
|---|---|---|---|
| `20261028_01_odometry_field_order.patch` | 修 ODOMETRY 的 `quality/reset_counter` 传反 | `vio` | 1 |
| `20261029_03_vio_heartbeat_clock.patch` | 心跳来源筛选、FcuClock 重启检测、串口自动重连、台架工具（假飞控/发送器/探针）、PTY 回归 | `vio` | 2（先应用 01） |
| `20261029_02_communicator_integration.patch` | 逐帧 IMU 队列、位姿质量/重置/协方差、心跳锁定、会话复位、PTY 回归 | `communication` | — |

应用方式（在 pure-uav 仓库里）：

```bash
git checkout vio
git apply .../20261028_01_odometry_field_order.patch
git apply .../20261029_03_vio_heartbeat_clock.patch

git checkout communication
git apply .../20261029_02_communicator_integration.patch
```

三个 patch 都在干净 worktree 上验证过：apply → 编译 → 测试通过。

台架工具（无飞机也能跑，见联调方案 §11）：

```bash
./bin/fake_fcu --link /tmp/fc_sim --duration 35 --save-raw /tmp/raw.bin
./bin/odometry_sender --serial /tmp/fc_sim --rate 30
./bin/odometry_probe --file /tmp/raw.bin --csv /tmp/pose.csv
```

`test_timebase_recovery`（离线模拟飞控重启）只在工作树里有：它需要
`VioEstimator::resetForNewTimebase()`，而分支快照的估计器还是旧版，等主工作树推上去时一起带上。

## 已废弃

`communicator_imu_callback.patch`（2026-10-18）**不要再用**：

- 它引用 `communicator.cpp` 里的 `data_mutex_`，而现在的实现已改成"每种数据一把锁"，
  `git apply` 会直接失败；
- 需要的能力已经由 `20261029_02_communicator_integration.patch` 提供，而且是**队列**
  而不是回调（接收线程只入队，VIO 线程自己取，不会在串口线程里做耗时工作）。

## 正式集成时 VIO 侧怎么接

```cpp
// 接收线程里 Communicator 已经自动逐帧入队，VIO 线程这样取：
pure_uav::Communicator::TimedImu item;
while (comm.popImu(item)) {
    estimator.feedImu(item.imu);      // item.imu.timestamp_us 是飞控 boot 时间
    clock.addSample(item.host_ns, item.imu.timestamp_us);  // 主机↔飞控时钟
}
// 位姿回传：
pose.quality = 90; pose.reset_counter = rc; pose.pos_sigma_m = 0.15f; pose.vel_sigma_mps = 0.25f;
comm.pushVisionPose(pose);
```

注意：`Communicator` 目前**没有串口自动重连**（`MavlinkLink` 已有）。正式集成时要么把
`MavlinkLink` 的 `handleLinkLoss/tryReopen` 策略搬过去，要么明确飞控重启后重启上位机程序。
