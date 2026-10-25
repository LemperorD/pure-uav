# 给 Communicator 增加 IMU 回调（VIO 用）

## 为什么需要

`Communicator::getImu()` 每次只返回**最新一帧** IMU：

```cpp
case MAVLINK_MSG_ID_HIGHRES_IMU: {
    std::lock_guard<std::mutex> lk(data_mutex_);
    imu_.timestamp_us = imu.time_usec;
    ...
}
```

VIO 线程无论按什么频率轮询，都可能在某两次轮询之间丢掉 IMU 帧；而 MSCKF 的传播对
IMU 完整性比较敏感（尤其是 100/200 Hz 下丢 1~2 帧）。

## patch 内容

1. `communicator.hpp`
   - `#include <functional>`
   - 新增 `void setImuCallback(std::function<void(const ImuData&)> cb);`
   - 私有成员 `std::function<void(const ImuData&)> imu_callback_;`
2. `communicator.cpp`
   - `HIGHRES_IMU` 分支里先组好 `ImuData sample`，加锁写入 `imu_` 并拷贝回调，
     **在锁外**调用 `cb(sample)`（避免 VIO 入队时死锁）。
   - 实现 `setImuCallback`。

## 应用方式

communication 分支（不是 git 仓库也能用 `patch`）：

```bash
cd ~/桌面/pure-uav/pure-uav-communication
patch -p1 < ~/calib/vio_online/patches/communicator_imu_callback.patch
```

如果已经是 git 仓库：

```bash
git apply ~/calib/vio_online/patches/communicator_imu_callback.patch
```

## VIO 侧用法

```cpp
std::mutex q_mtx;
std::deque<pure_uav::ImuData> imu_q;

comm.setImuCallback([&](const pure_uav::ImuData& imu) {
    std::lock_guard<std::mutex> lk(q_mtx);
    imu_q.push_back(imu);              // 只入队；不要做耗时操作
});
```

VIO 线程从 `imu_q` 取数据喂给 `VioEstimator::feedImu()`，图像同理从相机线程入队。

## 说明

- `~/calib/vio_online` 里的 `vio_live` **没有**用这个 patch：它自带 `MavlinkLink`，
  自己开串口，方便在不动通信模块的情况下先上真机验证。
- 正式单进程版本应让 `Communicator` 做唯一的串口拥有者，避免两个进程/两个对象
  同时读写同一个 `/dev/ttyACM*`。
- `getImu()` 保留不变，其他模块（日志/监控）仍可轮询最新值。
