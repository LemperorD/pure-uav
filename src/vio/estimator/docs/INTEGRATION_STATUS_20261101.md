# VIO → 飞控集成修复与验收记录（2026-11-01）

本轮基于 `vio` 的 `c9e35c67a073b219cd6856f305aca3acb4901487`。以 Git 仓库中的
`src/vio/estimator`、`src/driver/communicate` 和 `src/common/msg.hpp` 为唯一代码基线。
桌面的独立工作树、`estimator_upload` 和嵌套快照保留为历史副本；本轮没有同步这些目录。

## 已完成的改动

- 删除已迁走的 `src/interface` 构建入口；补齐共享 `msg.hpp` 的 include。
- 恢复 patch 02/05 的通信实现：逐帧 IMU、主机接收时间、质量/重置/协方差、身份锁定和重连。
  这两份补丁已包含在源码中，不要再向本分支重复应用。
- 所有数据流订阅、制导指令和起飞/降落/模式/解锁命令统一使用已确认的飞控 sysid/compid。
  普通 `sendArm(true/false)` 的 force 参数为 0，遵守飞控自身安全检查。
- `MavlinkLink` 与 `Communicator` 在首个有效目标心跳前拒收遥测；连接后同时核对 sysid 和 compid。
  心跳超时清空旧 IMU，解除身份锁，递增会话代数。失联保留最后已知 armed 状态，避免空中开启 ZUPT。
- 两种链路的 `TimedImu.session` 标记接收时会话；`reconnectCount()` 现在包含物理重开及心跳超时。
  `vio_live` 按会话变化重建，并拒绝出队过程中失效的旧会话样本。
- VIO 链路写入增加总期限和停止取消；串口背压不会再无限卡住 `stop()`。
  共享串口 fd 的读写/关闭/重开有互斥；MAVLink 打包有互斥，接收使用独立解析状态。
- 物理异常（非有限位姿、持续超速）锁定 LOST，视觉特征重新变好不能直接解除；估计器重建后解除。
  普通视觉丢失仍保留原有软恢复。重建会清空旧视觉失效计时窗口。
- `FcuClock` 同时检测显著回退及相对主机时钟的大幅向前跳变；真实 IMU 中断超过 100ms 时
  `vio_live` 也重建。去抖网格在短丢帧后重新锚定，不用旧 PLL 网格追赶新时钟。
- 图像只在实际送入估计器的 IMU 覆盖 `t_camera + camera_to_imu_offset` 时处理；100ms 仍不覆盖则丢图。
  等待期间固定图像时间戳，避免同一图像重复污染相机时钟平滑窗口。
- ODOMETRY `reset_counter` 按 uint8 回绕（254→255→0→1），不再在 255 饱和。
  `--safe-tum-out` 只记串口发送成功的位姿；`--no-send` 不产生发送轨迹。串口成功不等于飞控已接收/融合。
- `odometry_sender` 持续跟踪真实 IMU 时间基准；重启/时间跳变/失联/IMU 超过 500ms 不更新时
  中止并返回非零，请重新运行。初始等待 IMU 受 `--hz-wait` 限制。
- 根目录启用 CTest，仅注册不接硬件的测试；ZUPT 测试使用构建目录中的专用配置，
  不依赖生产配置的 `try_zupt` 开关。本轮没有修改标定或生产 YAML。
- Python 工具不再硬编码用户名；独立 CMake 优先使用同仓库驱动，不再自动借用桌面历史副本。
  独立拷贝时显式设置 `PURE_UAV_DRIVER_DIR` 和 `PURE_UAV_MAVLINK_DIR`。

## 软件验收

全量构建包括 `guidance`、`app`、WiFi 测试程序、VIO 离线/实时程序和台架工具。
配置 `loop5` 数据集后，根目录 CTest 共 **20 项，全部通过**：

- 原有坐标转换、打包、链路、台架轨迹、数学/制导/适配器测试；
- 真实数据回放的时间基准重建与 LOST 重建后 ZUPT 保留；
- 新增物理失效锁定、向前跳变/短丢帧/图像覆盖/重置回绕；
- 两种链路的心跳前隔离、错误来源拒收、超时会话清理、同路径设备挂断/重开；
- 串口背压下发送期限和停机；9/42 非默认目标的全部命令及普通解锁/加锁；
- 实际 `odometry_sender` 的正常运行、时间回退、向前跳变、无 IMU、IMU 停更五种场景。

从仓库根目录复现（不需要飞控或相机）：

```bash
cmake -S . -B build -DPURE_VIO_BUILD_LIVE=ON   -DPURE_VIO_TEST_DATASET=/path/to/recorded/loop5
cmake --build build -j2
ctest --test-dir build --output-on-failure -j2
```

不提供 `PURE_VIO_TEST_DATASET` 时跳过两项真实数据回放测试，其余 18 项仍可运行。
也已单独配置 estimator 目录，并构建/运行其链路会话测试，确认能从同一仓库找到驱动。

## 和通信/主程序同学对接

`vio_live` 仍是本轮相机+IMU+ODOMETRY 台架/真机测试入口。生产 `src/app` 主循环尚未接入 VIO。
正式单进程集成时，应让 `Communicator` 独占串口，替换试验用 `MavlinkLink` 的数据源和输出：

1. 用 `popImu()` 顺序取样，同时将 `host_ns` 和飞控时间送给 `FcuClock`。
2. 核对 `TimedImu.session` / `reconnectCount()`；会话或时间基变化时暂停发布、清队列和相机缓存，
   调用 `resetForNewTimebase()`；时钟有效且估计器初始化后恢复。
3. 图像同样使用实际送入估计器的 IMU 覆盖门限，包含标定时间偏移。
4. `isArmed()` 控制运行期 ZUPT；`publishable()`、连接和会话共同控制发送。
5. 将质量、按 256 回绕的重建计数、位置/速度 sigma 填入共享 `VisionPose`，通过 `pushVisionPose()` 输出。
6. `sendArm` 等命令返回 true 只表示收到了 ACK，调用者仍需检查 `result == MAV_RESULT_ACCEPTED`。

## 分支交接和真机待验

本轮只在 `vio` 工作区落文件，没有提交、推送或修改 `main`。
`patches/20261101_06_main_build.patch` 是给当前本地 `origin/main` 的两处最小构建修复，
由 main 维护者应用；本分支已经修好，不要重复套用。

必须在实际硬件上另做：飞控日志确认 ODOMETRY 接收与帧率，前/右/上/偏航动作核对轴向，
EKF 对外部定位的实际融合、失效后拒绝融合及恢复后重锚定，真实 USB 拔插和飞控重启。
PTY 验证覆盖软件协议和恢复流程，不能替代固件融合与真实 USB 枚举验证。
