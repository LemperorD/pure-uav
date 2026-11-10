# 主任务框架（第一版）

任务层负责 **起飞 → 找装甲板 → 撞板 → 返航 → 降落** 的调度。
`MissionController::tick()` 只计算本拍意图，不直接打开设备、发送命令或等待 ACK。
当前提供独立演示和无硬件测试，`main` 中已标注接入位置，尚未启用自动任务。

## 文件与运行

| 文件 | 用途 |
| --- | --- |
| `mission_types.hpp` | 状态、参数、输入快照、输出意图 |
| `mission_controller.{hpp,cpp}` | 单线程、非阻塞状态机 |
| `mission_example.cpp` | 合成反馈演示，包含一次合成撞击事件 |
| `src/test/test_app/test_mission.cpp`（从仓库根目录计） | 正常流程、时间戳、目标丢失、超时和故障回归 |

在仓库根目录运行：

```bash
cmake -S . -B build
cmake --build build --target mission_example test_mission -j4
./bin/mission_example
ctest --test-dir build -R '^mission(_example)?$' --output-on-failure
```

演示用完美位置跟随代替飞控，用合成事件代替撞击传感器，不验证 PN 制导或飞行动力学。
任务核心不依赖 ROS、相机或串口；仓库根 CMake 仍会配置工程已有的其他依赖。

## 状态流转

```text
Idle --显式启动--> Preflight --自检/定位/落地/未解锁--> Offboard
Offboard --实际 setpoint 预热就绪 + 模式反馈--> Arming
Arming --飞控确认已解锁--> Takeoff
Takeoff --到达相对高度 + 低速持续稳定 + 已离地--> Search
Search --连续 N 个新目标帧 + 测距/制导就绪--> Strike
Strike --外部撞击确认--> ReturnHome
Strike --丢失目标超过窗口--> Search
Search / Strike --各阶段超时--> ReturnHome
ReturnHome --回到起飞点上方且稳定--> Landing
Landing --飞控确认已落地且已上锁--> Completed
```

- 飞行中收到 `return_requested`：进入返航；还没离地则直接交飞控降落。
- 总任务时限也会触发返航；返航本身超时则请求飞控在当前位置自主降落。
- 人工接管、掉线、控制命令失败、定位过期、异常上锁或意外退出 Offboard：进入 `Aborted` 并停止输出。
- `Completed` 表示流程已结束，撞板是否确认要单独检查 `strike_confirmed`；超时返航不会伪造撞板成功。
- 终态不会因启动标志仍为真而重复执行；宿主停止执行旧输出后，显式 `reset()` 才能再启动。

## 坐标、时间和目标约定

所有任务位置使用与飞控反馈一致的本地 **NED**：米、米/秒，z 向下。
自检通过时记录当前位置 `home_ned`；例如起飞点 z=-0.4 m、起飞高度 1.5 m，目标 z=-1.9 m。
返航回到同一个起飞点上方，之后由飞控自主降落。
VIO 的原生 ENU/FRD 结果不能未经换算直接填入。

`now_s`、`pose_time_s`、`target_time_s` 必须用同一个 `steady_clock` 原点。
后两者是采样时刻映射，不是主循环读取缓存的时刻。
`ins.boot_us` / `target.boot_us` 仍保留飞控 boot 微秒口径；目标确认只统计严格递增的新帧。
反复读取同一张图不会增加确认次数，旧帧、未来帧、低置信度、错误类别和无测距目标不会进入撞板阶段。
目标源应先选择并持续跟踪同一块装甲板；第一版没有跨检测框的身份关联。

默认连续确认 3 帧、起飞高度 1.5 m、搜索超时 30 s、撞板超时 5 s。
参数集中在 `MissionConfig`，构造时校验。第一版未接 YAML/CLI。

## 接入现有主循环

建议以现有 50 Hz 公共节拍调用，保留 VIO 发布和进程监督。每拍顺序：

1. 收集飞控、目标和操作者输入，构造 `MissionInput`。
2. 调用 `MissionController::tick(input)`。
3. 若 `reset_guidance` 为真，创建新的 `GuidanceLoop` 实例并调用 `init()`，清空上一段制导记忆。
   当前 `init()` 对已初始化实例直接返回，不会重置；适配层也可后续补显式 reset 接口。
4. 若 `run_guidance` 为真，调用 `GuidanceLoop::run_once()`，本拍只让它使用共享指令出口。
5. 否则若 `setpoint` 有值，通过 `Communicator::pushGuidanceCmd()` 发送任务位置目标。
6. 将非空 `request` 交给命令执行器；下一拍反馈执行失败或真实飞控状态。

`setpoint` 与 `run_guidance` 互斥，避免任务和制导在同一拍抢占出口。
不要让另一个线程中的 `GuidanceLoop::run()` 在任务返航后继续发送。
暂停期间须停止执行制导；重新进入撞板阶段先重置记忆，避免复用旧视线角速度。
`GuidanceLoop` 初始化/输出失败必须反馈到任务，不能忽略错误后继续把 `guidance_ready` 设为真。

| 任务接口 | 现有模块 / 待接工作 |
| --- | --- |
| `ins` | `runtime/guidance_ports.hpp::makeGuidanceIns()` 可合成飞控姿态和位置；宿主补采样年龄检查 |
| `target` | 复用 `GuidanceTargetSample`；独立 `detect_rt` 的结果传输、时间映射和测距仍待接入 |
| `fcu_connected / armed / offboard` | 通信心跳、`isArmed()` 与飞控模式反馈；均须有新鲜度保证 |
| `landed` | 需补飞控落地状态反馈，目前任务层只预留输入，不靠高度猜测 |
| `preflight_ready` | 宿主汇总定位、操作开关和整机自检；状态机另外检查位置新鲜度及静止条件 |
| `offboard_stream_ready` | 只在持续实际发送的预热流达标时置真；发送中断/失败须复位，不能按调用次数猜测 |
| `guidance_ready` | 制导已初始化、标定和测距有效，输入健康且可运行；目标有距离才允许确认 |
| `FlightRequest::kOffboard` | `sendSetMode(px4_mode::OFFBOARD)` |
| `FlightRequest::kArm` | `sendArm(true)`；状态转换等真实 armed 反馈 |
| `FlightRequest::kLand` | `sendLand()`；停止 Offboard setpoint，等待落地和飞控正常上锁 |
| `strike_confirmed` | 撞击传感器、裁判系统或仿真事件；任务层不从距离推断撞击成功 |

现有 `sendSetMode/sendArm/sendLand` 会等待 ACK，应放到独立命令执行器，避免阻塞 VIO 和 setpoint 流。
`request` 是节流重试的请求，每个请求最多保留一个在途执行；ACK 接受也不等于动作完成。
进入终态或人工接管时，宿主须取消尚未发送的旧任务请求，防止延迟执行旧解锁命令。
第一版没有自动强制上锁，落地后等待飞控自身上锁确认。

## 第一版边界与参考

搜索阶段先保持起飞后的观察点；丢目标后在当前位置悬停并重新确认。
巡航搜索点、偏航扫描、避障、撞板退出动作和实机撞击判据尚未实现。
当前通信 `GuidanceCmd` 没有航向字段，任务层没有假设已能发送偏航扫描。
返航是本地位置直返，不使用依赖 GPS 的 RTL，也不规划障碍物绕行。
接入这些能力时保持任务阶段、制导计算和通信执行三层分开即可。

参考了本机 `repos-for-learning/Fast-Drone-250-master` 的
`src/realflight_modules/px4ctrl/src/PX4CtrlFSM.{h,cpp}`：
借鉴起飞/悬停/命令/降落的阶段划分、起飞点记录及依靠反馈转换的思路。
这里为当前 C++17 工程重新实现，没有引入它的 ROS/MAVROS 控制器或源码依赖。
