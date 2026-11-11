# 主任务与 main 接入

任务流程为 **起飞 → 找装甲板 → 撞板 → 返航 → 降落**。
`main` 已通过 `MissionService` 接入任务运行层，共用现有串口、VIO 与检测进程。
默认继续运行定位和检测；显式加 `--mission` 才启动任务。任务开始前等待定位就绪。

## 调用关系

```text
main.cpp（模块启动、固定节拍公共循环、退出清理）
  └─ MissionService::tick()（VIO 发布锁 + 目标接收 + 时间映射）
       └─ MissionRuntime::tick()（反馈快照、预热、控制权仲裁、发送）
            ├─ MissionController::tick()：纯状态机
            ├─ MavlinkMissionPort：main 共用的 Communicator
            └─ MissionGuidanceRunner：只计算，结果交回 Runtime 发送

detect_rt --独立 Unix datagram 通道--> MissionService
VioRuntime::PublicationGuard --同一时钟映射--> 目标 boot_us
```

任务模块不创建后台命令线程，不排队解锁指令。发送模式、解锁和降落请求时，
使用新增的 `request*ForSession()`，只发送消息，不等待 ACK。
任务依靠新鲜的模式、armed、落地和位置反馈推进；反馈一直不满足则阶段超时。
ACK 拒绝尚未单独接入任务原因码，会表现为动作反馈不满足而超时。
原有同步 `send*()` 接口保留，任务路径不调用它们。

## 文件

| 文件 | 用途 |
| --- | --- |
| `mission_types.hpp`、`mission_controller.{hpp,cpp}` | 参数、输入输出、纯状态机 |
| `mission_runtime.{hpp,cpp}` | main 与任务间的运行契约、发送仲裁和故障处理 |
| `mission_config.{hpp,cpp}` | 严格 YAML 参数加载与校验 |
| `../glue/mission_service.{hpp,cpp}` | VIO、检测 IPC 与任务运行层的组合门面 |
| `../glue/mission_ports.{hpp,cpp}` | 真实飞控适配器和制导计算适配器 |
| `../../common/mission_target_channel.hpp` | 本机父子进程目标协议；有界、非阻塞接收 |
| `mission_example.cpp` | 合成反馈演示，包含一次合成撞击事件 |
| `src/test/test_app/test_mission*.cpp`（仓库根目录计） | 状态机、运行层、真实协议回归 |

## 配置与运行

从仓库根目录执行：

```bash
cmake -S . -B build
cmake --build build --target main mission_example test_mission \
  test_mission_runtime test_mission_wire -j4

# 不打开设备：校验主入口的实际任务配置
./bin/main --mission --no-send --check-config

# 相机服务先启动；运行任务调度检查，禁止发送位姿、setpoint 和动作请求
./bin/main --mission --no-send

# 显式启用任务控制：定位就绪后开始自检和起飞
./bin/main --mission --mission-config config/mission.yaml \
  --guidance-config config/onboard-guidance.yaml

# 独立合成演示，不打开设备
./bin/mission_example
ctest --test-dir build -R '^mission(_example|_runtime|_wire)?$' --output-on-failure
```

`mission_wire` 使用本地 Unix socket 和 PTY，需要运行环境允许相应系统调用。

`config/mission.yaml` 管理任务参数，未知字段、非有限值及非法范围会在打开设备前报错。
`--mission-config`、`--guidance-config` 中的相对路径从仓库根目录解析。
`--check-config` 只有配合 `--mission` 才加载这两份配置。
`--no-send` 不伪造发送成功，因此不会达到 Offboard 预热条件，也不会自动解锁。
通信模块自身的心跳和遥测订阅请求仍会运行。

默认连续确认 3 个新目标帧、起飞高度 1.5 m、搜索超时 30 s、撞板超时 5 s。
Offboard 预热默认要求连续成功发送 1.5 s，中间超过 0.2 s 没发送则重新起算。
主入口与任务适配器统一支持 PX4；其他固件在连接相机前即被主入口拒绝。
`--check-config` 会打印 `flight_stack: PX4`，飞控参数不会被程序自动修改。

运行中向 `main` 的 PID 发送 `SIGUSR1` 可请求返航；Ctrl-C / SIGTERM 停止程序并释放输出。
任务完成或中止会结束本次 `main`，关闭检测、VIO 和通信。

## 状态与反馈

```text
Idle → Preflight → Offboard → Arming → Takeoff → Search → Strike
                                                    ↑        │
                                                    └─丢目标─┘
Strike --撞击确认--> ReturnHome → Landing → Completed
Search / Strike --超时--> ReturnHome
```

- 起飞条件：新鲜定位、未解锁、真实落地反馈及静止条件通过。
- 起飞完成：到达起飞点上方、低速持续稳定，并且飞控反馈已离地。
- 撞板开始：连续新帧、类别/置信度/帧龄/测距满足条件，制导已初始化。
- 撞板期间：只运行制导计算；Runtime 独占指令发送。丢目标立即定点等待，超时回搜索。
- 返航：返回起飞点上方；若未离地就取消则直接交飞控降落。
- 降落完成：`EXTENDED_SYS_STATE` 确认在地面且心跳确认已上锁，不能用高度代替。
- 掉线、定位过期、人工接管、发送失败、意外退出 Offboard：停止输出并中止任务。
- 飞控会话变化或 VIO 定位基准重建：中止本轮任务，不沿用旧返航点。
- `Completed` 表示流程结束；必须另外看 `strike_confirmed`，超时返航不会伪造成功。

Runtime 先检查状态，再发送本拍唯一的任务或制导指令；模式/动作请求也在同拍执行。
停止以后没有尚待执行的后台请求；已发到飞控的命令不能被撤回，后续依靠飞控接管策略。
实际串口写还会校验期望的飞控会话，重连后旧指令不能发给新会话。

## 坐标、时间和进程协议

任务使用飞控 `LOCAL_POSITION_NED` 的本地 NED，单位米/米每秒，z 向下。
自检通过记录起飞位置；例如起飞点 z=-0.4 m、起飞高度 1.5 m，目标 z=-1.9 m。
VIO 只提供导航健康和时间基准门控，不把原生 FRD/ENU 位姿直接作为 NED 指令。

`MissionService` 持有 `VioRuntime::PublicationGuard` 直到本拍最终发送，
定位重建不能越过检查。目标采样时刻通过同一 `FcuClock` 映射，额外保留单调时钟年龄。
飞控姿态/位置接收时刻在缓存锁内读取；重复读取或相同 boot 时间戳不刷新位置年龄。

开启任务后，main 创建私有 `socketpair`，通过 `--mission-target-fd` 传给检测子进程。
仅发布端 fd 在 exec 时保留；监督线程重启子进程时继续使用同一通道。
消息带协议版本、采集单调时钟、序号、主目标像素、置信度、类别和可选测距。
消息通道不解析日志，不传相机大图，单拍最多处理 64 条；错误长度、旧/未来帧和非法值会丢弃。

## 当前能力边界

**main、任务、飞控和检测之间的运行通路已实现；实机撞板所需的数据仍需补齐：**

- 当前检测发布像素框，没有测距，协议明确 `has_range=false`；不会凭空估算距离。
- `config/onboard-guidance.yaml` 尚无完整彩色相机内参和机体外参，不能用 VIO 灰度标定直接冒充。
  未配置时 `guidance_ready=false`，仍可检查起飞/搜索/超时返航，不进入撞板。
- `MissionService::tick(return_requested, manual_takeover, strike_confirmed)` 已留操作者和撞击事件接口。
  main 当前只接了返航信号；撞击传感器/裁判事件和独立操作者接管信号还没有真实来源。
  飞控模式退出 Offboard 已能通过反馈触发控制权释放。
- 搜索先定点观察；巡航搜索、偏航扫描、避障、目标身份关联和撞击退出动作待实现。
- 返航先走本地位置直返，不规划绕障，也不调用依赖 GPS 的 RTL。

制导进入/重入时会重新构造 `GuidanceLoop` 并 `init()`，清空旧视线差分记忆。
制导计算使用空发送出口，所有实际指令统一回 Runtime 仲裁；不存在另一个制导线程继续发包。

初版参考了本机 Fast-Drone-250 的 `PX4CtrlFSM.{h,cpp}` 的阶段划分和反馈转换思路，
这里为当前 C++17 工程实现，没有引入 ROS/MAVROS 控制器源码。
