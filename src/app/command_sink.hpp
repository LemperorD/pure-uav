#pragma once
// =============================================================================
//  指令出口接口
//
//  【契约】
//    `send()` 每拍调一次，**非阻塞、不分配、不 printf**；失败返回 false（由主循环
//    计数并决定是否中止 / 上报），**不要在里面抛异常或重试**。
//
//  【⚠ 四个必须记住的事实】
//    1. **PX4 offboard 的硬要求是"指令持续到达"**（最低 2 Hz，否则退出 offboard；
//       主流实现如 MAVSDK 官方插件是 **20 Hz** 自动重发）。
//       我们建议与主循环同频（约 **50 Hz**），不要"只在有新数据时才发"。
//    2. **纯加速度 setpoint 是纯前馈**：`a_z` 不管就掉高、也不会自动保持位置
//       （仿真里已经踩过同一个坑）。⇒ 真"保命悬停"要发**零速度**（`ControlMode::kVelocity`），
//       走 PX4 速度环闭环。
//    3. 加速度 setpoint 在 MAVLink 里走 `SET_POSITION_TARGET_LOCAL_NED` + `type_mask`
//       忽略位置/速度/yaw，只保留 `af*`。
//    4. ⚠ 它用的是**飞控 local 系（无 GPS 时 = 初始机头对齐、z 朝下）**，
//       见《后续注意问题.md》§4.1。
//
//  【★ 本层不做翻译 —— 翻译在实现层，且只有实现层 include 通信头文件】
//    `ICommandSink` 只认我们自己中立的 `GuidanceOutput`；把它翻译成通信模块的
//    `GuidanceCmd` 是 `mavlink_sink.cpp` 的事（唯一 include `communicator.hpp` / `msg.hpp` 的地方）。
//    理由见 output.hpp 头注释（通信类型演进 / 命名冲突 / 仿真复用 / 单测依赖）。
//
//  【翻译对照表（实现 `mavlink_sink.cpp` 时照抄）】
//    ┌──────────────────┬────────────────────────┬──────────────────────────────┐
//    │ GuidanceOutput   │ 通信侧 GuidanceCmd      │ 填哪些字段                    │
//    ├──────────────────┼────────────────────────┼──────────────────────────────┤
//    │ kAcceleration    │ mode = ACCELERATION    │ afx/afy/afz = a_cmd_ned      │
//    │                  │                        │ （位置/速度/yaw/yaw_rate 被忽略）│
//    │ kVelocity        │ mode = VELOCITY        │ vx/vy/vz = v_cmd_ned（=0 悬停）│
//    │                  │                        │ （位置/加速度/yaw/yaw_rate 全忽略）│
//    └──────────────────┴────────────────────────┴──────────────────────────────┘
//
//  【★ 已落地（2026-10-20，communication 分支 `6ad420b`）】
//    **通信侧已把 yaw 彻底移除**：
//      · `GuidanceCmd` 里删掉了 `float yaw` 字段；
//      · `pushGuidanceCmd()` 的 `type_mask` 对**所有模式**都置了
//        `YAW_IGNORE | YAW_RATE_IGNORE`，pack 时航向传 0。
//    ⇒ 我们发零速度悬停（`kSafeHover`）时**不会再甩头**，也不需要在 sink 里填任何航向。
//
//    ⚠ 实现 sink 时注意两点：
//      1. **不要写 `c.yaw = ...`** —— 该字段在合并 communication 后会消失，写了就编不过；
//         本地工作树目前这份 `communicator.hpp` 还是旧版（还带 yaw），但别依赖它。
//      2. `GuidanceOutput::hold_yaw`（默认 true）现在纯粹是"意图标记"，翻译时什么都不用做。
//
//    背景（为什么这件事重要）：旧版 `type_mask` 在 VELOCITY 分支只忽略位置/加速度，
//    `yaw` 是生效的。若发零速度时 yaw 默认 0，会被理解成"转回初始机头方向"（无 GPS 时
//    局部系 x 轴 = 起飞瞬间机头）⇒ 刹车同时甩头，相机跟着转、目标出画。现已从根上消除。
// =============================================================================

#include "output.hpp"

namespace pure_uav {
namespace app {

class ICommandSink {
public:
    virtual ~ICommandSink() = default;

    // 发出一个周期指令。返回 true = 已成功交给下层（串口写成功 / topic 已 publish）。
    // ⚠ 必须根据 `out.ctrl` 决定发加速度还是发速度（见翻译对照表）。
    virtual bool send(const GuidanceOutput &out) = 0;

    // 通道是否还可用（串口是否打开、心跳是否还在）
    virtual bool alive() const = 0;
};

// ---------------------------------------------------------------- 具体实现（只声明）

// 真机：翻译成通信模块的 `GuidanceCmd{mode=ACCELERATION/VELOCITY, ...}` 并发出去
//   · 唯一 include `communicator.hpp` / `msg.hpp` 的实现类；
//   · 翻译表与 yaw 坑见文件头。
class MavlinkAccelSink final : public ICommandSink {
public:
    explicit MavlinkAccelSink(/* Communicator& */);   // 具体类型在 .cpp 里补
    ~MavlinkAccelSink() override = default;
    bool send(const GuidanceOutput &out) override;
    bool alive() const override;
};

// 仿真：发到 ROS2 topic（XTDrone2 的 cmd_accel_ned），`kVelocity` 时发速度 topic
class Ros2AccelSink final : public ICommandSink {
public:
    Ros2AccelSink() = default;
    ~Ros2AccelSink() override = default;
    bool send(const GuidanceOutput &out) override;
    bool alive() const override;
};

// 离线：只落盘（回放、回归测试、单元测试用；不连任何硬件）
//   ⚠ 2026-10-21：本类搬到 `csv_io.hpp` 了 —— 实现需要持有一个 `std::FILE*`
//     和一个行计数，属于"具体实现"，不该写进契约头文件。这里的空实现仍然有效：
//     `NullSink`（或直接用 `csv_io.hpp` 的 `CsvAccelSink`）。

// 空实现：调试"只想看制导算得对不对、不发指令"时用
class NullSink final : public ICommandSink {
public:
    NullSink() = default;
    ~NullSink() override = default;
    bool send(const GuidanceOutput &out) override;
    bool alive() const override;
};

}  // namespace app
}  // namespace pure_uav
