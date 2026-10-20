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
//    │                  │                        │ （位置/加速度 被忽略；yaw 待对方   │
//    │                  │                        │   改完后也忽略，见下面 ★）        │
//    └──────────────────┴────────────────────────┴──────────────────────────────┘
//
//  【★ 已确认（2026-10-20，与通信侧口头确认）】
//    **`VELOCITY` 模式后续会忽略 `yaw`** —— 对方会让"速度输入"不再需要一个航向值。
//    背景（为什么需要这个改动）：现在他们 `type_mask` 在 VELOCITY 分支里只忽略
//    `X/Y/Z` 与 `AX/AY/AZ`，**`yaw` 是生效的**（只忽略了 `YAW_RATE`）。⇒ 我们发零速度
//    （`kSafeHover`）时如果不显式填 yaw，默认值 0 会被理解成"转回初始机头方向"
//    （无 GPS 时局部系 x 轴 = 起飞瞬间机头），飞机会在刹车的同时甩头：
//    相机跟着转 → 目标出画 → 检测丢失 → 制导进 SEARCH，而这时我们本来就在降级。
//
//    ⇒ 现在按"**对方会忽略 yaw**"来设计：
//        · `GuidanceOutput::hold_yaw = true`（默认）表示"我不干预航向"，
//          sink 在对方改完之后**不需要再填 `c.yaw`**；
//        · 在对方改完**之前**联调时，若发现机头会转，临时在 sink 里把 `c.yaw`
//          填成"当前航向"（`Communicator::getAttitude()` 的 `yaw`）即可规避；
//        · 若将来真有"控航向"的需求，再把这个字段的语义扩成三态（保持/忽略/指定）。
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
class CsvAccelSink final : public ICommandSink {
public:
    explicit CsvAccelSink(const char *path);
    ~CsvAccelSink() override = default;
    bool send(const GuidanceOutput &out) override;
    bool alive() const override;
};

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
