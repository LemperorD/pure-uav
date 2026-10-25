#pragma once
// =============================================================================
//  制导模块对外的输出契约
//
//  【★ 本层不碰通信模块的任何类型】
//    这里只定义**我们自己的**中立输出 `GuidanceOutput`；把它翻译成通信模块的
//    `GuidanceCmd{mode=..., afx/afy/afz 或 vx/vy/vz}` 是**实现层**（`mavlink_sink.cpp`）
//    的事 —— 那是唯一 include `communicator.hpp` / `msg.hpp` 的地方。
//    理由（都是实际踩过的）：
//      ① 通信侧类型还在演进（刚把结构体从 communicator.hpp 搬到 msg.hpp）；
//      ② 两边的 `GuidanceCmd/GuidanceMode` 曾经同名同命名空间，**让整个仓库编译不过**；
//      ③ 仿真侧没有 `GuidanceCmd`（走 ROS2 topic），直连就没法仿真/真机共用一套 loop；
//      ④ 单测不需要拖进 `third_party/mavlink`。
//
//  【两个容易混的"模式"，不要混用】
//    · `PureGuidanceMode`（PN / APN / HOLD / SEARCH）—— **导引律层面**：我这拍是怎么算的；
//    · `ControlMode`（kAcceleration / kVelocity）    —— **控制指令层面**：我发的是什么量。
//    例：目标丢失（PureGuidanceMode = SEARCH）时，控制层既可以发零加速度、
//        也可以发零速度（kSafeHover，见下）—— 两者是正交的两件事。
//
//  【坐标系：已经是 NED】
//    内部（制导核）用 ENU；`GuidanceLoop` 在出口处调用 `adapter::cmd_accel_to_ned()`
//    完成**唯一一次**换算。⇒ 上层拿到的 `a_cmd_ned` 直接下发即可，**严禁再转一次**。
//
//  【为什么需要 `kVelocity` / 零速度兜底】
//    `HOLD` 是"保持上一拍**加速度**"，`SEARCH` 是"输出**零加速度**"，但
//    **零加速度 ≠ 悬停**：纯加速度是纯前馈，`a_z = 0` 就会掉高（仿真里踩过）。
//    真正的"保命悬停"要发**零速度 setpoint**，交给 PX4 的速度环闭环 ⇒ `kVelocity` + `v_cmd = 0`。
// =============================================================================

#include <cstdint>

#include "guidance_types.hpp"   // PureGuidanceMode（本模块自有类型，不涉及外部依赖）
#include "vec3.hpp"

namespace pure_uav {
namespace app {

// ---------------------------------------------------------------- 输入健康
//
// 由 `quick_check()`（几 ns、零分配）在回路内判定；不通过时按 `LoopConfig::on_unhealthy`
// 的配置降级。完整版 `check_guidance_state()`（约 69 ns、失败才建字符串）只在异常或
// 降频（1~5 Hz）时调用，用于黑匣子。
enum class InputHealth : std::uint8_t {
    kOk = 0,            // 通过
    kNotFinite,         // t / los / range / v_M / q 里有 NaN / Inf
    kLosZero,           // 视线是零矢量（没有方向可言）
    kLosNotUnit,        // |los| 偏离 1（上游没单位化）
    kQuatNotUnit,       // |q| 偏离 1（上游没归一化）
    kRangeNegative,     // 距离为负
    kStale,             // 数据太旧（超过 LoopConfig::max_sample_age_s）
    kTimeBackwards,     // 时间戳倒退 / 跨源（见《后续注意问题.md》§3.1 时基冲突）
};

// ---------------------------------------------------------------- 控制指令层面
enum class ControlMode : std::uint8_t {
    kAcceleration = 0,   // 发加速度：`a_cmd_ned` 有效（正常制导）
    kVelocity     = 1,   // 发速度：  `v_cmd_ned` 有效（零速度兜底 / 悬停）
};

// ---------------------------------------------------------------- 输出
struct GuidanceOutput {
    // ★ 本指令对应的采样时刻（飞控 boot 时钟 µs），方便上层做延迟统计
    std::uint64_t boot_us = 0;

    // ★ 控制指令层面：到底发什么（见 ControlMode）
    ControlMode ctrl = ControlMode::kAcceleration;

    // ★ 期望加速度（NED，m/s²）—— 已过 ENU→NED 换算；`ctrl == kAcceleration` 时有效
    Vec3 a_cmd_ned;

    // ★ 期望速度（NED，m/s）—— `ctrl == kVelocity` 时有效；零速度兜底 = {0,0,0}
    Vec3 v_cmd_ned;

    // ★ 航向策略：true = "我不干预航向"（默认）。
    //   communication 分支 `6ad420b` 已把所有模式的 yaw 忽略，且 `GuidanceCmd` 删掉了
    //   `yaw` 字段 ⇒ sink 不需要（也不能）填航向。该字段保留为"意图标记"。
    bool hold_yaw = true;

    double a_cmd_norm = 0.0;          // |a_cmd_ned|（限幅后）；kVelocity 时为 0
    bool   sat        = false;        // 本周期是否撞到 a_max（"要的加速度给不出"）

    // 导引律层面的模式与有效性（语义见 guidance_loop.hpp / README）
    PureGuidanceMode mode  = PureGuidanceMode::SEARCH;
    bool             valid = false;   // 本周期输入是否可用于制导

    // 调试 / 监控量
    double t_go    = 0.0;             // 剩余飞行时间（s）
    double Vc      = 0.0;             // 接近速度（m/s）
    double R       = 0.0;             // 相对距离（m）
    double lam_dot = 0.0;             // 视线角速率模长（rad/s）

    InputHealth health = InputHealth::kOk;   // 本帧体检结果
};

// ---------------------------------------------------------------- 主循环状态码
//
// 机载不用异常 ⇒ 全部用返回码表达失败原因（构造即有效 + 显式错误码）。
enum class LoopStatus : std::uint8_t {
    kOk = 0,
    kNotInitialized,      // init() 没成功就调 run/run_once
    kStopped,             // 已收到 request_stop()
    kSensorTimeout,       // 超过 timeout 仍拿不到新帧（传感器掉线）
    kUnhealthyInput,      // 输入连续不健康且策略要求中止
    kSinkError,           // 指令发不出去（串口断 / 写失败）
    kEstimatorReset,      // 估计器重置 ⇒ 坐标系跳变（无 GPS 时必须监控，见 §4.1）
};

}  // namespace app
}  // namespace pure_uav
