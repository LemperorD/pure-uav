#pragma once
// =============================================================================
//  合成世界（闭环演示 / 端到端回归用）
//
//  【它是什么】
//    一个类同时扮演三个角色，把整条链路在**零外部依赖**（无 Gazebo / 无 PX4 /
//    无相机 / 无串口）的情况下跑起来：
//
//        IInsSource    : 给出真值 p / v / q
//        ITargetSource : 把真值目标"拍"成像素 (u,v) + 距离 + 时间戳
//        ICommandSink  : 接住制导指令，供下一拍积分用（理想执行机构）
//
//         真值点质量积分（p, v）  ← 用上一拍指令积分
//              │
//              ├─► 世界视线 → 机体 → 相机系 → 针孔投影 ⇒ 像素 (u,v)
//              │                                            │
//              └──────────── GuidanceLoop ◄─────────────────┘
//                         (像素 → 反投影 → 制导核 → NED 加速度)
//
//  ⇒ 除"真值 → 像素"这一步是模拟的，其余全是**真实代码路径**：
//    像素反投影、to_guidance_state、guidance_step、cmd_accel_to_ned、降级/重发。
//
//  【为什么从 main.cpp 挪出来】
//    原来它写在 `main()` 里，测试拿不到 ⇒ "min R = 0.033 m" 只能靠肉眼确认，
//    以后任何一次重构都没人复核。挪进 `app_guidance` 库之后：
//      · `main.cpp` 只剩命令行解析与打印（薄壳）；
//      · `test_guidance` §17 用**同一份代码**跑闭环并断言（回归保护）。
//
//  【⚠ 它是"理想点质量"，不是飞行动力学】
//    直接把指令当实际加速度：没有姿态环、没有阻力、没有重力（a_z=0 就不掉高）。
//    ⇒ 验证的是**制导逻辑**（ω 方向、饱和、HOLD、降级、重发），**不是**飞行性能。
//    真性能只能在 XTDrone2/PX4 或实飞上验。
// =============================================================================

#include <cstdint>

#include "guidance_loop.hpp"   // CameraModel / LoopConfig / GuidanceLoop
#include "sensor_source.hpp"   // IInsSource / ITargetSource
#include "command_sink.hpp"    // ICommandSink

namespace pure_uav {
namespace app {

// ---------------------------------------------------------------- 相机
//
// 相机约定：x 右 / y 下 / z 前（光轴）。外参给出"相机三轴在机体 FRD 下的坐标"：
//   camX(右)     → bodyY(右) = (0,1,0)
//   camY(下)     → bodyZ(下) = (0,0,1)
//   camZ(前/光轴) → bodyX(前) = (1,0,0)
// ⇒ 行主序 R_body_cam = [0 0 1; 1 0 0; 0 1 0]
//
// ⚠ 这是**演示用**的近似值（f=600、主点 640×480 的一半）。真机必须用 VIO 标定结果：
//   `src/vio/extrinsic/results/final/camchain-openvins-final.yaml`（内参）
//   `src/vio/extrinsic/results/final/final_extrinsics.json`（外参）
CameraModel demo_camera();

// ---------------------------------------------------------------- 场景
struct SceneConfig {
    Vec3   p0{0.0, 0.0, -5.0};    // 起点（NED；z=-5 ⇒ 高 5 m）
    Vec3   v0{8.0, 0.0, 0.0};     // 初速（朝 +x = 北 8 m/s）
    Vec3   plate{40.0, 3.0, -5.0};// 装甲板位置（40 m 前方、右偏 3 m、同高度）
    double dt = 0.02;             // 积分步长 = 1/loop_hz
    std::uint64_t t0_us = 1000000ull;  // 起始时间戳（从 1 s 起算，避开 0="未初始化"）
};

// ---------------------------------------------------------------- 合成世界
class SyntheticWorld final : public IInsSource,
                             public ITargetSource,
                             public ICommandSink {
public:
    explicit SyntheticWorld(const CameraModel &cam, const SceneConfig &sc = SceneConfig{});

    // 每拍推进真值（在 loop.run_once 之前调用）：v += a·dt, p += v·dt。
    // 指令只在一拍内有效（下一拍若没新指令就是 0，模拟"指令断流"的极端情况）。
    void step();

    // ---- IInsSource ----
    bool poll(InsSample &out) override;
    bool alive() const override { return true; }

    // ---- ITargetSource：真值目标 → 像素 ----
    bool poll(TargetSample &out) override;

    // ---- ICommandSink：接住制导指令（下一拍积分用）----
    bool send(const GuidanceOutput &out) override;

    // ---- 观测量（演示/断言用）----
    Vec3   pos() const { return p_; }
    Vec3   vel() const { return v_; }
    double true_range() const { return norm(plate_ - p_); }
    const GuidanceOutput &last() const { return last_; }

    int sends() const { return sends_; }   // 收到的指令数（= sink 被调用次数）
    int sat() const { return sat_; }       // 饱和拍数
    int hold() const { return hold_; }     // HOLD 拍数
    int search() const { return search_; } // SEARCH 拍数

private:
    // R^T × v（外参是 相机→机体，投影时要用 机体→相机）
    static Vec3 mat3_mul_transpose(const double R[9], const Vec3 &v);

    CameraModel cam_;
    Vec3   p_, v_, plate_;
    double q_[4] = {1.0, 0.0, 0.0, 0.0};
    double dt_;
    std::uint64_t t_us_;
    Vec3   a_cmd_{};          // 上一拍指令（用于积分）
    GuidanceOutput last_{};
    int sends_ = 0, sat_ = 0, hold_ = 0, search_ = 0;
};

// ---------------------------------------------------------------- 闭环驱动
//
// 把"推进真值 → run_once → 统计"这段循环也挪进库里，让 main 和测试跑**同一份**驱动
// （否则测试里重写一遍循环，就变成"测的是另一段代码"，回归保护会打折）。
struct ClosedLoopReport {
    int ticks = 0;             // 成功跑完的拍数（不含触发终止的那一拍）
    double min_r = 1e18;       // 全程最近距离（m）
    int min_r_tick = -1;       // 最近距离出现在第几拍
    LoopStatus stop = LoopStatus::kOk;  // 终止状态（kOk = 跑满 ticks）
    bool target_lost = false;  // 终止原因是目标离开视野（kSensorTimeout）
};

// 每拍回调（main 用来打印表格；传 nullptr 则不回调）
using TickHook = void (*)(void *user, int k,
                          const SyntheticWorld &world, const GuidanceOutput &out);

ClosedLoopReport run_synthetic_closed_loop(SyntheticWorld &world, GuidanceLoop &loop,
                                           int ticks, TickHook hook = nullptr,
                                           void *user = nullptr);

}  // namespace app
}  // namespace pure_uav
