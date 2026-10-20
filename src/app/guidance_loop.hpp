#pragma once
// =============================================================================
//  GuidanceLoop —— 机载主循环的"外壳"（面向对象的那一层）
//
//  【★ 这是哪个"主循环"】
//    本类是**运行在伴飞计算机（香橙派）上的用户态进程循环**，也就是 `main()` 里那个 while。
//    它和系统里其它循环的关系：
//
//      ① PX4 姿态/角速率环   ~1 kHz   飞控内部，**不归我们管**
//      ② PX4 位置/速度环    ~100 Hz  飞控内部，**不归我们管**
//      ③ 通信接收线程       事件驱动  收一帧处理一帧（姿态/位置被请求到约 50 Hz）
//      ④ 视觉/检测          ~30 Hz    相机帧率，检测侧
//      ⑤ ★ 本循环 loop_hz   **我们**：每 1/loop_hz 秒做一次"取数→算制导→发指令"
//
//    ⇒ 我们是**外环**：只输出"期望加速度（或零速度）"，由 ①② 负责变成姿态/力矩。
//      所以 loop_hz 提高**不会**让飞机飞得更稳，只让"我们的期望"更新得更勤。
//      也正因为是外环，我们的回路带宽只有 0.5~2 Hz（t_go ≈ 0.5~2 s），
//      50 Hz 采样已有 25~100 倍裕度（依据见《后续注意问题.md》§6 的实测数据）。
//
//  【架构：函数式内核 + 面向对象外壳】
//      外壳（本类）   ：取数据 → 换算 → 调核 → 体检 → 发指令 → 统计
//      内核（纯函数） ：adapter::to_guidance_state() → guidance_step() → cmd_accel_to_ned()
//    内核**一行不改**（铁律 §1.5），测试仍按 1439 条断言逐例验证；
//    本类只负责"有状态、有生命周期、可替换数据源"的那部分脏活。
//
//  【一拍做什么（run_once 的顺序，不允许颠倒）】
//      ① poll 两个数据源（拿"最近一帧"；没有新帧就复用上一帧 —— 零阶保持）
//      ② 目标：像素 (u,v) → 相机系 → 机体 → 世界（用本帧 `q`；内参/外参见 LoopConfig::cam）
//      ③ 组装 adapter::RawFrame（外部口径 → 内部口径的唯一换算点）
//      ④ quick_check()（几 ns、零分配）—— 不健康就按 `on_unhealthy` 降级
//      ⑤ guidance_step()（纯函数，约 7 ns）
//      ⑥ adapter::cmd_accel_to_ned()（唯一一次出口换算）
//      ⑦ sink->send()（PX4 需要"持续到达"⇒ 每拍都发，哪怕数据没更新）
//      ⑧ 统计（计数、延迟、饱和率）
//
//  【实时性约束（写实现时必须遵守）】
//    · 循环内**零分配、零 printf、零 std::string**；日志写预分配环形缓冲，循环外落盘；
//    · 节拍用**绝对时间睡眠**（`clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)`），
//      不要 `std::this_thread::sleep_for`（会累积漂移）；
//    · 时钟用**单调时钟**；数据时间戳按 `boot_us` 约定（见 sample.hpp / §3.1）；
//    · 不用异常：构造/初始化用 `bool init()` + `LoopStatus` 表达失败。
//
//  【本文件目前只有声明】
//    实现放在 `guidance_loop.cpp`；落地后再把 `src/app/CMakeLists.txt` 改成
//    链接 `guidance_flight`（= guidance_core + state_adapter + mavlink_units）。
// =============================================================================

#include <cstdint>

#include "command_sink.hpp"
#include "guidance_types.hpp"   // GuidanceParams / GuidanceMemory
#include "output.hpp"
#include "sample.hpp"
#include "sensor_source.hpp"
#include "state_adapter.hpp"    // adapter::StateTimeTracker

namespace pure_uav {
namespace app {

// ---------------------------------------------------------------- 相机模型
//
// 目标数据是**像素**，所以主循环需要"像素 → 世界单位矢量"的换算参数。
// 内参/外参**都有现成标定结果**（VIO 分支）：
//   · 内参：`src/vio/extrinsic/results/final/camchain-openvins-final.yaml`
//   · 外参：`src/vio/extrinsic/results/final/final_extrinsics.json`
// 落地时从配置加载，代码里不写死。
struct CameraModel {
    // 针孔内参
    double fx = 0.0, fy = 0.0;
    double cx = 0.0, cy = 0.0;

    // 外参：相机系 → **机体**系的旋转矩阵（行主序 9 个 double，R_body_cam）
    // 之后再乘 `R(q)`（机体→世界）才能得到世界系视线。
    double R_body_cam[9] = {1, 0, 0,
                            0, 1, 0,
                            0, 0, 1};

    bool valid() const { return fx > 0.0 && fy > 0.0; }
};

// ---------------------------------------------------------------- 配置
struct LoopConfig {
    // 主循环频率。**默认 50 Hz**，依据：
    //   · PX4 的硬要求只是"指令持续到达"（>2 Hz 否则退出 offboard）；
    //   · MAVSDK 官方 Offboard 插件默认 20 Hz 自动重发（业界主流量级）；
    //   · 我们的回路带宽 0.5~2 Hz ⇒ 50 Hz 有 25~100 倍裕度；
    //   · 末端交会 t_go≈0.5 s，20 ms 一拍 ≈ 4%（20 Hz 则 10%，偏粗）；
    //   · 与仿真 `phase_b_guidance.py --rate 50` **一致**，参数不用重调。
    // 可选：20（最保守）/ 100（更平滑）/ 250（I/O 时间只剩 4 ms，不推荐）。
    int loop_hz = 50;

    // 一帧数据"太旧"的阈值：超过就判 kStale（不拿过期数据做制导）。
    // 视觉常见 30 Hz（33 ms），这里给足余量；100 ms 已经能容忍丢一帧。
    double max_sample_age_s = 0.10;

    // 连续这么久没有新帧 ⇒ kSensorTimeout（传感器掉线，通常是串口/相机问题）
    double sensor_timeout_s = 0.50;

    // ★ 输入不健康（NaN / 未单位化 / 负距离 / 过期）时的策略。
    //
    //   分层兜底的思路（业界通行）：
    //     第 1 层（本类）：输入体检 + 发**安全的合法 setpoint**（不要断流！）
    //     第 2 层（飞控）：offboard 失联 failsafe（`COM_OF_LOSS_T` → `COM_OBL_ACT`）
    //                     必须在飞控侧**单独配置**，我们替代不了
    //     第 3 层（飞手）：RC override
    enum class OnUnhealthy : std::uint8_t {
        // 保持上一拍的加速度指令（平滑，不中断 offboard）。**受 hold_last_max_s 限时**
        kHoldLastCommand = 0,
        // ★ 主力降级：发**零速度** setpoint（`ControlMode::kVelocity`, v=0），
        //   交给 PX4 速度环把飞机刹住并近似悬停。
        //   ⚠ 注意：零加速度**不是**悬停（纯前馈会掉高），所以这里必须是"零速度"而不是"零加速度"。
        kSafeHover = 1,
        // 直接不发（返回 kUnhealthyInput，把主动权交给飞控 failsafe）。
        // ⚠ 飞行中慎用：PX4 收不到 setpoint 会在超时后掉出 offboard。
        kStop = 2,
    };
    OnUnhealthy on_unhealthy = OnUnhealthy::kHoldLastCommand;

    // `kHoldLastCommand` 的最长时间：超过就自动转 `kSafeHover`（防止"抱着错指令一直推"）
    double hold_last_max_s = 0.20;

    // 完整体检（check_guidance_state，约 69 ns、失败才建字符串）的降频执行频率；
    // 0 = 从不。回路内永远只用 quick_check（约 7 ns）。
    double full_check_hz = 2.0;

    // 是否每拍都发指令（PX4 offboard 需要持续到达 ⇒ 默认 true）
    bool send_on_every_tick = true;

    // 相机模型（目标数据是像素，必须给；`valid()==false` 时无法制导）
    CameraModel cam;
};

// ---------------------------------------------------------------- 统计
//
// 只做累加，不打印。落地时建议再加一个"环形缓冲日志"，异常前后各留 N 秒。
struct LoopStats {
    std::uint64_t ticks        = 0;   // 主循环拍数
    std::uint64_t ins_frames   = 0;   // 收到多少帧飞控数据
    std::uint64_t tgt_frames   = 0;   // 收到多少帧目标数据（`present=true` 的才计）
    std::uint64_t sent         = 0;   // 成功发出的指令数
    std::uint64_t saturated    = 0;   // 撞 a_max 的拍数
    std::uint64_t hold         = 0;   // 末端冻结拍数
    std::uint64_t search       = 0;   // 目标丢失拍数
    std::uint64_t unhealthy    = 0;   // 输入体检不通过拍数
    std::uint64_t safe_hover   = 0;   // 降级为"零速度悬停"的拍数
    std::uint64_t sink_errors  = 0;   // 发送失败次数
    std::uint64_t max_latency_us = 0; // 采样时刻 → 发出时刻 的最大值（用于看时延）
};

// ---------------------------------------------------------------- 主循环
class GuidanceLoop {
public:
    // 依赖注入：参数 + 配置 + 三个外部组件（数据源×2 + 出口）。
    // ⚠ 只持有引用，**不接管生命周期**；对象必须在 loop 之前构造、之后析构。
    GuidanceLoop(const GuidanceParams &params, const LoopConfig &cfg,
                 IInsSource &ins, ITargetSource &tgt, ICommandSink &sink);
    ~GuidanceLoop() = default;

    GuidanceLoop(const GuidanceLoop &) = delete;
    GuidanceLoop &operator=(const GuidanceLoop &) = delete;

    // 构造即有效：校验参数与相机模型、复位时基、清零统计。失败返回 false。
    bool init();

    // 跑一拍。成功返回 kOk；需要上层介入时返回对应状态码。
    // `out` 可空 —— 调试/回放时想拿到本拍结果就传指针。
    LoopStatus run_once(GuidanceOutput *out = nullptr);

    // 阻塞跑主循环（内部按 loop_hz 定时，绝对时间睡眠）。
    LoopStatus run();

    // 请求停止（信号处理 / 上层安全逻辑调用；实现里用 std::atomic<bool>）
    void request_stop();

    const LoopStats &stats() const { return stats_; }
    bool initialized() const { return inited_; }

private:
    // ---- 外部依赖（引用，不拥有）----
    const GuidanceParams params_;
    LoopConfig           cfg_;
    IInsSource    &ins_;
    ITargetSource &tgt_;
    ICommandSink  &sink_;

    // ---- 内核状态 ----
    GuidanceMemory memory_{};                 // 制导核的跨周期记忆（纯函数要求调用方持有）
    adapter::StateTimeTracker tracker_{};     // 时钟：**只喂飞控帧**（见 §3.1 时基冲突）

    // ---- 最近一帧采样（"零阶保持"：没有新帧就复用）----
    InsSample    ins_now_{};
    TargetSample tgt_now_{};
    bool have_ins_ = false;
    bool have_tgt_ = false;
    GuidanceOutput last_out_{};               // 上一拍输出（kHoldLastCommand 策略要用）
    double last_health_s_ = 0.0;              // 上次完整体检的时刻（降频用）

    // ---- 生命周期 ----
    bool inited_         = false;
    bool stop_requested_ = false;             // 实现里应为 std::atomic<bool>

    LoopStats stats_{};
};

}  // namespace app
}  // namespace pure_uav
