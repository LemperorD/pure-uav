// VioHealth：运行时失效判定 + 恢复状态机。
//
// 背景（session29 实测）：视觉断供 15 秒后，滤波器把速度积分到 3.3 m/s 还照发，
// 谁也不知道它是错的，尺度一路偏到 5 倍。这个模块的唯一职责是：
//   **在它开始骗人的时候说"别信我"**，而不是让它变准。
//
// 判据（都可以实测，不依赖真值）：
//   features        —— 这一帧跟踪器里还剩多少特征
//   msckf_updated   —— 这一帧有没有真的产生一次 MSCKF 更新
//                     （有特征但三角化全失败 = 没更新，这才是真正的"没进滤波器"）
//
// 状态：
//   UNKNOWN  还没收到有效帧
//   OK       有特征、近期有更新
//   DEGRADED 特征数掉到 min_features 以下，但还没到 lost_after → 仍在发，标记降级
//   LOST     特征不足持续超过 degrade_after，或已经 lost_after 秒没有任何更新
//            → **立刻停发**，并准备恢复
//
// 滞回：恢复要求特征数连续 recover_after 秒 ≥ min_features_recover 且期间至少有一次更新，
// 避免在悬崖边上反复抖动。

#pragma once

#include <cstdint>
#include <vector>

namespace pure_uav {

enum class VioHealthState { UNKNOWN, OK, DEGRADED, LOST };

const char* toString(VioHealthState s);

struct HealthConfig {
    bool enabled = true;

    // 特征数阈值（跟踪器里还在跟的特征）
    int min_features = 60;          // 低于这个算"特征不足"
    int min_features_recover = 90;  // 恢复要求更高，形成滞回

    // 更新率阈值（滑窗）：这一窗口内真正产生 MSCKF 更新的帧占比。
    // 这是比"距上次更新多久"更靠谱的判据 —— session29 的 10~11 s 秒级只有 1 次更新，
    // "距上次更新"永远不到 1.5 s，但视觉其实已经基本没进滤波器了。
    double update_rate_window_s = 2.0;
    double min_update_rate = 0.15;      // 低于 15% 算"视觉没进滤波器"
    double min_update_rate_recover = 0.35;

    // 时间阈值（秒）
    double degrade_after_s = 0.4;   // 特征不足持续多久 → DEGRADED
    double lost_after_s = 1.5;      // 多久没有 MSCKF 更新 / 持续特征不足 → LOST

    // 【2026-10-25】深度一致性门限。这是唯一能抓到"稳定但错"的指标：
    //   run4 全程 0 次重建、特征 169–237（健康门限全过），但 deperr 一直 17–23 px，
    //   速度从 0.65 爬到 1.61 —— 输出的是"看起来连续、其实是错的"轨迹。
    //   好数据实测 1.3–4 px，坏数据 9–23 px，界线很清楚。
    double deperr_degrade_px = 12.0;   // 近 2 s 深度一致性超过它 → DEGRADED（仍发布，标记不可信）
    double deperr_lost_px = 15.0;      // 超过它并持续 deperr_lost_s → LOST（停止发布）
                                       // 实测：好数据 1.3–4 px、run3 峰值 9.4、run4 全程 17–23
    double deperr_lost_s = 2.0;

    // 【2026-10-22】物理合理性保护（纯安全项，只会有正面效应）：
    // 实测过"健康门限报 OK、但估计速度从 0.18 一路爬到 4.9 m/s、闭环误差 94%"的
    // 静默发散。这种位姿发出去比不发达危险，所以这里按物理上限硬拦：
    //   - 位姿出现 NaN/Inf      → 立即判 LOST（绝不外发）
    //   - |v| > max_speed_mps 持续 max_speed_hold_s → 判 LOST
    // 手持/室内无人机正常速度都远低于 3 m/s；真高速飞行时把这个值调大即可。
    double max_speed_mps = 3.0;
    double max_speed_hold_s = 0.5;

    // 【2026-10-25】重力泄漏门限（只在准静止时刻可测，见 VioEstimator::tiltCheck）。
    //
    // 为什么必须有这条：turn4/turn5 实测"人完全静止、面板上 tilt err 10.8°"——姿态错了
    // 10.8°，就有 g·sinθ = 1.84 m/s² 的重力没被扣掉，被当成真实加速度积分。实测速度
    // 13 秒从 0.13 涨到 5.92 m/s、位置飘 32.7 m，而这 13 秒里健康状态是 DEGRADED（照发）。
    // 这条是纯粹的"别信我"开关：泄漏超过 tilt_lost_deg 并持续 tilt_lost_s → 停发。
    // 参考刻度：倾角差 1° ↔ 0.17 m/s² 假加速度；5.7° ↔ 1 m/s²。
    double tilt_degrade_deg = 5.0;   // 超过 → DEGRADED（仍发，标记不可信）
    double tilt_lost_deg = 8.0;      // 超过并持续 tilt_lost_s → LOST（停发）
    double tilt_lost_s = 1.0;
    double recover_after_s = 1.0;   // 恢复需要连续正常多久

    // 挂掉后恢复时是否重建估计器（否则只把状态改回 OK，轨迹会带着跑偏的状态）。
    // 默认关：重建是重操作，必须先确认阈值不会误报；实时程序用 --reinit-on-recover 打开。
    // 【2026-10-21】默认打开：卡方门限一旦锁死（更新率 0/30），没有重建就再也回不来。
    // 实测手持数据：关闭时 37 秒死锁、终点 321 m；打开后 8 秒自愈、终点 1.97 m。
    bool reinit_on_recover = true;
    // 【2026-10-21】"软恢复"（LOST 之后自己回来了）是否也重建。
    // 默认 **false**：短时 LOST 期间滤波器的状态一直都在，恢复后继续用就行，
    // 重建反而把状态和锚点都丢掉、还要重新走动态初始化（2–3 秒）。
    // 实测 clean1（88 s）：打开时重建 **11 次**、原因几乎全是"曾经 LOST 后恢复"，
    // 等于每 8 秒重启一次 —— 这是可用率低的真正原因，不是卡方门限。
    // 硬恢复（LOST 持续超过 force_reinit_after_s）仍然会重建，那条是必要的。
    bool reinit_on_soft_recover = false;

    // 【2026-10-21】快速自救（比重建轻得多）：LOST 持续 inflate_after_s 就把协方差放大一次。
    // 卡方门限锁死时，更新率恒 0、状态开环发散；把 P 放大 => S 变大 => 门限重新打开，
    // 测量立刻能重新进来。实测比"重建估计器"快一个数量级，且不用重新初始化、不丢锚点。
    // 【实测结论】默认关闭：放大 P 让第一次恢复快 0.26 秒，但整体 OK 帧占比从 40% 掉到 32%、
    // 15-25s 的更新率从 3.10 掉到 1.60——增益太大会把状态推过头，然后更快再锁死。
    // 想复现就设 true（或改这里）。
    bool inflate_enable = false;
    double inflate_after_s = 0.5;    // LOST 持续这么久就放大一次
    double inflate_scale = 100.0;    // 方差放大倍数（sigma ×10）
    int max_inflate_times = 3;       // 最多连续放大几次；还不行就交给 force_reinit 重建

    // LOST 持续这么久还没自己恢复 → 强制重建估计器重新初始化。
    // 必须有这条：实测 session29 从 18 s 起特征还有 130~220 个，但 MSCKF 更新恒为 0，
    // "等它自己恢复"是等不到的 —— 更新管线已经卡死，只有重建能退出这个状态。
    double force_reinit_after_s = 3.0;
    // 两次重建之间的最小间隔，避免抖动时反复重建
    double min_rebuild_interval_s = 2.0;
};

struct HealthReport {
    VioHealthState state = VioHealthState::UNKNOWN;
    bool publishable = false;       // 是否应该把位姿发给飞控
    int features = 0;               // 最近一帧的特征数
    bool updated_this_frame = false;
    double since_update_s = 0.0;    // 距离上一次 MSCKF 更新多久
    double since_state_change_s = 0.0;
    int recoveries = 0;             // 已经重初始化几次
    int lost_count = 0;             // 进入 LOST 的次数
};

class HealthMonitor {
public:
    HealthMonitor() = default;
    explicit HealthMonitor(const HealthConfig& cfg);

    void configure(const HealthConfig& cfg);
    const HealthConfig& config() const { return cfg_; }

    // 每来一帧相机调用一次。time_s 必须单调递增（相机时间域）。
    /// @param visual_updated 这一帧是否产生了**任意**视觉 EKF 更新（MSCKF 或 SLAM）。
    ///        注意：不要只传 MSCKF 的更新标志 —— max_slam>0 时只走 SLAM 的帧也是有效视觉信息。
    /// @param tilt_deg 准静止时刻"滤波器重力方向 vs 加速度计"的夹角（度）；<0 表示本帧测不准（在动）
    void update(double time_s, int features, bool visual_updated, double deperr_px = -1.0,
                double tilt_deg = -1.0);

    /// 物理合理性违规时强制判失效（NaN/Inf 或速度超出物理上限）
    void mark_implausible(double time_s);

    VioHealthState state() const { return state_; }
    bool publishable() const { return state_ != VioHealthState::LOST && state_ != VioHealthState::UNKNOWN; }
    HealthReport report(double now_s) const;

    // 恢复动作由上层执行（重建估计器 + 重锚定），执行完调用这个把状态切回 OK。
    void notify_recovered();

    void reset();

private:
    HealthConfig cfg_;
    VioHealthState state_ = VioHealthState::UNKNOWN;

    // 滑窗更新率：窗口内每帧是否有更新
    std::vector<double> win_time_;
    std::vector<unsigned char> win_updated_;
    double updateRate() const;
    void pushWindow(double t, bool updated);

    double start_time_ = -1.0;
    double last_time_ = -1.0;
    double last_update_time_ = -1.0;   // 上一次真正的 MSCKF 更新
    double bad_since_ = -1.0;          // 特征不足的起点
    double bad_dep_since_ = -1.0;      // 深度一致性崩掉的起点
    double bad_tilt_since_ = -1.0;     // 倾角差超限的起点
    double good_since_ = -1.0;         // 恢复候选的起点
    double state_changed_at_ = -1.0;

    int features_ = 0;
    bool updated_ = false;
    int recoveries_ = 0;
    int lost_count_ = 0;
    bool need_recover_ = false;        // LOST 之后、尚未完成重建
};

}  // namespace pure_uav
