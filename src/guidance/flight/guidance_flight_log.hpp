#pragma once
// =============================================================================
//  guidance_flight_log.hpp —— 制导中间量日志（可观测性外壳，不参与制导计算）
//
//  为什么放在 flight/ 而不是 app/：
//    · 它只消费 GuidanceOutput / GuidanceParams / GuidanceLoopStats（本模块类型）；
//    · 机载入口（MissionGuidanceRunner）和仿真入口可以共用同一份格式；
//    · 格式与推导可单测，不需要起真实数据源。
//
//  为什么不需要改制导核：
//    GuidanceOutput 已经有 û（los_hat_ned）、‖ω‖（lam_dot）、R、Vc、t_go；
//    本文件用**相邻两次计算拍**的 û 差分还原 ω 矢量与未滤波模长：
//        dlos  = û_k − û_{k−1}
//        dt    = boot_us_k − boot_us_{k−1}          （目标帧时戳之差）
//        w_raw = ‖dlos‖ / dt
//        omega = û_k × (dlos / dt)
//    叉乘对旋转不变（R(a×b) = Ra×Rb），所以 NED 下算与内核 ENU 下算数值相同。
//
//  三个“与内核口径一致”的细节（否则数据对不上）：
//    1) 只有真的重算的拍才参与差分——重复帧的 boot_us 不变，直接跳过；
//    2) 无效拍（valid=false 或 health!=kOk）清空差分历史，与内核 SEARCH 分支清 memory 对齐；
//    3) dt 用目标帧 boot_us 之差，和内核 tracker 累加出来的 Δt 同源。
//
//  抽稀规则：按**目标帧时戳**（boot_us）做时间抽稀，相邻两行至少间隔 1/rate_hz 秒。
//    不按“计算拍计数”抽稀，是因为计算拍频率随检测帧率变化（10/30 Hz），
//    计数抽稀会让 rate_hz 的实际频率随配置漂移。
//    重复重发帧（boot_us 相同）直接跳过，否则会算出 dt=20 ms、dlos=0 的假 ω。
//
//  唯一的 I/O 入口是本类的 record/event；GuidanceLoop 内仍然零 I/O。
// =============================================================================

#include <cstdint>
#include <cstdio>
#include <string>

#include "guidance_flight_types.hpp"  // GuidanceOutput / GuidanceInputHealth
#include "guidance_loop.hpp"          // GuidanceLoopStats
#include "guidance_types.hpp"         // GuidanceParams / GuidanceLaw
#include "vec3.hpp"

namespace pure_uav::guidance::flight {

// YAML 键：log.enabled / log.rate_hz / log.file（见 assets/docs/app/usage.md §5）。
struct GuidanceLogConfig {
    bool enabled = false;   // 总开关；false = 不输出、不建文件
    double rate_hz = 10.0;  // 每秒最多输出行数（按目标帧时戳抽稀）；<=0 也视为关闭
    std::string file;       // 空 = 只打 stdout；非空 = 同时写 CSV（追加、自动补表头）
};

// 相邻两拍推导出的观测量（也是单测的直接输入）。
struct GuidanceLogDerived {
    Vec3 dlos;               // û_k − û_{k−1}（NED，无量纲）
    Vec3 omega;              // 视线角速率矢量（NED，rad/s）
    double dt_s = 0.0;       // 差分间隔（s）；0 表示本拍没有可用差分
    double w_raw = 0.0;      // ‖ω‖（rad/s），未滤波
    bool have_prev = false;  // 是否真的用了上一拍（false ⇒ 上面三项全零）
};

// 纯函数：由当前拍与上一拍推导观测量。
// @param have_prev 上一拍是否可用于差分（false 时只返回零值）。
// @note 不做有效性/时间合法性判断，调用方负责。
[[nodiscard]] GuidanceLogDerived guidance_log_derive(const GuidanceOutput& cur, const GuidanceOutput& prev, bool have_prev);

// 纯函数：一行给人看的摘要（不含换行）。
[[nodiscard]] std::string guidance_log_line(const GuidanceOutput& out, const GuidanceLogDerived& d, std::uint64_t seq, std::uint64_t frame, const GuidanceParams& params, const GuidanceLoopStats& stats);

// 纯函数：CSV 一行（不含换行）；表头由 guidance_log_csv_header() 给出。
[[nodiscard]] std::string guidance_log_csv_row(const GuidanceOutput& out, const GuidanceLogDerived& d, std::uint64_t seq, std::uint64_t frame, const GuidanceParams& params, const GuidanceLoopStats& stats);
[[nodiscard]] const char* guidance_log_csv_header();

// 枚举名（日志、单测共用；未知值返回 "?"）。
[[nodiscard]] const char* guidance_mode_name(PureGuidanceMode mode);
[[nodiscard]] const char* guidance_health_name(GuidanceInputHealth health);
[[nodiscard]] const char* guidance_law_name(GuidanceLaw law);

// 制导中间量日志。
// 每次 record() 前应检查 enabled()（record 内部也会自检）；close() 可重复调用。
// 单线程使用：只在主循环线程里调用，与 GuidanceLoop 同线程。
//
// 抽稀按**目标帧时戳**（boot_us）做时间抽稀：相邻两行至少间隔 1/rate_hz 秒。
// 这样与“计算拍频率”无关——检测 10 Hz 或 30 Hz 时，rate_hz 的语义都是“每秒最多几行”。
class GuidanceFlightLog {
  public:
    explicit GuidanceFlightLog(const GuidanceLogConfig& config);
    ~GuidanceFlightLog();

    GuidanceFlightLog(const GuidanceFlightLog&) = delete;
    GuidanceFlightLog& operator=(const GuidanceFlightLog&) = delete;

    // 打开文件并补表头（文件非空时不重复写表头）。
    // @return false 表示文件打不开；此时仍可继续用 stdout，调用方应提示一次。
    bool init(std::string& error);

    // 事件行（进撞板 / 复位），不受 rate_hz 限制；只在 enabled 时输出。
    void event(const char* what);

    // 记录一拍。
    // @note 重复帧（boot_us 与上一拍相同，即零阶保持重发）直接跳过，不参与差分也不输出；
    //       无效拍（valid=false 或 health!=kOk）会清空差分历史（与内核清 memory 对齐）；
    //       输出按 boot_us 时间抽稀，语义见类注释。
    void record(const GuidanceOutput& out, const GuidanceParams& params, const GuidanceLoopStats& stats);

    void close();

    [[nodiscard]] bool enabled() const {
        return enabled_;
    }
    [[nodiscard]] bool file_ok() const {
        return file_ != nullptr;
    }
    [[nodiscard]] const std::string& file() const {
        return config_.file;
    }

  private:
    GuidanceLogConfig config_;
    bool enabled_ = false;
    std::uint64_t period_us_ = 100000;  // 两行之间的最小间隔（µs），由 rate_hz 换算
    std::FILE* file_ = nullptr;
    std::uint64_t frame_ = 0;  // 计算拍计数（CSV 的 frame 列）
    std::uint64_t seq_ = 0;    // 已输出的行号
    bool have_prev_ = false;   // 上一拍是否可用于差分
    GuidanceOutput prev_{};
    std::uint64_t last_boot_us_ = 0;  // 上一拍的 boot_us（判重复帧 / 时间倒退）
    std::uint64_t last_log_us_ = 0;   // 上一行输出的目标帧时戳（0 = 还没输出过）
};

}  // namespace pure_uav::guidance::flight
