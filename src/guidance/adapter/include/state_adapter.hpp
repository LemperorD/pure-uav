#pragma once
// =============================================================================
//  state_adapter.hpp —— 外部口径 → GuidanceState 的唯一换算点
//
//  坐标/姿态/时间/单位约定与换算公式见 assets/docs/interfaces.md。
//  本文件只做换算；输入体检见 state_check.hpp。
// =============================================================================

#include "guidance_types.hpp"      // GuidanceState / PureGuidanceCmd / Vec3
#include "mavlink_units.hpp"       // 单位换算与哨兵（本文件全部转调它）

namespace pure_uav {
namespace adapter {

// ============================================================================
//  §0  口径枚举
// ============================================================================

// 惯性系：NED（外部）/ ENU（内部）。无 GPS 时 NED 实为“初始机头对齐 + z 下”，
// 与真 NED 只差固定偏航角；详见 assets/docs/interfaces.md §2。
enum class Frame {
    kNed = 0,   // 北-东-下（无 GPS 时实为“初始机头对齐 + z 下”的局部系）
    kEnu = 1,   // 东-北-上（内部口径）
};

// 外部机体系 → 光轴系。默认 kFrd；真实值必须由通信/视觉确认（Q8）。
// 各系定义与换算见 assets/docs/interfaces.md §3。
enum class AttitudeFrame {
    kFrd = 0,               // x 前 / y 右 / z 下
    kFlu = 1,               // x 前 / y 左 / z 上
    kCamFrd = 2,            // 光轴系（相机约定，按 FRD 语义描述）
    kCamFlu = 3,            // 光轴系（相机约定，按 FLU 语义描述）
    kOpticalPassthrough = 4,// 已是光轴系 → 直通
};

// 时间戳单位
enum class TimeUnit {
    kSeconds = 0,   // s
    kMillis  = 1,   // ms（MAVLink `time_boot_ms`：uint32，约 49.7 天回绕）
    kMicros  = 2,   // µs（PX4 uORB `timestamp`：uint64，实际不回绕）
};

// 时间基准：只用于记录/断言“两端是否同源”；跨源相减会静默给出垃圾。
// 详见 assets/docs/interfaces.md §4。
enum class TimeBase {
    kSameSource = 0,    // 与参考时标同源（唯一安全的情况）
    kSinceBoot  = 1,    // 开机以来（MAVLink time_boot_ms）
    kUnixEpoch  = 2,    // Unix epoch（本机 SITL 实测 uORB timestamp 表现为这个）
    kUnknown    = 3,    // 未知 → 使用前后应警觉
};

// 关键不变量：**同一帧数据的所有时间戳必须同源**。
//    跨源相减不会报错，只会静默给出垃圾差值。
inline bool is_safe_to_difference(TimeBase a, TimeBase b) noexcept {
    return a == TimeBase::kSameSource && b == TimeBase::kSameSource;
}

// ============================================================================
//  §1  坐标系换算（NED ↔ ENU）
// ============================================================================
//
// NED↔ENU 的映射是**自逆**的（转两次 = 恒等），所以只有一对函数：
//     ENU = (NED.y, NED.x, −NED.z)
//      NED = (ENU.y, ENU.x, −ENU.z)
// 位置 / 速度 / 加速度 / 目标速度 **都适用**（都是"同一个矢量在不同系的坐标"）。

// 把矢量从 `from` 系换算到 `to` 系；from == to 时原样返回。
Vec3 convert_vec(const Vec3 &v, Frame from, Frame to) noexcept;

// 便捷写法：NED → ENU（内部最常用）
inline Vec3 ned_to_enu(const Vec3 &v) noexcept { return convert_vec(v, Frame::kNed, Frame::kEnu); }

// 便捷写法：ENU → NED（发指令用）
inline Vec3 enu_to_ned(const Vec3 &v) noexcept { return convert_vec(v, Frame::kEnu, Frame::kNed); }

// 位置 / 速度 / 加速度语义上都是矢量，同一套换算；分开写只为可读性与自解释。
inline Vec3 position_from(const Vec3 &p, Frame from, Frame to) noexcept { return convert_vec(p, from, to); }
inline Vec3 velocity_from(const Vec3 &v, Frame from, Frame to) noexcept { return convert_vec(v, from, to); }
inline Vec3 accel_from(const Vec3 &a, Frame from, Frame to) noexcept { return convert_vec(a, from, to); }

// ============================================================================
//  §2  姿态换算（外部机体系 → 光轴系，且世界系 NED → ENU）
// ============================================================================

// --- 先给出四元数旋转工具 ---
//
// 本层**自带**一份实现（而不是调用 guidance_libs 的 `quat_rotate_vec`），理由：
//   ① 零链接依赖：只用纯头文件的 vec3.hpp + mavlink_units，
//      解析侧 / 机载侧单独编译本层时不必拖上整个制导库；
//   ② 语义明确：跨模块口径换算必须用**数学定义** R(q)·v 的那一个。
// 2026-10-16 用三条独立路径实测（各 200 例随机姿态、最大偏差 2.7e-15）：
//   仓库 `quat_rotate_vec` ≡ 本层 `rotate_vec_by_quat` ≡ 标准旋转矩阵
// ⇒ 三者语义一致，可放心混用；一致性由 test_state_adapter.cpp §4 钉住。

// 四元数旋转：v' = R(q)·v（q = [w,x,y,z]，机体→世界）
Vec3 rotate_vec_by_quat(const double q[4], const Vec3 &v) noexcept;

// 逆旋转：世界→机体（v' = R(q)ᵀ·v）
Vec3 rotate_vec_by_quat_inv(const double q[4], const Vec3 &v) noexcept;

// 共轭（单位四元数时即逆）
void quat_conjugate(const double q[4], double q_out[4]) noexcept;

// 外部姿态 q_in（[w,x,y,z]，**机体→世界**，世界系由 `world_in` 指定，
// 机体轴由 `frame_in` 指定）→ 制导核期望的姿态。
//
// 输出：`q_out` = [w,x,y,z]，**光轴系 → 世界系(ENU)**，|q_out| ≈ 1；
//       `world_out` = Frame::kEnu（恒为 ENU，返回它是为了调用方不必硬编码）。
//
// 实现：把"机体系三轴"各自表示成世界系矢量 → 在目标世界系里重建旋转矩阵
//       → 提取四元数。这样一次性同时完成**世界系换算**与**机体轴换算**，
//       且不依赖对"分量怎么重排"的记忆（重排最容易记反）。
//
// 返回 false = `frame_in == kOpticalPassthrough` 之外的输入不构成正规旋转
//       （|q_in| 偏离 1 过多 / 出现 NaN）。此时 `q_out` 被置为单位四元数。
//
// ⚠ kOpticalPassthrough 只做**世界系**换算（NED→ENU），机体轴不动。
bool convert_attitude(const double q_in[4], Frame world_in, AttitudeFrame frame_in,
                      double q_out[4], Frame &world_out) noexcept;

// 便捷版：外部按 NED + FRD 给（PX4 / MAVLink 最常见）→ 内部 ENU + 光轴系
inline bool attitude_from_ned_frd(const double q_in[4], double q_out[4]) noexcept {
    Frame dump = Frame::kNed;
    return convert_attitude(q_in, Frame::kNed, AttitudeFrame::kFrd, q_out, dump);
}

// 单位化四元数并返回**归一化前的模长**。
// 用于自检：|q| 明显偏离 1 说明上游给了脏数据（外推/插值未归一化）。
// 零四元数（模 < 1e-12）会被置为单位四元数，返回 0。
double normalize_quat(double q[4]) noexcept;

// ============================================================================
//  §3  时间对齐（单位换算 + 回绕安全差分）
// ============================================================================

// 外部时间戳 → 内部秒（double）。`t_ref` 是同源的参考时标（可为 0）。
// ⚠ 若两端基准不同源，相减会得到垃圾 —— 用 is_safe_to_difference() 先自检。
double timestamp_to_s(double raw, TimeUnit unit, double t_ref = 0.0) noexcept;

// uint32 版本（MAVLink `time_boot_ms`）。`t_ref` 与返回值同单位 raw。
// ⚠ 没有单位参数：uint32 时间戳的既有约定就是 ms，写死可避免调用方传错。
double timestamp_u32_ms_to_s(uint32_t raw_ms, uint32_t t_ref_ms = 0u) noexcept;

// uint64 版本（PX4 uORB `timestamp`，µs）。
double timestamp_u64_us_to_s(uint64_t raw_us, uint64_t t_ref_us = 0u) noexcept;

// 时间轴跟踪器：累加每帧差值，天然容纳 uint32 回绕、避免 2^53 精度问题。
// 接口/用法见 assets/docs/interfaces.md §4.4。
class StateTimeTracker {
public:
    // ---- 输入：每帧推一个原始时间戳 ----
    void push_u64_us(uint64_t raw_us) noexcept;   // PX4 uORB（µs, uint64）
    void push_u32_ms(uint32_t raw_ms) noexcept;   // MAVLink（ms, uint32，回绕安全）
    void push_s(double raw_s) noexcept;           // 已经是秒

    // ---- 输出 ----
    // 相对**参考点**的内部秒。未设参考点时参考点 = 首帧（即 t_s() 从 0 起算）。
    double t_s() const noexcept {
        return static_cast<double>(t_us_ - ref_u64_us_) * 1e-6;
    }
    uint64_t elapsed_us() const noexcept { return t_us_; }
    bool empty() const noexcept { return frames_ == 0; }
    uint32_t frames() const noexcept { return frames_; }
    uint32_t out_of_order() const noexcept { return out_of_order_; }

    // 最近一次 push 的"原始值差"是否可疑（大跳变）——只提示，不影响累加。
    bool last_jump_suspect() const noexcept { return last_jump_suspect_; }

    // ---- 参考点 setter：把"当前原始时间戳"定为 t = 0 ----
    // 典型用法：目标发射那一刻记下发射时标，之后 tr.t_s() 即"发射以来几秒"。
    void set_reference_u64_us(uint64_t raw_us) noexcept;
    void set_reference_u32_ms(uint32_t raw_ms) noexcept;
    void reset() noexcept;

    TimeBase base() const noexcept { return base_; }
    void set_base(TimeBase b) noexcept { base_ = b; }

private:
    uint64_t t_us_ = 0;                  // 累加得到的内部时间（µs）
    bool     has_prev_ = false;
    uint64_t prev_u64_ = 0;              // 上一帧原始值（µs 域）
    uint64_t ref_u64_us_ = 0;            // 已从累加值中扣除的参考偏移（µs）
    uint32_t frames_ = 0;
    uint32_t out_of_order_ = 0;
    bool     last_jump_suspect_ = false;
    TimeBase base_ = TimeBase::kUnknown;
};

// ============================================================================
//  §4  组装 GuidanceState（外部口径 → 内部口径）
// ============================================================================
//
// 输入由调用方按**外部口径**填（NED、ms/µs、deg、mm…），本文件负责全部换算。
// 每个函数都返回"只填了哪些字段、哪些字段缺失"的位掩码，配合
// `state_check.hpp` 可以一眼看出"这一帧够不够制导用"。

// 已填充字段位掩码（缺失的字段会保持 GuidanceState 的默认值 = 0）
enum : uint32_t {
    kHaveTime    = 1u << 0,   // t
    kHaveRange   = 1u << 1,   // range
    kHaveLos     = 1u << 2,   // los_hat
    kHavePM      = 1u << 3,   // p_M
    kHaveVM      = 1u << 4,   // v_M
    kHaveQ       = 1u << 5,   // q
    kHaveVT      = 1u << 6,   // v_T（仿真真值 / 真机不可得）
    kHaveATPerp  = 1u << 7,   // a_T_perp（APN 用）
};

// ---- 从原始时间戳填 t ----
inline void fill_time(GuidanceState &st, const StateTimeTracker &tr) noexcept { st.t = tr.t_s(); }

// ---- 4.1 飞控惯导：位置 / 速度 / 姿态（PX4 uORB 或 MAVLink 裸字段）----
//
// `p_raw` / `v_raw` 在 `world_in` 系（通常 NED）；单位 m / (m/s)。
// `q_raw` 为 [w,x,y,z]，机体（`att_in`）→ 世界（`world_in`）。
//
// ⚠ 只填 p_M / v_M / q，**不碰** los_hat / range（那是视觉的事）。
//   `st.valid` 不由本函数设置（有效性是"目标是否可观测"的语义，见 4.3）。
void fill_ins(GuidanceState &st,
              const Vec3 &p_raw, const Vec3 &v_raw,
              const double q_raw[4],
              Frame world_in, AttitudeFrame att_in,
              bool has_position = true, bool has_attitude = true) noexcept;

// ---- 4.2 目标 / 视觉：视线方向 + 距离 → los_hat + range + (可选) v_T ----
//
// `los_raw` **不必是单位矢量**（反投影结果常有模长误差），会自动单位化；
// 零矢量 ⇒ `valid = false`（视作目标丢失，制导进 SEARCH）。
//
// `opt_range_m`：有直接测距（深度相机 / 激光 / size 反推）时填，`has_range = true`。
//   ⚠ 没有 range 时制导仍可跑（PPN 不含距离项），但 TPN/APN 与 t_go 会失准。
//
// `opt_vT_raw`：仅"真值法"（FROM_RELATIVE_VELOCITY，仿真）可得；真机置 false。
//
// 返回本次填充的位掩码。
uint32_t fill_target(GuidanceState &st,
                     const Vec3 &los_raw, Frame from,
                     bool has_range, double opt_range_m,
                     bool has_vT, const Vec3 &opt_vT_raw,
                     bool target_present = true) noexcept;

// ---- 4.3 APN 的目标横向加速度 ----
// 真机需估计（可由弹道外推得到）；估计不出来就别开 APN。
void fill_target_accel(GuidanceState &st, const Vec3 &a_T_raw, Frame from,
                       bool has_ata = true) noexcept;

// ---- 4.4 制导指令 → 外部 NED（发指令前唯一的一次换算）----
//
// `a_cmd`（内部 ENU, m/s²）→ NED 三分量，可直接写进
// `cmd_accel_ned` 的 linear.x/y/z 或 MAVLink `SET_POSITION_TARGET_LOCAL_NED`。
void cmd_accel_to_ned(const PureGuidanceCmd &cmd, double out_xyz[3]) noexcept;

// 只转矢量（不想带整个 cmd 结构时用）
inline Vec3 accel_cmd_ned(const Vec3 &a_cmd_enu) noexcept { return enu_to_ned(a_cmd_enu); }

// ============================================================================
//  §5  一步到位的"整帧"入口
// ============================================================================
//
// 把 §4 的调用顺序固定下来，避免各处自己拼装（拼装最容易漏一个换算）。
// 字段缺失时按缺省值处理，不返回值之外的东西；详细体检请用 state_check.hpp。

struct InsRaw {          // 飞控惯导原始帧
    Vec3   p;            // 位置（`world` 系，m）
    Vec3   v;            // 速度（`world` 系，m/s）
    double q[4] = {1.0, 0.0, 0.0, 0.0};  // 姿态 [w,x,y,z]，机体→世界
    bool   has_position = true;
    bool   has_attitude = true;
};

struct TargetRaw {       // 视觉/目标检测原始帧
    Vec3   los;          // 视线方向（`world` 系，**不必单位化**）
    bool   has_range = false;
    double range_m = 0.0;
    bool   has_vT = false;
    Vec3   vT;           // 目标速度（`world` 系，m/s）
    bool   present = true;
};

struct RawFrame {        // 一帧完整外部输入
    Frame         world = Frame::kNed;                 // 外部世界系
    AttitudeFrame att   = AttitudeFrame::kFrd;         // 外部机体系
    InsRaw        ins;
    TargetRaw     target;
};

GuidanceState to_guidance_state(const RawFrame &raw) noexcept;

}  // namespace adapter
}  // namespace pure_uav
