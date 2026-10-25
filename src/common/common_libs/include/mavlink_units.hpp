#pragma once
// =============================================================================
//  MAVLink 单位换算（解析侧专用）
//
//  【职责】只做"数值 + 单位"的换算，不做任何别的事：
//    · 不解析报文（解析在调用方）        · 不做坐标变换（NED/ENU 属另一模块）
//    · 不做状态机 / 不做滤波             · 不依赖任何外部库（只用 <cmath> / <cstdint>）
//
//  【输入/输出单位约定】函数名即约定：<源单位>_to_<目标单位>
//    · 长度  : m(米) / mm(毫米) / cm(厘米)
//    · 速度  : m_s(米/秒) / cm_s(厘米/秒)
//    · 角度  : rad(弧度) / deg(度) / cdeg(1e-2 度，即 MAVLink 的 degrees*100)
//              / deg_e7(1e-7 度，即 MAVLink 的 lat/lon)
//    · 角速度: rad_s(弧度/秒) / deg_s(度/秒)
//    · 时间  : s(秒) / ms(毫秒) / us(微秒)
//    · 角度归一化结果一律为 **rad**
//
//  【为什么独立成模块】MAVLink 的单位在不同报文里**不统一**，散落在解析代码里极易出错。
//    集中到一个文件后：① 每个换算只有一处实现；② 边界情况可单独写测试钉死；
//    ③ 解析代码变成"读字段 → 调函数"，可读性大幅提升。
//
//  【MAVLink 单位速查】（摘自 message_definitions/v1.0/common.xml，2026-10-16 逐行核对）
//    ┌──────────────────────────────┬────────┬────────────────────────────┐
//    │ 报文.字段                    │ 单位   │ 备注                       │
//    ├──────────────────────────────┼────────┼────────────────────────────┤
//    │ ATTITUDE.roll/pitch/yaw      │ rad    │ 已是 SI                    │
//    │ ATTITUDE.*speed              │ rad/s  │ 已是 SI                    │
//    │ LOCAL_POSITION_NED.x/y/z     │ m      │ 已是 SI                    │
//    │ LOCAL_POSITION_NED.vx/vy/vz  │ m/s    │ 已是 SI                    │
//    │ GLOBAL_POSITION_INT.lat/lon  │ degE7  │ ×1e-7                      │
//    │ GLOBAL_POSITION_INT.alt      │ mm     │ ×1e-3                      │
//    │ GLOBAL_POSITION_INT.vx/vy/vz │ cm/s   │ ×1e-2                      │
//    │ GLOBAL_POSITION_INT.hdg      │ cdeg   │ 0..35999；UINT16_MAX = 无效│
//    │ GPS_RAW_INT.lat/lon          │ degE7  │ 规范**无** invalid 哨兵     │
//    │ GPS_RAW_INT.alt              │ mm     │                            │
//    │ GPS_RAW_INT.vel              │ cm/s   │ UINT16_MAX = 无效          │
//    │ GPS_RAW_INT.cog              │ cdeg   │ UINT16_MAX = 无效          │
//    │ GPS_RAW_INT.yaw              │ cdeg   │ ⚠ 0 = 无此数据；**36000 = 正北** │
//    │ VFR_HUD.airspeed/groundspeed │ m/s    │ 已是 SI                    │
//    │ VFR_HUD.heading              │ deg    │ ⚠ **不是 rad**             │
//    │ ODOMETRY.time_usec           │ us     │                            │
//    └──────────────────────────────┴────────┴────────────────────────────┘
//
//  【精度说明】
//    · 换算全部用 double，误差在 1 ulp 量级（相对 ~1e-16），远小于 MAVLink 字段
//      自身的量化步长（cdeg=0.01°≈1.7e-4 rad、degE7=1e-7°≈1.7e-9 rad），可忽略。
//    · 唯一要小心的是**写回** MAVLink 整型字段：必须四舍五入 + 饱和/环绕，见 §7。
//
//  【本模块显式处理的边界情况】
//    · 哨兵值（invalid）：`hdg`/`cog` 的 UINT16_MAX 等，见 §8
//    · 角度环绕：写回 cdeg 时 359.999° 会四舍五入到 36000，必须绕回 0，见 §7
//    · 整型溢出：deg → degE7 在 |deg| > 214.7 时溢出 int32，必须饱和，见 §7
//    · 非有限值：NaN / ±Inf **原样透传**（不伪造数值），由调用方决定如何处理
//    · uint32 毫秒回绕：约 49.7 天回绕一次，见 §6
//
//  【不做的事】坐标变换（NED↔ENU）、四元数、时间戳对齐 —— 各自有独立模块。
// =============================================================================

#include <cstdint>

namespace pure_uav {
namespace mavlink_units {

// ============================================================================
//  §0  常量
// ============================================================================

inline constexpr double kPi    = 3.14159265358979323846;
inline constexpr double kTwoPi = 6.28318530717958647692;

// 换算因子（显式列出，避免调用方到处写魔法数）
// 说明：`cdeg→rad` 用**一步到位**的合并常数（只舍入一次），比 `cdeg*1e-2*kDegToRad`
//       少一次舍入；其余换算在 .cpp 里用**除以精确的 10 的幂**实现（比乘倒数更准）。
inline constexpr double kDegToRad  = kPi / 180.0;      // deg  → rad
inline constexpr double kRadToDeg  = 180.0 / kPi;      // rad  → deg
inline constexpr double kCDegToRad = kPi / 18000.0;    // cdeg → rad
inline constexpr double kRadToCDeg = 18000.0 / kPi;    // rad  → cdeg

// ============================================================================
//  §1  长度
// ============================================================================

double mm_to_m(double mm) noexcept;
double m_to_mm(double m) noexcept;
double cm_to_m(double cm) noexcept;
double m_to_cm(double m) noexcept;

// ============================================================================
//  §2  速度
// ============================================================================

double cm_s_to_m_s(double cm_s) noexcept;
double m_s_to_cm_s(double m_s) noexcept;

// ============================================================================
//  §3  角度（rad / deg / cdeg / degE7）
// ============================================================================

double deg_to_rad(double deg) noexcept;
double rad_to_deg(double rad) noexcept;

double cdeg_to_rad(double cdeg) noexcept;   // MAVLink 的 degrees*100
double rad_to_cdeg(double rad) noexcept;
double cdeg_to_deg(double cdeg) noexcept;
double deg_to_cdeg(double deg) noexcept;

double deg_e7_to_deg(double deg_e7) noexcept;   // MAVLink 的 lat/lon
double deg_to_deg_e7(double deg) noexcept;

// ============================================================================
//  §4  角速度
// ============================================================================

double deg_s_to_rad_s(double deg_s) noexcept;
double rad_s_to_deg_s(double rad_s) noexcept;

// ============================================================================
//  §5  角度归一化与角差（全部输入/输出 **rad**）
// ============================================================================

// 归一化到 **[-π, π]**（闭区间；+π 与 −π 是同一个物理方向）。
// ⚠ 端点归属不确定：输入为 π 的奇数倍时（±π、3π、7π…）可能落到任一端点，
//    **不要依赖具体是哪一端**；需要统一时请自行再判一次。
// ⚠ 内部用 std::remainder（对大角度也能保持精度），不是 fmod。
// ⚠ NaN / ±Inf 原样返回。
double wrap_pi(double rad) noexcept;

// 归一化到 [0, 2π)。
double wrap_2pi(double rad) noexcept;

// 最短有符号角差 wrap_pi(a − b)，结果在 [-π, π]。
// 用于"两个角度差了多少"，避免 ±π 处的 358° 假跳变。
// ⚠ 符号约定：**正值 = a 在 b 的逆时针方向（角度增大方向）**。
//    例：angle_diff_pi(179°, −179°) = −2°（顺时针 2°），不是 +358°。
double angle_diff_pi(double a_rad, double b_rad) noexcept;

// ============================================================================
//  §6  时间（s / ms / us）
// ============================================================================

double ms_to_s(double ms) noexcept;
double us_to_s(double us) noexcept;
double s_to_ms(double s) noexcept;
double s_to_us(double s) noexcept;

// --- uint32 毫秒回绕安全运算（MAVLink `time_boot_ms` 是 uint32，约 49.7 天回绕）---

// 无符号差（单位 ms）。即使 now < prev（刚回绕过）结果仍正确。
// ⚠ 前提：真实间隔 < 2^31 ms ≈ 24.8 天。超出则无法区分，比赛场景不涉及。
uint32_t ms_diff_u32(uint32_t now_ms, uint32_t prev_ms) noexcept;

// 同上，直接换算成秒。
double ms_diff_to_s(uint32_t now_ms, uint32_t prev_ms) noexcept;

// 回绕安全的"是否已过截止时刻"判断。
// 返回 true 表示 now_ms 已经到达/越过 deadline_ms。
// ⚠ 不要写 `now_ms > deadline_ms` —— 回绕后会永远为假。
bool ms_elapsed(uint32_t now_ms, uint32_t deadline_ms) noexcept;

// ============================================================================
//  §7  取整 / 饱和（**写回** MAVLink 整型字段时用）
// ============================================================================

// 四舍五入到最近整数（half away from zero），并饱和到 int32 范围。
// NaN → 0；±Inf → 饱和到对应端点。
int32_t round_to_i32_sat(double v) noexcept;

// 四舍五入并饱和到 uint16 范围 [0, 65535]。NaN → 0。
uint16_t round_to_u16_sat(double v) noexcept;

// 度 → cdeg（写回 hdg / cog）：四舍五入 + **环绕**到 [0, 36000)。
// 例：359.999° → 36000 → 绕回 0。NaN → 0。
uint16_t deg_to_cdeg_wrapped(double deg) noexcept;

// 度 → degE7（写回 lat / lon）：四舍五入 + **饱和**到 int32。
// |deg| > 214.7483647 时饱和（正常经纬度不会触发）。NaN → 0。
int32_t deg_to_deg_e7_round(double deg) noexcept;

// ============================================================================
//  §8  哨兵值（MAVLink 的 invalid 约定）
// ============================================================================

inline constexpr uint16_t kInvalidU16 = 0xFFFFu;      // hdg / cog / vel / eph / epv ...
inline constexpr int32_t  kInt32Max   = 0x7FFFFFFF;   // 2147483647

// hdg / cog 是否有效（规范：invalid = UINT16_MAX）。
bool is_valid_heading_cdeg(uint16_t raw_cdeg) noexcept;

// 带哨兵检查的安全换算。返回 false = 原始字段无效，**out_* 不被写入**。
// 这样调用方可以写：if (heading_cdeg_to_rad(msg.hdg, hdg_rad)) { 使用 hdg_rad; }
bool heading_cdeg_to_rad(uint16_t raw_cdeg, double &out_rad) noexcept;
bool heading_cdeg_to_deg(uint16_t raw_cdeg, double &out_deg) noexcept;

// lat/lon 的"防御性"有效性检查。
// ⚠⚠ 规范**没有**为 lat/lon 定义 invalid 哨兵 —— 有效性应由 `fix_type`（GPS_RAW_INT）
//     或 `xy_global`（PX4 uORB）表示。本函数只排除部分栈会发的 INT32_MAX / INT32_MIN，
//     属于**防御性**约定，不是规范要求。严格遵循规范时请直接用 deg_e7_to_deg()。
bool is_plausible_latlon_deg_e7(int32_t raw_deg_e7) noexcept;

}  // namespace mavlink_units

// 便捷别名：调用方可以写 pure_uav::mu::deg_to_rad(...)
namespace mu = mavlink_units;

}  // namespace pure_uav
