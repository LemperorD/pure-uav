#include "state_adapter.hpp"

#include <cmath>
#include <cstddef>

namespace pure_uav {
namespace adapter {

// ============================================================================
//  §0  四元数工具
// ============================================================================

// 四元数旋转：v' = R(q)·v，q = [w,x,y,z]（机体→世界）。
//
// 【为什么本层自带一份实现】
//   ① 零链接依赖：只用纯头文件的 vec3.hpp + mavlink_units，
//      解析侧 / 机载侧单独编译本层时不必拖上整个 guidance_libs；
//   ② 语义明确：跨模块口径换算必须用**数学定义**的那一个。
//
// 【与 guidance_libs::quat_rotate_vec 的关系（2026-10-16 三条路径实测）】
//   仓库的 `quat_rotate_vec` 与本文实现、与"标准旋转矩阵"三者**完全一致**
//   （各 200 例随机姿态，最大偏差 2.7e-15），即它确实是 R(q)·v，不是转置。
//   三个实现互相钉住：见 test_state_adapter.cpp §4。
Vec3 rotate_vec_by_quat(const double q[4], const Vec3 &v) noexcept {
    const double w = q[0];
    const Vec3   u(q[1], q[2], q[3]);
    // R(q)·v = v + 2w·(u×v) + 2·u×(u×v)
    const Vec3 uv  = cross(u, v);
    const Vec3 uuv = cross(u, uv);
    return v + uv * (2.0 * w) + uuv * 2.0;
}

Vec3 rotate_vec_by_quat_inv(const double q[4], const Vec3 &v) noexcept {
    const double qc[4] = {q[0], -q[1], -q[2], -q[3]};
    return rotate_vec_by_quat(qc, v);
}

void quat_conjugate(const double q[4], double q_out[4]) noexcept {
    q_out[0] = q[0];
    q_out[1] = -q[1];
    q_out[2] = -q[2];
    q_out[3] = -q[3];
}

// ============================================================================
//  内部工具
// ============================================================================
namespace {
// 制导核**光轴系**的三条定义轴（相机约定）：+x = 右、+y = 下、+z = 前(光轴)。
//
// 换算的核心问题：外部给的 q 是关于**外部机体系**的旋转，而我们最终要的是
// "光轴系三轴在世界(ENU)下的指向"。所以要先把"光轴系第 k 根轴"用**外部机体系**
// 的坐标表示出来，再用 q 把它旋到世界系 —— 见 optical_axis_in_external_frame()。
enum class OpticalAxis { kRight, kDown, kForward };

// 光轴系第 `k` 根轴，在**外部机体系**下的坐标。
//
// 先把"物理指向"写清楚（这是唯一需要记对的地方），再由它按各系的基读出坐标：
//   物理轴       | 光轴系哪根轴 | FRD(x前 y右 z下) | FLU(x前 y左 z上)
//   -------------|-------------|------------------|-----------------
//   机头方向(前) | +z (光轴)   | (1, 0, 0)        | (1, 0, 0)
//   机右方向     | +x          | (0, 1, 0)        | (0,-1, 0)
//   机下方向     | +y          | (0, 0, 1)        | (0, 0,-1)
//
//   ⇒ 光轴系 ↔ FRD 是**循环置换** (x,y,z)_cam = (y,z,x)_frd，det = +1（不是镜像）；
//     光轴系 ↔ FLU 同样是循环置换，只是"下"的符号随 FLU 的 z 向上而翻转。
//   ⚠ 三根轴的正交性/手性由构造保证：(x̂_cam × ŷ_cam = ẑ_cam)
//     = (机右 × 机下 = 机头方向) —— 自己乘一遍即可确认，无需再做归一化。
Vec3 optical_axis_in_external_frame(OpticalAxis axis, AttitudeFrame f) noexcept {
    switch (f) {
        case AttitudeFrame::kFrd:
            switch (axis) {
                case OpticalAxis::kRight:   return Vec3(0.0, 1.0, 0.0);
                case OpticalAxis::kDown:    return Vec3(0.0, 0.0, 1.0);
                case OpticalAxis::kForward: return Vec3(1.0, 0.0, 0.0);
            }
            break;
        case AttitudeFrame::kFlu:
            switch (axis) {
                case OpticalAxis::kRight:   return Vec3(0.0, -1.0, 0.0);
                case OpticalAxis::kDown:    return Vec3(0.0, 0.0, -1.0);
                case OpticalAxis::kForward: return Vec3(1.0, 0.0, 0.0);
            }
            break;
        case AttitudeFrame::kCamFrd:
            // 外部**已经是光轴系**（按 FRD 语系书写）⇒ 光轴系三轴 = 自己的基
            switch (axis) {
                case OpticalAxis::kRight:   return Vec3(1.0, 0.0, 0.0);
                case OpticalAxis::kDown:    return Vec3(0.0, 1.0, 0.0);
                case OpticalAxis::kForward: return Vec3(0.0, 0.0, 1.0);
            }
            break;
        case AttitudeFrame::kCamFlu:
            // 同上，按 FLU 语系书写。两者都退化为单位基 —— 差别在 q 的语义，
            // 而 q 是原样拿到的，所以这里不做任何变换（见 convert_attitude）。
            switch (axis) {
                case OpticalAxis::kRight:   return Vec3(1.0, 0.0, 0.0);
                case OpticalAxis::kDown:    return Vec3(0.0, 1.0, 0.0);
                case OpticalAxis::kForward: return Vec3(0.0, 0.0, 1.0);
            }
            break;
        case AttitudeFrame::kOpticalPassthrough:
            // 对方已按光轴系给了 q ⇒ 只做世界系换算，机体轴不动
            switch (axis) {
                case OpticalAxis::kRight:   return Vec3(1.0, 0.0, 0.0);
                case OpticalAxis::kDown:    return Vec3(0.0, 1.0, 0.0);
                case OpticalAxis::kForward: return Vec3(0.0, 0.0, 1.0);
            }
            break;
    }
    return Vec3(0.0, 0.0, 1.0);   // 不可达
}

// 世界系矢量：从 `from` 转到 `to`（当前只是 NED↔ENU，但写成通用形状）
Vec3 world_to_world(const Vec3 &v, Frame from, Frame to) noexcept {
    if (from == to) return v;
    // ENU = (N.y, N.x, −N.z)；自逆 —— 反向同样用这个式子再转一次即可。
    if (from == Frame::kNed && to == Frame::kEnu) return Vec3(v.y, v.x, -v.z);
    if (from == Frame::kEnu && to == Frame::kNed) return Vec3(v.y, v.x, -v.z);
    return v;
}

}  // namespace

// ============================================================================
//  §1  坐标系换算
// ============================================================================
Vec3 convert_vec(const Vec3 &v, Frame from, Frame to) noexcept {
    return world_to_world(v, from, to);
}

// ============================================================================
//  §2  姿态换算
// ============================================================================

double normalize_quat(double q[4]) noexcept {
    const double n2 = q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3];
    if (!std::isfinite(n2)) {
        q[0] = 1.0; q[1] = q[2] = q[3] = 0.0;
        return 0.0;
    }
    if (n2 < 1e-24) {   // 零四元数：没有姿态信息，给恒等而不是 NaN
        q[0] = 1.0; q[1] = q[2] = q[3] = 0.0;
        return 0.0;
    }
    const double n = std::sqrt(n2);
    q[0] /= n; q[1] /= n; q[2] /= n; q[3] /= n;
    return n;
}

bool convert_attitude(const double q_in[4], Frame world_in, AttitudeFrame frame_in,
                      double q_out[4], Frame &world_out) noexcept {
    world_out = world_in;   // 失败时保持"未转换"的语义，调用方不会误以为转过了

    for (int i = 0; i < 4; ++i) {
        if (!std::isfinite(q_in[i])) {
            q_out[0] = 1.0; q_out[1] = q_out[2] = q_out[3] = 0.0;
            return false;
        }
    }

    // |q| 必须"像个四元数"：
    //   · 太接近 0 ⇒ 没有姿态信息（零/退化输入）
    //   · 数量级差太远 ⇒ 大概率不是四元数（把欧拉角或别的结构填进来了）
    // ⚠ 容差不设成 1e-3：**未归一化的四元数很常见**（外推、插值、整数标定），
    //   只要方向对，归一化后就是同一个姿态 ⇒ 应当接受并归一化，不该拒。
    const double n2 = q_in[0] * q_in[0] + q_in[1] * q_in[1] +
                      q_in[2] * q_in[2] + q_in[3] * q_in[3];
    const double n = std::sqrt(n2);
    if (!(n > 1e-6) || !(n < 1e+3)) {   // 写成 !() 形式以同时挡住 NaN
        q_out[0] = 1.0; q_out[1] = q_out[2] = q_out[3] = 0.0;
        return false;
    }

    // 单位化，避免上游微小的模长误差被带进制导
    double qn[4] = {q_in[0] / n, q_in[1] / n, q_in[2] / n, q_in[3] / n};

    // ---- 光轴系三轴在**内部世界系(ENU)** 下的坐标 = 目标旋转矩阵 R 的三列 ----
    //   col[0] = 光轴系 +x（机右）在世界系下的指向
    //   col[1] = 光轴系 +y（机下）
    //   col[2] = 光轴系 +z（光轴 = 机头方向）
    // 取列（而不是行）是因为 q 的语义是"机体→世界"：R·v_body = v_world。
    Vec3 col[3];
    {
        const OpticalAxis axes[3] = {OpticalAxis::kRight, OpticalAxis::kDown,
                                     OpticalAxis::kForward};
        for (int i = 0; i < 3; ++i) {
            // ① 光轴系第 i 根轴在**外部机体系**下的坐标
            const Vec3 in_body = optical_axis_in_external_frame(axes[i], frame_in);
            // ② 用外部姿态旋到**外部世界系**（R(q)·v = 机体→世界）
            const Vec3 in_world = rotate_vec_by_quat(qn, in_body);
            // ③ 外部世界系 → 内部世界系(ENU)。
            //    kCamFrd / kCamFlu / kOpticalPassthrough 的 ① 已是单位基，
            //    所以这一步等价于"直接把 q 当光轴系→世界的旋转"，符合预期。
            col[i] = world_to_world(in_world, world_in, Frame::kEnu);
        }
    }

    // ---- 从旋转矩阵重建四元数（Shepperd 法：挑最大的对角项分支，数值最稳）----
    const double m00 = col[0].x, m10 = col[0].y, m20 = col[0].z;
    const double m01 = col[1].x, m11 = col[1].y, m21 = col[1].z;
    const double m02 = col[2].x, m12 = col[2].y, m22 = col[2].z;

    double w = 1.0, x = 0.0, y = 0.0, z = 0.0;
    const double tr = m00 + m11 + m22;
    if (tr > 0.0) {
        const double s = std::sqrt(tr + 1.0) * 2.0;   // s = 4w
        w = 0.25 * s;
        x = (m21 - m12) / s;
        y = (m02 - m20) / s;
        z = (m10 - m01) / s;
    } else if (m00 > m11 && m00 > m22) {
        const double s = std::sqrt(1.0 + m00 - m11 - m22) * 2.0;   // s = 4x
        w = (m21 - m12) / s;
        x = 0.25 * s;
        y = (m01 + m10) / s;
        z = (m02 + m20) / s;
    } else if (m11 > m22) {
        const double s = std::sqrt(1.0 + m11 - m00 - m22) * 2.0;   // s = 4y
        w = (m02 - m20) / s;
        x = (m01 + m10) / s;
        y = 0.25 * s;
        z = (m12 + m21) / s;
    } else {
        const double s = std::sqrt(1.0 + m22 - m00 - m11) * 2.0;   // s = 4z
        w = (m10 - m01) / s;
        x = (m02 + m20) / s;
        y = (m12 + m21) / s;
        z = 0.25 * s;
    }

    q_out[0] = w; q_out[1] = x; q_out[2] = y; q_out[3] = z;
    normalize_quat(q_out);   // 清掉重建过程的舍入误差，保证 |q| 严格接近 1
    world_out = Frame::kEnu;
    return true;
}

// ============================================================================
//  §3  时间对齐
// ============================================================================

double timestamp_to_s(double raw, TimeUnit unit, double t_ref) noexcept {
    switch (unit) {
        case TimeUnit::kSeconds: return raw - t_ref;
        case TimeUnit::kMillis:  return mavlink_units::ms_diff_to_s(
                                     static_cast<uint32_t>(raw),
                                     static_cast<uint32_t>(t_ref));
        case TimeUnit::kMicros:  return (raw - t_ref) * 1e-6;
    }
    return raw - t_ref;
}

double timestamp_u32_ms_to_s(uint32_t raw_ms, uint32_t t_ref_ms) noexcept {
    return mavlink_units::ms_diff_to_s(raw_ms, t_ref_ms);
}

double timestamp_u64_us_to_s(uint64_t raw_us, uint64_t t_ref_us) noexcept {
    return static_cast<double>(raw_us - t_ref_us) * 1e-6;
}

// ---- StateTimeTracker ----

void StateTimeTracker::push_u64_us(uint64_t raw_us) noexcept {
    ++frames_;
    last_jump_suspect_ = false;

    if (!has_prev_) {           // 第一帧：t = 0 起算
        has_prev_ = true;
        prev_u64_ = raw_us;
        return;
    }

    const uint64_t d = raw_us - prev_u64_;   // 无符号差：天然模 2^64，容纳回绕
    // 判"乱序"：差值大得离谱（> 1e7 s ≈ 115 天）不可能是真实增量
    if (d > 10000000000000ull) {
        ++out_of_order_;
        last_jump_suspect_ = true;
        return;                                 // 不累加、不更新 prev
    }
    t_us_ += d;
    prev_u64_ = raw_us;
    if (d > 5000000ull) last_jump_suspect_ = true;   // > 5 s：可疑（丢帧/时钟跳变）
}

void StateTimeTracker::push_u32_ms(uint32_t raw_ms) noexcept {
    ++frames_;
    last_jump_suspect_ = false;

    const uint64_t raw_us = static_cast<uint64_t>(raw_ms) * 1000ull;
    if (!has_prev_) {
        has_prev_ = true;
        prev_u64_ = raw_us;
        return;
    }

    // 32 位回绕安全：先在 ms 域做无符号差，再扩到 µs
    const uint32_t prev_ms = static_cast<uint32_t>(prev_u64_ / 1000ull);
    const uint32_t d_ms = mavlink_units::ms_diff_u32(raw_ms, prev_ms);

    if (d_ms > 2147483647u) {        // 落在"过去"半区 ⇒ 乱序
        ++out_of_order_;
        last_jump_suspect_ = true;
        return;
    }
    t_us_ += static_cast<uint64_t>(d_ms) * 1000ull;
    prev_u64_ = raw_us;
    if (d_ms > 5000u) last_jump_suspect_ = true;
}

void StateTimeTracker::push_s(double raw_s) noexcept {
    ++frames_;
    last_jump_suspect_ = false;

    if (!std::isfinite(raw_s)) {   // 非法值直接忽略
        return;
    }
    // 统一在 µs 域累加（prev_u64_ 一直是"上一帧原始时标的 µs 表示"）
    const uint64_t raw_us = static_cast<uint64_t>(raw_s * 1e6);
    if (!has_prev_) {
        has_prev_ = true;
        prev_u64_ = raw_us;
        return;
    }
    const double d_s = raw_s - static_cast<double>(prev_u64_) * 1e-6;
    if (d_s < 0.0) {
        ++out_of_order_;
        last_jump_suspect_ = true;
        return;
    }
    t_us_ += static_cast<uint64_t>(d_s * 1e6);
    prev_u64_ = raw_us;
    if (d_s > 5.0) last_jump_suspect_ = true;
}

void StateTimeTracker::set_reference_u64_us(uint64_t raw_us) noexcept {
    if (!has_prev_) {
        // 还没数据：把参考点设为该值，下一帧自然从 0 附近开始
        prev_u64_ = raw_us;
        has_prev_ = true;
        t_us_ = 0;
        ref_u64_us_ = 0;
        return;
    }
    // 把内部时间推进到"这一原始时刻"（含回绕安全的无符号差）
    const uint64_t d = raw_us - prev_u64_;
    if (d <= 10000000000000ull) {      // 正常范围才推进（防乱序把参考点算歪）
        t_us_ += d;
        prev_u64_ = raw_us;
    }
    ref_u64_us_ = t_us_;               // 之后 t_s() = (t_us_ − ref)/1e6
}

void StateTimeTracker::set_reference_u32_ms(uint32_t raw_ms) noexcept {
    set_reference_u64_us(static_cast<uint64_t>(raw_ms) * 1000ull);
}

void StateTimeTracker::reset() noexcept {
    t_us_ = 0; has_prev_ = false; prev_u64_ = 0; ref_u64_us_ = 0;
    frames_ = 0; out_of_order_ = 0; last_jump_suspect_ = false;
    base_ = TimeBase::kUnknown;
}

// ============================================================================
//  §4  组装 GuidanceState
// ============================================================================

void fill_ins(GuidanceState &st,
              const Vec3 &p_raw, const Vec3 &v_raw,
              const double q_raw[4],
              Frame world_in, AttitudeFrame att_in,
              bool has_position, bool has_attitude) noexcept {
    if (has_position) st.p_M = world_to_world(p_raw, world_in, Frame::kEnu);
    st.v_M = world_to_world(v_raw, world_in, Frame::kEnu);
    if (has_attitude) {
        Frame dummy = Frame::kEnu;
        convert_attitude(q_raw, world_in, att_in, st.q, dummy);
    }
}

uint32_t fill_target(GuidanceState &st,
                     const Vec3 &los_raw, Frame from,
                     bool has_range, double opt_range_m,
                     bool has_vT, const Vec3 &opt_vT_raw,
                     bool target_present) noexcept {
    uint32_t filled = 0;

    const Vec3 los_enu = world_to_world(los_raw, from, Frame::kEnu);
    const double los_norm = norm(los_enu);
    if (los_norm > 1e-9) {
        st.los_hat = los_enu / los_norm;
        filled |= kHaveLos;
    } else {
        st.los_hat = Vec3{};        // 零矢量：没有方向可言
    }

    if (has_range) {
        st.range = opt_range_m;
        filled |= kHaveRange;
    }

    if (has_vT) {
        st.v_T = world_to_world(opt_vT_raw, from, Frame::kEnu);
        filled |= kHaveVT;
    }

    // 有效性：目标必须"存在且方向可用"
    st.valid = target_present && (los_norm > 1e-9);
    return filled;
}

void fill_target_accel(GuidanceState &st, const Vec3 &a_T_raw, Frame from,
                       bool has_ata) noexcept {
    st.a_T_perp = has_ata ? world_to_world(a_T_raw, from, Frame::kEnu) : Vec3{};
}

void cmd_accel_to_ned(const PureGuidanceCmd &cmd, double out_xyz[3]) noexcept {
    const Vec3 a_ned = world_to_world(cmd.a_cmd, Frame::kEnu, Frame::kNed);
    out_xyz[0] = a_ned.x;
    out_xyz[1] = a_ned.y;
    out_xyz[2] = a_ned.z;
}

// ============================================================================
//  §5  整帧入口
// ============================================================================

GuidanceState to_guidance_state(const RawFrame &raw) noexcept {
    GuidanceState st;

    // ① 惯导（位置 / 速度 / 姿态）
    fill_ins(st, raw.ins.p, raw.ins.v, raw.ins.q,
             raw.world, raw.att, raw.ins.has_position, raw.ins.has_attitude);

    // ② 目标（视线 / 距离 / 目标速度）—— 会设置 st.valid，放最后
    fill_target(st, raw.target.los, raw.world,
                raw.target.has_range, raw.target.range_m,
                raw.target.has_vT, raw.target.vT,
                raw.target.present);

    return st;
}

}  // namespace adapter
}  // namespace pure_uav
