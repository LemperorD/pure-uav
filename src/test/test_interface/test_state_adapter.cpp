// 数据对齐适配层单元测试 —— 王乙芃
//
// 风格：无外部测试框架，纯 main() + 轻量断言（与仓库 test_math / test_guidance /
//       test_mavlink_units 一致）。
//
// 覆盖：
//    1. 坐标系换算 NED↔ENU（自逆、往返、轴向映射）
//    2. 时间对齐：单位换算、uint32 回绕、StateTimeTracker（含"先相减再转 double"陷阱）
//    3. ★★ 姿态换算：外部机体系 → 光轴系（5 种外部口径 × 2 种世界系），用
//       "光轴系三轴在世界系下的指向"做**独立参考实现**逐例比对
//    4. ★ 与 guidance_libs 的 quat_rotate_vec 交叉验证（两份实现必须一致）
//    5. 组装 GuidanceState（fill_ins / fill_target / to_guidance_state）
//    6. 输出换算 accel_enu → cmd_accel_ned
//    7. ★ 不变量自检（state_check）：正常帧全过、各类脏数据必须被抓出来
//    8. ★ 上游模拟器：同一场景用"不同外部口径"描述，适配后必须得到**同一个**
//       GuidanceState —— 这是本模块存在意义的直接证明
//
// 运行：./bin/test/test_state_adapter   （退出码 0=全过，1=有失败）

#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>

#include "state_adapter.hpp"
#include "state_check.hpp"
#include "quat.hpp"          // guidance_core：交叉验证四元数旋转（通用数学，不在仿真里）
#include "guidance.hpp"      // guidance_step()

using namespace pure_uav;
using namespace pure_uav::adapter;

// ============================================================
//  轻量断言
// ============================================================
static int g_pass = 0;
static int g_fail = 0;

static void ok(bool cond, const char *what) {
    if (cond) {
        ++g_pass;
    } else {
        ++g_fail;
        std::printf("  [FAIL] %s\n", what);
    }
}

static void near(double got, double want, double tol, const char *what) {
    if (std::fabs(got - want) <= tol) {
        ++g_pass;
    } else {
        ++g_fail;
        std::printf("  [FAIL] %s: got %.12g, want %.12g (tol %.2g)\n",
                    what, got, want, tol);
    }
}

static void near_vec(const Vec3 &got, const Vec3 &want, double tol, const char *what) {
    const double d = norm(got - want);
    if (d <= tol) {
        ++g_pass;
    } else {
        ++g_fail;
        std::printf("  [FAIL] %s: got (%.12g, %.12g, %.12g), want (%.12g, %.12g, %.12g) "
                    "(|d|=%.3g, tol=%.2g)\n",
                    what, got.x, got.y, got.z, want.x, want.y, want.z, d, tol);
    }
}

static void near_q(const double got[4], const double want[4], double tol, const char *what) {
    // q 与 −q 表示同一旋转 ⇒ 允许整体符号差
    double d1 = 0.0, d2 = 0.0;
    for (int i = 0; i < 4; ++i) {
        d1 += (got[i] - want[i]) * (got[i] - want[i]);
        d2 += (got[i] + want[i]) * (got[i] + want[i]);
    }
    const double d = std::sqrt(std::fmin(d1, d2));
    if (d <= tol) {
        ++g_pass;
    } else {
        ++g_fail;
        std::printf("  [FAIL] %s: got [%.9g,%.9g,%.9g,%.9g], want [%.9g,%.9g,%.9g,%.9g]"
                    " (|d|=%.3g, tol=%.2g)\n",
                    what, got[0], got[1], got[2], got[3],
                    want[0], want[1], want[2], want[3], d, tol);
    }
}

static void section(const char *name) { std::printf("\n== %s ==\n", name); }

static constexpr double PI_REF = 3.14159265358979323846;

// ============================================================
//  本地参考实现（**故意不复用被测代码**）
// ============================================================

// 参考四元数旋转：v' = R(q)·v，q = [w,x,y,z]（机体→世界）
static Vec3 ref_rotate(const double q[4], const Vec3 &v) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    const double R[3][3] = {
        {1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)},
        {2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)},
        {2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)},
    };
    return Vec3(R[0][0] * v.x + R[0][1] * v.y + R[0][2] * v.z,
                R[1][0] * v.x + R[1][1] * v.y + R[1][2] * v.z,
                R[2][0] * v.x + R[2][1] * v.y + R[2][2] * v.z);
}

// 参考旋转矩阵（q 对应的 R，行优先）—— 从矩阵直接读"三列的叉积/行列式"
static void ref_matrix(const double q[4], double R[3][3]) {
    const double w = q[0], x = q[1], y = q[2], z = q[3];
    R[0][0] = 1 - 2 * (y * y + z * z); R[0][1] = 2 * (x * y - z * w);     R[0][2] = 2 * (x * z + y * w);
    R[1][0] = 2 * (x * y + z * w);     R[1][1] = 1 - 2 * (x * x + z * z); R[1][2] = 2 * (y * z - x * w);
    R[2][0] = 2 * (x * z - y * w);     R[2][1] = 2 * (y * z + x * w);     R[2][2] = 1 - 2 * (x * x + y * y);
}

// 从四元数取"某机体轴在世界系下的指向"（列向量）
static Vec3 ref_axis_in_world(const double q[4], int axis) {
    double R[3][3];
    ref_matrix(q, R);
    return Vec3(R[0][axis], R[1][axis], R[2][axis]);
}

static double ref_det3(const Vec3 &a, const Vec3 &b, const Vec3 &c) {
    return dot(a, cross(b, c));
}

// NED ↔ ENU 的参考实现（独立写一遍，不复用被测函数）
static Vec3 ref_ned_to_enu(const Vec3 &v) { return Vec3(v.y, v.x, -v.z); }

// 参考：外部姿态（world_in + frame_in）→ 光轴系姿态（ENU）下"光轴系三轴的世界指向"
// 返回 [光轴系+x, 光轴系+y, 光轴系+z] 在 ENU 下的指向
static void ref_optical_axes(const double q_in[4], Frame world_in, AttitudeFrame f,
                             Vec3 out[3]) {
    // 光轴系第 k 根轴在**外部机体系**下的坐标
    Vec3 in_body[3];
    switch (f) {
        case AttitudeFrame::kFrd:
            in_body[0] = Vec3(0, 1, 0); in_body[1] = Vec3(0, 0, 1); in_body[2] = Vec3(1, 0, 0);
            break;
        case AttitudeFrame::kFlu:
            in_body[0] = Vec3(0, -1, 0); in_body[1] = Vec3(0, 0, -1); in_body[2] = Vec3(1, 0, 0);
            break;
        case AttitudeFrame::kCamFrd:
        case AttitudeFrame::kCamFlu:
        case AttitudeFrame::kOpticalPassthrough:
            in_body[0] = Vec3(1, 0, 0); in_body[1] = Vec3(0, 1, 0); in_body[2] = Vec3(0, 0, 1);
            break;
    }
    for (int k = 0; k < 3; ++k) {
        const Vec3 w = ref_rotate(q_in, in_body[k]);      // 外部世界系
        out[k] = (world_in == Frame::kNed) ? ref_ned_to_enu(w) : w;
    }
}

// 随机单位四元数（可复现）
static uint32_t g_seed = 20261016u;
static double rnd01() {
    g_seed = g_seed * 1664525u + 1013904223u;
    return static_cast<double>(g_seed >> 8) / 16777216.0;   // 2^24
}
static void random_quat(double q[4]) {
    // 均匀四元数（Shoemake 法）
    const double u1 = rnd01(), u2 = rnd01(), u3 = rnd01();
    const double s1 = std::sqrt(1.0 - u1), s2 = std::sqrt(u1);
    q[0] = s1 * std::sin(2 * PI_REF * u2);   // w
    q[1] = s1 * std::cos(2 * PI_REF * u2);   // x
    q[2] = s2 * std::sin(2 * PI_REF * u3);   // y
    q[3] = s2 * std::cos(2 * PI_REF * u3);   // z
}

// 绕单位轴 axis 转 angle 的四元数 [w,x,y,z]
static void quat_axis_angle(const Vec3 &axis_in, double angle, double q[4]) {
    const Vec3 a = normalized(axis_in);
    const double s = std::sin(angle * 0.5);
    q[0] = std::cos(angle * 0.5);
    q[1] = a.x * s;
    q[2] = a.y * s;
    q[3] = a.z * s;
}

// 单位四元数（避免用 C99 复合字面量 —— C++ 不支持）
static const double kIdentityQuat[4] = {1.0, 0.0, 0.0, 0.0};

// 各外部机体口径下"光轴系三轴"在**外部机体坐标**里的取值。
// 用于手性（det）检查：必须是 +1（正规旋转），不能是 −1（镜像）。
static void ref_optical_axes_in_body(int idx, const AttitudeFrame *frames, Vec3 out[3]) {
    switch (frames[idx]) {
        case AttitudeFrame::kFrd:
            out[0] = Vec3(0, 1, 0); out[1] = Vec3(0, 0, 1); out[2] = Vec3(1, 0, 0);
            break;
        case AttitudeFrame::kFlu:
            out[0] = Vec3(0, -1, 0); out[1] = Vec3(0, 0, -1); out[2] = Vec3(1, 0, 0);
            break;
        default:   // kCamFrd / kCamFlu / kOpticalPassthrough：就是自己的单位基
            out[0] = Vec3(1, 0, 0); out[1] = Vec3(0, 1, 0); out[2] = Vec3(0, 0, 1);
            break;
    }
}

// ============================================================
//  1. 坐标系换算
// ============================================================
static void test_frame_conversion() {
    section("1. 坐标系换算 NED ↔ ENU");

    // 轴向映射：NED(x北 y东 z下) → ENU(x东 y北 z上)
    near_vec(ned_to_enu(Vec3(1, 0, 0)), Vec3(0, 1, 0), 0.0, "NED 北 → ENU +y(北)");
    near_vec(ned_to_enu(Vec3(0, 1, 0)), Vec3(1, 0, 0), 0.0, "NED 东 → ENU +x(东)");
    near_vec(ned_to_enu(Vec3(0, 0, 1)), Vec3(0, 0, -1), 0.0, "NED 下 → ENU −z(下)");
    near_vec(enu_to_ned(Vec3(0, 0, 1)), Vec3(0, 0, -1), 0.0, "ENU 上 → NED −z(上)");

    // 自逆：转两次 = 恒等
    const Vec3 v(1.5, -2.25, 3.125);
    near_vec(convert_vec(convert_vec(v, Frame::kNed, Frame::kEnu), Frame::kEnu, Frame::kNed),
             v, 0.0, "NED→ENU→NED = 恒等");
    // 同系 = 原样
    near_vec(convert_vec(v, Frame::kEnu, Frame::kEnu), v, 0.0, "ENU→ENU = 原样");
    // 模长不变
    near(norm(ned_to_enu(v)), norm(v), 0.0, "换算保模长");
    // 零矢量
    near_vec(ned_to_enu(Vec3{}), Vec3{}, 0.0, "零矢量换算仍为零");
    // 语义包装函数
    near_vec(position_from(v, Frame::kNed, Frame::kEnu), ned_to_enu(v), 0.0, "position_from = ned_to_enu");
    near_vec(velocity_from(v, Frame::kNed, Frame::kEnu), ned_to_enu(v), 0.0, "velocity_from = ned_to_enu");
    near_vec(accel_from(v, Frame::kNed, Frame::kEnu), ned_to_enu(v), 0.0, "accel_from = ned_to_enu");
}

// ============================================================
//  2. 时间对齐
// ============================================================
static void test_time() {
    section("2. 时间对齐（单位 / 回绕 / 跟踪器）");

    near(timestamp_to_s(2500.0, TimeUnit::kMillis), 2.5, 0.0, "ms → s");
    near(timestamp_to_s(2500000.0, TimeUnit::kMicros), 2.5, 0.0, "µs → s");
    near(timestamp_to_s(2.5, TimeUnit::kSeconds), 2.5, 0.0, "s → s");
    near(timestamp_to_s(3000.0, TimeUnit::kMillis, 500.0), 2.5, 0.0, "ms → s（带参考）");
    near(timestamp_to_s(3000000.0, TimeUnit::kMicros, 500000.0), 2.5, 0.0, "µs → s（带参考）");
    near(timestamp_to_s(3.0, TimeUnit::kSeconds, 0.5), 2.5, 0.0, "s → s（带参考）");

    // ---- uint32 ms 回绕（约 49.7 天）----
    near(timestamp_u32_ms_to_s(1000u, 0u), 1.0, 0.0, "u32 ms 常规差");
    // 4294967290 → 5：真实间隔 = (0xFFFFFFFF − 4294967290) + 5 + 1 = 11 ms
    near(timestamp_u32_ms_to_s(5u, 4294967290u), 0.011, 1e-12, "u32 ms 回绕差 = 11 ms");
    near(timestamp_u32_ms_to_s(4294967295u, 0u), 4294967.295, 1e-6, "u32 ms 最大差");

    // ---- uint64 µs ----
    near(timestamp_u64_us_to_s(2500000ull, 0ull), 2.5, 0.0, "u64 µs → s");
    // ★ 关键：本机 SITL 实测 timestamp ≈ Unix epoch µs（1.79e15）。先相减再转 double 才安全。
    {
        const uint64_t t0 = 1792136351209102ull;
        const uint64_t t1 = t0 + 456789ull;                 // +0.456789 s
        near(timestamp_u64_us_to_s(t1, t0), 0.456789, 1e-9, "u64 µs 大数差（epoch 量级）");

        // 反例：绝对量级大到 double 无法表示 1 µs 时，"先转 double 再相减"会得到 0
        //   （2^53 µs ≈ 285 年，此时 double 的 ULP 已 > 1 µs）
        const uint64_t big0 = 9007199254740992ull;          // 2^53
        const uint64_t big1 = big0 + 1ull;                  // +1 µs
        near(timestamp_u64_us_to_s(big1, big0), 1e-6, 1e-18,
             "u64 µs 大数差：整数相减保住 1 µs");
        ok(static_cast<double>(big1) == static_cast<double>(big0),
           "参考：先转 double 再相减会丢 1 µs（故本模块先在整数域相减）");
    }

    // ---- StateTimeTracker：基本累加 ----
    {
        StateTimeTracker tr;
        ok(tr.empty(), "跟踪器初始为空");
        tr.push_u64_us(1000ull);              // 首帧 → t = 0
        near(tr.t_s(), 0.0, 0.0, "首帧 t = 0");
        tr.push_u64_us(1000ull + 20000ull);   // +20 ms
        near(tr.t_s(), 0.020, 1e-15, "累加 20 ms");
        tr.push_u64_us(1000ull + 50000ull);   // +30 ms
        near(tr.t_s(), 0.050, 1e-15, "累加 50 ms");
        ok(tr.frames() == 3u, "帧计数");
        ok(tr.out_of_order() == 0u, "无乱序");
    }

    // ---- StateTimeTracker：uint32 回绕 ----
    {
        StateTimeTracker tr;
        tr.push_u32_ms(4294967290u);          // 距回绕 5 ms
        near(tr.t_s(), 0.0, 0.0, "u32 首帧 t = 0");
        tr.push_u32_ms(10u);                  // 回绕后真实间隔 16 ms
        near(tr.t_s(), 0.016, 1e-12, "u32 回绕累加 16 ms（关键用例）");
        tr.push_u32_ms(20u);
        near(tr.t_s(), 0.026, 1e-12, "u32 回绕后继续累加");
        ok(tr.out_of_order() == 0u, "回绕不算乱序");
    }

    // ---- StateTimeTracker：乱序必须被拒（且不倒退）----
    {
        StateTimeTracker tr;
        tr.push_u64_us(1000000ull);
        tr.push_u64_us(1002000ull);           // +2 ms
        const double t_before = tr.t_s();
        tr.push_u64_us(1000ull);              // 明显乱序（早于首帧）
        ok(tr.out_of_order() == 1u, "乱序被计数");
        near(tr.t_s(), t_before, 0.0, "乱序不改变已累加时间（不会倒退）");
        ok(tr.last_jump_suspect(), "乱序被标记为可疑跳变");
    }

    // ---- StateTimeTracker：参考点（发射时刻 → t = 0）----
    {
        StateTimeTracker tr;
        const uint64_t fmu_t = 1792136351209102ull;   // epoch µs 量级
        tr.push_u64_us(fmu_t);
        tr.push_u64_us(fmu_t + 1000000ull);            // 1 s 后
        near(tr.t_s(), 1.0, 1e-9, "参考点前的绝对时间（相对首帧）");
        tr.set_reference_u64_us(fmu_t + 1000000ull);  // 此刻定为 0
        near(tr.t_s(), 0.0, 1e-12, "参考点归零（发射时刻 = t_ref）");
        tr.push_u64_us(fmu_t + 1456789ull);            // 再 +0.456789 s
        near(tr.t_s(), 0.456789, 1e-9, "参考点后的相对时间（保持 µs 精度）");
        ok(!tr.last_jump_suspect(), "正常帧不报可疑");
    }

    // ---- StateTimeTracker：大跳变提示 ----
    {
        StateTimeTracker tr;
        tr.push_u64_us(0ull);
        tr.push_u64_us(6000000ull);           // +6 s > 5 s 阈值
        ok(tr.last_jump_suspect(), "6 s 跳变被提示（丢帧/时钟跳变的信号）");
        near(tr.t_s(), 6.0, 1e-12, "跳变仍被累加（只提示不丢弃）");
    }

    // ---- 基准自检 ----
    ok(is_safe_to_difference(TimeBase::kSameSource, TimeBase::kSameSource),
       "同源可相减");
    ok(!is_safe_to_difference(TimeBase::kSinceBoot, TimeBase::kUnixEpoch),
       "boot 与 epoch 跨源相减被拦下");
}

// ============================================================
//  3. ★★ 姿态换算（外部机体系 → 光轴系）
// ============================================================
static void test_attitude() {
    section("3. 姿态换算（外部机体系 → 光轴系）");

    // ---- 3.1 单位四元数直通 ----
    {
        const double qid[4] = {1, 0, 0, 0};
        double qout[4];
        Frame wout = Frame::kNed;
        ok(convert_attitude(qid, Frame::kEnu, AttitudeFrame::kOpticalPassthrough, qout, wout),
           "透传：返回成功");
        near_q(qout, qid, 1e-15, "透传：ENU + 光轴系 → 恒等");
        ok(wout == Frame::kEnu, "透传：输出世界系 = ENU");
    }

    // ---- 3.2 ★ 手工算例：无人机水平、机头朝正东（ENU）----
    // 机头朝东 ⇒ 光轴系 +z(光轴) = 东；+x(机右) = 南；+y(机下) = 下
    //
    // 手工推导（逐轴走一遍"机体系 → 外部世界系 → ENU"，不依赖矩阵记忆）：
    //   外部 q_frd = [√2/2,0,0,√2/2]（绕 NED z 下轴 +90° = 北→东）
    //   光轴+x(机右) = FRD(0,1,0) → R → NED(−1,0,0) → ENU(0,−1,0) = 南
    //   光轴+y(机下) = FRD(0,0,1) → R → NED(0,0,1)  → ENU(0,0,−1) = 下
    //   光轴+z(光轴) = FRD(1,0,0) → R → NED(0,1,0)  → ENU(1,0,0)  = 东
    //   ⇒ 目标矩阵 K 三列 = (0,−1,0)、(0,0,−1)、(1,0,0)
    //   ⇒ R2q(K) = [−0.5, 0.5, −0.5, 0.5]（已用独立数值程序验证 R(q)=K，偏差 0）
    {
        // 外部按 NED + FRD 给：无人机水平朝东 ⇒ 机体→NED = 绕 z_NED(下)、即"向北转东" +90°
        const double q_frd[4] = {0.70710678118654752440, 0.0, 0.0,
                                 0.70710678118654752440};
        double qout[4];
        Frame wout = Frame::kNed;
        ok(convert_attitude(q_frd, Frame::kNed, AttitudeFrame::kFrd, qout, wout),
           "FRD→光轴系：返回成功");
        const double expect[4] = {-0.5, 0.5, -0.5, 0.5};
        near_q(qout, expect, 1e-15, "FRD→光轴系：水平朝东 的手算例");
        // 三轴指向复核（用独立参考实现读 q 的列）
        near_vec(ref_axis_in_world(qout, 0), Vec3(0, -1, 0), 1e-15, "  光轴系+x(机右) = 南");
        near_vec(ref_axis_in_world(qout, 1), Vec3(0, 0, -1), 1e-15, "  光轴系+y(机下) = 下");
        near_vec(ref_axis_in_world(qout, 2), Vec3(1, 0, 0), 1e-15, "  光轴系+z(光轴) = 东");
    }

    // ---- 3.3 ★★ 随机姿态 × 5 种外部口径 × 2 种世界系，与独立参考实现逐例比对 ----
    {
        const Frame worlds[2] = {Frame::kNed, Frame::kEnu};
        const AttitudeFrame frames[5] = {AttitudeFrame::kFrd, AttitudeFrame::kFlu,
                                         AttitudeFrame::kCamFrd, AttitudeFrame::kCamFlu,
                                         AttitudeFrame::kOpticalPassthrough};
        double worst = 0.0;
        int cases = 0;
        for (int wi = 0; wi < 2; ++wi) {
            for (int fi = 0; fi < 5; ++fi) {
                for (int trial = 0; trial < 20; ++trial) {
                    double q_in[4];
                    random_quat(q_in);
                    double q_out[4];
                    Frame wout = Frame::kNed;
                    const bool okc = convert_attitude(q_in, worlds[wi], frames[fi],
                                                      q_out, wout);
                    ok(okc, "  随机姿态换算必须成功");
                    if (!okc) continue;
                    ok(wout == Frame::kEnu, "  输出世界系恒为 ENU");

                    Vec3 want[3];
                    ref_optical_axes(q_in, worlds[wi], frames[fi], want);
                    for (int k = 0; k < 3; ++k) {
                        const Vec3 got = ref_axis_in_world(q_out, k);
                        const double d = norm(got - want[k]);
                        if (d > worst) worst = d;
                        ++cases;
                        if (d > 1e-12) {
                            ok(false, "  随机姿态：光轴系三轴与参考不一致");
                        } else {
                            ok(true, "  随机姿态：光轴系三轴与参考一致");
                        }
                    }
                }
            }
        }
        std::printf("  （随机姿态交叉比对 %d 例，最大偏差 %.3g）\n", cases, worst);
    }

    // ---- 3.4 ★ 三种"已是光轴系"的口径必须给出同一个结果 ----
    // CamFrd / CamFlu / OpticalPassthrough 的 ① 都是单位基 ⇒ 结果应完全相同；
    // 差别只在调用方如何**解释** q，而 q 是原样传入的。
    {
        double q_in[4];
        random_quat(q_in);
        double q_a[4], q_b[4], q_c[4];
        Frame wa = Frame::kNed, wb = Frame::kNed, wc = Frame::kNed;
        convert_attitude(q_in, Frame::kEnu, AttitudeFrame::kCamFrd, q_a, wa);
        convert_attitude(q_in, Frame::kEnu, AttitudeFrame::kCamFlu, q_b, wb);
        convert_attitude(q_in, Frame::kEnu, AttitudeFrame::kOpticalPassthrough, q_c, wc);
        near_q(q_a, q_b, 1e-15, "CamFrd 与 CamFlu 结果一致（都不动机体轴）");
        near_q(q_a, q_c, 1e-15, "CamFrd 与透传结果一致");
    }

    // ---- 3.5 ★ 已知姿态：外部只做世界系换算时，机体轴关系可预测 ----
    // NED + FRD 的恒等姿态（机体与 NED 对齐：x=北 y=东 z=下）
    // ⇒ 光轴系 +z(光轴=机头) 应指向北 ⇒ ENU 的 +y
    {
        const double qid[4] = {1, 0, 0, 0};
        double qout[4];
        Frame wout = Frame::kNed;
        ok(convert_attitude(qid, Frame::kNed, AttitudeFrame::kFrd, qout, wout),
           "恒等姿态换算成功");
        near_vec(ref_axis_in_world(qout, 2), Vec3(0, 1, 0), 1e-12,
                 "恒等 FRD：光轴指向北(ENU +y)");
        near_vec(ref_axis_in_world(qout, 0), Vec3(1, 0, 0), 1e-12,
                 "恒等 FRD：光轴系+x(机右) 指向东(ENU +x)");
        near_vec(ref_axis_in_world(qout, 1), Vec3(0, 0, -1), 1e-12,
                 "恒等 FRD：光轴系+y(机下) 指向下(ENU −z)");

        // ENU + FLU 对齐（x=东 y=北 z=上）：光轴(机头) 指向东 ⇒ ENU +x
        double qout2[4];
        Frame wout2 = Frame::kNed;
        ok(convert_attitude(qid, Frame::kEnu, AttitudeFrame::kFlu, qout2, wout2),
           "ENU+FLU 恒等姿态换算成功");
        near_vec(ref_axis_in_world(qout2, 2), Vec3(1, 0, 0), 1e-12,
                 "恒等 FLU：光轴指向东(ENU +x)");
    }

    // ---- 3.6 ★ 手性检查：所有口径都必须是正规旋转（det = +1），不能是镜像 ----
    {
        const AttitudeFrame frames[4] = {AttitudeFrame::kFrd, AttitudeFrame::kFlu,
                                         AttitudeFrame::kCamFrd, AttitudeFrame::kCamFlu};
        for (int fi = 0; fi < 4; ++fi) {
            Vec3 e[3];
            ref_optical_axes_in_body(fi, frames, e);
            const double det = ref_det3(e[0], e[1], e[2]);
            near(det, 1.0, 0.0, "  外部机体口径 → 光轴系 的映射 det = +1（正规旋转）");
        }
    }

    // ---- 3.7 非法输入必须被拒；"未归一化但方向合理"必须被接受 ----
    {
        double qout[4];
        Frame wout = Frame::kEnu;

        const double q_zero[4] = {0, 0, 0, 0};
        ok(!convert_attitude(q_zero, Frame::kNed, AttitudeFrame::kFrd, qout, wout),
           "零四元数被拒（没有姿态信息）");
        near_q(qout, kIdentityQuat, 0.0, "  被拒时输出单位四元数");
        ok(wout == Frame::kNed, "  被拒时不谎报已转换（world_out 不变）");

        const double q_nan[4] = {1.0, 0.0, std::numeric_limits<double>::quiet_NaN(), 0.0};
        ok(!convert_attitude(q_nan, Frame::kNed, AttitudeFrame::kFrd, qout, wout),
           "含 NaN 的四元数被拒");

        const double q_huge[4] = {1e9, 0.0, 0.0, 0.0};
        ok(!convert_attitude(q_huge, Frame::kNed, AttitudeFrame::kFrd, qout, wout),
           "量级离谱（|q|=1e9）被判为非四元数并拒绝");

        // ★ 未归一化但**方向合理**：必须接受并归一化（外推/插值的常见情形）
        const double q_scaled[4] = {0.5, 0.0, 0.0, -0.5};   // |q| = 0.7071
        ok(convert_attitude(q_scaled, Frame::kNed, AttitudeFrame::kFrd, qout, wout),
           "未归一化（|q|=0.707）被接受");
        const double q_unit[4] = {0.70710678118654752440, 0.0, 0.0,
                                  -0.70710678118654752440};
        double q_ref[4];
        Frame wr = Frame::kNed;
        convert_attitude(q_unit, Frame::kNed, AttitudeFrame::kFrd, q_ref, wr);
        near_q(qout, q_ref, 1e-15, "  归一化后与单位四元数结果完全一致");

        const double q_slight[4] = {1.0 + 1e-9, 0, 0, 0};
        ok(convert_attitude(q_slight, Frame::kNed, AttitudeFrame::kFrd, qout, wout),
           "轻微未归一化被接受");
        near(std::sqrt(qout[0] * qout[0] + qout[1] * qout[1] +
                       qout[2] * qout[2] + qout[3] * qout[3]),
             1.0, 1e-15, "  归一化后 |q| = 1");
    }

    // ---- 3.8 normalize_quat ----
    {
        double q[4] = {0, 0, 0, 5};
        const double n = normalize_quat(q);
        near(n, 5.0, 0.0, "normalize_quat 返回归一化前模长");
        // ⚠ normalize_quat 是**原地**归一化 ⇒ 不能拿 q 与单位四元数比"是否没变"
        near_vec(Vec3(q[0], q[1], q[2]), Vec3(0, 0, 0), 1e-15, "  归一化后矢量部分为 0");
        near(q[3], 1.0, 1e-15, "  归一化后 q[3] = 1（原地写回）");
        double z[4] = {0, 0, 0, 0};
        near(normalize_quat(z), 0.0, 0.0, "零四元数返回 0");
        near_q(z, kIdentityQuat, 0.0, "  零四元数 → 单位四元数（不是 NaN）");
        double nanq[4] = {std::numeric_limits<double>::quiet_NaN(), 0, 0, 0};
        near(normalize_quat(nanq), 0.0, 0.0, "NaN 四元数返回 0");
        near_q(nanq, kIdentityQuat, 0.0, "  NaN → 单位四元数");
    }

    // ---- 3.9 便捷包装 attitude_from_ned_frd ----
    {
        // 水平朝东（NED + FRD）：机体→NED = 绕 z_NED(下/正东转) 转 +90°
        const double q_frd[4] = {0.70710678118654752440, 0.0, 0.0,
                                 0.70710678118654752440};
        double q_direct[4], q_wrap[4];
        Frame w = Frame::kNed;
        convert_attitude(q_frd, Frame::kNed, AttitudeFrame::kFrd, q_direct, w);
        ok(attitude_from_ned_frd(q_frd, q_wrap), "attitude_from_ned_frd 成功");
        near_q(q_wrap, q_direct, 1e-15, "attitude_from_ned_frd = convert_attitude(NED, FRD)");
        // 与 3.2 的手工算例互证：同一物理姿态 ⇒ 同一四元数（含 ±q 等价）
        near_q(q_wrap, (const double[4]){-0.5, 0.5, -0.5, 0.5}, 1e-15,
               "  与 3.2 手工算例一致（等价输入 ⇒ 同一姿态）");
    }
}

// ============================================================
//  4. ★ 与 guidance_libs 交叉验证（两份四元数实现必须一致）
// ============================================================
static void test_quat_cross_check() {
    section("4. 四元数旋转：与 guidance_libs 交叉验证 + 语义差异锁定");

    // ---- 4.1 本层的 R(q)·v 必须与独立参考实现（矩阵写法）一致 ----
    {
        double worst = 0.0;
        int n = 0;
        for (int trial = 0; trial < 200; ++trial) {
            double q[4];
            random_quat(q);
            const Vec3 v(rnd01() * 10.0 - 5.0, rnd01() * 10.0 - 5.0, rnd01() * 10.0 - 5.0);

            const Vec3 a = rotate_vec_by_quat(q, v);   // 被测：本层实现
            const Vec3 b = ref_rotate(q, v);           // 参考：矩阵写法
            const double d = norm(a - b);
            if (d > worst) worst = d;
            ++n;
            if (d > 1e-13) ok(false, "rotate_vec_by_quat 与参考实现不一致");
        }
        std::printf("  （%d 例，rotate_vec_by_quat vs 矩阵参考 最大偏差 %.3g）\n", n, worst);
    }

    // ---- 4.2 ★★ 锁定仓库 quat_rotate_vec 的语义（必须与"矩阵参考"一致）----
    //
    // 这一节原本是为了排查一个"看似是转置"的怀疑，最终用完整矩阵对比证明：
    //   **仓库的 quat_rotate_vec 是标准 R(q)·v，不是转置。**
    // 把三条独立路径钉在一起，防止以后有人改坏其中任一实现：
    //   ① sensor.cpp 的 quat_rotate_vec   ② 本层 rotate_vec_by_quat   ③ 本测试的矩阵写法
    {
        double q90[4];
        quat_axis_angle(Vec3(0, 0, 1), PI_REF / 2, q90);   // 绕 +z 转 +90°

        // 三条路径对同一输入必须给出同一结果
        const Vec3 a = quat_rotate_vec(q90, Vec3(1, 0, 0));          // ①
        const Vec3 b = rotate_vec_by_quat(q90, Vec3(1, 0, 0));       // ②
        const Vec3 c = ref_rotate(q90, Vec3(1, 0, 0));               // ③
        near_vec(a, c, 1e-15, "① 仓库 quat_rotate_vec = ③ 矩阵参考");
        near_vec(b, c, 1e-15, "② 本层 rotate_vec_by_quat = ③ 矩阵参考");
        // 解析值：绕 +z 转 90° 把 +x 送到 +y
        near_vec(c, Vec3(0, 1, 0), 1e-15, "绕 +z 转 90°：(1,0,0) → (0,1,0)");
        // 逆旋转把 +x 送到 −y
        near_vec(rotate_vec_by_quat_inv(q90, Vec3(1, 0, 0)), Vec3(0, -1, 0), 1e-15,
                 "本层逆旋转：(1,0,0) → (0,−1,0)");
        near_vec(quat_rotate_vec(q90, Vec3(0, 1, 0)),
                 rotate_vec_by_quat(q90, Vec3(0, 1, 0)), 1e-15,
                 "仓库实现 = 本层实现（同一数学定义）");

        // 逐例三方比对
        double worst_ab = 0.0, worst_bc = 0.0;
        int n = 0;
        for (int trial = 0; trial < 200; ++trial) {
            double q[4];
            random_quat(q);
            const Vec3 v(rnd01() * 4.0 - 2.0, rnd01() * 4.0 - 2.0, rnd01() * 4.0 - 2.0);
            const double dab = norm(quat_rotate_vec(q, v) - rotate_vec_by_quat(q, v));
            const double dbc = norm(rotate_vec_by_quat(q, v) - ref_rotate(q, v));
            if (dab > worst_ab) worst_ab = dab;
            if (dbc > worst_bc) worst_bc = dbc;
            ++n;
            if (dab > 1e-13 || dbc > 1e-13) ok(false, "三方四元数旋转实现不一致");
        }
        std::printf("  （%d 例：仓库 vs 本层 最大偏差 %.3g；本层 vs 矩阵参考 最大偏差 %.3g）\n",
                    n, worst_ab, worst_bc);
    }

    // ---- 4.3 单位四元数 / 恒等 ----
    {
        const double qid[4] = {1, 0, 0, 0};
        near_vec(rotate_vec_by_quat(qid, Vec3(1, 2, 3)), Vec3(1, 2, 3), 0.0,
                 "恒等四元数：v 不变");
        near_vec(rotate_vec_by_quat_inv(qid, Vec3(1, 2, 3)), Vec3(1, 2, 3), 0.0,
                 "恒等四元数：逆旋转也不变");

        // 旋转保模长、正交（RᵀR = I）
        double q[4];
        random_quat(q);
        const Vec3 v(1.5, -2.5, 3.5);
        near(norm(rotate_vec_by_quat(q, v)), norm(v), 1e-15, "旋转保模长");
        near_vec(rotate_vec_by_quat_inv(q, rotate_vec_by_quat(q, v)), v, 1e-14,
                 "R(q)ᵀ·R(q)·v = v（可逆）");

        // 顺序锁定：绕 +z 转 90° 的四元数必须是 [cos45, 0, 0, sin45]
        double q90[4];
        quat_axis_angle(Vec3(0, 0, 1), PI_REF / 2, q90);
        near(q90[0], std::sqrt(0.5), 1e-15, "四元数顺序锁定 q[0] = w = cos45");
        near(q90[3], std::sqrt(0.5), 1e-15, "四元数顺序锁定 q[3] = z = sin45");

        // 共轭
        double qc[4];
        quat_conjugate(q, qc);
        near_q(qc, (const double[4]){q[0], -q[1], -q[2], -q[3]}, 0.0, "共轭定义");
    }
}

// ============================================================
//  5. 组装 GuidanceState
// ============================================================
static void test_fill_state() {
    section("5. 组装 GuidanceState（外部 NED → 内部 ENU）");

    // ---- 5.1 fill_ins ----
    {
        static const double kCamFacingEast[4] = {0.70710678118654752440, -0.70710678118654752440,
                                                 0.0, 0.0};
        GuidanceState st;
        // NED：北 10、东 20、上 30（NED z = −30）
        const Vec3 p_ned(10.0, 20.0, -30.0);
        const Vec3 v_ned(1.0, 2.0, -3.0);
        const double q_frd[4] = {1, 0, 0, 0};
        fill_ins(st, p_ned, v_ned, q_frd, Frame::kNed, AttitudeFrame::kFrd);
        near_vec(st.p_M, Vec3(20.0, 10.0, 30.0), 0.0, "p_M：NED→ENU");
        near_vec(st.v_M, Vec3(2.0, 1.0, 3.0), 0.0, "v_M：NED→ENU");
        near_q(st.q, kCamFacingEast, 1e-12,
               "q：FRD→光轴系（恒等 FRD ⇒ 循环置换）");

        // 已经就是内部口径：应当原样
        GuidanceState st2;
        fill_ins(st2, Vec3(1, 2, 3), Vec3(4, 5, 6), q_frd, Frame::kEnu,
                 AttitudeFrame::kOpticalPassthrough);
        near_vec(st2.p_M, Vec3(1, 2, 3), 0.0, "内部口径位置原样");
        near_vec(st2.v_M, Vec3(4, 5, 6), 0.0, "内部口径速度原样");
        near_q(st2.q, q_frd, 1e-15, "内部口径姿态原样");

        // has_position / has_attitude = false 时不写
        GuidanceState st3;
        st3.p_M = Vec3(9, 9, 9);
        fill_ins(st3, Vec3(1, 1, 1), Vec3(0, 0, 0), q_frd, Frame::kNed,
                 AttitudeFrame::kFrd, /*has_position=*/false, /*has_attitude=*/false);
        near_vec(st3.p_M, Vec3(9, 9, 9), 0.0, "has_position=false 不覆盖位置");
        near_q(st3.q, kIdentityQuat, 0.0,
               "has_attitude=false 不覆盖姿态（保持默认单位四元数）");
        near_vec(st3.v_M, Vec3(0, 0, 0), 0.0, "速度无 has_ 开关，总是写");
    }

    // ---- 5.2 fill_target ----
    {
        GuidanceState st;
        // NED 下的视线（机→目标）：北 0、东 300、上 0 ⇒ ENU 应为 (300, 0, 0)
        const uint32_t f1 = fill_target(st, Vec3(0.0, 300.0, 0.0), Frame::kNed,
                                        true, 300.0, false, Vec3{});
        ok((f1 & kHaveLos) != 0u, "fill_target 填了 los_hat");
        ok((f1 & kHaveRange) != 0u, "fill_target 填了 range");
        near_vec(st.los_hat, Vec3(1, 0, 0), 1e-15, "los_hat：NED→ENU（已单位化）");
        near(st.range, 300.0, 0.0, "range 原样");
        ok(st.valid, "目标存在 ⇒ valid");

        // ★ 未单位化的视线（模长 300）必须被自动单位化
        GuidanceState st2;
        fill_target(st2, Vec3(0.0, 300.0, 0.0), Frame::kNed, true, 300.0, false, Vec3{});
        near(norm(st2.los_hat), 1.0, 1e-15, "未单位化输入被自动单位化");

        // 零视线 ⇒ valid = false（目标丢失）
        GuidanceState st3;
        st3.valid = true;
        fill_target(st3, Vec3{}, Frame::kNed, true, 10.0, false, Vec3{});
        ok(!st3.valid, "零视线 ⇒ valid = false（制导会进 SEARCH）");
        near_vec(st3.los_hat, Vec3{}, 0.0, "  零视线给出零矢量（不是 NaN）");

        // present = false ⇒ valid = false，但 los_hat 仍被填（供日志）
        GuidanceState st4;
        fill_target(st4, Vec3(1, 0, 0), Frame::kEnu, true, 5.0, false, Vec3{},
                    /*target_present=*/false);
        ok(!st4.valid, "present=false ⇒ valid = false");
        near_vec(st4.los_hat, Vec3(1, 0, 0), 0.0, "  仍填 los_hat（便于日志/调试）");

        // 目标速度（真值法才有）
        GuidanceState st5;
        const uint32_t f5 = fill_target(st5, Vec3(1, 0, 0), Frame::kEnu, true, 5.0,
                                        true, Vec3(0, 0, -9.8));
        ok((f5 & kHaveVT) != 0u, "fill_target 填了 v_T");
        near_vec(st5.v_T, Vec3(0, 0, -9.8), 0.0, "  v_T 原样（已在 ENU）");

        // 缺 range：仍应可用（PPN 不需距离）
        GuidanceState st6;
        const uint32_t f6 = fill_target(st6, Vec3(1, 0, 0), Frame::kEnu, false, 0.0,
                                        false, Vec3{});
        ok((f6 & kHaveRange) == 0u, "无测距 ⇒ 未填 range");
        ok(st6.valid, "  无测距仍算有效帧（PPN 不含距离项）");
    }

    // ---- 5.3 fill_target_accel ----
    {
        GuidanceState st;
        fill_target_accel(st, Vec3(0.0, 0.0, -9.8), Frame::kNed, true);
        near_vec(st.a_T_perp, Vec3(0.0, 0.0, 9.8), 0.0,
                 "a_T_perp：NED 的重力(−z) → ENU 的 −z");
        GuidanceState st2;
        st2.a_T_perp = Vec3(7, 7, 7);
        fill_target_accel(st2, Vec3(1, 1, 1), Frame::kNed, false);
        near_vec(st2.a_T_perp, Vec3{}, 0.0, "has_ata=false ⇒ 清零（不要用脏值算 APN）");
    }

    // ---- 5.4 to_guidance_state（整帧入口）----
    {
        RawFrame raw;
        raw.world = Frame::kNed;
        raw.att = AttitudeFrame::kFrd;
        raw.ins.p = Vec3(10.0, 20.0, -30.0);      // NED
        raw.ins.v = Vec3(1.0, 2.0, -3.0);
        raw.ins.q[0] = 1.0;
        raw.target.los = Vec3(0.0, 300.0, 0.0);   // NED，未单位化
        raw.target.has_range = true;
        raw.target.range_m = 300.0;

        const GuidanceState st = to_guidance_state(raw);
        near_vec(st.p_M, Vec3(20.0, 10.0, 30.0), 0.0, "整帧：位置");
        near_vec(st.v_M, Vec3(2.0, 1.0, 3.0), 0.0, "整帧：速度");
        near_vec(st.los_hat, Vec3(1, 0, 0), 1e-15, "整帧：视线");
        near(st.range, 300.0, 0.0, "整帧：距离");
        ok(st.valid, "整帧：valid");
    }

    // ---- 5.5 输出换算 ----
    {
        GuidanceCmd cmd;
        cmd.a_cmd = Vec3(1.0, 2.0, -3.0);        // ENU
        double xyz[3];
        cmd_accel_to_ned(cmd, xyz);
        near(xyz[0], 2.0, 0.0, "指令出：a_NED.x = a_ENU.y（东→NED y）");
        near(xyz[1], 1.0, 0.0, "指令出：a_NED.y = a_ENU.x");
        near(xyz[2], 3.0, 0.0, "指令出：a_NED.z = −a_ENU.z");
        near_vec(accel_cmd_ned(Vec3(1, 2, -3)), Vec3(2, 1, 3), 0.0, "accel_cmd_ned 一致");
    }
}

// ============================================================
//  6. ★ 上游模拟器：不同口径描述同一场景 → 必须得到同一个内部状态
// ============================================================
static void test_upstream_simulator() {
    section("6. 上游模拟器（同一场景 × 多口径 → 同一个 GuidanceState）");

    // 场景：无人机在 ENU (东 0, 北 0, 上 100)，速度朝东 20 m/s，**机头朝东**、水平
    //       目标在 ENU (东 300, 北 0, 上 100)。
    //
    // ★ 三个口径必须描述**同一个物理姿态**，适配后得到同一个 GuidanceState。
    //   本条测试是本模块存在意义的直接证明（"外部口径差异被适配层吃掉了"）。
    //   三种输入四元数（都用程序反解 + 回代验证过，适配层输出完全一致）：
    //     A) NED + FRD : 绕 NED z(下) +90°（北→东）   → [√2/2, 0, 0, √2/2]
    //     B) ENU + 光轴: 直接给内部口径（理想情况）    → [−0.5, 0.5, −0.5, 0.5]
    //     C) NED + FLU : 同一物理姿态在 FLU 下         → [0, √2/2, √2/2, 0]
    //   期望输出（光轴系→ENU）——物理标准，与运动方向无关：
    //     光轴系 +z(光轴/机头) = 东 (1,0,0)、+x(机右) = 南 (0,−1,0)、+y(机下) = 下 (0,0,−1)
    //     ⇒ q = [−0.5, 0.5, −0.5, 0.5]（经 R 矩阵回代验证三列严格一致）
    GuidanceState ref;
    ref.p_M = Vec3(0.0, 0.0, 100.0);
    ref.v_M = Vec3(20.0, 0.0, 0.0);
    ref.q[0] = -0.5;
    ref.q[1] = 0.5; ref.q[2] = -0.5; ref.q[3] = 0.5;
    ref.los_hat = Vec3(1.0, 0.0, 0.0);
    ref.range = 300.0;
    ref.valid = true;

    // 口径 A：NED + FRD（PX4 / MAVLink 最常见）
    {
        RawFrame raw;
        raw.world = Frame::kNed;
        raw.att = AttitudeFrame::kFrd;
        // 位置：ENU (0,0,100) → NED (北 0, 东 0, 下 −100)
        raw.ins.p = Vec3(0.0, 0.0, -100.0);
        // 速度：ENU (20,0,0) → NED (vN 0, vE 20, vD 0)
        raw.ins.v = Vec3(0.0, 20.0, 0.0);
        // 姿态：水平朝东的 FRD 四元数（绕 NED 下轴、即"北→东" +90°）
        raw.ins.q[0] = 0.70710678118654752440;
        raw.ins.q[1] = 0.0; raw.ins.q[2] = 0.0; raw.ins.q[3] = 0.70710678118654752440;
        // 视线：ENU (1,0,0) → NED (北 0, 东 1, 下 0)，故意不单位化（×123.4）
        raw.target.los = Vec3(0.0, 123.4, 0.0);
        raw.target.has_range = true;
        raw.target.range_m = 300.0;

        const GuidanceState st = to_guidance_state(raw);
        near_vec(st.p_M, ref.p_M, 1e-12, "口径A(NED+FRD)：位置");
        near_vec(st.v_M, ref.v_M, 1e-12, "口径A(NED+FRD)：速度");
        near_vec(st.los_hat, ref.los_hat, 1e-12, "口径A(NED+FRD)：视线");
        near(st.range, ref.range, 1e-12, "口径A(NED+FRD)：距离");
        near_q(st.q, ref.q, 1e-12, "口径A(NED+FRD)：姿态");
        ok(invariants_hold(st, GuidanceCmd{}), "口径A：不变量全部成立");
    }

    // 口径 B：ENU + 光轴系（对方已经按我们的口径给 —— 理想情况）
    {
        RawFrame raw;
        raw.world = Frame::kEnu;
        raw.att = AttitudeFrame::kOpticalPassthrough;
        raw.ins.p = ref.p_M;
        raw.ins.v = ref.v_M;
        for (int i = 0; i < 4; ++i) raw.ins.q[i] = ref.q[i];
        raw.target.los = Vec3(300.0, 0.0, 0.0);   // 未单位化
        raw.target.has_range = true;
        raw.target.range_m = 300.0;

        const GuidanceState st = to_guidance_state(raw);
        near_vec(st.p_M, ref.p_M, 1e-12, "口径B(ENU+光轴)：位置");
        near_vec(st.v_M, ref.v_M, 1e-12, "口径B(ENU+光轴)：速度");
        near_vec(st.los_hat, ref.los_hat, 1e-12, "口径B(ENU+光轴)：视线");
        near_q(st.q, ref.q, 1e-15, "口径B(ENU+光轴)：姿态（原样透传）");
        ok(invariants_hold(st, GuidanceCmd{}), "口径B：不变量全部成立");
    }

    // 口径 C：NED + FLU（ROS 惯例）
    //   同一物理姿态（机头朝东、水平）在 FLU 下的四元数 = [0, √2/2, √2/2, 0]
    //   （由"给定期望输出反解 R(q_in) = P·R(期望)·Pᵀ"得到，并已回代验证）
    {
        RawFrame raw;
        raw.world = Frame::kNed;
        raw.att = AttitudeFrame::kFlu;
        raw.ins.p = Vec3(0.0, 0.0, -100.0);
        raw.ins.v = Vec3(0.0, 20.0, 0.0);
        raw.ins.q[0] = 0.0;
        raw.ins.q[1] = 0.70710678118654752440;
        raw.ins.q[2] = 0.70710678118654752440;
        raw.ins.q[3] = 0.0;
        raw.target.los = Vec3(0.0, 123.4, 0.0);
        raw.target.has_range = true;
        raw.target.range_m = 300.0;

        const GuidanceState st = to_guidance_state(raw);
        near_vec(st.p_M, ref.p_M, 1e-12, "口径C(NED+FLU)：位置");
        near_vec(st.los_hat, ref.los_hat, 1e-12, "口径C(NED+FLU)：视线");
        near_q(st.q, ref.q, 1e-12, "口径C(NED+FLU)：姿态（FLU→光轴系）");
    }

    // 口径 D：真值法（带目标速度与加速度）—— NED 下目标自由落体
    {
        RawFrame raw;
        raw.world = Frame::kNed;
        raw.att = AttitudeFrame::kFrd;
        raw.ins.p = Vec3(0.0, 0.0, -100.0);
        raw.ins.v = Vec3(0.0, 20.0, 0.0);
        raw.ins.q[0] = 0.70710678118654752440;
        raw.ins.q[3] = 0.70710678118654752440;
        raw.target.los = Vec3(0.0, 1.0, 0.0);
        raw.target.has_range = true;
        raw.target.range_m = 300.0;
        raw.target.has_vT = true;
        raw.target.vT = Vec3(0.0, 5.0, 0.0);      // NED：向东北飞 5 m/s

        GuidanceState st = to_guidance_state(raw);
        near_vec(st.v_T, Vec3(5.0, 0.0, 0.0), 1e-12, "口径D：v_T NED→ENU");
        fill_target_accel(st, Vec3(0.0, 0.0, -9.8), Frame::kNed, true);
        near_vec(st.a_T_perp, Vec3(0.0, 0.0, 9.8), 1e-12, "口径D：重力 NED→ENU");
    }
}

// ============================================================
//  7. ★ 不变量自检（state_check）
// ============================================================
static void test_checks() {
    section("7. 不变量自检");

    // ---- 7.1 正常帧：全过 ----
    {
        GuidanceState st;
        st.t = 0.456;
        st.los_hat = Vec3(1, 0, 0);
        st.lam_dot = 0.012;
        st.range = 12.3;
        st.valid = true;
        st.p_M = Vec3(0, 0, 10);
        st.v_M = Vec3(20, 0, 0);
        st.q[0] = 1.0;

        const Report rep = check_guidance_state(st);
        if (!rep.ok()) std::printf("%s\n", rep.detail().c_str());
        ok(rep.ok(), "正常 state 无 error");
        ok(rep.n_error == 0, "  错误计数为 0");
    }

    // ---- 7.2 |q| 偏离 1 必须被抓住 ----
    {
        GuidanceState st;
        st.los_hat = Vec3(1, 0, 0);
        st.range = 10.0;
        st.q[0] = 2.0;                              // |q| = 2
        const Report rep = check_guidance_state(st);
        ok(!rep.ok(), "|q|=2 被判不通过");
        ok(rep.has(CheckCode::kQuatNotUnit), "  且 code = kQuatNotUnit");
    }

    // ---- 7.3 |los_hat| 偏离 1（上游没单位化）----
    {
        GuidanceState st;
        st.los_hat = Vec3(300, 0, 0);
        st.range = 300.0;
        ok(!check_guidance_state(st).ok(), "|los_hat|=300 被判不通过");
        ok(check_guidance_state(st).has(CheckCode::kLosNotUnit), "  code = kLosNotUnit");
        // 允许"不要求单位矢量"的宽松模式（给非单位矢量上游用）
        ok(check_guidance_state(st, /*expected_los_units=*/false).ok(),
           "  宽松模式下只要求非零");
        GuidanceState z;
        z.los_hat = Vec3{};
        z.range = 10.0;
        ok(!check_guidance_state(z, false).ok(), "  宽松模式仍拦零矢量");
    }

    // ---- 7.4 range 为负 ----
    {
        GuidanceState st;
        st.los_hat = Vec3(1, 0, 0);
        st.range = -1.0;
        ok(check_guidance_state(st).has(CheckCode::kRangeNegative), "range<0 被抓");
    }

    // ---- 7.5 NaN 必须被抓 ----
    {
        GuidanceState st;
        st.los_hat = Vec3(std::numeric_limits<double>::quiet_NaN(), 0, 0);
        st.range = 10.0;
        const Report rep = check_guidance_state(st);
        ok(!rep.ok(), "los_hat 含 NaN 被判不通过");
        ok(rep.has(CheckCode::kNotFinite), "  code = kNotFinite");

        GuidanceState st2;
        st2.los_hat = Vec3(1, 0, 0);
        st2.range = 10.0;
        st2.v_M = Vec3(std::numeric_limits<double>::infinity(), 0, 0);
        ok(check_guidance_state(st2).has(CheckCode::kNotFinite), "v_M 含 Inf 被抓");
    }

    // ---- 7.6 报告文本 ----
    {
        GuidanceState st;
        st.los_hat = Vec3(3, 0, 0);
        st.range = 10.0;
        const Report rep = check_guidance_state(st);
        ok(rep.detail().find("los_hat") != std::string::npos, "报告里点名了出问题的字段");
        ok(rep.summary().find("错误") != std::string::npos, "摘要里含错误计数");
        GuidanceState good;
        good.los_hat = Vec3(1, 0, 0);
        good.range = 10.0;
        ok(check_guidance_state(good).detail().find("全部通过") != std::string::npos,
           "全过时给出'全部通过'");
    }

    // ---- 7.7 cmd 检查：限幅自洽 ----
    {
        GuidanceCmd cmd;
        cmd.a_cmd = Vec3(10.0, 0.0, 0.0);
        cmd.a_cmd_norm = 10.0;
        cmd.a_cmd_unsat = Vec3(10.0, 0.0, 0.0);
        cmd.mode = GuidanceMode::PN;
        cmd.valid = true;
        ok(check_guidance_cmd(cmd, 10.0, true).ok(), "恰好饱和的 cmd 自洽");

        GuidanceCmd over = cmd;
        over.a_cmd = Vec3(20.0, 0.0, 0.0);
        over.a_cmd_norm = 20.0;
        over.a_cmd_unsat = Vec3(20.0, 0.0, 0.0);
        over.sat = true;
        const Report rep = check_guidance_cmd(over, 10.0, true);
        ok(!rep.ok(), "限幅后仍超限被判不通过");
        ok(rep.has(CheckCode::kCmdOverLimit), "  code = kCmdOverLimit");

        GuidanceCmd mismatch = cmd;
        mismatch.a_cmd_norm = 99.0;
        ok(check_guidance_cmd(mismatch).has(CheckCode::kCmdNormMismatch),
           "|a_cmd| 与 a_cmd_norm 不一致被抓");

        GuidanceCmd fake_sat = cmd;
        fake_sat.sat = true;                    // 声称饱和，实际没超限
        ok(!check_guidance_cmd(fake_sat, 10.0, true).ok(),
           "sat=true 但限幅前未超限被抓");

        GuidanceCmd search;
        search.mode = GuidanceMode::SEARCH;
        search.valid = false;
        ok(check_guidance_cmd(search).ok(), "SEARCH + 零指令 + valid=false 自洽");
        GuidanceCmd bad_search;
        bad_search.mode = GuidanceMode::SEARCH;
        bad_search.a_cmd = Vec3(1, 0, 0);
        bad_search.a_cmd_norm = 1.0;
        ok(!check_guidance_cmd(bad_search).ok(), "SEARCH 却输出非零指令被抓");
    }

    // ---- 7.8 ★ 真闭环：用手工几何跑 guidance_step，再体检指令 ----
    {
        GuidanceState st;
        st.t = 1.0;
        st.los_hat = Vec3(1, 0, 0);
        st.range = 100.0;
        st.valid = true;
        st.v_M = Vec3(20.0, 0.0, 0.0);
        st.v_T = Vec3(-5.0, 3.0, 0.0);          // 有横向相对运动 ⇒ ω ≠ 0
        st.q[0] = 1.0;

        GuidanceParams pp;
        pp.law = GuidanceLaw::PPN;
        pp.N = 4.0;
        pp.a_max = 30.0;
        GuidanceMemory mem;
        const GuidanceCmd cmd = guidance_step(st, pp, mem);

        ok(cmd.mode == GuidanceMode::PN, "PPN 闭环：mode = PN");
        ok(norm(cmd.a_cmd) > 0.0, "PPN 闭环：有非零指令");
        const Report rep = check_frame(st, cmd, pp.a_max, true);
        if (!rep.ok()) std::printf("%s\n", rep.detail().c_str());
        ok(rep.ok(), "PPN 闭环：state+cmd 不变量全部成立");
        ok(invariants_hold(st, cmd, pp.a_max, true), "invariants_hold() 一致");

        // 违反限幅的构造必须被 invariants_hold 抓到
        GuidanceState st2 = st;
        GuidanceParams pp2 = pp;
        pp2.a_max = 0.5;                        // 强行把上限压到很小 ⇒ 必然饱和
        GuidanceMemory mem2;
        const GuidanceCmd cmd2 = guidance_step(st2, pp2, mem2);
        ok(cmd2.sat, "  小 a_max 触发饱和");
        ok(invariants_hold(st2, cmd2, pp2.a_max, true), "  饱和后不变量仍成立（矢量等比缩放）");

        // 目标丢失 ⇒ SEARCH + 零指令
        GuidanceState st3 = st;
        st3.valid = false;
        GuidanceMemory mem3;
        const GuidanceCmd cmd3 = guidance_step(st3, pp, mem3);
        ok(cmd3.mode == GuidanceMode::SEARCH, "valid=false ⇒ SEARCH");
        ok(norm(cmd3.a_cmd) == 0.0, "  SEARCH 输出零指令");
        ok(invariants_hold(st3, cmd3, pp.a_max, true), "  SEARCH 帧不变量成立");
    }
}

// ============================================================
//  main
// ============================================================
int main() {
    std::printf("数据对齐适配层单元测试 test_state_adapter\n");
    std::printf("（内部口径：ENU + 四元数[w,x,y,z] + 秒 + SI + 光轴系）\n");

    test_frame_conversion();
    test_time();
    test_attitude();
    test_quat_cross_check();
    test_fill_state();
    test_upstream_simulator();
    test_checks();

    std::printf("\n----------------------------------------\n");
    std::printf("通过 %d, 失败 %d\n", g_pass, g_fail);
    std::printf("%s\n", (g_fail == 0) ? "ALL PASS" : "SOME TESTS FAILED");
    return (g_fail == 0) ? 0 : 1;
}
