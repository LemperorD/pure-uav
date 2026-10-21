#pragma once
// =============================================================================
//  一次采样的数据载体（外部口径）
//
//  【为什么单独拆一个文件】
//    制导核的输入契约是 `adapter::RawFrame`（p / v / q / los / range / v_T），
//    但它**没有时间戳字段**，而机载上"飞控帧"和"视觉帧"是**各自到达、频率不同**的。
//    所以这里把"数据 + 采样时刻"绑定成一个采样包，两种采样分开：
//        · InsSample    飞控惯导（约 50 Hz）
//        · TargetSample 视觉/检测（约 30 Hz，取决于相机）
//    ⇒ 主循环永远拿"最近一帧"，**不要每帧等新数据**（否则制导周期被相机拖死）。
//
//  【★ 目标数据为什么是"像素"而不是"视线矢量"】
//    与视觉组的既有约定（《视觉制导接口约定》§6）是：视觉给 **曝光时刻 + 检测框中心像素 (u,v)**
//    （+ 可选框尺寸、置信度、距离）。**像素 → 相机系 → 机体 → 世界** 这三步由我们做，
//    理由：内参/外参是我们这边现成的（VIO 分支 `src/vio/extrinsic/results/final/`），
//    而 `R_惯性←机体` 必须用**我们的**姿态 `q`（时延最可控）—— 让检测侧去转会把跨模块
//    时延和坐标责任混在一起（正是《数据对齐问题清单》#20 那个坑）。
//
//    换算链（实现时照这个做）：
//        d_cam = ( (u − cx)/fx , (v − cy)/fy , 1 )  → 归一化
//        d_body = R_body←cam · d_cam                 （外参，相机→机体）
//        û_world = R(q) · d_body                     （姿态 q：机体→世界）
//        然后交给 adapter::fill_target(û_world, ...) 做单位化与有效性判定
//
//  【★ 时间戳约定（必须遵守，见《后续注意问题.md》§3.1）】
//    `boot_us` = **飞控 boot 时钟的微秒**（不是相机时钟、不是 Unix epoch）。
//      · 飞控回传只给 `time_boot_ms`（uint32 ms）⇒ ×1000 转 µs 即可，**同一时钟**；
//      · 相机/视觉时钟（可能是 Unix µs 或相机自身时钟）必须先用 `SystemTime`
//        （同帧给出 time_unix_usec + time_boot_ms）换算过来：
//            offset_us = time_unix_usec − time_boot_ms × 1000
//            boot_us   = 视觉时间戳 − offset_us
//      · 换算是**单向**的（只做 相机→boot），且 `offset_us` 要持续维护（SYSTEM_TIME 约 1 Hz）。
//
//  【★ 采样时刻 ≠ 收到时刻】
//    这里填的是**数据产生的那一刻**（飞控/相机打的时间戳），不是"我们读到的时刻"。
//    用收到时刻会让采样抖动直接变成 ω 差分的噪声（ω = û × dû/dt，分母是 dt）。
//
//  【为什么用 uint64 µs 而不是 double 秒】
//    内部制导要 double 秒，但**先做整数相减**再转 double 才不丢精度（µs 大数 ≈1e15 已逼近 2^53）。
// =============================================================================

#include <cstdint>

#include "vec3.hpp"   // pure_uav::Vec3（src/common/common_libs/include/vec3.hpp）

namespace pure_uav {
namespace app {

// ---------------------------------------------------------------- 飞控惯导采样
//
// 对应通信模块的 `LocalPosition` + `Attitude`（两者都带 time_boot_ms）。
// ⚠ 位置/速度的系统与"初始机体坐标系"约定有关：无 GPS 时飞控的 local 系
//   以**初始机头**为 x 轴、z 朝下（详见《后续注意问题.md》§4.1）。
//   这里按**外部口径**原样承载，换算统一交给 `adapter::to_guidance_state()`。
struct InsSample {
    // ★ 采样时刻（飞控 boot 时钟，µs）。0 表示"尚未收到过任何帧"
    std::uint64_t boot_us = 0;

    Vec3   p;                                  // 位置（m，外部 NED 口径）
    Vec3   v;                                  // 速度（m/s，外部 NED 口径）
    double q[4] = {1.0, 0.0, 0.0, 0.0};        // 姿态四元数 [w,x,y,z]（机体→世界）

    bool has_position = false;                 // 该帧是否带位置（飞控断流时会缺）
    bool has_attitude = false;                 // 该帧是否带姿态
};

// ---------------------------------------------------------------- 目标采样（检测输出）
//
// ⚠ 目前检测模块**还没有代码**（`detect` 分支为空，只有 `assets/docs/detection.md` 任务书），
//   这里的字段是按《视觉制导接口约定》§6 与任务书（YOLO + PnP 测距）定的期望；
//   真正接线前先和检测组确认一遍。
//
// 缺数据时的降级（主循环必须知道）：
//   · 没有 `v_T`（真机常态）⇒ `GuidanceParams::los_rate_source` 必须是 `FROM_LOS_DIFFERENCE`；
//   · 没有 `range`            ⇒ `t_go` 与 TPN/APN 的 `Vc` 都失准 ⇒ 实际只能用 PPN；
//   · `present = false`       ⇒ 制导进 SEARCH（输出零指令），**上层必须自己接管**。
struct TargetSample {
    // ★ 采样时刻（同上，飞控 boot 时钟 µs）。视觉帧在这里填**曝光时刻**换算后的值
    std::uint64_t boot_us = 0;

    // ---- 检测框（相机像素域）----
    double u_px = 0.0;                  // 检测框中心 x（像素）  ← 约定里的必须项
    double v_px = 0.0;                  // 检测框中心 y（像素）  ← 约定里的必须项
    bool   has_box  = false;            // 是否带框（带框才能用尺寸反推距离）
    double w_px     = 0.0;              // 框宽（像素）：R ≈ fx · S / w_px（S = 目标物理尺寸）
    double h_px     = 0.0;              // 框高（像素）
    double conf     = 0.0;              // 置信度 0~1（给滤波器加权 / 门限）
    int    class_id = -1;               // 类别（大装甲板 / 弹丸 / …）；-1 = 未给

    bool present = false;               // 本帧是否真的看到目标（检测置信度低于门限 ⇒ false）

    // ---- 距离（可选；三种来源任选其一）----
    bool   has_range = false;           // 是否测得距离
    double range_m   = 0.0;             // 距离（m）：PnP / 尺寸反推 / 直接测距（激光、深度）

    // ---- 目标速度（m/s，外部 NED 口径）----
    // **真机拿不到** ⇒ has_v_T 恒为 false；仅仿真真值源会填。
    // 填 false 时制导核会自动改用 LOS 差分法求 ω。
    bool has_v_T = false;
    Vec3 v_T;
};

}  // namespace app
}  // namespace pure_uav
