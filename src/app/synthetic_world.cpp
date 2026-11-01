// =============================================================================
//  合成世界实现（详见头注释）
//
//  三个角色都在本文件：真值惯导 / 真值视觉（→ 像素）/ 理想执行机构。
//  ⚠ 这里**故意不做**：姿态环、阻力、重力、噪声、延时。
//    目的是让"制导算得对不对"成为唯一变量；逼真度交给 XTDrone2 仿真。
// =============================================================================

#include "synthetic_world.hpp"

#include <cmath>

#include "quat.hpp"   // quat_rotate_vec_inv()

namespace pure_uav {
namespace app {

// ---------------------------------------------------------------- 相机
CameraModel demo_camera() {
    CameraModel c;
    c.fx = 600.0;
    c.fy = 600.0;
    c.cx = 320.0;   // 640×480 的主点
    c.cy = 240.0;
    const double R[9] = {0, 0, 1, 1, 0, 0, 0, 1, 0};   // 行主序：相机(x右,y下,z前) → 机体 FRD
    for (int i = 0; i < 9; ++i) c.R_body_cam[i] = R[i];
    return c;
}

// ---------------------------------------------------------------- 构造
SyntheticWorld::SyntheticWorld(const CameraModel &cam, const SceneConfig &sc)
    : cam_(cam), p_(sc.p0), v_(sc.v0), plate_(sc.plate), dt_(sc.dt), t_us_(sc.t0_us) {
    q_[0] = 1.0;   // 机体与局部系对齐（单位四元数）
}

// ---------------------------------------------------------------- 推进真值
void SyntheticWorld::step() {
    v_ = v_ + a_cmd_ * dt_;
    p_ = p_ + v_ * dt_;
    t_us_ += static_cast<std::uint64_t>(dt_ * 1e6);
    a_cmd_ = Vec3{};   // 指令不重复生效
}

// ---------------------------------------------------------------- 真值惯导
bool SyntheticWorld::poll(InsSample &out) {
    out.boot_us      = t_us_;
    out.p            = p_;
    out.v            = v_;
    for (int i = 0; i < 4; ++i) out.q[i] = q_[i];
    out.has_position = true;
    out.has_attitude = true;
    return true;   // 合成世界每拍都有新帧
}

// ---------------------------------------------------------------- 真值视觉 → 像素
bool SyntheticWorld::poll(TargetSample &out) {
    const Vec3 d_world = plate_ - p_;              // 世界系视线（未归一化）
    const double R = norm(d_world);
    if (!(R > 1e-6)) return false;
    const Vec3 los_world = d_world / R;

    // 世界 → 机体（用 q 的逆）→ 相机系（用外参的逆）
    const Vec3 d_body = quat_rotate_vec_inv(q_, los_world);
    const Vec3 d_cam  = mat3_mul_transpose(cam_.R_body_cam, d_body);
    if (!(d_cam.z > 1e-6)) return false;            // 目标在相机后方（飞越后即如此）

    // 针孔投影
    out.boot_us   = t_us_;
    out.u_px      = cam_.cx + cam_.fx * (d_cam.x / d_cam.z);
    out.v_px      = cam_.cy + cam_.fy * (d_cam.y / d_cam.z);
    out.has_box   = false;                          // 演示里不造框尺寸
    out.w_px      = 0.0;
    out.h_px      = 0.0;
    out.conf      = 1.0;
    out.class_id  = 0;                              // 假定就是装甲板
    out.present   = true;
    out.has_range = true;                           // 真值测距（相当于激光/PnP）
    out.range_m   = R;
    out.has_v_T   = true;                           // 靶静止 ⇒ 可用真值法算 ω
    out.v_T       = Vec3(0.0, 0.0, 0.0);
    return true;
}

// ---------------------------------------------------------------- 理想执行机构
bool SyntheticWorld::send(const GuidanceOutput &out) {
    ++sends_;
    last_ = out;
    if (out.ctrl == ControlMode::kAcceleration) {
        a_cmd_ = out.a_cmd_ned;
    } else {
        // kVelocity（零速度悬停）：演示里当"零加速度"处理。
        // ⚠ 真机上零速度会走 PX4 速度环把飞机刹住，这里没有速度环，只能近似。
        a_cmd_ = Vec3{};
    }
    if (out.sat) ++sat_;
    if (out.mode == PureGuidanceMode::HOLD) ++hold_;
    if (out.mode == PureGuidanceMode::SEARCH) ++search_;
    return true;
}

// ---------------------------------------------------------------- 工具
Vec3 SyntheticWorld::mat3_mul_transpose(const double R[9], const Vec3 &v) {
    return Vec3(R[0] * v.x + R[3] * v.y + R[6] * v.z,
                R[1] * v.x + R[4] * v.y + R[7] * v.z,
                R[2] * v.x + R[5] * v.y + R[8] * v.z);
}

// ---------------------------------------------------------------- 闭环驱动
ClosedLoopReport run_synthetic_closed_loop(SyntheticWorld &world, GuidanceLoop &loop,
                                           int ticks, TickHook hook, void *user) {
    ClosedLoopReport rep;
    for (int k = 0; k < ticks; ++k) {
        world.step();                       // 推进真值（用上一拍指令）
        GuidanceOutput out;
        const LoopStatus st = loop.run_once(&out);
        if (st != LoopStatus::kOk) {
            // kSensorTimeout 在演示里是**正常现象**：飞越目标后目标出画 ⇒ 不再有新帧。
            rep.stop = st;
            rep.target_lost = (st == LoopStatus::kSensorTimeout);
            break;
        }

        const double R = world.true_range();
        if (R < rep.min_r) {
            rep.min_r = R;
            rep.min_r_tick = k;
        }
        if (hook) hook(user, k, world, out);
        ++rep.ticks;
    }
    return rep;
}

}  // namespace app
}  // namespace pure_uav
