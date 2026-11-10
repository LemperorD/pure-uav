#pragma once

#include "mission/mission_types.hpp"

namespace pure_uav::app::mission {

// 单线程调用；tick 不阻塞、不发 MAVLink、不读取设备、不分配内存。
class MissionController {
  public:
    // 非法/非有限参数抛出 std::invalid_argument，仅在构造时校验。
    explicit MissionController(MissionConfig config = {});
    MissionOutput tick(const MissionInput& input);
    // 只能在宿主已停止执行上一轮任务的输出后调用。
    void reset();

    [[nodiscard]] MissionState state() const {
        return state_;
    }
    [[nodiscard]] MissionReason reason() const {
        return reason_;
    }
    [[nodiscard]] bool hasHome() const {
        return have_home_;
    }
    [[nodiscard]] Vec3 homeNed() const {
        return home_ned_;
    }

  private:
    void enter(MissionState state, double now_s);
    void abort(MissionReason reason, double now_s);
    void returnHome(MissionReason reason, const MissionInput& input);
    bool poseUsable(const MissionInput& input) const;
    bool targetUsable(const MissionInput& input) const;
    bool settled(const MissionInput& input, const Vec3& destination);
    bool requestDue(double now_s);
    MissionOutput output(const MissionInput& input, MissionState previous);

    MissionConfig config_;
    MissionState state_ = MissionState::kIdle;
    MissionReason reason_ = MissionReason::kNone;
    double entered_s_ = 0.0;
    double started_s_ = 0.0;
    double last_tick_s_ = -1.0;
    double settled_since_s_ = -1.0;
    double last_request_s_ = -1.0;
    double last_target_s_ = -1.0;
    std::uint64_t last_target_boot_us_ = 0;
    unsigned confirmed_frames_ = 0;
    bool have_home_ = false;
    bool strike_confirmed_ = false;
    bool holding_after_loss_ = false;
    Vec3 home_ned_;
    Vec3 takeoff_ned_;
    Vec3 search_ned_;
    Vec3 loss_hold_ned_;
};

}  // namespace pure_uav::app::mission
