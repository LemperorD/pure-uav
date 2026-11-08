#pragma once
#include "communicator.hpp"
#include "pure_uav/vio_runtime.hpp"
namespace pure_uav::app {
enum class HeartbeatWaitResult { connected, stopped, timeout };
HeartbeatWaitResult waitForHeartbeat(const Communicator& comm, const std::atomic<bool>& stop, double timeout_s);

class CommImuSource final : public VioImuSource {
  public:
    explicit CommImuSource(Communicator& comm) : comm_(comm) {
    }
    bool pop(TimedImu& out) override;
    uint64_t session() const override {
        return comm_.reconnectCount();
    }
    bool armed() const override {
        return comm_.isArmed();
    }
    uint64_t dropped() const override {
        return comm_.droppedImu();
    }

  private:
    Communicator& comm_;
};
}  // namespace pure_uav::app
