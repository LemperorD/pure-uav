#pragma once

#include <sys/types.h>

#include <atomic>
#include <string>

namespace pure_uav::camera {

class CameraServerProcess {
  public:
    CameraServerProcess() = default;
    ~CameraServerProcess();
    CameraServerProcess(const CameraServerProcess&) = delete;
    CameraServerProcess& operator=(const CameraServerProcess&) = delete;

    bool start(const std::string& config, const std::string& channel, int timeout_ms, const std::atomic<bool>& interrupted, std::string& error);
    void stop() noexcept;

  private:
    pid_t pid_ = -1;
};

}  // namespace pure_uav::camera
