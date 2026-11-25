#pragma once

#include <atomic>
#include <exception>
#include <string>
#include <thread>
#include <utility>

#include "detection_process.hpp"

namespace pure_uav::app {
// 把原有阻塞监督循环放到独立线程，让 main 继续执行 VIO 发布与输入监督。
// 只有监督线程访问 DetectionProcess；析构先通知停止并 join，再析构进程对象。
class DetectionSupervisor {
  public:
    explicit DetectionSupervisor(DetectionProcessOptions options) : process_(std::move(options)) {
    }
    ~DetectionSupervisor() {
        stop();
    }
    DetectionSupervisor(const DetectionSupervisor&) = delete;
    DetectionSupervisor& operator=(const DetectionSupervisor&) = delete;

    bool start(std::string* error) {
        if (worker_.joinable()) {
            if (error != nullptr) {
                *error = "检测监督线程已启动";
            }
            return false;
        }
        stop_ = false;
        done_ = false;
        if (!process_.start(error)) {
            return false;
        }
        try {
            worker_ = std::thread([this] {
                result_ = process_.supervise([this] { return stop_.load(); });
                done_.store(true, std::memory_order_release);
            });
            return true;
        } catch (const std::exception& e) {
            process_.stop();
            if (error != nullptr) {
                *error = std::string("检测监督线程启动失败: ") + e.what();
            }
            return false;
        }
    }
    void stop() {
        stop_ = true;
        if (worker_.joinable()) {
            worker_.join();
        }
    }
    bool finished(SuperviseResult& result) const {
        if (!done_.load(std::memory_order_acquire)) {
            return false;
        }
        result = result_;
        return true;
    }

  private:
    DetectionProcess process_;
    std::atomic<bool> stop_{false}, done_{false};
    SuperviseResult result_;
    std::thread worker_;
};
}  // namespace pure_uav::app
