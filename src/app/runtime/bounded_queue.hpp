#pragma once

#include <cstddef>
#include <deque>
#include <mutex>
#include <utility>

namespace pure_uav::app {

// 采集线程只投递；慢消费者丢最旧帧，队列长度有界且可监测丢帧。
template <typename T>
class BoundedQueue {
  public:
    explicit BoundedQueue(std::size_t capacity) : capacity_(capacity) {
    }
    void push(T value) {
        std::scoped_lock lock(mutex_);
        if (capacity_ == 0) {
            ++dropped_;
            return;
        }
        if (queue_.size() == capacity_) {
            queue_.pop_front();
            ++dropped_;
        }
        queue_.push_back(std::move(value));
    }
    bool pop(T& value) {
        std::scoped_lock lock(mutex_);
        if (queue_.empty()) {
            return false;
        }
        value = std::move(queue_.front());
        queue_.pop_front();
        return true;
    }
    void clear() {
        std::scoped_lock lock(mutex_);
        queue_.clear();
    }
    std::size_t dropped() const {
        std::scoped_lock lock(mutex_);
        return dropped_;
    }

  private:
    const std::size_t capacity_;
    mutable std::mutex mutex_;
    std::deque<T> queue_;
    std::size_t dropped_ = 0;
};

}  // namespace pure_uav::app
