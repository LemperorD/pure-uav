// POSIX 共享内存帧通道：单一写者 + 多个只读消费者。
//
// 不使用 PTHREAD_PROCESS_SHARED 互斥锁：若持有者（相机服务或某个消费者）崩溃，
// 跨进程互斥锁会永久死锁，直接违背「检测侧异常不得拖垮定位」的进程隔离目标。
// 改为 seqlock + 三缓冲轮转，写者永不阻塞、读者永不撕裂，天然实现「丢最旧」。
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

#include "camera_frame.hpp"

namespace pure_uav::camera {

// 删除残留通道。启动清理与正常退出都会调用；已被 mmap 的读方不受影响。
bool unlinkChannel(const std::string& name) noexcept;
bool channelExists(const std::string& name) noexcept;

class ShmChannelWriter {
  public:
    ShmChannelWriter() = default;
    ~ShmChannelWriter();
    ShmChannelWriter(const ShmChannelWriter&) = delete;
    ShmChannelWriter& operator=(const ShmChannelWriter&) = delete;
    ShmChannelWriter(ShmChannelWriter&&) = delete;
    ShmChannelWriter& operator=(ShmChannelWriter&&) = delete;

    /// 独占创建通道。已残留的同名对象会先被删除，保证只有一个写者。
    bool create(const std::string& name, ChannelType type, const StreamMeta& stream, size_t payload_bytes, int slot_count, std::string* error);

    /// 发布一帧。gray/depth/color 三段按该顺序拼接写入同一个槽，长度取自 meta。
    /// 只在采集回调内调用：O(payload) 一次 memcpy，无堆分配、无日志、无阻塞。
    bool publish(const FrameMeta& meta, const uint8_t* gray, const uint8_t* depth, const uint8_t* color) noexcept;

    /// 只能在 markReady 之前调用：补齐启动时才拿到的静态元数据（深度内参/外参）。
    void setStreamMeta(const StreamMeta& meta) noexcept;
    void setSession(uint64_t session, uint64_t generation) noexcept;
    void markReady() noexcept;
    void markClosing() noexcept;
    [[nodiscard]] uint64_t published() const noexcept;
    [[nodiscard]] uint64_t dropped() const noexcept;
    [[nodiscard]] bool valid() const noexcept {
        return base_ != nullptr;
    }
    [[nodiscard]] const std::string& name() const noexcept {
        return name_;
    }

    /// unmap + shm_unlink；重复调用安全。
    void close() noexcept;

  private:
    std::string name_;
    int fd_ = -1;
    void* base_ = nullptr;
    size_t size_ = 0;
    ShmHeader* header_ = nullptr;
    uint8_t* slots_ = nullptr;
    int slot_count_ = 0;
    int next_slot_ = 0;
};

class ShmChannelReader {
  public:
    ShmChannelReader() = default;
    ~ShmChannelReader();
    ShmChannelReader(const ShmChannelReader&) = delete;
    ShmChannelReader& operator=(const ShmChannelReader&) = delete;
    ShmChannelReader(ShmChannelReader&&) = delete;
    ShmChannelReader& operator=(ShmChannelReader&&) = delete;

    /// 以只读方式打开通道，等待写者创建并 markReady，超时返回 false。
    bool open(const std::string& name, ChannelType expected, int timeout_ms, std::string* error);

    /// 取最新一帧；没有比上次返回更新的帧时返回 false。
    /// payload 内为 gray|depth|color 三段，按 meta 的 length 切分。
    bool readLatest(FrameMeta& meta, std::vector<uint8_t>& payload, uint64_t& session, uint64_t& generation) noexcept;

    [[nodiscard]] const StreamMeta& streamMeta() const noexcept;
    [[nodiscard]] bool writerAlive(uint64_t timeout_ns = kHeartbeatTimeoutNs) const noexcept;
    [[nodiscard]] bool closing() const noexcept;
    [[nodiscard]] uint64_t session() const noexcept;
    [[nodiscard]] uint64_t generation() const noexcept;
    [[nodiscard]] uint64_t published() const noexcept;
    [[nodiscard]] uint64_t dropped() const noexcept;
    [[nodiscard]] bool valid() const noexcept {
        return base_ != nullptr;
    }
    void close() noexcept;

  private:
    [[nodiscard]] const SlotHeader* slot(int index) const noexcept;

    int fd_ = -1;
    void* base_ = nullptr;
    size_t size_ = 0;
    const ShmHeader* header_ = nullptr;
    const uint8_t* slots_ = nullptr;
    int slot_count_ = 0;
    uint64_t last_seq_ = 0;
    uint64_t last_session_ = 0;
};

}  // namespace pure_uav::camera
