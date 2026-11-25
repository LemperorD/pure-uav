#include "camera_shm.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>
#include <ctime>
#include <new>

namespace pure_uav::camera {
namespace {

constexpr int kMaxSlots = 16;
constexpr int kMaxReadAttempts = 4;
constexpr long kPollIntervalNs = 10000000L;  // 10 ms

void sleepPoll() noexcept {
    timespec ts{kPollIntervalNs / 1000000000L, kPollIntervalNs % 1000000000L};
    while (nanosleep(&ts, &ts) == -1 && errno == EINTR) {
    }
}

void setError(std::string* error, const std::string& message) {
    if (error != nullptr) {
        *error = message;
    }
}

std::string errnoText(const char* what) {
    return std::string(what) + ": " + std::strerror(errno);
}

}  // namespace

bool unlinkChannel(const std::string& name) noexcept {
    if (name.empty() || name.front() != '/') {
        return false;
    }
    return shm_unlink(name.c_str()) == 0 || errno == ENOENT;
}

bool channelExists(const std::string& name) noexcept {
    const int fd = shm_open(name.c_str(), O_RDONLY, 0);
    if (fd < 0) {
        return false;
    }
    close(fd);
    return true;
}

// ---------------------------------------------------------------- ShmChannelWriter

ShmChannelWriter::~ShmChannelWriter() {
    close();
}

bool ShmChannelWriter::create(const std::string& name, ChannelType type, const StreamMeta& stream, size_t payload_bytes, int slot_count, std::string* error) {
    close();
    if (name.empty() || name.front() != '/') {
        setError(error, "camera shm name must start with '/'");
        return false;
    }
    if (slot_count < 1 || slot_count > kMaxSlots || payload_bytes < 1024U || payload_bytes > 64U * 1024U * 1024U) {
        setError(error, "camera shm invalid slot count or payload size");
        return false;
    }

    // 上次异常退出可能留下同名对象；独占创建前清掉，保证只有一个写者。
    if (!unlinkChannel(name)) {
        setError(error, errnoText("shm_unlink"));
        return false;
    }

    const size_t stride = alignUp(kSlotHeaderBytes + payload_bytes, kCacheLine);
    const size_t total = kHeaderBytes + static_cast<size_t>(slot_count) * stride;

    int fd = shm_open(name.c_str(), O_CREAT | O_EXCL | O_RDWR, 0660);
    if (fd < 0) {
        setError(error, errnoText("shm_open"));
        return false;
    }
    if (ftruncate(fd, static_cast<off_t>(total)) != 0) {
        setError(error, errnoText("ftruncate"));
        ::close(fd);
        shm_unlink(name.c_str());
        return false;
    }
    void* base = mmap(nullptr, total, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    if (base == MAP_FAILED) {
        setError(error, errnoText("mmap"));
        ::close(fd);
        shm_unlink(name.c_str());
        return false;
    }

    auto* header = new (base) ShmHeader{};
    auto* slots = static_cast<uint8_t*>(base) + kHeaderBytes;
    for (int index = 0; index < slot_count; ++index) {
        new (slots + static_cast<size_t>(index) * stride) SlotHeader{};
    }

    header->header_bytes = static_cast<uint32_t>(kHeaderBytes);
    header->slot_count = static_cast<uint32_t>(slot_count);
    header->slot_payload_bytes = payload_bytes;
    header->slot_stride = stride;
    header->total_bytes = total;
    header->channel_type = static_cast<uint32_t>(type);
    header->creator_pid = static_cast<uint32_t>(getpid());
    header->stream = stream;
    header->heartbeat_ns.store(static_cast<uint64_t>(monotonicNs()), std::memory_order_relaxed);

    name_ = name;
    fd_ = fd;
    base_ = base;
    size_ = total;
    header_ = header;
    slots_ = slots;
    slot_count_ = slot_count;
    next_slot_ = 0;
    if (error != nullptr) {
        error->clear();
    }
    return true;
}

bool ShmChannelWriter::publish(const FrameMeta& meta, const uint8_t* gray, const uint8_t* depth, const uint8_t* color) noexcept {
    if (header_ == nullptr || slots_ == nullptr) {
        return false;
    }
    const uint64_t total = static_cast<uint64_t>(meta.gray_bytes) + meta.depth_bytes + meta.color_bytes;
    if (total > header_->slot_payload_bytes || total == 0) {
        header_->dropped.fetch_add(1, std::memory_order_relaxed);
        return false;
    }
    if ((meta.gray_bytes != 0 && gray == nullptr) || (meta.depth_bytes != 0 && depth == nullptr) || (meta.color_bytes != 0 && color == nullptr)) {
        header_->dropped.fetch_add(1, std::memory_order_relaxed);
        return false;
    }

    const int index = next_slot_;
    next_slot_ = (next_slot_ + 1) % slot_count_;
    auto* slot = reinterpret_cast<SlotHeader*>(slots_ + static_cast<size_t>(index) * header_->slot_stride);
    auto* payload = reinterpret_cast<uint8_t*>(slot) + kSlotHeaderBytes;

    // 写者从崩溃遗留的奇数序号中恢复：下一个"写入中"序号必须是奇数。
    uint64_t version = slot->seq_lock.load(std::memory_order_relaxed);
    if ((version & 1U) != 0U) {
        ++version;
    }
    slot->seq_lock.store(version + 1U, std::memory_order_relaxed);
    std::atomic_thread_fence(std::memory_order_release);

    slot->meta = meta;
    size_t offset = 0;
    if (meta.gray_bytes != 0) {
        std::memcpy(payload + offset, gray, meta.gray_bytes);
        offset += meta.gray_bytes;
    }
    if (meta.depth_bytes != 0) {
        std::memcpy(payload + offset, depth, meta.depth_bytes);
        offset += meta.depth_bytes;
    }
    if (meta.color_bytes != 0) {
        std::memcpy(payload + offset, color, meta.color_bytes);
    }

    slot->seq_lock.store(version + 2U, std::memory_order_release);
    header_->published.fetch_add(1, std::memory_order_relaxed);
    header_->heartbeat_ns.store(static_cast<uint64_t>(monotonicNs()), std::memory_order_release);
    return true;
}

void ShmChannelWriter::setStreamMeta(const StreamMeta& meta) noexcept {
    if (header_ != nullptr && header_->ready.load(std::memory_order_relaxed) == 0) {
        header_->stream = meta;
    }
}

void ShmChannelWriter::setSession(uint64_t session, uint64_t generation) noexcept {
    if (header_ == nullptr) {
        return;
    }
    header_->session.store(session, std::memory_order_release);
    header_->generation.store(generation, std::memory_order_release);
}

void ShmChannelWriter::markReady() noexcept {
    if (header_ != nullptr) {
        header_->ready.store(1, std::memory_order_release);
    }
}

void ShmChannelWriter::markClosing() noexcept {
    if (header_ != nullptr) {
        header_->closing.store(1, std::memory_order_release);
        header_->heartbeat_ns.store(static_cast<uint64_t>(monotonicNs()), std::memory_order_release);
    }
}

uint64_t ShmChannelWriter::published() const noexcept {
    return header_ != nullptr ? header_->published.load(std::memory_order_relaxed) : 0;
}

uint64_t ShmChannelWriter::dropped() const noexcept {
    return header_ != nullptr ? header_->dropped.load(std::memory_order_relaxed) : 0;
}

void ShmChannelWriter::close() noexcept {
    if (base_ != nullptr && base_ != MAP_FAILED) {
        munmap(base_, size_);
    }
    if (fd_ >= 0) {
        ::close(fd_);
    }
    if (!name_.empty()) {
        shm_unlink(name_.c_str());
    }
    name_.clear();
    fd_ = -1;
    base_ = nullptr;
    size_ = 0;
    header_ = nullptr;
    slots_ = nullptr;
    slot_count_ = 0;
    next_slot_ = 0;
}

// ---------------------------------------------------------------- ShmChannelReader

ShmChannelReader::~ShmChannelReader() {
    close();
}

bool ShmChannelReader::open(const std::string& name, ChannelType expected, int timeout_ms, std::string* error) {
    close();
    if (name.empty() || name.front() != '/') {
        setError(error, "camera shm name must start with '/'");
        return false;
    }

    const int64_t deadline = monotonicNs() + static_cast<int64_t>(timeout_ms) * 1000000LL;
    int fd = -1;
    while (true) {
        fd = shm_open(name.c_str(), O_RDONLY, 0);
        if (fd >= 0) {
            break;
        }
        if (errno != ENOENT || monotonicNs() >= deadline) {
            setError(error, "camera shm '" + name + "' not available: " + errnoText("shm_open") + " (camera_server must be started first)");
            return false;
        }
        sleepPoll();
    }

    struct stat status{};
    if (fstat(fd, &status) != 0 || status.st_size < static_cast<off_t>(kHeaderBytes)) {
        setError(error, errnoText("fstat"));
        ::close(fd);
        return false;
    }
    const auto size = static_cast<size_t>(status.st_size);
    void* base = mmap(nullptr, size, PROT_READ, MAP_SHARED, fd, 0);
    if (base == MAP_FAILED) {
        setError(error, errnoText("mmap"));
        ::close(fd);
        return false;
    }

    const auto* header = reinterpret_cast<const ShmHeader*>(base);
    if (header->magic != kShmMagic || header->version != kShmVersion || header->header_bytes != kHeaderBytes || header->slot_count == 0 || header->slot_count > kMaxSlots || header->channel_type != static_cast<uint32_t>(expected) || header->slot_stride <= kSlotHeaderBytes ||
        header->slot_payload_bytes == 0 || header->total_bytes != size || header->total_bytes != kHeaderBytes + static_cast<size_t>(header->slot_count) * header->slot_stride) {
        setError(error, "camera shm layout/version mismatch: rebuild every process with the same camera_frame.hpp");
        munmap(base, size);
        ::close(fd);
        return false;
    }

    while (header->ready.load(std::memory_order_acquire) == 0) {
        if (monotonicNs() >= deadline) {
            setError(error, "camera shm '" + name + "' never became ready");
            munmap(base, size);
            ::close(fd);
            return false;
        }
        sleepPoll();
    }

    fd_ = fd;
    base_ = base;
    size_ = size;
    header_ = header;
    slots_ = static_cast<const uint8_t*>(base) + kHeaderBytes;
    slot_count_ = static_cast<int>(header->slot_count);
    last_seq_ = 0;
    last_session_ = header->session.load(std::memory_order_acquire);
    if (error != nullptr) {
        error->clear();
    }
    return true;
}

const SlotHeader* ShmChannelReader::slot(int index) const noexcept {
    return reinterpret_cast<const SlotHeader*>(slots_ + static_cast<size_t>(index) * header_->slot_stride);
}

bool ShmChannelReader::readLatest(FrameMeta& meta, std::vector<uint8_t>& payload, uint64_t& session, uint64_t& generation) noexcept {
    if (header_ == nullptr) {
        return false;
    }
    session = header_->session.load(std::memory_order_acquire);
    generation = header_->generation.load(std::memory_order_acquire);
    if (session != last_session_) {
        last_session_ = session;
        last_seq_ = 0;  // 新会话允许从最小序号重新接收
    }

    // 第一遍只挑序号最大的稳定槽，不拷贝负载。
    int best_slot = -1;
    uint64_t best_seq = last_seq_;
    for (int index = 0; index < slot_count_; ++index) {
        const auto* candidate = slot(index);
        const uint64_t before = candidate->seq_lock.load(std::memory_order_acquire);
        if ((before & 1U) != 0U) {
            continue;
        }
        const uint64_t seq = candidate->meta.seq;
        if (candidate->seq_lock.load(std::memory_order_acquire) != before) {
            continue;
        }
        if (seq > best_seq) {
            best_seq = seq;
            best_slot = index;
        }
    }
    if (best_slot < 0) {
        return false;
    }

    // 第二遍只对该槽做一次负载拷贝，并用 seqlock 前后校验排除撕裂。
    const auto* best = slot(best_slot);
    for (int attempt = 0; attempt < kMaxReadAttempts; ++attempt) {
        const uint64_t before = best->seq_lock.load(std::memory_order_acquire);
        if ((before & 1U) != 0U) {
            continue;
        }
        const FrameMeta candidate = best->meta;
        const uint64_t total = static_cast<uint64_t>(candidate.gray_bytes) + candidate.depth_bytes + candidate.color_bytes;
        if (candidate.seq != best_seq || total == 0 || total > header_->slot_payload_bytes) {
            return false;  // 该槽已被写者覆盖，下一轮再取
        }
        const auto* source = reinterpret_cast<const uint8_t*>(best) + kSlotHeaderBytes;
        try {
            payload.resize(static_cast<size_t>(total));
        } catch (...) {
            return false;
        }
        std::memcpy(payload.data(), source, static_cast<size_t>(total));
        if (best->seq_lock.load(std::memory_order_acquire) != before) {
            continue;  // 写者并发覆盖，重试同一槽
        }
        meta = candidate;
        last_seq_ = candidate.seq;
        return true;
    }
    return false;
}

const StreamMeta& ShmChannelReader::streamMeta() const noexcept {
    static const StreamMeta empty{};
    return header_ != nullptr ? header_->stream : empty;
}

bool ShmChannelReader::writerAlive(uint64_t timeout_ns) const noexcept {
    if (header_ == nullptr) {
        return false;
    }
    const auto beat = static_cast<int64_t>(header_->heartbeat_ns.load(std::memory_order_acquire));
    const int64_t now = monotonicNs();
    if (beat <= 0 || now < beat) {
        return false;
    }
    return static_cast<uint64_t>(now - beat) <= timeout_ns;
}

bool ShmChannelReader::closing() const noexcept {
    return header_ != nullptr && header_->closing.load(std::memory_order_acquire) != 0;
}

uint64_t ShmChannelReader::session() const noexcept {
    return header_ != nullptr ? header_->session.load(std::memory_order_acquire) : 0;
}

uint64_t ShmChannelReader::generation() const noexcept {
    return header_ != nullptr ? header_->generation.load(std::memory_order_acquire) : 0;
}

uint64_t ShmChannelReader::published() const noexcept {
    return header_ != nullptr ? header_->published.load(std::memory_order_acquire) : 0;
}

uint64_t ShmChannelReader::dropped() const noexcept {
    return header_ != nullptr ? header_->dropped.load(std::memory_order_acquire) : 0;
}

void ShmChannelReader::close() noexcept {
    if (base_ != nullptr && base_ != MAP_FAILED) {
        munmap(base_, size_);
    }
    if (fd_ >= 0) {
        ::close(fd_);
    }
    fd_ = -1;
    base_ = nullptr;
    size_ = 0;
    header_ = nullptr;
    slots_ = nullptr;
    slot_count_ = 0;
    last_seq_ = 0;
    last_session_ = 0;
}

}  // namespace pure_uav::camera
