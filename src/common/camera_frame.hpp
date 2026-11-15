// 相机服务与消费者之间的跨进程帧协议。
//
// 布局（单通道）：
//   [0, kHeaderBytes)                         ShmHeader（创建后只读，除原子状态字段）
//   [kHeaderBytes + i*slot_stride, +kSlotHeaderBytes)  SlotHeader（seqlock + FrameMeta）
//   [+kSlotHeaderBytes, +slot_payload_bytes)          负载：gray | depth | color
//
// 所有跨进程共享字段都是定长 POD；只有 seqlock / 状态位用 lock-free std::atomic
// 访问。写入方以 placement new 构造这些对象，读方通过 reinterpret_cast 访问同一
// 块 MAP_SHARED 内存，因此两端必须使用同一份头文件（同一 kShmVersion）。
#pragma once

#include <atomic>
#include <chrono>
#include <cstddef>
#include <cstdint>

namespace pure_uav::camera {

// POSIX shm 名字必须以 '/' 开头；两个通道互相独立，慢消费者不会占用对方槽位。
constexpr const char* kVioChannel = "/pure_uav_camera_vio";
constexpr const char* kColorChannel = "/pure_uav_camera_color";

constexpr uint32_t kShmMagic = 0x5055434DU;  // "PUCM"
constexpr uint32_t kShmVersion = 2U;
constexpr int kDefaultSlotCount = 3;
constexpr size_t kDefaultVioPayloadBytes = 8U * 1024U * 1024U;    // 1920x1080 gray + CV_16U depth
constexpr size_t kDefaultColorPayloadBytes = 8U * 1024U * 1024U;  // 原生深度 + MJPG 单帧
constexpr uint64_t kHeartbeatTimeoutNs = 2000000000ULL;           // 2 s

enum class ChannelType : uint32_t {
    vio = 0,    // 负载 = gray(CV_8UC1) + depth_mm(CV_16UC1，可为空)
    color = 1,  // 负载 = depth_mm（可为空）| 相机原始编码字节
};

constexpr size_t kCacheLine = 64U;

constexpr size_t alignUp(size_t value, size_t alignment) noexcept {
    return (value + alignment - 1U) / alignment * alignment;
}

// 与 std::chrono::steady_clock 同源（Linux 上即 CLOCK_MONOTONIC），跨进程可比。
// DetectionRuntime::steadyNs() 与 OrbbecCamera::steadyNowNs() 使用同一时基。
inline int64_t monotonicNs() noexcept {
    return std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now().time_since_epoch()).count();
}

// 静态流信息：创建通道时写一次，ready 之后不再变化。
// 深度内参/外参随通道传递，消费者无需再向相机服务查询。
struct StreamMeta {
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t fps = 0;
    uint32_t color_encoding = 0;  // static_cast<uint32_t>(pure_uav::ColorEncoding)
    uint32_t has_depth = 0;
    uint32_t reserved0 = 0;
    double fx = 0, fy = 0, cx = 0, cy = 0;
    double R_c2d[9] = {1, 0, 0, 0, 1, 0, 0, 0, 1};
    double t_mm[3] = {0.0, 0.0, 0.0};
    uint32_t depth_width = 0;
    uint32_t depth_height = 0;
    uint32_t reserved1 = 0;
    // V2：完整测距标定；仅在 SDK 的内参、畸变和 D2C 外参均可用时置 1。
    uint32_t range_calibration_valid = 0;
    uint32_t reserved2 = 0;
    double depth_distortion[8] = {};  // OpenCV: k1,k2,p1,p2,k3,k4,k5,k6
    double color_fx = 0, color_fy = 0, color_cx = 0, color_cy = 0;
    double color_distortion[8] = {};
    uint64_t reserved3[4] = {0, 0, 0, 0};
};

// 每帧元数据。三个 length 描述负载内连续排布的三段字节。
struct FrameMeta {
    uint64_t seq = 0;             // 从 1 起单调递增，跨两个通道取同一帧集编号
    int64_t host_ns = 0;          // CLOCK_MONOTONIC，采集时刻
    int64_t device_us = 0;        // SDK timeStampUs()，仅可作相对量
    int64_t sys_us = 0;           // SDK systemTimeStampUs()，主机时基（VIO --cam-ts device 用）
    int64_t depth_device_us = 0;  // SDK 深度帧 timeStampUs()
    uint32_t gray_bytes = 0;
    uint32_t depth_bytes = 0;
    uint32_t color_bytes = 0;
    uint32_t flags = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t color_encoding = 0;
    uint32_t reserved0 = 0;
    uint64_t reserved1[3] = {0, 0, 0};
};

// 帧标志。
constexpr uint32_t kFrameHasDepth = 1U << 0;

struct ShmHeader {
    uint32_t magic = kShmMagic;
    uint32_t version = kShmVersion;
    uint32_t header_bytes = 0;
    uint32_t slot_count = 0;
    uint64_t slot_payload_bytes = 0;
    uint64_t slot_stride = 0;
    uint64_t total_bytes = 0;
    uint32_t channel_type = 0;
    uint32_t creator_pid = 0;
    StreamMeta stream{};

    // 以下字段跨进程共享。持有者崩溃时不会留下内核锁，读方靠心跳判定失联。
    std::atomic<uint64_t> session{0};
    std::atomic<uint64_t> generation{0};
    std::atomic<uint64_t> heartbeat_ns{0};
    std::atomic<uint64_t> published{0};
    std::atomic<uint64_t> dropped{0};
    std::atomic<uint64_t> ready{0};
    std::atomic<uint64_t> closing{0};
};

// seqlock：偶数表示槽稳定，奇数表示写者正在写入。
// 写者只做 fetch_add + memcpy，永不阻塞；读者校验前后序号，不读撕裂帧。
struct SlotHeader {
    std::atomic<uint64_t> seq_lock{0};
    FrameMeta meta{};
};

constexpr size_t kHeaderBytes = alignUp(sizeof(ShmHeader), kCacheLine);
constexpr size_t kSlotHeaderBytes = alignUp(sizeof(SlotHeader), kCacheLine);

static_assert(std::atomic<uint64_t>::is_always_lock_free, "camera shm requires lock-free 64-bit atomics for cross-process seqlock");
static_assert(sizeof(FrameMeta) % 8 == 0, "FrameMeta must be 8-byte aligned");
static_assert(sizeof(StreamMeta) % 8 == 0, "StreamMeta must be 8-byte aligned");

}  // namespace pure_uav::camera
