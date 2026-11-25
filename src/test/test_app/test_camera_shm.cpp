// camera_ipc 无硬件回归：帧通道布局、跨进程读取、seqlock 一致性、会话失效、
// 长度边界、双通道隔离与清理。不打开任何相机或推理设备。
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include "camera_shm.hpp"
#include "color_frame.hpp"

namespace {
using namespace pure_uav::camera;

int g_failures = 0;

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++g_failures;                                                        \
        }                                                                        \
    } while (false)

#define REQUIRE(cond)                                                             \
    do {                                                                          \
        if (!(cond)) {                                                            \
            std::fprintf(stderr, "FATAL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            return 1;                                                             \
        }                                                                         \
    } while (false)

FrameMeta makeMeta(uint64_t seq, uint32_t bytes) {
    FrameMeta meta;
    meta.seq = seq;
    meta.host_ns = monotonicNs();
    meta.device_us = static_cast<int64_t>(seq * 100);
    meta.sys_us = static_cast<int64_t>(seq * 100);
    meta.depth_device_us = meta.device_us;
    meta.gray_bytes = bytes;
    meta.width = 640;
    meta.height = 480;
    return meta;
}

int testChannelBasics(const std::string& vio_name, const std::string& color_name) {
    std::string error;
    StreamMeta stream;
    stream.width = 640;
    stream.height = 480;
    stream.fps = 30;
    stream.fx = 640.5;
    stream.has_depth = 1;
    stream.depth_width = 640;
    stream.depth_height = 400;

    ShmChannelWriter vio_writer;
    ShmChannelWriter color_writer;
    REQUIRE(vio_writer.create(vio_name, ChannelType::vio, stream, 64U * 1024U, 3, &error));
    REQUIRE(color_writer.create(color_name, ChannelType::color, stream, 64U * 1024U, 3, &error));
    vio_writer.setSession(1, 0);
    color_writer.setSession(1, 0);
    vio_writer.markReady();
    color_writer.markReady();
    REQUIRE(channelExists(vio_name));
    REQUIRE(channelExists(color_name));

    ShmChannelReader vio_reader;
    ShmChannelReader color_reader;
    REQUIRE(vio_reader.open(vio_name, ChannelType::vio, 1000, &error));
    REQUIRE(color_reader.open(color_name, ChannelType::color, 1000, &error));
    CHECK(vio_reader.streamMeta().fx == 640.5);
    CHECK(vio_reader.streamMeta().depth_height == 400);
    CHECK(vio_reader.writerAlive());
    CHECK(!vio_reader.closing());

    // 通道类型不符必须拒绝，避免消费者读错布局。
    ShmChannelReader wrong_type;
    CHECK(!wrong_type.open(vio_name, ChannelType::color, 100, &error));
    wrong_type.close();

    FrameMeta meta;
    std::vector<uint8_t> payload;
    uint64_t session = 0;
    uint64_t generation = 0;
    CHECK(!vio_reader.readLatest(meta, payload, session, generation));  // 尚无帧

    // 两条通道各写各的，互不干扰。
    std::vector<uint8_t> gray(1000, 0x5A);
    std::vector<uint8_t> depth(800, 0x11);
    std::vector<uint8_t> jpeg(512, 0xE7);
    FrameMeta vio_meta = makeMeta(1, static_cast<uint32_t>(gray.size()));
    vio_meta.depth_bytes = static_cast<uint32_t>(depth.size());
    vio_meta.flags = kFrameHasDepth;
    REQUIRE(vio_writer.publish(vio_meta, gray.data(), depth.data(), nullptr));
    FrameMeta color_meta = makeMeta(1, 0);
    color_meta.color_bytes = static_cast<uint32_t>(jpeg.size());
    color_meta.color_encoding = static_cast<uint32_t>(pure_uav::ColorEncoding::jpeg);
    REQUIRE(color_writer.publish(color_meta, nullptr, nullptr, jpeg.data()));

    REQUIRE(vio_reader.readLatest(meta, payload, session, generation));
    CHECK(meta.seq == 1);
    CHECK(meta.device_us == 100);
    CHECK(meta.flags == kFrameHasDepth);
    CHECK(payload.size() == gray.size() + depth.size());
    CHECK(payload[0] == 0x5A);
    CHECK(payload[gray.size()] == 0x11);
    CHECK(session == 1 && generation == 0);
    CHECK(!vio_reader.readLatest(meta, payload, session, generation));  // 无更新帧

    REQUIRE(color_reader.readLatest(meta, payload, session, generation));
    CHECK(meta.seq == 1);
    CHECK(meta.color_bytes == jpeg.size());
    CHECK(payload.size() == jpeg.size());
    CHECK(payload[0] == 0xE7);
    CHECK(vio_reader.published() == 1);
    CHECK(color_reader.published() == 1);

    // 超长负载必须被拒绝并计数，不能越界写共享内存。
    FrameMeta too_big = makeMeta(2, 128U * 1024U);
    CHECK(!vio_writer.publish(too_big, gray.data(), nullptr, nullptr));
    CHECK(vio_writer.dropped() == 1);
    CHECK(vio_reader.readLatest(meta, payload, session, generation) == false);

    // 会话切换使旧序号失效：新会话允许从 seq=1 重新接收。
    vio_writer.setSession(2, 1);
    FrameMeta restarted = makeMeta(1, 100);
    REQUIRE(vio_writer.publish(restarted, gray.data(), nullptr, nullptr));
    REQUIRE(vio_reader.readLatest(meta, payload, session, generation));
    CHECK(meta.seq == 1);
    CHECK(session == 2 && generation == 1);
    CHECK(vio_reader.session() == 2);

    // 跨进程：子进程独立 shm_open + mmap，验证布局与帧可读。
    const pid_t child = fork();
    REQUIRE(child >= 0);
    if (child == 0) {
        ShmChannelReader child_reader;
        std::string child_error;
        if (!child_reader.open(vio_name, ChannelType::vio, 1000, &child_error)) {
            _exit(2);
        }
        FrameMeta child_meta;
        std::vector<uint8_t> child_payload;
        uint64_t child_session = 0;
        uint64_t child_generation = 0;
        if (!child_reader.readLatest(child_meta, child_payload, child_session, child_generation)) {
            _exit(3);
        }
        if (child_meta.seq != 1 || child_session != 2 || child_payload.size() != static_cast<size_t>(child_meta.gray_bytes) + child_meta.depth_bytes) {
            _exit(4);
        }
        _exit(0);
    }
    int status = 0;
    REQUIRE(waitpid(child, &status, 0) == child);
    CHECK(WIFEXITED(status));
    if (WIFEXITED(status) && WEXITSTATUS(status) != 0) {
        std::fprintf(stderr, "cross-process reader exited with %d\n", WEXITSTATUS(status));
    }
    CHECK(WIFEXITED(status) && WEXITSTATUS(status) == 0);

    vio_writer.markClosing();
    CHECK(vio_reader.closing());
    vio_reader.close();
    color_reader.close();
    vio_writer.close();
    color_writer.close();
    CHECK(!channelExists(vio_name));
    CHECK(!channelExists(color_name));
    return 0;
}

int testConcurrentSeqlock(const std::string& name) {
    constexpr uint64_t kFrames = 20000;
    constexpr uint32_t kPayload = 2048;
    std::string error;
    ShmChannelWriter writer;
    REQUIRE(writer.create(name, ChannelType::vio, StreamMeta{}, 16U * 1024U, 3, &error));
    writer.markReady();

    ShmChannelReader reader;
    REQUIRE(reader.open(name, ChannelType::vio, 1000, &error));

    std::atomic<uint64_t> corrupt{0};
    std::atomic<uint64_t> accepted{0};
    std::thread producer([&] {
        std::vector<uint8_t> buffer(kPayload);
        for (uint64_t seq = 1; seq <= kFrames; ++seq) {
            std::fill(buffer.begin(), buffer.end(), static_cast<uint8_t>(seq & 0xFFU));
            const FrameMeta meta = makeMeta(seq, kPayload);
            writer.publish(meta, buffer.data(), nullptr, nullptr);
            if ((seq & 0xFFU) == 0U) {
                std::this_thread::yield();
            }
        }
    });
    std::thread consumer([&] {
        FrameMeta meta;
        std::vector<uint8_t> payload;
        uint64_t session = 0;
        uint64_t generation = 0;
        for (uint64_t attempt = 0; attempt < kFrames; ++attempt) {
            if (!reader.readLatest(meta, payload, session, generation)) {
                std::this_thread::yield();
                continue;
            }
            ++accepted;
            const auto expected = static_cast<uint8_t>(meta.seq & 0xFFU);
            if (payload.size() != kPayload) {
                ++corrupt;
                continue;
            }
            for (const uint8_t value : payload) {
                if (value != expected) {
                    ++corrupt;
                    break;
                }
            }
        }
    });
    producer.join();
    consumer.join();

    CHECK(writer.published() == kFrames);
    CHECK(accepted.load() > 0);
    CHECK(corrupt.load() == 0);  // 任何一次撕裂都会被 seqlock 校验拦下
    reader.close();
    writer.close();
    CHECK(!channelExists(name));
    return 0;
}

pid_t spawnConsumer(const std::string& name, ChannelType type, size_t expected_bytes) {
    const pid_t pid = fork();
    if (pid != 0) {
        return pid;
    }
    ShmChannelReader reader;
    std::string error;
    if (!reader.open(name, type, 2000, &error)) {
        _exit(2);
    }
    uint64_t accepted = 0;
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::seconds(5);
    FrameMeta meta;
    std::vector<uint8_t> payload;
    uint64_t session = 0;
    uint64_t generation = 0;
    while (accepted < 20 && std::chrono::steady_clock::now() < deadline) {
        if (!reader.readLatest(meta, payload, session, generation)) {
            std::this_thread::yield();
            continue;
        }
        if (payload.size() != expected_bytes) {
            _exit(3);
        }
        const auto expected = static_cast<uint8_t>(meta.seq & 0xFFU);
        for (const uint8_t value : payload) {
            if (value != expected) {
                _exit(4);  // 读到撕裂帧
            }
        }
        ++accepted;
    }
    _exit(accepted >= 20 ? 0 : 5);
}

// 模拟 camera_server：一个写者同时喂 VIO 与检测两个独立消费者进程。
// 两个消费者必须都能独立取到完整帧，任一侧慢/退出都不影响另一侧。
int testTwoConsumers(const std::string& vio_name, const std::string& color_name) {
    constexpr size_t kVioBytes = 1024;
    constexpr size_t kColorBytes = 256;
    std::string error;
    ShmChannelWriter vio_writer;
    ShmChannelWriter color_writer;
    REQUIRE(vio_writer.create(vio_name, ChannelType::vio, StreamMeta{}, 16U * 1024U, 3, &error));
    REQUIRE(color_writer.create(color_name, ChannelType::color, StreamMeta{}, 16U * 1024U, 3, &error));
    vio_writer.markReady();
    color_writer.markReady();

    const pid_t vio_pid = spawnConsumer(vio_name, ChannelType::vio, kVioBytes);
    const pid_t color_pid = spawnConsumer(color_name, ChannelType::color, kColorBytes);
    REQUIRE(vio_pid > 0 && color_pid > 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    std::vector<uint8_t> gray(kVioBytes);
    std::vector<uint8_t> jpeg(kColorBytes);
    for (uint64_t seq = 1; seq <= 3000; ++seq) {
        const auto tag = static_cast<uint8_t>(seq & 0xFFU);
        std::fill(gray.begin(), gray.end(), tag);
        std::fill(jpeg.begin(), jpeg.end(), tag);
        const FrameMeta vio_meta = makeMeta(seq, static_cast<uint32_t>(kVioBytes));
        vio_writer.publish(vio_meta, gray.data(), nullptr, nullptr);
        if (seq % 3 == 0) {  // 彩色通道按 1/3 限频，模拟服务端 max_hz 门限
            FrameMeta color_meta = makeMeta(seq, 0);
            color_meta.color_bytes = static_cast<uint32_t>(kColorBytes);
            color_writer.publish(color_meta, nullptr, nullptr, jpeg.data());
        }
        if ((seq & 0x3FU) == 0U) {
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }

    int vio_status = 0;
    int color_status = 0;
    REQUIRE(waitpid(vio_pid, &vio_status, 0) == vio_pid);
    REQUIRE(waitpid(color_pid, &color_status, 0) == color_pid);
    CHECK(WIFEXITED(vio_status) && WEXITSTATUS(vio_status) == 0);
    CHECK(WIFEXITED(color_status) && WEXITSTATUS(color_status) == 0);
    CHECK(vio_writer.published() == 3000);
    CHECK(color_writer.published() == 1000);

    vio_writer.close();
    color_writer.close();
    CHECK(!channelExists(vio_name));
    CHECK(!channelExists(color_name));
    return 0;
}

}  // namespace

int main() {
    const std::string suffix = std::to_string(static_cast<long long>(getpid()));
    const std::string vio_name = "/pure_uav_test_vio_" + suffix;
    const std::string color_name = "/pure_uav_test_color_" + suffix;
    const std::string stress_name = "/pure_uav_test_stress_" + suffix;
    const std::string dual_vio_name = "/pure_uav_test_dual_vio_" + suffix;
    const std::string dual_color_name = "/pure_uav_test_dual_color_" + suffix;
    unlinkChannel(vio_name);
    unlinkChannel(color_name);
    unlinkChannel(stress_name);
    unlinkChannel(dual_vio_name);
    unlinkChannel(dual_color_name);

    if (testChannelBasics(vio_name, color_name) != 0) {
        return 1;
    }
    if (testConcurrentSeqlock(stress_name) != 0) {
        return 1;
    }
    if (testTwoConsumers(dual_vio_name, dual_color_name) != 0) {
        return 1;
    }

    // 不存在的通道：短超时返回明确失败，消费者据此提示先启动 camera_server。
    ShmChannelReader missing;
    std::string error;
    CHECK(!missing.open("/pure_uav_test_missing_" + suffix, ChannelType::vio, 100, &error));
    CHECK(!error.empty());
    missing.close();

    if (g_failures != 0) {
        std::fprintf(stderr, "test_camera_shm: %d failure(s)\n", g_failures);
        return 1;
    }
    std::printf("test_camera_shm: OK\n");
    return 0;
}
