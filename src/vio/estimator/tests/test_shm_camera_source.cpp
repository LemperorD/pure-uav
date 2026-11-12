// 使用真实 V1 共享内存通道验证 VIO 输入，不打开相机或串口。
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#include <atomic>
#include <chrono>
#include <cstdio>
#include <stdexcept>
#include <string>
#include <thread>

#include "shm_camera_source.hpp"

namespace {
using namespace pure_uav;
using namespace pure_uav::camera;

#define CHECK(condition)                                                                                                      \
    do {                                                                                                                      \
        if (!(condition)) throw std::runtime_error(std::string(__func__) + ":" + std::to_string(__LINE__) + ": " #condition); \
    } while (false)

struct Fixture {
    ShmChannelWriter writer;
    ShmCameraSource source;
    ShmCameraSource::Config config;
    StreamMeta stream;
    std::string error;
    uint8_t gray[12] = {1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12};
    uint16_t depth[4] = {1000, 2000, 3000, 4000};

    Fixture() {
        static int next = 0;
        config.channel = "/pure_uav_vio_source_test_" + std::to_string(getpid()) + "_" + std::to_string(++next);
        config.width = 4;
        config.height = 3;
        config.fps = 30;
        config.timeout_ms = 20;
        config.use_depth = true;
        stream.width = 4;
        stream.height = 3;
        stream.fps = 30;
        stream.has_depth = 1;
        stream.depth_width = 2;
        stream.depth_height = 2;
        stream.fx = 100;
        stream.fy = 101;
        stream.cx = 1;
        stream.cy = 1;
        stream.t_mm[0] = 25;
        stream.t_mm[1] = -10;
        stream.t_mm[2] = 5;
    }
    void create(ChannelType type = ChannelType::vio) {
        CHECK(writer.create(config.channel, type, stream, 1024, 3, &error));
        writer.setSession(1, 0);
        writer.markReady();
    }
    FrameMeta meta(uint64_t seq) const {
        FrameMeta m;
        m.seq = seq;
        m.host_ns = monotonicNs();
        m.device_us = 100000 + static_cast<int64_t>(seq) * 33333;
        m.depth_device_us = m.device_us - 500;
        m.sys_us = 1700000000000000LL + m.device_us;
        m.width = 4;
        m.height = 3;
        m.gray_bytes = sizeof(gray);
        m.depth_bytes = sizeof(depth);
        m.flags = kFrameHasDepth;
        return m;
    }
    void publish(const FrameMeta& m) {
        CHECK(writer.publish(m, gray, reinterpret_cast<const uint8_t*>(depth), nullptr));
    }
};

void framesAndTimebase() {
    Fixture f;
    f.create();
    CHECK(f.source.start(f.config, &f.error));
    const auto calibration = f.source.depthInfo();
    CHECK(calibration.valid());
    CHECK(calibration.width == 2 && calibration.height == 2);
    CHECK(calibration.t_c2d_m[0] == 0.025 && calibration.t_c2d_m[1] == -0.01);
    auto m = f.meta(1);
    f.publish(m);
    VioFrame frame;
    CHECK(f.source.pop(frame, 20));
    CHECK(frame.gray.type() == CV_8UC1 && frame.gray.rows == 3 && frame.gray.cols == 4);
    CHECK(frame.gray.at<uint8_t>(2, 3) == 12);
    CHECK(frame.depth_mm.type() == CV_16UC1 && frame.depth_mm.at<uint16_t>(1, 1) == 4000);
    CHECK(frame.host_ns == m.host_ns && frame.device_us == m.device_us);
    CHECK(frame.system_us == m.sys_us && frame.depth_device_us == m.depth_device_us);
    CHECK(frame.device_us - frame.depth_device_us == 500);
    CHECK(f.source.lastFrameHostNs() == m.host_ns);
    VioFrame duplicate;
    CHECK(!f.source.pop(duplicate, 0));
    f.gray[0] = 99;
    m = f.meta(4);
    f.publish(m);
    VioFrame newest;
    CHECK(f.source.pop(newest, 20));
    CHECK(f.source.dropped() == 2);
    CHECK(newest.gray.at<uint8_t>(0, 0) == 99);
    CHECK(frame.gray.at<uint8_t>(0, 0) == 1);  // 先前返回的图像不被覆盖
    m = f.meta(5);
    m.depth_bytes = m.flags = 0;
    f.publish(m);
    CHECK(f.source.pop(newest, 20) && newest.depth_mm.empty());
    f.source.stop();
    CHECK(channelExists(f.config.channel));  // 消费者关闭不能 unlink 服务端通道
}

void startupValidation(const std::string& calibration_dir) {
    Fixture f;
    CHECK(!f.source.start(f.config, &f.error));
    CHECK(f.error.find("camera_server must be started first") != std::string::npos);
    f.create();
    auto wrong = f.config;
    wrong.width = 5;
    CHECK(!f.source.start(wrong, &f.error));
    CHECK(f.error.find("resolution/fps") != std::string::npos);
    wrong = f.config;
    wrong.fps = 60;
    CHECK(!f.source.start(wrong, &f.error));
    CHECK(f.error.find("resolution/fps") != std::string::npos);
    if (!calibration_dir.empty()) {
        wrong = f.config;
        wrong.config_dir = calibration_dir;
        CHECK(!f.source.start(wrong, &f.error));
        CHECK(f.error.find("cam0.resolution") != std::string::npos);
        wrong.width = wrong.height = 0;
        CHECK(!f.source.start(wrong, &f.error));
        CHECK(f.error.find("resolution/fps") != std::string::npos);
    }
    CHECK(f.source.start(f.config, &f.error));
    CHECK(!f.source.start(f.config, &f.error));
    CHECK(f.source.running());
    f.source.stop();
    f.writer.close();
    f.create(ChannelType::color);
    CHECK(!f.source.start(f.config, &f.error));
    CHECK(f.error.find("layout/version") != std::string::npos);
}

void depthValidation() {
    Fixture f;
    f.stream.R_c2d[0] = 0;
    f.create();
    CHECK(!f.source.start(f.config, &f.error));
    CHECK(f.error.find("depth calibration") != std::string::npos);
    f.writer.close();
    f.stream.R_c2d[0] = 1;
    f.stream.has_depth = 0;
    f.create();
    CHECK(!f.source.start(f.config, &f.error));
    f.config.use_depth = false;
    CHECK(f.source.start(f.config, &f.error));
    CHECK(!f.source.depthInfo().valid());
    auto m = f.meta(1);
    m.depth_bytes = m.flags = 0;
    f.publish(m);
    VioFrame out;
    CHECK(f.source.pop(out, 0) && out.depth_mm.empty());
    f.source.stop();
    f.writer.close();
    f.stream.has_depth = 1;
    f.create();
    CHECK(f.source.start(f.config, &f.error));
    f.publish(f.meta(1));
    CHECK(f.source.pop(out, 0) && out.depth_mm.empty());
}

void discardStopAndRestart() {
    Fixture f;
    f.create();
    CHECK(f.source.start(f.config, &f.error));
    auto m = f.meta(1);
    f.source.discardBefore(m.host_ns + 1);
    f.publish(m);
    VioFrame out;
    CHECK(!f.source.pop(out, 0));
    CHECK(f.source.dropped() == 1);
    m = f.meta(2);
    f.publish(m);
    CHECK(f.source.pop(out, 20));
    std::atomic<bool> waiting{false};
    std::atomic<bool> returned{false};
    std::thread reader([&] {
        waiting = true;
        VioFrame empty;
        returned = !f.source.pop(empty, 5000);
    });
    while (!waiting.load()) {
        std::this_thread::yield();
    }
    f.source.stop();
    reader.join();
    CHECK(returned.load());
    CHECK(f.source.start(f.config, &f.error));
    CHECK(f.source.pop(out, 0));
    CHECK(f.source.dropped() == 0);
    f.writer.markClosing();
    CHECK(!f.source.running());
    CHECK(!f.source.pop(out, 0));
    CHECK(!f.source.lastError().empty());
}

void expiredHeartbeat() {
    Fixture f;
    f.create();
    CHECK(f.source.start(f.config, &f.error));
    const int fd = shm_open(f.config.channel.c_str(), O_RDWR, 0);
    CHECK(fd >= 0);
    void* memory = mmap(nullptr, kHeaderBytes, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
    close(fd);
    CHECK(memory != MAP_FAILED);
    auto* header = static_cast<ShmHeader*>(memory);
    header->heartbeat_ns.store(static_cast<uint64_t>(monotonicNs()) - kHeartbeatTimeoutNs - 1);
    munmap(memory, kHeaderBytes);
    CHECK(!f.source.running());
    VioFrame out;
    CHECK(!f.source.pop(out, 0));
    CHECK(f.source.lastError().find("timed out") != std::string::npos);
    f.source.stop();
    CHECK(!f.source.start(f.config, &f.error));
    CHECK(f.error.find("heartbeat has expired") != std::string::npos);
}

void changedSessionsAndMalformedFrames() {
    for (int mode = 0; mode < 5; ++mode) {
        Fixture f;
        f.create();
        CHECK(f.source.start(f.config, &f.error));
        auto m = f.meta(1);
        if (mode == 0) {
            f.writer.setSession(2, 0);
        }
        if (mode == 1) {
            f.writer.setSession(1, 1);
        }
        if (mode == 2) {
            --m.gray_bytes;
        }
        if (mode == 3) {
            m.depth_bytes -= 2;
        }
        if (mode == 4) {
            m.width = 5;
        }
        f.publish(m);
        VioFrame out;
        CHECK(!f.source.pop(out, 0));
        CHECK(!f.source.running());
        CHECK(!f.source.lastError().empty());
    }
}
}  // namespace

int main(int argc, char** argv) {
    try {
        framesAndTimebase();
        startupValidation(argc > 1 ? argv[1] : "");
        depthValidation();
        discardStopAndRestart();
        expiredHeartbeat();
        changedSessionsAndMalformedFrames();
        std::puts("test_shm_camera_source: OK");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "test_shm_camera_source: %s\n", e.what());
        return 1;
    }
}
