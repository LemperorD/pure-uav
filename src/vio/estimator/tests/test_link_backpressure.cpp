#include <future>

#include "mavlink_link.hpp"
#include "pty_test_helpers.hpp"
using namespace wire_test;
int main() {
    try {
        Pty pty;
        pure_uav::MavlinkLink link;
        pure_uav::MavlinkLink::Config cfg;
        cfg.device = pty.slave;
        cfg.send_heartbeat = false;
        cfg.send_timeout_ms = 100;
        cfg.heartbeat_timeout_ms = 5000;
        std::string error;
        check(link.start(cfg, &error), "start");
        heartbeat(pty.master);
        check(waitFor([&] { return link.connected(); }), "connect");
        Reader reader(pty.master);
        reader.drain();
        // 故意不读输出，直到 PTY 背压。必须在总期限内返回失败。
        pure_uav::VisionPose pose{};
        bool blocked = false;
        for (int i = 0; i < 1000; ++i) {
            const auto start = steady_clock::now();
            const bool sent = link.sendVisionPose(pose);
            check(steady_clock::now() - start < milliseconds(500), "send exceeded bounded deadline");
            if (!sent) {
                blocked = true;
                break;
            }
        }
        check(blocked, "test did not fill PTY output");
        // 再次处于背压时，stop 必须能取消写并结束接收线程；不能靠 drain 来解锁。
        auto send = std::async(std::launch::async, [&] { return link.sendVisionPose(pose); });
        std::this_thread::sleep_for(milliseconds(10));
        const auto start = steady_clock::now();
        link.stop();
        check(steady_clock::now() - start < milliseconds(700), "stop blocked behind serial output");
        check(send.wait_for(milliseconds(500)) == std::future_status::ready, "sender did not terminate");
        check(!send.get(), "stopped send succeeded");
        std::puts("PASS: backpressure send deadline and cancellation");
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "FAIL: %s\n", e.what());
        return 1;
    }
}
