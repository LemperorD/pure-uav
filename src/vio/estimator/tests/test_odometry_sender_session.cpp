#include <csignal>
#include <sys/wait.h>
#include "pty_test_helpers.hpp"
using namespace wire_test;

struct Child {
    pid_t pid = -1;
    bool reaped = false;
    ~Child() { if (pid > 0 && !reaped) { kill(pid, SIGKILL); waitpid(pid, nullptr, 0); } }
};
enum class Scenario { Normal, Rollback, ForwardJump, NoImu, StaleImu };
void runScenario(const char* executable, Scenario scenario) {
    Pty pty;
    Child child;
    child.pid = fork(); check(child.pid >= 0, "fork sender");
    if (child.pid == 0) {
        ::close(pty.master);
        int sink = ::open("/dev/null", O_WRONLY);
        if (sink >= 0) { dup2(sink, STDOUT_FILENO); dup2(sink, STDERR_FILENO); ::close(sink); }
        execl(executable, executable, "--serial", pty.slave.c_str(), "--duration", "1.8",
              "--hz-wait", "0.3", static_cast<char*>(nullptr));
        std::_Exit(127);
    }
    const auto start = steady_clock::now();
    Reader reader(pty.master);
    int status = 0, ticks = 0, before = 0, late = 0;
    uint64_t last_timestamp = 0;
    while (steady_clock::now() - start < seconds(4)) {
        const double elapsed = duration<double>(steady_clock::now() - start).count();
        if (elapsed > 0.05) {
            if (ticks % 20 == 0) heartbeat(pty.master);
            if (scenario != Scenario::NoImu && !(scenario == Scenario::StaleImu && elapsed > 0.8)) {
                uint64_t timestamp = 100000000ULL + static_cast<uint64_t>(elapsed * 1e6);
                if (elapsed >= 0.8 && scenario == Scenario::Rollback)
                    timestamp = 1000000ULL + static_cast<uint64_t>((elapsed - 0.8) * 1e6);
                if (elapsed >= 0.8 && scenario == Scenario::ForwardJump) timestamp += 120000000ULL;
                imu(pty.master, timestamp);
            }
        }
        mavlink_message_t message{};
        while (reader.next(message, 1)) {
            if (message.msgid != MAVLINK_MSG_ID_ODOMETRY) continue;
            mavlink_odometry_t odom{}; mavlink_msg_odometry_decode(&message, &odom);
            check(odom.time_usec > last_timestamp, "sender timestamps not monotonic");
            last_timestamp = odom.time_usec;
            if (elapsed < 0.8) ++before;
            if (elapsed > (scenario == Scenario::StaleImu ? 1.45 : 1.0)) ++late;
        }
        if (waitpid(child.pid, &status, WNOHANG) == child.pid) { child.reaped = true; break; }
        ++ticks;
        std::this_thread::sleep_for(milliseconds(4));
    }
    check(child.reaped, "sender did not exit within deadline");
    check(WIFEXITED(status), "sender terminated abnormally");
    if (scenario == Scenario::Normal) {
        check(WEXITSTATUS(status) == 0 && before > 5 && late > 5, "normal sender run failed");
    } else {
        check(WEXITSTATUS(status) == 1, "invalid session did not return failure");
        check(late == 0, "sender continued transmitting on invalid epoch/stale IMU");
        if (scenario != Scenario::NoImu) check(before > 5, "scenario never reached normal sending");
        else check(last_timestamp == 0, "sender transmitted without IMU");
    }
}
int main(int argc, char** argv) {
    if (argc != 2) return 2;
    try {
        for (auto scenario : {Scenario::Normal, Scenario::Rollback, Scenario::ForwardJump,
                              Scenario::NoImu, Scenario::StaleImu}) runScenario(argv[1], scenario);
        std::puts("PASS: actual sender normal, rollback, forward jump, missing and stale IMU");
        return 0;
    } catch (const std::exception& e) { std::fprintf(stderr, "FAIL: %s\n", e.what()); return 1; }
}
