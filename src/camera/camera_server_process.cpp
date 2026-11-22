#include "camera_server_process.hpp"

#include <fcntl.h>
#include <poll.h>
#include <signal.h>
#include <spawn.h>
#include <sys/wait.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <thread>

namespace pure_uav::camera {
namespace {
using Clock = std::chrono::steady_clock;
using namespace std::chrono_literals;

bool waitForReady(int fd, int timeout_ms, const std::atomic<bool>& interrupted, std::string& error) {
    const auto deadline = Clock::now() + std::chrono::milliseconds(timeout_ms);
    while (!interrupted.load()) {
        const auto remaining = std::chrono::ceil<std::chrono::milliseconds>(deadline - Clock::now());
        pollfd ready{fd, POLLIN, 0};
        const int pending = poll(&ready, 1, static_cast<int>(std::clamp(remaining, 0ms, 50ms).count()));
        if (pending > 0) {
            char marker = 0;
            if ((ready.revents & POLLIN) != 0 && read(fd, &marker, sizeof(marker)) == sizeof(marker) && marker == 'R') {
                return true;
            }
            error = "camera_server 在就绪前退出，请查看相机服务日志";
            return false;
        }
        if (pending < 0 && errno != EINTR) {
            error = std::string("等待相机服务失败: ") + std::strerror(errno);
            return false;
        }
        if (Clock::now() >= deadline) {
            error = "等待 camera_server 就绪超时（" + std::to_string(timeout_ms) + " ms）";
            return false;
        }
    }
    error = "相机启动已取消";
    return false;
}
}  // namespace

CameraServerProcess::~CameraServerProcess() {
    stop();
}

bool CameraServerProcess::start(const std::string& config, const std::string& channel, int timeout_ms, const std::atomic<bool>& interrupted, std::string& error) {
    stop();
    constexpr int child_ready_fd = STDERR_FILENO + 1;
    const auto executable = (std::filesystem::read_symlink("/proc/self/exe").parent_path() / "camera_server").string();
    std::array<std::string, 7> args{executable, "--config", config, "--vio-channel", channel, "--ready-fd", std::to_string(child_ready_fd)};
    std::array<char*, args.size() + 1> argv{};
    for (size_t index = 0; index < args.size(); ++index) {
        argv[index] = args[index].data();
    }
    int ready_pipe[2];
    if (pipe2(ready_pipe, O_CLOEXEC) != 0) {
        error = std::string("创建相机就绪管道失败: ") + std::strerror(errno);
        return false;
    }
    posix_spawn_file_actions_t actions;
    int status = posix_spawn_file_actions_init(&actions);
    if (status == 0) {
        status = posix_spawn_file_actions_adddup2(&actions, ready_pipe[1], child_ready_fd);
        if (status == 0) {
            status = posix_spawn(&pid_, argv[0], &actions, nullptr, argv.data(), environ);
        }
        posix_spawn_file_actions_destroy(&actions);
    }
    close(ready_pipe[1]);
    if (status != 0) {
        close(ready_pipe[0]);
        pid_ = -1;
        error = args.front() + ": " + std::strerror(status);
        return false;
    }
    const bool ready = waitForReady(ready_pipe[0], timeout_ms, interrupted, error);
    close(ready_pipe[0]);
    if (!ready) {
        stop();
    }
    return ready;
}

void CameraServerProcess::stop() noexcept {
    if (pid_ <= 0) {
        return;
    }
    kill(pid_, SIGTERM);
    const auto deadline = Clock::now() + 3s;
    do {
        const pid_t done = waitpid(pid_, nullptr, WNOHANG);
        if (done == pid_ || (done < 0 && errno != EINTR)) {
            pid_ = -1;
            return;
        }
        std::this_thread::sleep_for(10ms);
    } while (Clock::now() < deadline);

    std::fprintf(stderr, "[CAM] camera_server 未及时退出，强制回收\n");
    kill(pid_, SIGKILL);
    while (waitpid(pid_, nullptr, 0) < 0 && errno == EINTR) {
    }
    pid_ = -1;
}

}  // namespace pure_uav::camera
