#pragma once
// 检测子进程生命周期管理（header-only）。
// bin/main 与无硬件单测共用同一实现：main 只负责启动、监督、重启与停止
// bin/detect_rt；检测监督部分不解析算法配置、不打开帧通道、不参与推理。
#include <fcntl.h>
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <csignal>
#include <cstdint>
#include <cstdio>
#include <cstring>
#include <functional>
#include <string>
#include <thread>
#include <utility>
#include <vector>

namespace pure_uav::app {

struct DetectionProcessOptions {
    std::string executable;
    std::vector<std::string> arguments;  // 仅运行参数透传，不含检测算法参数
    int max_restarts = 3;
    int restart_delay_ms = 2000;
    int stop_timeout_ms = 3000;
    int poll_interval_ms = 500;
    int stop_signal = SIGTERM;
    int target_publish_fd = -1;  // 父进程持有；仅此 fd 在检测 exec 时保留。
};

enum class SuperviseStatus {
    exited,         // 子进程正常退出（exit 0）
    stopped,        // 收到停止请求，子进程已回收
    restart_limit,  // 异常退出次数超过上限
    error,          // fork / waitpid / 重新启动失败
};

struct SuperviseResult {
    SuperviseStatus status = SuperviseStatus::error;
    std::string error;
    uint64_t restarts = 0;
    int last_exit_code = 0;
};

class DetectionProcess {
  public:
    explicit DetectionProcess(DetectionProcessOptions options) : options_(std::move(options)) {
    }
    ~DetectionProcess() {
        stop();
    }

    DetectionProcess(const DetectionProcess&) = delete;
    DetectionProcess& operator=(const DetectionProcess&) = delete;

    bool start(std::string* error) noexcept {
        try {
            return startImpl(error);
        } catch (...) {
            setError(error, "启动检测子进程时发生异常");
            return false;
        }
    }

    SuperviseResult supervise(std::function<bool()> should_stop) noexcept {
        SuperviseResult result;
        try {
            superviseImpl(std::move(should_stop), result);
        } catch (...) {
            result.status = SuperviseStatus::error;
            setError(&result.error, "监督检测子进程时发生异常");
        }
        result.restarts = restarts_;
        return result;
    }

    void stop() noexcept {
        if (pid_ <= 0) {
            return;
        }
        const pid_t child = pid_;
        signalChild(options_.stop_signal);
        int status = 0;
        bool reaped = waitWithTimeout(child, options_.stop_timeout_ms, status);
        if (!reaped) {
            std::fprintf(stderr, "[main] 检测子进程未在 %d ms 内退出，强制回收\n", options_.stop_timeout_ms);
            signalChild(SIGKILL);
            while (waitpid(child, &status, 0) < 0 && errno == EINTR) {
            }
        }
        pid_ = -1;
        grouped_ = false;
    }

    [[nodiscard]] bool running() const noexcept {
        return pid_ > 0;
    }
    [[nodiscard]] uint64_t restarts() const noexcept {
        return restarts_;
    }
    [[nodiscard]] pid_t pid() const noexcept {
        return pid_;
    }

  private:
    static void setError(std::string* error, const std::string& reason) noexcept {
        if (error == nullptr) {
            return;
        }
        try {
            *error = reason;
        } catch (...) {
        }
    }

    static bool stopRequested(const std::function<bool()>& should_stop) {
        const bool has_stop = static_cast<bool>(should_stop);
        return has_stop && should_stop();
    }

    static bool sleepWithStopCheck(int milliseconds, const std::function<bool()>& should_stop) {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(milliseconds > 0 ? milliseconds : 0);
        do {
            if (stopRequested(should_stop)) {
                return true;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } while (std::chrono::steady_clock::now() < deadline);
        return stopRequested(should_stop);
    }

    static bool waitWithTimeout(pid_t child, int timeout_ms, int& status) noexcept {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(timeout_ms > 0 ? timeout_ms : 0);
        do {
            const pid_t done = waitpid(child, &status, WNOHANG);
            if (done == child) {
                return true;
            }
            if (done < 0 && errno != EINTR) {
                return true;  // ECHILD：子进程已被回收
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        } while (std::chrono::steady_clock::now() < deadline);
        return false;
    }

    void signalChild(int signal_number) const noexcept {
        if (pid_ <= 0) {
            return;
        }
        if (grouped_ && kill(-pid_, signal_number) == 0) {
            return;
        }
        kill(pid_, signal_number);
    }

    bool startImpl(std::string* error) {
        if (pid_ > 0) {
            return true;
        }
        if (options_.executable.empty()) {
            setError(error, "检测子程序路径为空");
            return false;
        }
        if (access(options_.executable.c_str(), X_OK) != 0) {
            setError(error, "检测子程序不可执行: " + options_.executable);
            return false;
        }

        // argv 必须在 fork 之前准备好：main 进程内 Communicator 已启动工作线程，
        // 子进程里做内存分配可能碰到被持有的分配器锁。
        std::vector<std::string> args;
        args.reserve(options_.arguments.size() + 2);
        args.push_back(options_.executable);
        for (const auto& argument : options_.arguments) {
            args.push_back(argument);
        }
        std::vector<char*> argv;
        argv.reserve(args.size() + 1);
        for (auto& argument : args) {
            argv.push_back(argument.data());
        }
        argv.push_back(nullptr);

        std::fflush(nullptr);
        const pid_t child = fork();
        if (child < 0) {
            setError(error, std::string("fork 失败: ") + std::strerror(errno));
            return false;
        }
        if (child == 0) {
            // 自成进程组，避免终端信号同时投递给父子进程造成重复处理。
            setpgid(0, 0);
            if (options_.target_publish_fd >= 0 && fcntl(options_.target_publish_fd, F_SETFD, 0) < 0) {
                _exit(127);
            }
            execv(options_.executable.c_str(), argv.data());
            std::fprintf(stderr, "[main] 启动检测子进程失败: %s: %s\n", options_.executable.c_str(), std::strerror(errno));
            _exit(127);
        }
        const int group = setpgid(child, child);
        grouped_ = (group == 0 || errno == EACCES);
        pid_ = child;
        std::fprintf(stderr, "[main] 检测子进程已启动: pid=%d %s\n", static_cast<int>(child), options_.executable.c_str());
        return true;
    }

    void superviseImpl(const std::function<bool()>& should_stop, SuperviseResult& result) {
        const uint64_t limit = options_.max_restarts > 0 ? static_cast<uint64_t>(options_.max_restarts) : 0;
        for (;;) {
            if (stopRequested(should_stop)) {
                stop();
                result.status = SuperviseStatus::stopped;
                return;
            }
            if (pid_ <= 0) {
                result.status = SuperviseStatus::error;
                setError(&result.error, "检测子进程未启动或已被回收");
                return;
            }

            int status = 0;
            const pid_t done = waitpid(pid_, &status, WNOHANG);
            if (done == 0) {
                if (sleepWithStopCheck(options_.poll_interval_ms, should_stop)) {
                    stop();
                    result.status = SuperviseStatus::stopped;
                    return;
                }
                continue;
            }
            if (done < 0) {
                if (errno == EINTR) {
                    continue;
                }
                result.status = SuperviseStatus::error;
                setError(&result.error, std::string("waitpid 失败: ") + std::strerror(errno));
                return;
            }

            pid_ = -1;
            grouped_ = false;
            const bool exited_normally = WIFEXITED(status) && WEXITSTATUS(status) == 0;
            result.last_exit_code = WIFEXITED(status) ? WEXITSTATUS(status) : -1;
            if (exited_normally) {
                std::fprintf(stderr, "[main] 检测子进程正常退出\n");
                result.status = SuperviseStatus::exited;
                return;
            }
            if (restarts_ >= limit) {
                std::fprintf(stderr, "[main] 检测子进程异常退出(code=%d)，已达重启上限 %llu 次\n", result.last_exit_code, static_cast<unsigned long long>(limit));
                result.status = SuperviseStatus::restart_limit;
                setError(&result.error, "检测子进程反复异常退出，已达重启上限");
                return;
            }

            ++restarts_;
            std::fprintf(stderr, "[main] 检测子进程异常退出(code=%d)，第 %llu/%llu 次重启\n", result.last_exit_code, static_cast<unsigned long long>(restarts_), static_cast<unsigned long long>(limit));
            if (sleepWithStopCheck(options_.restart_delay_ms, should_stop)) {
                stop();
                result.status = SuperviseStatus::stopped;
                return;
            }
            std::string error;
            if (!start(&error)) {
                result.status = SuperviseStatus::error;
                setError(&result.error, error);
                return;
            }
        }
    }

    DetectionProcessOptions options_;
    pid_t pid_ = -1;
    bool grouped_ = false;
    uint64_t restarts_ = 0;
};

}  // namespace pure_uav::app
