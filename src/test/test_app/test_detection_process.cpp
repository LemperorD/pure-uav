// 检测子进程管理无硬件回归：启动、回收、重启上限、停止、信号转发与参数透传。
// 用 /bin/true 与临时 shell 脚本作子进程，不依赖飞控、相机或 NPU。
#include <sys/wait.h>
#include <unistd.h>

#include <cerrno>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include "detection_process.hpp"

namespace {
using pure_uav::app::DetectionProcess;
using pure_uav::app::DetectionProcessOptions;
using pure_uav::app::SuperviseStatus;

int g_failures = 0;
std::vector<std::string> g_temp_files;

#define CHECK(cond)                                                              \
    do {                                                                         \
        if (!(cond)) {                                                           \
            std::fprintf(stderr, "FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond); \
            ++g_failures;                                                        \
        }                                                                        \
    } while (false)

std::string suffix() {
    return std::to_string(static_cast<long long>(getpid()));
}

std::string tempPath(const std::string& tag, const std::string& ext) {
    const auto path = std::filesystem::temp_directory_path() / ("pure_uav_det_proc_" + suffix() + "_" + tag + ext);
    const std::string result = path.string();
    g_temp_files.push_back(result);
    return result;
}

std::string makeScript(const std::string& tag, const std::string& body) {
    const std::string path = tempPath(tag, ".sh");
    std::ofstream out(path, std::ios::binary | std::ios::trunc);
    out << "#!/bin/sh\n" << body << "\n";
    out.close();
    std::error_code ec;
    std::filesystem::permissions(path, std::filesystem::perms::owner_all | std::filesystem::perms::group_read | std::filesystem::perms::group_exec | std::filesystem::perms::others_read | std::filesystem::perms::others_exec, std::filesystem::perm_options::add, ec);
    return path;
}

std::string readFile(const std::string& path) {
    std::ifstream in(path);
    std::ostringstream text;
    text << in.rdbuf();
    return text.str();
}

DetectionProcessOptions makeOptions(const std::string& executable) {
    DetectionProcessOptions options;
    options.executable = executable;
    options.max_restarts = 2;
    options.restart_delay_ms = 10;
    options.stop_timeout_ms = 500;
    options.poll_interval_ms = 10;
    return options;
}

bool reaped(pid_t child) {
    int status = 0;
    return waitpid(child, &status, WNOHANG) < 0 && errno == ECHILD;
}

void testNormalExit() {
    DetectionProcess process(makeOptions("/bin/true"));
    std::string error;
    CHECK(process.start(&error));
    CHECK(error.empty());
    CHECK(process.running());
    const pid_t child = process.pid();
    const auto result = process.supervise([] { return false; });
    CHECK(result.status == SuperviseStatus::exited);
    CHECK(result.restarts == 0);
    CHECK(!process.running());
    CHECK(reaped(child));  // 没有僵尸进程
}

void testMissingExecutable() {
    DetectionProcess process(makeOptions("/tmp/pure_uav_no_such_detect_binary"));
    std::string error;
    CHECK(!process.start(&error));
    CHECK(error.find("不可执行") != std::string::npos);
    CHECK(!process.running());

    DetectionProcessOptions empty = makeOptions("");
    DetectionProcess blank(empty);
    std::string blank_error;
    CHECK(!blank.start(&blank_error));
    CHECK(!blank_error.empty());
}

void testRestartLimit() {
    const std::string script = makeScript("fail", "exit 3");
    DetectionProcess process(makeOptions(script));
    std::string error;
    CHECK(process.start(&error));
    const auto result = process.supervise([] { return false; });
    CHECK(result.status == SuperviseStatus::restart_limit);
    CHECK(result.restarts == 2);
    CHECK(result.last_exit_code == 3);
    CHECK(!result.error.empty());
    CHECK(!process.running());
}

void testStopRequest() {
    const std::string script = makeScript("loop", "while :; do sleep 1; done");
    DetectionProcess process(makeOptions(script));
    std::string error;
    CHECK(process.start(&error));
    const pid_t child = process.pid();
    const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(80);
    const auto result = process.supervise([deadline] { return std::chrono::steady_clock::now() >= deadline; });
    CHECK(result.status == SuperviseStatus::stopped);
    CHECK(!process.running());
    CHECK(reaped(child));
    process.stop();  // 幂等
    process.stop();
}

void testSignalForwarding() {
    const std::string marker = tempPath("term_marker", ".txt");
    std::string body = "trap 'echo terminated > MARKER; exit 0' TERM\nwhile :; do sleep 0.1; done";
    const auto at = body.find("MARKER");
    CHECK(at != std::string::npos);
    body.replace(at, std::string("MARKER").size(), marker);
    const std::string script = makeScript("trap", body);

    DetectionProcess process(makeOptions(script));
    std::string error;
    CHECK(process.start(&error));
    const pid_t child = process.pid();
    std::this_thread::sleep_for(std::chrono::milliseconds(200));  // 等子进程装好 trap
    process.stop();
    CHECK(!process.running());
    CHECK(reaped(child));
    CHECK(readFile(marker).find("terminated") != std::string::npos);
}

void testArguments() {
    const std::string marker = tempPath("args_marker", ".txt");
    std::string body = "printf '%s\\n' \"$@\" > MARKER\nwhile :; do sleep 0.1; done";
    const auto at = body.find("MARKER");
    CHECK(at != std::string::npos);
    body.replace(at, std::string("MARKER").size(), marker);
    const std::string script = makeScript("args", body);

    DetectionProcessOptions options = makeOptions(script);
    options.arguments = {"--config", "alt.yaml", "--json"};
    DetectionProcess process(options);
    std::string error;
    CHECK(process.start(&error));
    std::this_thread::sleep_for(std::chrono::milliseconds(200));
    process.stop();

    const std::string args = readFile(marker);
    CHECK(args.find("--config") != std::string::npos);
    CHECK(args.find("alt.yaml") != std::string::npos);
    CHECK(args.find("--json") != std::string::npos);
    CHECK(args.find("--model") == std::string::npos);  // 不注入算法参数
}

}  // namespace

int main() {
    testNormalExit();
    testMissingExecutable();
    testRestartLimit();
    testStopRequest();
    testSignalForwarding();
    testArguments();

    for (const auto& path : g_temp_files) {
        std::error_code ec;
        std::filesystem::remove(path, ec);
    }
    if (g_failures != 0) {
        std::fprintf(stderr, "test_detection_process: %d failure(s)\n", g_failures);
        return 1;
    }
    std::printf("test_detection_process: OK\n");
    return 0;
}
