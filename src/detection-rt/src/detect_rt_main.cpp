// detect_rt：camera_server 彩色帧通道的实时检测消费者 / 离线单图检测 CLI。
// 本进程不再打开 Orbbec 设备；相机只由 camera_server 初始化一次。
// --json 时 stdout 只输出 JSON 行。参数与推流用法见同目录 README。
#include <unistd.h>

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <csignal>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <memory>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <string>
#include <thread>
#include <vector>

#include "camera_shm.hpp"
#include "detection_runtime.hpp"
#include "detection_types.hpp"
#include "detector_config.hpp"
#include "yolo_backend.hpp"
#include "yolo_postprocess.hpp"

namespace {

using pure_uav::ColorEncoding;
using pure_uav::camera::FrameMeta;
using pure_uav::camera::ShmChannelReader;
using pure_uav::detection::DetectionRuntime;
using pure_uav::detection::FrameResult;
using pure_uav::detection::InferenceBackend;
using pure_uav::detection::PostprocessParams;
using pure_uav::detection::RuntimeConfig;
using pure_uav::detection::TargetSnapshot;
using pure_uav::detection::YoloBackend;
using pure_uav::detection::YoloOutputView;

std::atomic<bool> g_sigint{false};

// --json 时把进程 stdout 重定向到 stderr，JSON 行写原始 stdout 的独立流，
// 这样驱动/VIPLite 自己打印到 stdout 的日志不会污染 JSON。
FILE* g_json_out = stdout;

void onSigint(int /*unused*/) {
    g_sigint.store(true);
}

uint64_t unixNowUs() {
    using namespace std::chrono;
    return static_cast<uint64_t>(duration_cast<microseconds>(system_clock::now().time_since_epoch()).count());
}

// 把现有 YoloBackend（CPU 的 ONNX Runtime 或板端 NPU）适配成 DetectionRuntime
// 需要的 InferenceBackend 接口。CPU 构建下 makeNpuBackend() 返回空，因此必须
// 走这里，否则检测运行时拿不到任何后端。
class RuntimeBackend final : public InferenceBackend {
  public:
    bool load(const RuntimeConfig& config, std::string& error) override {
        config_ = config;
        return backend_.load(config.model_path, config.imgsz, error);
    }
    bool infer(const cv::Mat& bgr, FrameResult& result, std::string& error) override {
        YoloOutputView view;
        if (!backend_.infer(bgr, view, error)) {
            return false;
        }
        PostprocessParams params;
        params.num_classes = config_.num_classes;
        params.conf_thr = config_.conf_thr;
        params.iou_thr = config_.iou_thr;
        params.max_det = config_.max_det;
        params.img_w = bgr.cols;
        params.img_h = bgr.rows;
        result.infer_ms = view.infer_ms;
        if (!postprocess(view, params, result)) {
            error = "Unsupported model output: expected decoded YOLO xywh + configured class probabilities";
            return false;
        }
        // 本树 yolo_postprocess.hpp 尚无 target_class 字段，而 DetectionRuntime 的
        // validResult() 要求启用类别过滤时不得出现其它类别，故在后端先行过滤。
        // 主目标字段随后由 DetectionRuntime::selectPrimary() 统一重算。
        if (config_.target_class >= 0) {
            int kept = 0;
            for (int index = 0; index < result.count; ++index) {
                if (result.boxes[index].class_id == config_.target_class) {
                    if (kept != index) {
                        result.boxes[kept] = result.boxes[index];
                    }
                    ++kept;
                }
            }
            result.count = kept;
            result.present = kept > 0;
        }
        return true;
    }
    void close() noexcept override {
        backend_.release();
    }
    [[nodiscard]] std::string info() const {
        return backend_.info();
    }

  private:
    RuntimeConfig config_;
    YoloBackend backend_;
};

void printBoxes(const FrameResult& r, bool json, int64_t captured_host_ns = 0) {
    FILE* out = json ? g_json_out : stdout;
    if (json) {
        // DetectionRuntime 把 host_us 置 0：单调时钟不能冒充 Unix/飞控时间。
        // 采集时刻改以 host_ns（CLOCK_MONOTONIC）输出，由调用方按需换算。
        std::fprintf(out,
                     "{\"host_us\":%llu,\"host_ns\":%lld,\"seq\":%llu,\"infer_ms\":%.3f,\"present\":%s,"
                     "\"count\":%d,\"boxes\":[",
                     static_cast<unsigned long long>(r.host_us), static_cast<long long>(captured_host_ns), static_cast<unsigned long long>(r.seq), static_cast<double>(r.infer_ms), r.present ? "true" : "false", r.count);
        for (int i = 0; i < r.count; ++i) {
            const auto& b = r.boxes[i];
            const double u = 0.5 * (b.x1 + b.x2);
            const double v = 0.5 * (b.y1 + b.y2);
            const double w = b.x2 - b.x1;
            const double h = b.y2 - b.y1;
            std::fprintf(out,
                         "%s{\"class_id\":%d,\"conf\":%.6f,\"x1\":%.3f,\"y1\":%.3f,"
                         "\"x2\":%.3f,\"y2\":%.3f,\"u\":%.3f,\"v\":%.3f,\"w\":%.3f,"
                         "\"h\":%.3f}",
                         (i != 0) ? "," : "", b.class_id, static_cast<double>(b.conf), static_cast<double>(b.x1), static_cast<double>(b.y1), static_cast<double>(b.x2), static_cast<double>(b.y2), u, v, w, h);
        }
        std::fprintf(out, "]}\n");
        std::fflush(out);
        return;
    }

    for (int i = 0; i < r.count; ++i) {
        const auto& b = r.boxes[i];
        std::printf(
            "box[%d] class=%d conf=%.4f x1=%.1f y1=%.1f x2=%.1f y2=%.1f "
            "center=(%.1f,%.1f) size=%.1fx%.1f\n",
            i, b.class_id, static_cast<double>(b.conf), static_cast<double>(b.x1), static_cast<double>(b.y1), static_cast<double>(b.x2), static_cast<double>(b.y2), 0.5 * (b.x1 + b.x2), 0.5 * (b.y1 + b.y2), static_cast<double>(b.x2 - b.x1), static_cast<double>(b.y2 - b.y1));
    }
    if (r.present) {
        std::printf("main target: class=%d conf=%.4f u=%.1f v=%.1f w=%.1f h=%.1f\n", r.class_id, r.conf, r.u_px, r.v_px, r.w_px, r.h_px);
    } else {
        std::printf("main target: present=false\n");
    }
}

cv::Mat drawAnnotated(const cv::Mat& bgr, const FrameResult& r) {
    cv::Mat vis = bgr.clone();
    for (int i = 0; i < r.count; ++i) {
        const auto& b = r.boxes[i];
        const cv::Scalar color = (b.class_id == 1) ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
        cv::rectangle(vis, cv::Point(static_cast<int>(b.x1), static_cast<int>(b.y1)), cv::Point(static_cast<int>(b.x2), static_cast<int>(b.y2)), color, 2);
        char text[64];
        std::snprintf(text, sizeof(text), "c%d %.2f", b.class_id, static_cast<double>(b.conf));
        cv::putText(vis, text, cv::Point(static_cast<int>(b.x1), static_cast<int>(b.y1) - 4), cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
    }
    return vis;
}

// 给 mjpg-streamer 提供最新带框图：先写 frame_tmp.bin 再原子 rename 成 frame.jpg。
// 临时文件不带 .jpg 扩展名，--existing 扫描不会把它当视频帧；rename 保证读端只看到完整帧。
bool writeStreamFrame(const cv::Mat& bgr, const FrameResult& r, const std::string& stream_dir) {
    const cv::Mat vis = drawAnnotated(bgr, r);
    const auto dir = std::filesystem::path(stream_dir);
    const std::string tmp = (dir / "frame_tmp.bin").string();
    const std::string dst = (dir / "frame.jpg").string();
    const std::vector<int> params{cv::IMWRITE_JPEG_QUALITY, 80};

    std::vector<unsigned char> jpg;
    if (!cv::imencode(".jpg", vis, jpg, params)) {
        std::fprintf(stderr, "[warn] JPEG 编码失败\n");
        return false;
    }
    {
        std::ofstream ofs(tmp, std::ios::binary | std::ios::trunc);
        if (!ofs) {
            std::fprintf(stderr, "[warn] 打开流帧临时文件失败: %s\n", tmp.c_str());
            return false;
        }
        ofs.write(reinterpret_cast<const char*>(jpg.data()), static_cast<std::streamsize>(jpg.size()));
        if (!ofs) {
            std::fprintf(stderr, "[warn] 写入流帧失败: %s\n", tmp.c_str());
            std::filesystem::remove(tmp);
            return false;
        }
    }
    std::error_code ec;
    std::filesystem::rename(tmp, dst, ec);
    if (ec) {
        std::fprintf(stderr, "[warn] 流帧 rename 失败: %s\n", ec.message().c_str());
        std::filesystem::remove(tmp, ec);
        return false;
    }
    return true;
}

void drawAndSave(const cv::Mat& bgr, const FrameResult& r, const std::string& dump_dir, uint64_t seq) {
    const cv::Mat vis = drawAnnotated(bgr, r);
    char name[128];
    std::snprintf(name, sizeof(name), "detect_%06llu.jpg", static_cast<unsigned long long>(seq));
    const std::string path = (std::filesystem::path(dump_dir) / name).string();
    if (!cv::imwrite(path, vis)) {
        std::fprintf(stderr, "[warn] 保存带框图失败: %s\n", path.c_str());
    }
}

// 离线单图与实时模式共用同一个 RuntimeConfig 与 RuntimeBackend，
// 保证阈值、类别过滤和后处理的解释完全一致。
int runImage(const std::string& image_path, const RuntimeConfig& cfg, bool json, const std::string& dump_dir, const std::string& stream_dir) {
    cv::Mat bgr = cv::imread(image_path, cv::IMREAD_COLOR);
    if (bgr.empty()) {
        std::fprintf(stderr, "[error] 无法读取图片: %s\n", image_path.c_str());
        return 1;
    }

    RuntimeBackend backend;
    std::string error;
    if (!backend.load(cfg, error)) {
        std::fprintf(stderr, "[error] 模型加载失败: %s\n", error.c_str());
        return 1;
    }
    std::fprintf(stderr, "[detect_rt] model=%s\n", cfg.model_path.c_str());
    std::fprintf(stderr, "[detect_rt] %s\n", backend.info().c_str());
    std::fprintf(stderr, "[detect_rt] image=%s %dx%d\n", image_path.c_str(), bgr.cols, bgr.rows);

    FrameResult r{};
    r.host_us = unixNowUs();
    r.seq = 1;
    r.width = bgr.cols;
    r.height = bgr.rows;
    if (!backend.infer(bgr, r, error)) {
        std::fprintf(stderr, "[error] 推理失败: %s\n", error.c_str());
        return 1;
    }

    printBoxes(r, json);
    if (!dump_dir.empty()) {
        drawAndSave(bgr, r, dump_dir, r.seq);
    }
    if (!stream_dir.empty()) {
        writeStreamFrame(bgr, r, stream_dir);
    }
    if (!json) {
        std::printf("summary: infer_ms=%.3f count=%d present=%s\n", static_cast<double>(r.infer_ms), r.count, r.present ? "true" : "false");
    }
    return 0;
}

int runCamera(const RuntimeConfig& cfg, const std::string& shm_name, int shm_timeout_ms, long frames, bool json, const std::string& dump_dir, const std::string& stream_dir, double stream_fps) {
    ShmChannelReader reader;
    std::string error;
    if (!reader.open(shm_name, pure_uav::camera::ChannelType::color, shm_timeout_ms, &error)) {
        std::fprintf(stderr, "[error] 连接相机服务彩色通道失败: %s\n", error.c_str());
        return 1;
    }
    const auto& stream = reader.streamMeta();
    std::fprintf(stderr, "[detect_rt] shm=%s stream=%ux%u@%u model=%s\n", shm_name.c_str(), stream.width, stream.height, stream.fps, cfg.model_path.c_str());

    DetectionRuntime runtime(cfg, std::make_unique<RuntimeBackend>());
    if (!runtime.start(&error)) {
        std::fprintf(stderr, "[error] 检测运行时启动失败: %s\n", error.c_str());
        reader.close();
        return 1;
    }

    long processed = 0;
    uint64_t total_boxes = 0;
    uint64_t present_frames = 0;
    uint64_t last_result_seq = 0;
    uint64_t last_session = 0;
    uint64_t last_generation = 0;
    uint64_t latest_bgr_seq = 0;
    cv::Mat latest_bgr;
    const auto t_start = std::chrono::steady_clock::now();
    auto last_report = t_start;
    std::chrono::steady_clock::time_point heartbeat_lost_since{};

    // 推流限速：JPEG 编码只在需要时做，默认最多 20 帧/秒。
    const std::chrono::duration<double> stream_period(1.0 / std::max(1.0, stream_fps));
    std::chrono::steady_clock::time_point last_stream{};

    while (!g_sigint.load()) {
        FrameMeta meta;
        std::vector<uint8_t> payload;
        uint64_t session = 0;
        uint64_t generation = 0;
        bool got_frame = reader.readLatest(meta, payload, session, generation);
        if (got_frame) {
            if (session != last_session || generation != last_generation) {
                // 相机服务重启/时间基切换：旧帧与旧检测结果一起作废。
                runtime.setEpoch(session, generation);
                last_session = session;
                last_generation = generation;
                last_result_seq = 0;
                latest_bgr.release();
            }
            if (meta.color_bytes != 0 && meta.width > 0 && meta.height > 0 && payload.size() >= meta.color_bytes) {
                pure_uav::EncodedColorFrame encoded;
                encoded.bytes = cv::Mat(1, static_cast<int>(meta.color_bytes), CV_8UC1, payload.data());
                encoded.encoding = static_cast<ColorEncoding>(meta.color_encoding);
                encoded.width = static_cast<int>(meta.width);
                encoded.height = static_cast<int>(meta.height);
                encoded.host_ns = meta.host_ns;
                encoded.device_us = meta.device_us;
                encoded.seq = meta.seq;
                runtime.submit(encoded);

                // DetectionRuntime 解码后不对外暴露 BGR；画框/推流需要自己解一份。
                // 彩色通道已在服务端限频（默认 10 Hz），额外解码开销可控。
                if (!dump_dir.empty() || !stream_dir.empty()) {
                    try {
                        cv::Mat bgr = pure_uav::detection::decodeColorFrame(encoded);
                        if (!bgr.empty()) {
                            latest_bgr = bgr;
                            latest_bgr_seq = meta.seq;
                        }
                    } catch (const std::exception& decode_error) {
                        std::fprintf(stderr, "[warn] 彩色帧解码失败: %s\n", decode_error.what());
                    }
                }
            }
        }

        TargetSnapshot snapshot;
        const bool got = runtime.latest(snapshot, pure_uav::camera::monotonicNs()) && snapshot.result.seq != last_result_seq;
        if (got) {
            last_result_seq = snapshot.result.seq;
            ++processed;
            total_boxes += static_cast<uint64_t>(snapshot.result.count);
            if (snapshot.result.present) {
                ++present_frames;
            }
            printBoxes(snapshot.result, json, snapshot.captured_host_ns);
            const bool frame_matches = !latest_bgr.empty() && latest_bgr_seq >= snapshot.result.seq;
            if (!dump_dir.empty() && frame_matches) {
                drawAndSave(latest_bgr, snapshot.result, dump_dir, snapshot.result.seq);
            }
            if (!stream_dir.empty() && frame_matches) {
                const auto now = std::chrono::steady_clock::now();
                if (last_stream.time_since_epoch().count() == 0 || now - last_stream >= stream_period) {
                    writeStreamFrame(latest_bgr, snapshot.result, stream_dir);
                    last_stream = now;
                }
            }
        }
        if (frames > 0 && processed >= frames) {
            break;
        }

        const auto now = std::chrono::steady_clock::now();
        if (now - last_report >= std::chrono::seconds(1)) {
            const auto stats = runtime.stats();
            std::fprintf(json ? stderr : stdout,
                         "[stats] submitted=%llu replaced=%llu rate_limited=%llu stale=%llu "
                         "processed=%llu detected=%llu infer=%.2f ms last_error=%s\n",
                         static_cast<unsigned long long>(stats.submitted), static_cast<unsigned long long>(stats.replaced), static_cast<unsigned long long>(stats.rate_limited), static_cast<unsigned long long>(stats.stale), static_cast<unsigned long long>(stats.processed),
                         static_cast<unsigned long long>(stats.detected), stats.last_infer_ms, stats.last_error.empty() ? "-" : stats.last_error.c_str());
            last_report = now;
        }

        // 相机服务失联超过 3 s 就退出：由 bin/main 监督重启（最多 3 次），
        // 超限或单独运行时交由外部托管方处置，避免消费者空转。
        if (!reader.writerAlive()) {
            if (heartbeat_lost_since.time_since_epoch().count() == 0) {
                heartbeat_lost_since = now;
            } else if (now - heartbeat_lost_since >= std::chrono::seconds(3)) {
                std::fprintf(stderr, "[error] camera_server 心跳丢失超过 3 s，退出消费者\n");
                break;
            }
        } else {
            heartbeat_lost_since = {};
        }

        // 没有新帧/新结果时短暂让出 CPU。
        if (!got && !got_frame) {
            std::this_thread::sleep_for(std::chrono::milliseconds(2));
        }
    }

    const auto stats = runtime.stats();
    runtime.stop();
    reader.close();

    std::fprintf(json ? stderr : stdout,
                 "summary: submitted=%llu replaced=%llu rate_limited=%llu stale=%llu "
                 "processed=%llu detected=%llu present_frames=%llu total_boxes=%llu\n",
                 static_cast<unsigned long long>(stats.submitted), static_cast<unsigned long long>(stats.replaced), static_cast<unsigned long long>(stats.rate_limited), static_cast<unsigned long long>(stats.stale), static_cast<unsigned long long>(stats.processed),
                 static_cast<unsigned long long>(stats.detected), static_cast<unsigned long long>(present_frames), static_cast<unsigned long long>(total_boxes));
    return 0;
}

void usage() {
    std::fprintf(stderr,
                 "用法: detect_rt [--config FILE] [--check-config]\n"
                 "                [--image FILE] [--frames N] [--json] [--dump-dir DIR]\n"
                 "                [--stream-dir DIR] [--stream-fps FPS]\n"
                 "                [--shm NAME] [--shm-timeout-ms N] [--verbose]\n"
                 "\n检测算法参数（model_path/imgsz/阈值/类别/限频/帧龄）唯一真源为\n"
                 "config/detector-rt.yaml，不提供对应命令行选项。\n"
                 "  --config FILE     指定配置文件路径（默认 config/detector-rt.yaml）\n"
                 "  --check-config    只校验配置，不加载模型、不连接相机服务\n"
                 "\n默认从 camera_server 的彩色共享内存通道取帧；相机只由 camera_server 打开。\n"
                 "--image 为离线单图模式，不连接相机服务。\n");
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, onSigint);

    RuntimeConfig cfg;
    std::string config_path = pure_uav::detection::kDefaultDetectorConfig;
    std::string image_path;
    std::string dump_dir;
    std::string stream_dir;
    std::string shm_name = pure_uav::camera::kColorChannel;
    int shm_timeout_ms = 5000;
    long frames = 0;
    double stream_fps = 20.0;
    bool json = false;
    bool check_config = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto requireValue = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "[error] %s 缺少参数值\n", name);
                std::exit(2);
            }
            return argv[++i];
        };

        if (arg == "--config") {
            config_path = requireValue("--config");
        } else if (arg == "--check-config") {
            check_config = true;
        } else if (arg == "--image") {
            image_path = requireValue("--image");
        } else if (arg == "--frames") {
            frames = std::atol(requireValue("--frames").c_str());
            frames = std::max<long>(frames, 0);
        } else if (arg == "--json") {
            json = true;
        } else if (arg == "--dump-dir") {
            dump_dir = requireValue("--dump-dir");
        } else if (arg == "--stream-dir") {
            stream_dir = requireValue("--stream-dir");
        } else if (arg == "--stream-fps") {
            stream_fps = std::atof(requireValue("--stream-fps").c_str());
        } else if (arg == "--shm") {
            shm_name = requireValue("--shm");
        } else if (arg == "--shm-timeout-ms") {
            shm_timeout_ms = std::atoi(requireValue("--shm-timeout-ms").c_str());
        } else if (arg == "--verbose") {
            // 保留以兼容旧脚本；相机日志由 camera_server 统一输出。
        } else if (arg == "--camera") {
            std::fprintf(stderr, "[error] --camera 已移除：分辨率/曝光由 config/camera.yaml 的 camera_server 统一配置\n");
            return 2;
        } else if (arg == "--help" || arg == "-h") {
            usage();
            return 0;
        } else {
            std::fprintf(stderr, "[error] 未知参数: %s\n", arg.c_str());
            usage();
            return 2;
        }
    }

    if (json) {
        // 保留原始 stdout 给 JSON 行，把进程 stdout（fd 1）指向 stderr，
        // 避免 VIPLite/驱动日志混进 JSON。
        std::fflush(stdout);
        const int fd = dup(STDOUT_FILENO);
        if (fd >= 0) {
            FILE* f = fdopen(fd, "w");
            if (f != nullptr) {
                g_json_out = f;
                std::setvbuf(g_json_out, nullptr, _IOLBF, 0);
            } else {
                close(fd);
            }
        }
        dup2(STDERR_FILENO, STDOUT_FILENO);
    }

    std::string config_error;
    if (!pure_uav::detection::loadDetectorConfig(config_path, cfg, config_error)) {
        std::fprintf(stderr, "[error] %s\n", config_error.c_str());
        return 2;
    }
    std::fprintf(stderr, "[detect_rt] %s\n", pure_uav::detection::describeDetectorConfig(cfg).c_str());
    if (check_config) {
        std::fprintf(stderr, "[detect_rt] 配置有效（未加载模型、未连接相机服务）\n");
        return 0;
    }

    if (stream_fps < 1.0 || stream_fps > 60.0) {
        std::fprintf(stderr, "[error] --stream-fps 必须在 [1,60] 内\n");
        return 2;
    }
    if (shm_timeout_ms < 100 || shm_timeout_ms > 120000) {
        std::fprintf(stderr, "[error] --shm-timeout-ms 必须在 100..120000 内\n");
        return 2;
    }
    if (shm_name.empty() || shm_name.front() != '/') {
        std::fprintf(stderr, "[error] --shm 通道名必须以 '/' 开头\n");
        return 2;
    }

    for (const auto& dir : {dump_dir, stream_dir}) {
        if (dir.empty()) {
            continue;
        }
        std::error_code ec;
        std::filesystem::create_directories(dir, ec);
        if (ec) {
            std::fprintf(stderr, "[error] 无法创建输出目录: %s\n", dir.c_str());
            return 2;
        }
    }

    if (!image_path.empty()) {
        return runImage(image_path, cfg, json, dump_dir, stream_dir);
    }
    return runCamera(cfg, shm_name, shm_timeout_ms, frames, json, dump_dir, stream_dir, stream_fps);
}
