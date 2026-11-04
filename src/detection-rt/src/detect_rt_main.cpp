// detect_rt：实时相机/离线单图检测 CLI；--json 时 stdout 只输出 JSON 行。
// 参数与推流用法见同目录 README。

#include <algorithm>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <csignal>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "pure_uav/detection/detector.hpp"
#include "pure_uav/detection/detection_types.hpp"
#include "pure_uav/detection/yolo_backend.hpp"
#include "yolo_postprocess.hpp"

namespace {

using pure_uav::detection::DetectionBox;
using pure_uav::detection::Detector;
using pure_uav::detection::DetectorConfig;
using pure_uav::detection::DetectorStats;
using pure_uav::detection::FrameResult;
using pure_uav::detection::PostprocessParams;
using pure_uav::detection::YoloBackend;

std::atomic<bool> g_sigint{false};

// --json 时把进程 stdout 重定向到 stderr，JSON 行写原始 stdout 的独立流，
// 这样驱动/VIPLite 自己打印到 stdout 的日志不会污染 JSON。
FILE* g_json_out = stdout;

void onSigint(int) { g_sigint.store(true); }

uint64_t unixNowUs() {
    using namespace std::chrono;
    return static_cast<uint64_t>(
        duration_cast<microseconds>(system_clock::now().time_since_epoch()).count());
}

bool fileExists(const std::string& p) {
    std::error_code ec;
    return std::filesystem::is_regular_file(p, ec);
}

#if defined(PURE_DETECTION_BACKEND_CPU)
constexpr const char* kDefaultModelRel = "assets/models/yolo26n.onnx";
#else
constexpr const char* kDefaultModelRel = "assets/models/yolo26n.nb";
#endif

// 默认模型：cwd 下的 assets/models/<模型>，其次从可执行文件向上找仓库根。
std::string resolveDefaultModel() {
    const std::string rel = kDefaultModelRel;
    if (fileExists(rel)) return rel;

    std::string exe;
    {
        char buf[4096];
        const ssize_t n = readlink("/proc/self/exe", buf, sizeof(buf) - 1);
        if (n > 0) {
            buf[n] = '\0';
            exe = buf;
        }
    }
    if (!exe.empty()) {
        std::filesystem::path dir = std::filesystem::path(exe).parent_path();
        for (int i = 0; i < 6 && !dir.empty(); ++i) {
            const auto candidate = dir / rel;
            if (fileExists(candidate.string())) return candidate.string();
            dir = dir.parent_path();
        }
    }
    return rel;
}

bool parseCamera(const std::string& s, int& w, int& h, int& fps) {
    return std::sscanf(s.c_str(), "%dx%d@%d", &w, &h, &fps) == 3 && w > 0 && h > 0 && fps > 0;
}

void printBoxes(const FrameResult& r, bool json) {
    FILE* out = json ? g_json_out : stdout;
    if (json) {
        std::fprintf(out,
                    "{\"host_us\":%llu,\"seq\":%llu,\"infer_ms\":%.3f,\"present\":%s,"
                    "\"count\":%d,\"boxes\":[",
                    static_cast<unsigned long long>(r.host_us),
                    static_cast<unsigned long long>(r.seq),
                    static_cast<double>(r.infer_ms), r.present ? "true" : "false", r.count);
        for (int i = 0; i < r.count; ++i) {
            const DetectionBox& b = r.boxes[i];
            const double u = 0.5 * (b.x1 + b.x2);
            const double v = 0.5 * (b.y1 + b.y2);
            const double w = b.x2 - b.x1;
            const double h = b.y2 - b.y1;
            std::fprintf(out,
                         "%s{\"class_id\":%d,\"conf\":%.6f,\"x1\":%.3f,\"y1\":%.3f,"
                         "\"x2\":%.3f,\"y2\":%.3f,\"u\":%.3f,\"v\":%.3f,\"w\":%.3f,"
                         "\"h\":%.3f}",
                         i ? "," : "", b.class_id, static_cast<double>(b.conf),
                         static_cast<double>(b.x1), static_cast<double>(b.y1),
                         static_cast<double>(b.x2), static_cast<double>(b.y2), u, v, w, h);
        }
        std::fprintf(out, "]}\n");
        std::fflush(out);
        return;
    }

    for (int i = 0; i < r.count; ++i) {
        const DetectionBox& b = r.boxes[i];
        std::printf("box[%d] class=%d conf=%.4f x1=%.1f y1=%.1f x2=%.1f y2=%.1f "
                    "center=(%.1f,%.1f) size=%.1fx%.1f\n",
                    i, b.class_id, static_cast<double>(b.conf),
                    static_cast<double>(b.x1), static_cast<double>(b.y1),
                    static_cast<double>(b.x2), static_cast<double>(b.y2),
                    0.5 * (b.x1 + b.x2), 0.5 * (b.y1 + b.y2),
                    static_cast<double>(b.x2 - b.x1), static_cast<double>(b.y2 - b.y1));
    }
    if (r.present) {
        std::printf("main target: class=%d conf=%.4f u=%.1f v=%.1f w=%.1f h=%.1f\n",
                    r.class_id, r.conf, r.u_px, r.v_px, r.w_px, r.h_px);
    } else {
        std::printf("main target: present=false\n");
    }
}

cv::Mat drawAnnotated(const cv::Mat& bgr, const FrameResult& r) {
    cv::Mat vis = bgr.clone();
    for (int i = 0; i < r.count; ++i) {
        const DetectionBox& b = r.boxes[i];
        const cv::Scalar color = (b.class_id == 1) ? cv::Scalar(0, 0, 255) : cv::Scalar(255, 0, 0);
        cv::rectangle(vis, cv::Point(static_cast<int>(b.x1), static_cast<int>(b.y1)),
                      cv::Point(static_cast<int>(b.x2), static_cast<int>(b.y2)), color, 2);
        char text[64];
        std::snprintf(text, sizeof(text), "c%d %.2f", b.class_id, static_cast<double>(b.conf));
        cv::putText(vis, text, cv::Point(static_cast<int>(b.x1), static_cast<int>(b.y1) - 4),
                    cv::FONT_HERSHEY_SIMPLEX, 0.5, color, 1);
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
        ofs.write(reinterpret_cast<const char*>(jpg.data()),
                 static_cast<std::streamsize>(jpg.size()));
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

void drawAndSave(const cv::Mat& bgr, const FrameResult& r, const std::string& dump_dir,
                 uint64_t seq) {
    const cv::Mat vis = drawAnnotated(bgr, r);
    char name[128];
    std::snprintf(name, sizeof(name), "detect_%06llu.jpg", static_cast<unsigned long long>(seq));
    const std::string path = (std::filesystem::path(dump_dir) / name).string();
    if (!cv::imwrite(path, vis)) {
        std::fprintf(stderr, "[warn] 保存带框图失败: %s\n", path.c_str());
    }
}

int runImage(const std::string& image_path, const DetectorConfig& cfg, bool json,
             const std::string& dump_dir, const std::string& stream_dir) {
    cv::Mat bgr = cv::imread(image_path, cv::IMREAD_COLOR);
    if (bgr.empty()) {
        std::fprintf(stderr, "[error] 无法读取图片: %s\n", image_path.c_str());
        return 1;
    }

    YoloBackend backend;
    std::string error;
    if (!backend.load(cfg.model_path, cfg.imgsz, error)) {
        std::fprintf(stderr, "[error] 模型加载失败: %s\n", error.c_str());
        return 1;
    }
    std::fprintf(stderr, "[detect_rt] model=%s\n", cfg.model_path.c_str());
    std::fprintf(stderr, "[detect_rt] %s\n", backend.info().c_str());
    std::fprintf(stderr, "[detect_rt] image=%s %dx%d\n", image_path.c_str(), bgr.cols, bgr.rows);

    pure_uav::detection::YoloOutputView view;
    if (!backend.infer(bgr, view, error)) {
        std::fprintf(stderr, "[error] 推理失败: %s\n", error.c_str());
        return 1;
    }

    PostprocessParams pp;
    pp.num_classes = cfg.num_classes;
    pp.conf_thr = cfg.conf_thr;
    pp.iou_thr = cfg.iou_thr;
    pp.max_det = cfg.max_det;
    pp.img_w = bgr.cols;
    pp.img_h = bgr.rows;

    FrameResult r{};
    r.host_us = unixNowUs();
    r.seq = 1;
    r.width = bgr.cols;
    r.height = bgr.rows;
    r.infer_ms = view.infer_ms;
    pure_uav::detection::postprocess(view, pp, r);

    printBoxes(r, json);
    if (!dump_dir.empty()) drawAndSave(bgr, r, dump_dir, r.seq);
    if (!stream_dir.empty()) writeStreamFrame(bgr, r, stream_dir);
    if (!json) {
        std::printf("summary: infer_ms=%.3f count=%d present=%s\n",
                    static_cast<double>(r.infer_ms), r.count, r.present ? "true" : "false");
    }
    return 0;
}

int runCamera(const DetectorConfig& cfg, long frames, bool json, const std::string& dump_dir,
              const std::string& stream_dir, double stream_fps) {
    Detector det;
    std::string error;
    if (!det.start(cfg, error)) {
        std::fprintf(stderr, "[error] 启动失败: %s\n", error.c_str());
        return 1;
    }

    std::fprintf(stderr, "[detect_rt] model=%s\n", cfg.model_path.c_str());
    std::fprintf(stderr, "[detect_rt] %s\n", det.modelInfo().c_str());
    std::fprintf(stderr, "[detect_rt] camera profile=%s\n", det.cameraProfile().c_str());

    long processed = 0;
    uint64_t total_boxes = 0;
    uint64_t present_frames = 0;
    const auto t_start = std::chrono::steady_clock::now();
    auto last_report = t_start;

    // 推流限速：JPEG 编码只在需要时做，默认最多 20 帧/秒。
    const std::chrono::duration<double> stream_period(1.0 / std::max(1.0, stream_fps));
    std::chrono::steady_clock::time_point last_stream{};

    while (!g_sigint.load()) {
        FrameResult r;
        bool got = det.poll(r);
        if (got) {
            ++processed;
            total_boxes += static_cast<uint64_t>(r.count);
            if (r.present) ++present_frames;
            printBoxes(r, json);
            if (!dump_dir.empty()) {
                cv::Mat frame;
                if (det.latest_frame(frame)) drawAndSave(frame, r, dump_dir, r.seq);
            }
            if (!stream_dir.empty()) {
                const auto now = std::chrono::steady_clock::now();
                if (last_stream.time_since_epoch().count() == 0 ||
                    now - last_stream >= stream_period) {
                    cv::Mat frame;
                    if (det.latest_frame(frame)) {
                        writeStreamFrame(frame, r, stream_dir);
                        last_stream = now;
                    }
                }
            }
        }
        if (frames > 0 && processed >= frames) break;

        const auto now = std::chrono::steady_clock::now();
        if (now - last_report >= std::chrono::seconds(1)) {
            const DetectorStats s = det.stats();
            std::fprintf(json ? stderr : stdout,
                         "[stats] capture=%.1f fps process=%.1f fps avg_infer=%.2f ms "
                         "frames=%llu/%llu\n",
                         s.capture_fps, s.process_fps, s.avg_infer_ms,
                         static_cast<unsigned long long>(s.process_frames),
                         static_cast<unsigned long long>(s.capture_frames));
            last_report = now;
        }

        // 没有新结果时短暂让出 CPU；有结果时不额外等待。
        if (!got) std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }

    const DetectorStats s = det.stats();
    det.stop();

    std::fprintf(json ? stderr : stdout,
                 "summary: capture_fps=%.1f process_fps=%.1f avg_infer_ms=%.3f "
                 "capture_frames=%llu process_frames=%llu present_frames=%llu total_boxes=%llu\n",
                 s.capture_fps, s.process_fps, s.avg_infer_ms,
                 static_cast<unsigned long long>(s.capture_frames),
                 static_cast<unsigned long long>(s.process_frames),
                 static_cast<unsigned long long>(present_frames),
                 static_cast<unsigned long long>(total_boxes));
    return 0;
}

void usage() {
    std::fprintf(stderr,
                 "用法: detect_rt [--model PATH] [--conf 0.25] [--iou 0.45]\n"
                 "                [--image FILE] [--frames N] [--json] [--dump-dir DIR]\n"
                 "                [--stream-dir DIR] [--stream-fps FPS]\n"
                 "                [--camera WxH@FPS] [--verbose]\n");
}

}  // namespace

int main(int argc, char** argv) {
    std::signal(SIGINT, onSigint);

    DetectorConfig cfg;
    std::string model;
    std::string image_path;
    std::string dump_dir;
    std::string stream_dir;
    std::string camera_arg;
    long frames = 0;
    double stream_fps = 20.0;
    bool json = false;
    bool verbose = false;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        const auto requireValue = [&](const char* name) -> std::string {
            if (i + 1 >= argc) {
                std::fprintf(stderr, "[error] %s 缺少参数值\n", name);
                std::exit(2);
            }
            return argv[++i];
        };

        if (arg == "--model") {
            model = requireValue("--model");
        } else if (arg == "--conf") {
            cfg.conf_thr = static_cast<float>(std::atof(requireValue("--conf").c_str()));
        } else if (arg == "--iou") {
            cfg.iou_thr = static_cast<float>(std::atof(requireValue("--iou").c_str()));
        } else if (arg == "--image") {
            image_path = requireValue("--image");
        } else if (arg == "--frames") {
            frames = std::atol(requireValue("--frames").c_str());
            if (frames < 0) frames = 0;
        } else if (arg == "--json") {
            json = true;
        } else if (arg == "--dump-dir") {
            dump_dir = requireValue("--dump-dir");
        } else if (arg == "--stream-dir") {
            stream_dir = requireValue("--stream-dir");
        } else if (arg == "--stream-fps") {
            stream_fps = std::atof(requireValue("--stream-fps").c_str());
        } else if (arg == "--camera") {
            camera_arg = requireValue("--camera");
        } else if (arg == "--verbose") {
            verbose = true;
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
            if (f) {
                g_json_out = f;
                std::setvbuf(g_json_out, nullptr, _IOLBF, 0);
            } else {
                close(fd);
            }
        }
        dup2(STDERR_FILENO, STDOUT_FILENO);
    }

    if (cfg.conf_thr <= 0.0f || cfg.conf_thr > 1.0f || cfg.iou_thr <= 0.0f || cfg.iou_thr > 1.0f) {
        std::fprintf(stderr, "[error] --conf/--iou 必须在 (0,1] 内\n");
        return 2;
    }
    if (stream_fps < 1.0 || stream_fps > 60.0) {
        std::fprintf(stderr, "[error] --stream-fps 必须在 [1,60] 内\n");
        return 2;
    }

    if (!camera_arg.empty()) {
        int w = 0, h = 0, fps = 0;
        if (!parseCamera(camera_arg, w, h, fps)) {
            std::fprintf(stderr, "[error] --camera 需要 WxH@FPS 形式，收到: %s\n",
                         camera_arg.c_str());
            return 2;
        }
        cfg.camera.width = w;
        cfg.camera.height = h;
        cfg.camera.fps = fps;
    }

    cfg.model_path = model.empty() ? resolveDefaultModel() : model;
    if (cfg.model_path.empty()) {
        std::fprintf(stderr, "[error] 无法解析默认模型路径\n");
        return 2;
    }
    cfg.camera.verbose = verbose;
    cfg.verbose = verbose;

    for (const auto& dir : {dump_dir, stream_dir}) {
        if (dir.empty()) continue;
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
    return runCamera(cfg, frames, json, dump_dir, stream_dir, stream_fps);
}
