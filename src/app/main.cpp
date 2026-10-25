// =============================================================================
//  pure-uav 机载主程序（制导）
//
//  【定位】**薄壳**：只做命令行解析 + 打印。真正的活儿都在库里：
//      合成世界/闭环驱动  → synthetic_world.{hpp,cpp}（闭环演示 + 回归测试共用）
//      CSV 录制/回放        → csv_io.{hpp,cpp}（黑匣子 / 离线复盘）
//      一拍做什么          → guidance_loop.cpp（取数→反投影→制导核→发指令）
//
//  【三种跑法】
//    ① 合成世界闭环（默认，零依赖）：
//         ./bin/main --law=0 --n=4 --a-max=10 --ticks=300
//    ② 边跑边录（黑匣子；录出来的 CSV 可以用 ③ 复盘）：
//         ./bin/main --record=flight.csv --out=cmd.csv
//    ③ 回放录制（不跑合成世界，只用录下来的输入重算一遍）：
//         ./bin/main --replay=flight.csv --out=replay_cmd.csv
//    ④ 参数文件（默认值 < YAML < 命令行）：
//         ./bin/main --config=guidance.yaml --print-params
//
//  【为什么现在就用合成世界而不是真机】
//    真机闭环缺三样外部东西（检测接口 / VIO 运行时 / 通信合并），都不在本模块内。
//    合成世界把"数据通路"先钉死：除"真值→像素"这一步是模拟的，其余全是真实代码。
//    ⇒ 等三个依赖就位后，把 SyntheticWorld 换成
//      MavlinkInsSource + VisionTargetSource + MavlinkAccelSink，
//      主程序这段几乎不用改（这也是为什么接口先定、实现后补）。
// =============================================================================

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <thread>

#include "csv_io.hpp"
#include "guidance_loop.hpp"
#include "params.hpp"
#include "synthetic_world.hpp"

using namespace pure_uav;
using namespace pure_uav::app;

namespace {

// ---------------------------------------------------------------- 极简参数解析
struct Args {
    int    hz    = 50;
    int    law   = 0;        // 0=PPN 1=TPN 2=APN
    double N     = 4.0;      // 导航比（工程 3~5）
    double a_max = 10.0;     // 过载上限（x500 量级）
    int    ticks = 300;      // 总拍数（50 Hz × 6 s）
    bool   verbose = true;
    bool   realtime = false; // 按真实时间节拍（默认全速跑，便于快速迭代）
    const char *record = nullptr;   // 输入录制 CSV
    const char *out    = nullptr;   // 指令落盘 CSV
    const char *replay = nullptr;   // 输入回放 CSV
    const char *config = nullptr;   // YAML 参数文件（默认值 < 文件 < 命令行）
    bool   print_params = false;    // 打印最终参数后退出
    // ★ 只有**显式给出**的命令行项才覆盖 YAML（否则会把文件里的值冲掉）
    bool   law_given = false;
    bool   n_given = false;
    bool   a_max_given = false;
    bool   hz_given = false;
};

bool parse_int(const char *s, int &out) {
    char *end = nullptr;
    const long v = std::strtol(s, &end, 10);
    if (!end || *end != '\0' || v < 0) return false;
    out = static_cast<int>(v);
    return true;
}

bool parse_double(const char *s, double &out) {
    char *end = nullptr;
    const double v = std::strtod(s, &end);
    if (!end || *end != '\0') return false;
    out = v;
    return true;
}

const char *status_name(LoopStatus s) {
    switch (s) {
        case LoopStatus::kOk:             return "kOk";
        case LoopStatus::kNotInitialized: return "kNotInitialized";
        case LoopStatus::kStopped:        return "kStopped";
        case LoopStatus::kSensorTimeout:  return "kSensorTimeout";
        case LoopStatus::kUnhealthyInput: return "kUnhealthyInput";
        case LoopStatus::kSinkError:      return "kSinkError";
        case LoopStatus::kEstimatorReset: return "kEstimatorReset";
    }
    return "?";
}

const char *mode_name(PureGuidanceMode m) {
    switch (m) {
        case PureGuidanceMode::PN:     return "PN  ";
        case PureGuidanceMode::APN:    return "APN ";
        case PureGuidanceMode::HOLD:   return "HOLD";
        case PureGuidanceMode::SEARCH: return "SECH";
    }
    return "?   ";
}

void usage(const char *prog) {
    std::printf(
        "用法: %s [选项]\n"
        "  --hz=N        主循环频率（默认 50）\n"
        "  --law=N       0=PPN 1=TPN 2=APN（默认 0）\n"
        "  --n=X         导航比（默认 4）\n"
        "  --a-max=X     过载上限 m/s²（默认 10）\n"
        "  --ticks=N     总拍数（默认 300）\n"
        "  --realtime    按真实节拍跑（默认全速）\n"
        "  --quiet       只打汇总\n"
        "  --record=F    把每拍输入录成 CSV（合成世界里也能录，便于试格式）\n"
        "  --out=F       把每拍指令落盘成 CSV\n"
        "  --replay=F    回放录制好的 CSV（不跑合成世界）\n"
        "  --config=F    YAML 参数文件（默认值 < 文件 < 命令行）\n"
        "  --print-params 打印最终参数后退出\n"
        "  --help        本帮助\n",
        prog);
}

GuidanceLaw law_from_int(int v) {
    return (v == 1) ? GuidanceLaw::TPN : (v == 2) ? GuidanceLaw::APN : GuidanceLaw::PPN;
}

const char *law_name(GuidanceLaw law) {
    switch (law) {
        case GuidanceLaw::PPN: return "PPN";
        case GuidanceLaw::TPN: return "TPN";
        case GuidanceLaw::APN: return "APN";
    }
    return "?";
}

// ---------------------------------------------------------------- 打点打印
struct Lat3 {
    unsigned long long p50 = 0;
    unsigned long long p95 = 0;
    unsigned long long max = 0;
};

Lat3 lat3(const LatencyStats &l, LatencyStats::Stage s) {
    Lat3 v;
    v.p50 = static_cast<unsigned long long>(l.percentile_ns(s, 0.50));
    v.p95 = static_cast<unsigned long long>(l.percentile_ns(s, 0.95));
    v.max = static_cast<unsigned long long>(l.max_ns(s));
    return v;
}

// ---------------------------------------------------------------- 打印
struct PrintCtx {
    int    every = 5;      // 每几拍打一行
    double dt    = 0.02;
};

void on_tick_print(void *user, int k, const SyntheticWorld &world, const GuidanceOutput &out) {
    const PrintCtx *ctx = static_cast<const PrintCtx *>(user);
    if (!ctx || ctx->every <= 0 || (k % ctx->every) != 0) return;
    std::printf("  %4d  %5.2f  %5.2f  %6.2f   %s  %6.3f   (%6.2f,%6.2f,%6.2f)  %s\n",
                k, k * ctx->dt, world.true_range(), out.Vc, mode_name(out.mode), out.t_go,
                out.a_cmd_ned.x, out.a_cmd_ned.y, out.a_cmd_ned.z, out.sat ? "Y" : ".");
}

void print_banner(const Args &a, const AppParams &ap) {
    std::printf("=== pure-uav 制导主程序（合成世界，闭环）===\n");
    std::printf("  律=%s  N=%.1f  a_max=%.1f m/s²  loop=%d Hz  拍数=%d\n",
                law_name(ap.guidance.law), ap.guidance.N, ap.guidance.a_max,
                ap.loop.loop_hz, a.ticks);
    std::printf("  起点 (0,0,-5) NED，初速 (8,0,0)，靶 (40,3,-5)\n");
    std::printf("  ⚠ 理想点质量积分（无姿态环/重力），目的在验证制导链路\n");
    if (!a.realtime) {
        std::printf("  ⚠ 全速模式：降级计时按**墙钟**计（hold_last_max_s / sensor_timeout），\n");
        std::printf("     全速下不会走完 ⇒ 只能看链路是否跑通；要看降级行为请加 --realtime\n");
    }
    std::printf("\n");
    std::printf("   拍   t(s)   R(m)   Vc(m/s)  mode  t_go(s)  a_ned=(x, y, z)      sat\n");
}

// ============================================================================
//  ① 合成世界闭环（可顺带录制）
// ============================================================================
int run_synthetic(const Args &a, const AppParams &ap) {
    const double dt = 1.0 / static_cast<double>(ap.loop.loop_hz);

    SceneConfig sc;
    sc.dt = dt;

    SyntheticWorld world(demo_camera(), sc);

    // ---- 录制（可选）：装饰器包在数据源外面，主循环代码一行不改 ----
    InputRecorder rec;
    const bool want_rec = (a.record != nullptr);
    if (want_rec && !rec.open(a.record)) {
        std::printf("!! 打不开录制文件：%s\n", a.record);
        return 1;
    }
    RecordingInsSource    rins(world, rec);
    RecordingTargetSource rtgt(world, rec);
    IInsSource    &ins = want_rec ? static_cast<IInsSource &>(rins)
                                  : static_cast<IInsSource &>(world);
    ITargetSource &tgt = want_rec ? static_cast<ITargetSource &>(rtgt)
                                  : static_cast<ITargetSource &>(world);

    // ---- 指令落盘（可选）：一声两处（执行器 + 文件）----
    CsvAccelSink csv_sink(a.out);
    if (a.out && !csv_sink.ok()) {
        std::printf("!! 打不开输出文件：%s\n", a.out);
        return 1;
    }
    TeeSink tee(world, csv_sink);
    ICommandSink &sink = a.out ? static_cast<ICommandSink &>(tee)
                               : static_cast<ICommandSink &>(world);

    GuidanceLoop loop(ap.guidance, ap.loop, ins, tgt, sink);
    if (!loop.init()) {
        std::printf("GuidanceLoop 初始化失败（检查相机模型 / 参数）\n");
        return 1;
    }

    print_banner(a, ap);
    if (want_rec) std::printf("  录制 → %s\n", a.record);
    if (a.out)    std::printf("  落盘 → %s\n", a.out);
    if (want_rec || a.out) std::printf("\n");

    PrintCtx ctx;
    ctx.every = a.verbose ? 5 : 0;
    ctx.dt    = dt;

    const ClosedLoopReport rep = run_synthetic_closed_loop(
        world, loop, a.ticks, a.verbose ? on_tick_print : nullptr, &ctx);

    // ---- 汇总 ----
    const LoopStats &s = loop.stats();
    std::printf("\n=== 汇总 ===\n");
    if (rep.stop != LoopStatus::kOk) {
        std::printf("  终止：%s%s\n", status_name(rep.stop),
                    rep.target_lost ? "（目标离开视野，演示里属正常）" : "");
    }
    std::printf("  拍数 ticks=%llu  ins=%llu  tgt=%llu  sent=%llu  sink_err=%llu\n",
                (unsigned long long)s.ticks, (unsigned long long)s.ins_frames,
                (unsigned long long)s.tgt_frames, (unsigned long long)s.sent,
                (unsigned long long)s.sink_errors);
    std::printf("  饱和 sat=%d  冻结 hold=%d  丢失 search=%d  降级 unhealthy=%llu  safe_hover=%llu\n",
                world.sat(), world.hold(), world.search(),
                (unsigned long long)s.unhealthy, (unsigned long long)s.safe_hover);
    std::printf("  最近距离 min R = %.3f m（第 %d 拍）  末位置 (%.2f, %.2f, %.2f)\n",
                rep.min_r, rep.min_r_tick, world.pos().x, world.pos().y, world.pos().z);
    std::printf("  末速度 (%.2f, %.2f, %.2f)  max_latency=%llu us\n", world.vel().x,
                world.vel().y, world.vel().z, (unsigned long long)s.max_latency_us);
    if (s.latency.count() > 0) {
        const Lat3 poll = lat3(s.latency, LatencyStats::Stage::kPoll);
        const Lat3 proj = lat3(s.latency, LatencyStats::Stage::kProject);
        const Lat3 guid = lat3(s.latency, LatencyStats::Stage::kGuidance);
        const Lat3 send = lat3(s.latency, LatencyStats::Stage::kSend);
        const Lat3 tot  = lat3(s.latency, LatencyStats::Stage::kTotal);
        std::printf("  打点 P50/P95/max (ns)  取数 %llu/%llu/%llu  反投影 %llu/%llu/%llu\n",
                    poll.p50, poll.p95, poll.max, proj.p50, proj.p95, proj.max);
        std::printf("                        制导 %llu/%llu/%llu  发送 %llu/%llu/%llu"
                    "  总 %llu/%llu/%llu\n",
                    guid.p50, guid.p95, guid.max, send.p50, send.p95, send.max,
                    tot.p50, tot.p95, tot.max);
    }
    if (want_rec) std::printf("  录制行数 %llu（%s）\n",
                              (unsigned long long)rec.rows(), rec.ok() ? "ok" : "写失败");
    if (a.out) std::printf("  落盘行数 %llu（%s）\n",
                           (unsigned long long)csv_sink.rows(),
                           csv_sink.ok() ? "ok" : "写失败");

    if (rep.min_r < 1.0)
        std::printf("  ⇒ 链路闭环成立：PN 把最近距离压到了 1 m 以内 ✅\n");
    else
        std::printf("  ⇒ 最近距离仍 > 1 m：检查初速方向 / a_max / N\n");

    // kSensorTimeout 在演示里是正常现象 ⇒ 退出码 0；其余异常码如实返回
    return rep.target_lost ? 0 : (rep.stop == LoopStatus::kOk ? 0 : 1);
}

// ============================================================================
//  ③ 回放
// ============================================================================
int run_replay(const Args &a, const AppParams &ap) {
    CsvReader reader;
    if (!reader.open(a.replay)) {
        std::printf("!! 打不开回放文件：%s\n", a.replay);
        return 1;
    }

    CsvReplaySource src(reader);
    CsvAccelSink csv_sink(a.out);
    if (a.out && !csv_sink.ok()) {
        std::printf("!! 打不开输出文件：%s\n", a.out);
        return 1;
    }
    NullSink null_sink;
    ICommandSink &sink = a.out ? static_cast<ICommandSink &>(csv_sink)
                               : static_cast<ICommandSink &>(null_sink);

    GuidanceLoop loop(ap.guidance, ap.loop, src, src, sink);
    if (!loop.init()) {
        std::printf("GuidanceLoop 初始化失败\n");
        return 1;
    }

    std::printf("=== 回放 %s ===\n", a.replay);
    std::printf("  ⚠ 回放全速跑：超时/保持窗口用的是本机时钟，几乎不前进 ⇒\n");
    std::printf("     这里只验「算得对不对」，降级时机要用 --realtime 或单测 §16\n");
    std::printf("\n");

    const int ticks = (a.ticks > 0) ? a.ticks : 1000000;
    int k = 0;
    LoopStatus stop = LoopStatus::kOk;
    for (; k < ticks; ++k) {
        const LoopStatus st = loop.run_once(nullptr);
        if (st != LoopStatus::kOk) {
            stop = st;
            break;
        }
        if (src.done()) break;
        if (a.realtime) {
            std::this_thread::sleep_for(std::chrono::nanoseconds(
                static_cast<long long>(1e9 / static_cast<double>(ap.loop.loop_hz))));
        }
    }

    const LoopStats &s = loop.stats();
    std::printf("\n=== 汇总 ===\n");
    if (src.error()) std::printf("  !! CSV 解析问题：%s\n", src.error());
    std::printf("  回放行数 %llu  循环拍数 %llu  ins=%llu  tgt=%llu  sent=%llu\n",
                (unsigned long long)src.rows(), (unsigned long long)s.ticks,
                (unsigned long long)s.ins_frames, (unsigned long long)s.tgt_frames,
                (unsigned long long)s.sent);
    std::printf("  饱和 %llu  冻结 %llu  丢失 %llu  降级 %llu  末状态 %s\n",
                (unsigned long long)s.saturated, (unsigned long long)s.hold,
                (unsigned long long)s.search, (unsigned long long)s.unhealthy,
                status_name(stop));
    if (a.out) std::printf("  落盘行数 %llu（%s）\n",
                           (unsigned long long)csv_sink.rows(),
                           csv_sink.ok() ? "ok" : "写失败");

    return (stop == LoopStatus::kOk || src.error() == nullptr) ? 0 : 1;
}

}  // namespace

// ============================================================================
int main(int argc, char **argv) {
    Args a;
    for (int i = 1; i < argc; ++i) {
        const char *s = argv[i];
        auto eq = [s](const char *key) { return std::strncmp(s, key, std::strlen(key)) == 0; };
        if (eq("--hz=")) {
            if (!parse_int(s + 5, a.hz)) { std::printf("!! --hz 解析失败：%s\n", s); return 2; }
            a.hz_given = true;
        } else if (eq("--law=")) {
            if (!parse_int(s + 6, a.law)) { std::printf("!! --law 解析失败：%s\n", s); return 2; }
            a.law_given = true;
        } else if (eq("--n=")) {
            if (!parse_double(s + 4, a.N)) { std::printf("!! --n 解析失败：%s\n", s); return 2; }
            a.n_given = true;
        } else if (eq("--a-max=")) {
            if (!parse_double(s + 8, a.a_max)) {
                std::printf("!! --a-max 解析失败：%s\n", s);
                return 2;
            }
            a.a_max_given = true;
        } else if (eq("--ticks=")) {
            if (!parse_int(s + 8, a.ticks)) { std::printf("!! --ticks 解析失败：%s\n", s); return 2; }
        } else if (eq("--record=")) {
            a.record = s + 9;
        } else if (eq("--out=")) {
            a.out = s + 6;
        } else if (eq("--replay=")) {
            a.replay = s + 9;
        } else if (eq("--config=")) {
            a.config = s + 9;
        } else if (std::strcmp(s, "--print-params") == 0) {
            a.print_params = true;
        } else if (std::strcmp(s, "--realtime") == 0) {
            a.realtime = true;
        } else if (std::strcmp(s, "--quiet") == 0) {
            a.verbose = false;
        } else if (std::strcmp(s, "--help") == 0 || std::strcmp(s, "-h") == 0) {
            usage(argv[0]);
            return 0;
        } else {
            std::printf("未知参数：%s（--help 看用法）\n", s);
            return 2;
        }
    }

    if (a.ticks <= 0) {
        std::printf("参数非法：--ticks 必须 > 0\n");
        return 2;
    }
    if (a.law_given && (a.law < 0 || a.law > 2)) {
        std::printf("参数非法：--law 只能是 0/1/2\n");
        return 2;
    }

    // ★ 默认值 → YAML → 命令行（只覆盖显式给出的项）
    AppParams ap = default_params();
    if (a.config != nullptr) {
        const ParamResult r = load_yaml_file(a.config, &ap);
        if (!r.ok) {
            std::printf("!! 配置加载失败：%s\n", r.error.c_str());
            return 2;
        }
    }
    if (a.law_given) ap.guidance.law = law_from_int(a.law);
    if (a.n_given) ap.guidance.N = a.N;
    if (a.a_max_given) ap.guidance.a_max = a.a_max;
    if (a.hz_given) ap.loop.loop_hz = a.hz;

    const ParamResult vr = validate(ap);
    if (!vr.ok) {
        std::printf("!! 参数校验失败：%s\n", vr.error.c_str());
        return 2;
    }

    // 合成世界 demo：配置里没给相机时回退演示相机。
    // ⚠ 真机必须在 YAML 里给 cam.fx/fy/cx/cy/R_body_cam（否则这里会用假内参）。
    if (!ap.loop.cam.valid()) ap.loop.cam = demo_camera();

    if (a.print_params) {
        std::printf("%s", to_string(ap).c_str());
        return 0;
    }

    return a.replay ? run_replay(a, ap) : run_synthetic(a, ap);
}
