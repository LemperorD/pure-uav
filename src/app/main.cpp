#include <cstdio>
#include <exception>

#include "startup/app_options.hpp"

#ifdef PURE_APP_HAS_VIO
#include "abstract_factory.hpp"
#include "runtime/uav_application.hpp"
#endif

using namespace pure_uav::app;

int main(int argc, char** argv) {
    AppOptions options;
    try {
        options = parseAppOptions(argc, argv);
    } catch (const std::exception& error) {
        std::fprintf(stderr, "%s\n", error.what());
        return 2;
    }
    if (options.help) {
        printAppUsage(argv[0]);
        return 0;
    }
#ifdef PURE_APP_HAS_VIO
    try {
        UavApplication app(options);
        auto factory = app.prepare();
        if (!factory) {
            return app.result();
        }

        auto vio = factory->createVIOFactory()->createVIO();
        auto guidance = options.mission ? factory->createGuidanceFactory()->createGuidance() : nullptr;
        auto detector = options.no_detection ? nullptr : factory->createDetectorFactory()->createDetector();

        app.init(vio, guidance, detector);
        return app.run();
    } catch (const std::exception& error) {
        std::fprintf(stderr, "[MAIN] 异常退出：%s\n", error.what());
        return 1;
    }
#else
    std::fprintf(stderr, "[MAIN] 当前构建未启用 VIO：缺少实时 VIO 依赖（pure_vio_runtime 目标）。\n");
    return 1;
#endif
}
