// =============================================================================
//  guidance_example.cpp —— 制导调用示例（独立可执行文件）
//
//  不占用 src/app/main.cpp 的 main()；main.cpp 负责机载并发运行。
// =============================================================================

#include "startup/guidance_application.hpp"

using pure_uav::app::GuidanceApplication;

int main(int argc, char **argv) {
    GuidanceApplication app;
    return app.run(argc, argv);
}
