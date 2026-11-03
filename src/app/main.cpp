// =============================================================================
//  main.cpp —— 入口（只负责启动 GuidanceApplication）
//
//  所有参数解析、配置加载、模式分发都在 guidance_application.cpp；
//  本文件保持 5 行，方便以后换 GUI / 换启动方式。
// =============================================================================

#include "startup/guidance_application.hpp"

int main(int argc, char **argv) {
    pure_uav::app::GuidanceApplication app;
    return app.run(argc, argv);
}
