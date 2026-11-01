// =============================================================================
//  main.cpp —— 入口（只负责启动 Application）
//
//  所有参数解析、配置加载、模式分发都在 application.cpp；
//  本文件保持 5 行，方便以后换 GUI / 换启动方式。
// =============================================================================

#include "application.hpp"

int main(int argc, char **argv) {
    pure_uav::app::Application app;
    return app.run(argc, argv);
}
