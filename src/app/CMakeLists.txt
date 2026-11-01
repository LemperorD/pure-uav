# ============================================================
#  机载主循环（app）—— 方案 A 合并后
#
#  文件：
#    app_types.hpp      类型 / 契约 / CameraModel / LoopConfig / AppParams
#    guidance_loop.*    主循环 + 内存黑匣子
#    tools.*            CSV 录制/回放 + 合成世界 + run_synthetic/run_replay
#    application.*      CLI + 参数加载/校验 + 模式分发
#    main.cpp           入口（只调 Application）
#
#  target：
#    app_core     机载运行时（guidance_loop.cpp）
#    app_tools    地面工具（tools.cpp）—— 可不上机
#    app_startup  启动层（application.cpp）
#    app_guidance 向后兼容的聚合名（= app_startup），test_guidance 继续链接它
# ============================================================

add_library(app_core STATIC
    guidance_loop.cpp
)
target_include_directories(app_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(app_core
    guidance_flight
)

add_library(app_tools STATIC
    tools.cpp
)
target_link_libraries(app_tools
    app_core
)

add_library(app_startup STATIC
    application.cpp
)
target_link_libraries(app_startup
    app_core
    app_tools
)

# 向后兼容：旧 CMake / 测试里写 app_guidance 的地方不用改。
add_library(app_guidance INTERFACE)
target_link_libraries(app_guidance INTERFACE
    app_startup
)

add_executable(main
    main.cpp
)
target_link_libraries(main PRIVATE
    app_startup
)

# 可执行文件统一放到 <仓库>/bin（与 test/CMakeLists.txt 的 TEST_OUTPUT_DIR 一致），
# 这样文档里的 `./bin/main` 在 Windows 与 WSL 上都成立。
set_target_properties(main PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
)
