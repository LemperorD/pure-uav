# ============================================================
#  机载主循环（app）
#
#  文件布局（2026-11-01 讨论后）：
#    src/guidance/flight/    flight_types.hpp / guidance_loop.{hpp,cpp}
#    src/app/
#      main.cpp              入口
#      app_params.hpp        宿主侧 AppParams
#      startup/              application.{hpp,cpp}（CLI + 参数 + 分发）
#      tools/                tools.{hpp,cpp}（CSV 录制/回放 + 合成世界）
#
#  target：
#    app_tools     地面工具（tools.cpp）—— 可不上机
#    app_startup   启动层（application.cpp）
#    app_guidance  向后兼容聚合名（= app_startup），test_guidance 继续链接它
#    main          可执行入口
# ============================================================

add_library(app_tools STATIC
    tools/tools.cpp
)
target_include_directories(app_tools PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/tools
)
target_link_libraries(app_tools PUBLIC
    guidance_flight
)

add_library(app_startup STATIC
    startup/application.cpp
)
target_include_directories(app_startup PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/startup
)
target_link_libraries(app_startup PUBLIC
    app_tools
    guidance_flight
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
