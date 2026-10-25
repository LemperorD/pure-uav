# ============================================================
#  机载主循环（app）
#
#  · 契约/骨架：sample.hpp / output.hpp / sensor_source.hpp / command_sink.hpp
#  · 主循环：    guidance_loop.{hpp,cpp}   （一拍做什么，纯我们自己的逻辑）
#  · 合成世界：  synthetic_world.{hpp,cpp} （闭环演示 + test_guidance §17 回归共用）
#  · 离线 IO：   csv_io.{hpp,cpp}          （录制/回放/落盘/空实现）
#  · 薄壳：      main.cpp                  （只做命令行解析与打印）
#
#  ⚠ 依赖方向：app → guidance_flight（= guidance_core + state_adapter + mavlink_units）
#    具体数据源/出口（mavlink_ins_source.cpp / mavlink_sink.cpp）落地后再加到这里，
#    且**只有那些 .cpp** 才 include 通信模块的头文件。
# ============================================================
add_library(app_guidance STATIC
    guidance_loop.cpp
    synthetic_world.cpp
    csv_io.cpp
    params.cpp
    ring_log.cpp
)
target_include_directories(app_guidance PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(app_guidance
    guidance_flight
)

add_executable(main
    main.cpp
)
target_link_libraries(main PRIVATE
    app_guidance
)

# 可执行文件统一放到 <仓库>/bin（与 test/CMakeLists.txt 的 TEST_OUTPUT_DIR 一致），
# 这样文档里的 `./bin/main` 在 Windows 与 WSL 上都成立。
set_target_properties(main PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
)
