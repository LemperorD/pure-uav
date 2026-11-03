# ============================================================
#  宿主应用与入口（app）
#
#  文件布局：
#    src/guidance/flight/    flight_types.hpp / guidance_loop.{hpp,cpp}
#    src/app/
#      main.cpp                入口（全局 main）
#      guidance_app_params.hpp 宿主侧整机配置
#      guidance_application.{hpp,cpp}  CLI + 参数 + 分发
#      guidance_sim_tools.{hpp,cpp}    CSV 录制/回放 + 合成世界
#
#  target：
#    guidance_sim_tools  地面工具（可不上机）
#    guidance_application 宿主启动层
#    main                可执行入口（全局 main）
# ============================================================

add_library(guidance_sim_tools STATIC
    guidance_sim_tools.cpp
)
target_include_directories(guidance_sim_tools PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(guidance_sim_tools PUBLIC
    guidance_flight
)

add_library(guidance_application STATIC
    guidance_application.cpp
)
target_include_directories(guidance_application PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(guidance_application PUBLIC
    guidance_sim_tools
    guidance_flight
)

add_executable(main
    main.cpp
)
target_link_libraries(main PRIVATE
    guidance_application
)

# 可执行文件统一放到 <仓库>/bin（与 test/CMakeLists.txt 的 TEST_OUTPUT_DIR 一致），
# 这样文档里的 `./bin/main` 在 Windows 与 WSL 上都成立。
set_target_properties(main PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
)
