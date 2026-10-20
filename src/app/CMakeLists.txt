# ============================================================
#  机载主循环（app）
#
#  · 接口骨架：sample.hpp / output.hpp / sensor_source.hpp /
#              command_sink.hpp / guidance_loop.hpp（只有声明与契约注释）
#  · 实现：guidance_loop.cpp（本轮唯一实现，纯我们自己的逻辑）
#
#  ⚠ 依赖方向：app → guidance_flight（= guidance_core + state_adapter + mavlink_units）
#    具体数据源/出口（mavlink_ins_source.cpp / mavlink_sink.cpp）落地后再加到这里，
#    且**只有那些 .cpp** 才 include 通信模块的头文件。
# ============================================================
add_library(app_guidance STATIC
    guidance_loop.cpp
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
