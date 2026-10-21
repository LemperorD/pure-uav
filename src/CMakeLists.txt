add_subdirectory(common)      # pure_math_tools / pure_vec3 / mavlink_units
add_subdirectory(driver)
add_subdirectory(guidance)    # guidance_core / guidance_sim / state_adapter / guidance_flight
# app 放 guidance 之后：机载主循环（app_guidance）链接 guidance_flight，
# CMake 的 target_link_libraries 要求被依赖的 target **先定义**。
add_subdirectory(app)
# test 放最后：test_math 依赖 common、test_guidance 依赖 guidance、
# test_state_adapter 依赖 guidance 的 state_adapter —— 确保被依赖的 target 先定义。
#
# ⚠ 顺序有意义（2026-10-20 起）：
#   common   必须先于 guidance（guidance_core 链接 pure_vec3、state_adapter 链接 mavlink_units）；
#   guidance 必须先于 app（app_guidance 链接 guidance_flight）；
#   app      必须先于 test（将来的 app 级测试要链接 app_guidance）。
#
# 2026-10-20 目录调整：原 `src/interface/` 已撤销 ——
#   · mavlink_units / vec3.hpp      → src/common/common_libs/（通用件）
#   · state_adapter / state_check   → src/guidance/adapter/（制导的输入边界）
add_subdirectory(test)
