add_subdirectory(app)
add_subdirectory(common)      # pure_math_tools / pure_vec3 / mavlink_units
add_subdirectory(driver)
add_subdirectory(guidance)    # guidance_core / guidance_sim / state_adapter / guidance_flight
# test 放最后：test_math 依赖 common、test_guidance 依赖 guidance、
# test_state_adapter 依赖 guidance 的 state_adapter —— 确保被依赖的 target 先定义。
#
# ⚠ 顺序有意义（2026-10-20 起）：
#   common 必须先于 guidance（guidance_core 链接 pure_vec3、state_adapter 链接 mavlink_units）；
#   guidance 必须先于 test。
#
# 2026-10-20 目录调整：原 `src/interface/` 已撤销 ——
#   · mavlink_units / vec3.hpp      → src/common/common_libs/（通用件）
#   · state_adapter / state_check   → src/guidance/adapter/（制导的输入边界）
add_subdirectory(test)
