add_subdirectory(app)
add_subdirectory(common)
add_subdirectory(driver)
add_subdirectory(guidance)
add_subdirectory(interface)
# test 放最后：test_math 依赖 common、test_guidance 依赖 guidance、
# test_mavlink_units 依赖 interface，确保被依赖的 target 先定义
add_subdirectory(test)
