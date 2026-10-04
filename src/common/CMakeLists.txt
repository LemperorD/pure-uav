add_library(pure_math_tools SHARED
    common_libs/src/pure_math_tools.cpp
)
target_include_directories(pure_math_tools PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/common_libs/include
)