add_subdirectory(common)
add_subdirectory(driver)
add_subdirectory(guidance)
add_subdirectory(vio/estimator)
add_subdirectory(camera)
add_subdirectory(detection-rt)

add_subdirectory(app)

add_subdirectory(test)

# 独立样例注册为 OBJECT 库，仅为 clang-tidy 提供编译参数，不参与编译。 删除之后 clang-tidy 会报错。
find_package(OpenCV 4 REQUIRED)
add_library(
  det_tooling OBJECT
  EXCLUDE_FROM_ALL
  test/test_app/test_detection_runtime.cpp
  test/test_app/test_input_quantization.cpp
  test/test_app/test_target_bridge.cpp
  detection-rt/src/detection_runtime.cpp
  detection-rt/src/input_quantization.cpp
  detection-rt/src/npu_backend.cpp
)
target_include_directories(
  det_tooling
  PRIVATE ${CMAKE_SOURCE_DIR}/src/detection-rt/include
          ${CMAKE_SOURCE_DIR}/src/detection-rt/src
          ${CMAKE_SOURCE_DIR}/src/common
          ${CMAKE_SOURCE_DIR}/src/guidance/flight
          ${CMAKE_SOURCE_DIR}/src/guidance/guidance_libs/include
          ${CMAKE_SOURCE_DIR}/src/common/common_libs/include
          ${OpenCV_INCLUDE_DIRS}
)
target_compile_definitions(det_tooling PRIVATE PURE_DETECTION_BACKEND_CPU=1)
