# 在宿主完成检测、VIO、制导模块注册后 add_subdirectory 本目录。
foreach(dependency detection_rt pure_vio_runtime guidance_flight)
  if(NOT TARGET ${dependency})
    message(FATAL_ERROR "factory_core requires target ${dependency}")
  endif()
endforeach()

add_library(factory_core STATIC product.cpp)
target_compile_features(factory_core PUBLIC cxx_std_17)
target_include_directories(factory_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_include_directories(
  factory_core PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/../detection-rt/src)
target_link_libraries(factory_core PUBLIC detection_rt pure_vio_runtime
                                          guidance_flight)

option(PURE_FACTORY_BUILD_TESTS "Build factory lifecycle regression tests" OFF)
if(PURE_FACTORY_BUILD_TESTS)
  add_executable(test_factory_core tests/test_factory_core.cpp)
  target_link_libraries(test_factory_core PRIVATE factory_core)
  add_test(
    NAME factory_core
    COMMAND
      test_factory_core
      ${CMAKE_CURRENT_SOURCE_DIR}/../vio/estimator/config/gemini_1920x1080)
  set_tests_properties(factory_core PROPERTIES TIMEOUT 30)
endif()
