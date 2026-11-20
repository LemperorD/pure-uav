add_library(
  factory_core STATIC
  product.cpp ${CMAKE_CURRENT_SOURCE_DIR}/../app/runtime/vio_pose_publisher.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../app/startup/preflight_config.cpp
)
target_include_directories(factory_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(factory_core PUBLIC app_support guidance_application)
if(TARGET detect_rt)
  target_compile_definitions(factory_core PRIVATE PURE_APP_HAS_DETECTION=1)
endif()

add_executable(
  test_factory_core ${CMAKE_SOURCE_DIR}/src/test/test_app/test_factory_core.cpp
)
target_link_libraries(test_factory_core PRIVATE factory_core)
add_test(NAME factory_core COMMAND test_factory_core)
set_tests_properties(factory_core PROPERTIES TIMEOUT 15)
