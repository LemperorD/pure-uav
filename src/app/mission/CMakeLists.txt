# 任务核心只依赖消息/采样类型，无 ROS、串口或推理运行时依赖。
add_library(mission_core STATIC mission_controller.cpp)
target_include_directories(
  mission_core
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..
         ${CMAKE_SOURCE_DIR}/src/common
         ${CMAKE_SOURCE_DIR}/src/common/common_libs/include
         ${CMAKE_SOURCE_DIR}/src/guidance/flight
         ${CMAKE_SOURCE_DIR}/src/guidance/guidance_libs/include
)

find_package(yaml-cpp REQUIRED)
add_library(mission_runtime STATIC mission_runtime.cpp mission_config.cpp)
target_link_libraries(
  mission_runtime
  PUBLIC mission_core
  PRIVATE yaml-cpp
)

add_library(
  mission_ports STATIC ../glue/mission_ports.cpp
                       ../glue/preflight_evaluator.cpp
)
target_link_libraries(
  mission_ports PUBLIC mission_runtime communicate guidance_flight
)

add_executable(mission_example mission_example.cpp)
target_link_libraries(mission_example PRIVATE mission_core)
set_target_properties(
  mission_example PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
)

add_executable(
  test_mission ${CMAKE_SOURCE_DIR}/src/test/test_app/test_mission.cpp
)
target_link_libraries(test_mission PRIVATE mission_core)
set_target_properties(
  test_mission PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin
)
add_test(NAME mission COMMAND test_mission)
set_tests_properties(mission PROPERTIES TIMEOUT 15)
add_test(NAME mission_example COMMAND mission_example)
set_tests_properties(mission_example PROPERTIES TIMEOUT 15)

add_executable(
  test_mission_runtime
  ${CMAKE_SOURCE_DIR}/src/test/test_app/test_mission_runtime.cpp
)
target_link_libraries(test_mission_runtime PRIVATE mission_ports)
add_test(NAME mission_runtime COMMAND test_mission_runtime)
set_tests_properties(mission_runtime PROPERTIES TIMEOUT 15)

add_executable(
  test_mission_wire ${CMAKE_SOURCE_DIR}/src/test/test_app/test_mission_wire.cpp
)
target_link_libraries(test_mission_wire PRIVATE mission_ports)
target_compile_options(test_mission_wire PRIVATE -Wno-address-of-packed-member)
add_test(NAME mission_wire COMMAND test_mission_wire)
set_tests_properties(mission_wire PROPERTIES TIMEOUT 15)

add_executable(
  test_preflight_evaluator
  ${CMAKE_SOURCE_DIR}/src/test/test_app/test_preflight_evaluator.cpp
)
target_link_libraries(test_preflight_evaluator PRIVATE mission_ports)
add_test(NAME preflight_evaluator COMMAND test_preflight_evaluator)
set_tests_properties(preflight_evaluator PROPERTIES TIMEOUT 15)
