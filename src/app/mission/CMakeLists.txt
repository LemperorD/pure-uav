# 任务核心只依赖消息/采样类型，无 ROS、串口或推理运行时依赖。
add_library(mission_core STATIC mission_controller.cpp)
target_include_directories(
  mission_core PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/..
                      ${CMAKE_SOURCE_DIR}/src/common
                      ${CMAKE_SOURCE_DIR}/src/common/common_libs/include
                      ${CMAKE_SOURCE_DIR}/src/guidance/flight
                      ${CMAKE_SOURCE_DIR}/src/guidance/guidance_libs/include)

add_executable(mission_example mission_example.cpp)
target_link_libraries(mission_example PRIVATE mission_core)
set_target_properties(
  mission_example PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)

add_executable(test_mission
               ${CMAKE_SOURCE_DIR}/src/test/test_app/test_mission.cpp)
target_link_libraries(test_mission PRIVATE mission_core)
set_target_properties(
  test_mission PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/bin)
add_test(NAME mission COMMAND test_mission)
set_tests_properties(mission PROPERTIES TIMEOUT 15)
add_test(NAME mission_example COMMAND mission_example)
set_tests_properties(mission_example PROPERTIES TIMEOUT 15)
