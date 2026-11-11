set(TEST_OUTPUT_DIR ${CMAKE_SOURCE_DIR}/bin/test)

add_executable(shm_write test_common/sw.cpp)
add_executable(shm_read test_common/sr.cpp)

add_executable(test_camera test_driver/test_camera.cpp)
target_include_directories(
  test_camera PUBLIC ${CMAKE_SOURCE_DIR}/src/driver/orbbec/include
)
target_link_libraries(
  test_camera
  # ${CMAKE_SOURCE_DIR}/src/driver/orbbec/lib/libOrbbecSDK.so
  OrbbecSDK
)

message(${CMAKE_CURRENT_SOURCE_DIR})

add_executable(test_math test_common/test_math.cpp)
# 2026-10-20：删掉了指向 ${CMAKE_CURRENT_SOURCE_DIR}/common_libs/include 的**死路径**
# （src/test/ 下并没有 common_libs 目录）；include 目录由 pure_math_tools 的 PUBLIC include
# 目录传递，无需在此重复写。
target_link_libraries(test_math pure_math_tools)

# 制导模块单元测试（讲义 §10 手算例 + 三条铁律 + 三律端到端 + §15 不变量 + §16 主循环外壳 + §17 端到端闭环回归 + §18
# CSV 录制/回放） ⚠ 本测试**确实需要仿真**：它用 TruthState / SensorConfig / Sensor 三级逼真度 以及
# TargetModel 做端到端，所以链接 guidance_sim（= core + sim）。 ★ 另链接
# guidance_application（宿主启动层），用于 §16 的 GuidanceLoop 行为验证 （像素反投影 / 降级策略 / 重发）、§17
# 的闭环回归（复用 synthetic_world） 和 §18 的 CSV 录制→回放（复用 csv_io）。guidance_application 由
# src/app 提供， src/CMakeLists.txt 里 app 必须排在 guidance 之后。
add_executable(test_guidance test_guidance/test_guidance.cpp)
target_include_directories(
  test_guidance PUBLIC ${CMAKE_SOURCE_DIR}/src/guidance/guidance_libs/include
)
target_link_libraries(
  test_guidance guidance_core guidance_sim guidance_application
)

# MAVLink 单位换算单元测试（公共件；2026-10-20 随库迁至 src/common/common_libs）
add_executable(test_mavlink_units test_adapter/test_mavlink_units.cpp)
# include 目录由 mavlink_units 的 PUBLIC include 目录传递。
target_link_libraries(test_mavlink_units mavlink_units)

# 数据对齐适配层单元测试（外部口径 → 内部口径） 覆盖：NED↔ENU / 时间对齐与回绕 / 姿态（外部机体系→光轴系）/ 四元数交叉验证 / 组装
# GuidanceState / 上游模拟器（多口径同一场景）/ 不变量自检 ★ 链接要求：guidance_state_adapter +
# mavlink_units + pure_vec3 + **guidance_core** —— 本测试属于**真机路径**，只该依赖核心。它需要
# guidance_step() 校验不变量、 以及 quat_rotate_vec 做交叉验证；这两样都在 guidance_core 里。
# 2026-10-20 起适配层已从 src/interface 迁到 src/guidance/adapter， Vec3 迁到
# src/common（pure_vec3），故 include 路径同步更新。 ⚠ 不要链接 guidance_sim —— 那会把仿真拖进真机路径的依赖，
# 正是 2026-10-16 把四元数工具从 sensor.* 移出核心所要修掉的问题。
add_executable(test_state_adapter test_adapter/test_state_adapter.cpp)
target_include_directories(
  test_state_adapter
  PUBLIC ${CMAKE_SOURCE_DIR}/src/guidance/adapter/include
         ${CMAKE_SOURCE_DIR}/src/common/common_libs/include
         ${CMAKE_SOURCE_DIR}/src/guidance/guidance_libs/include
)
target_link_libraries(
  test_state_adapter guidance_state_adapter mavlink_units pure_vec3
  guidance_core
)

add_executable(test_command test_driver/test_command.cpp)
target_include_directories(
  test_command PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../driver/communicate/include
)
target_link_libraries(test_command communicate)

add_executable(test_communicate test_driver/test_communicate.cpp)
target_include_directories(
  test_communicate
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../driver/communicate/include
)
target_link_libraries(test_communicate communicate)

add_executable(test_frequency test_driver/test_frequency.cpp)
target_include_directories(
  test_frequency
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../driver/communicate/include
)
target_link_libraries(test_frequency communicate)

add_executable(test_multithread test_driver/test_multithread.cpp)
target_include_directories(
  test_multithread
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../driver/communicate/include
)
target_link_libraries(test_multithread communicate)

# 【2026-10-28】无飞控联调回归：PTY 模拟飞控，验证 · 心跳解锁位 → isArmed() · HIGHRES_IMU → 逐帧队列
# popImu()（顺序/主机时钟/溢出计数） · pushVisionPose() → ODOMETRY 的 quality / reset_counter
# / 协方差 不需要真机、不需要摄像头。VIO↔通信交接时先跑它。
add_executable(test_vision_pose_wire test_driver/test_vision_pose_wire.cpp)
target_include_directories(
  test_vision_pose_wire
  PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/../driver/communicate/include
         ${CMAKE_SOURCE_DIR}/src/common
)
target_link_libraries(test_vision_pose_wire communicate)

set_target_properties(
  shm_write
  shm_read
  test_camera
  test_guidance
  test_mavlink_units
  test_state_adapter
  test_math
  test_command
  test_communicate
  test_frequency
  test_multithread
  test_vision_pose_wire
  PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
)

# 无硬件测试纳入根目录 ctest；串口/相机实机程序仍由操作者显式运行。
foreach(test_name IN ITEMS math guidance mavlink_units state_adapter
                           vision_pose_wire
)
  add_test(NAME ${test_name} COMMAND test_${test_name})
  set_tests_properties(${test_name} PROPERTIES TIMEOUT 120)
endforeach()

# 用替代驱动验证真实推理线程的外部帧模式，不依赖硬件或 ONNX Runtime。
find_package(OpenCV 4 REQUIRED)
add_executable(
  test_onboard_runtime
  test_driver/test_onboard_runtime.cpp ../detection-rt/src/detector.cpp
  ../detection-rt/src/yolo_postprocess.cpp
)
target_compile_definitions(
  test_onboard_runtime PRIVATE PURE_DETECTION_BACKEND_CPU=1
)
target_include_directories(
  test_onboard_runtime
  PRIVATE ${CMAKE_SOURCE_DIR}/src/detection-rt/include
          ${CMAKE_SOURCE_DIR}/src/detection-rt/src ${OpenCV_INCLUDE_DIRS}
)
target_link_libraries(
  test_onboard_runtime PRIVATE guidance_application communicate ${OpenCV_LIBS}
)
set_target_properties(
  test_onboard_runtime PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
)
add_test(NAME onboard_runtime COMMAND test_onboard_runtime)
set_tests_properties(
  onboard_runtime PROPERTIES WORKING_DIRECTORY ${CMAKE_SOURCE_DIR} TIMEOUT 15
)

add_executable(test_camera_shm test_app/test_camera_shm.cpp)
target_include_directories(test_camera_shm PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(test_camera_shm PRIVATE pure_camera_ipc ${OpenCV_LIBS})
set_target_properties(
  test_camera_shm PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
)
add_test(NAME camera_shm COMMAND test_camera_shm)
set_tests_properties(camera_shm PROPERTIES TIMEOUT 15)

# 检测配置：字段解析、严格白名单、范围校验与模型路径解析（无硬件）。
find_package(yaml-cpp REQUIRED)
add_executable(test_detector_config test_driver/test_detector_config.cpp)
target_include_directories(test_detector_config PRIVATE ${OpenCV_INCLUDE_DIRS})
target_link_libraries(test_detector_config PRIVATE detection_rt yaml-cpp)
set_target_properties(
  test_detector_config PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
)
add_test(NAME detector_config COMMAND test_detector_config)
set_tests_properties(
  detector_config PROPERTIES WORKING_DIRECTORY ${CMAKE_SOURCE_DIR} TIMEOUT 30
)

# 检测子进程管理：启动、回收、重启上限、停止与信号转发（无硬件）。
add_executable(test_detection_process test_app/test_detection_process.cpp)
target_include_directories(
  test_detection_process PRIVATE ${CMAKE_SOURCE_DIR}/src/app
)
set_target_properties(
  test_detection_process PROPERTIES RUNTIME_OUTPUT_DIRECTORY ${TEST_OUTPUT_DIR}
)
add_test(NAME detection_process COMMAND test_detection_process)
set_tests_properties(detection_process PROPERTIES TIMEOUT 30)

add_executable(test_detection_supervisor test_app/test_detection_supervisor.cpp)
target_include_directories(
  test_detection_supervisor PRIVATE ${CMAKE_SOURCE_DIR}/src/app
)
target_link_libraries(test_detection_supervisor PRIVATE pthread)
set_target_properties(
  test_detection_supervisor PROPERTIES RUNTIME_OUTPUT_DIRECTORY
                                       ${TEST_OUTPUT_DIR}
)
add_test(NAME detection_supervisor COMMAND test_detection_supervisor)
set_tests_properties(detection_supervisor PROPERTIES TIMEOUT 15)
