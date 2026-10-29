cmake_minimum_required(VERSION 3.16)
project(pure_uav_vio_online LANGUAGES CXX)

# ==============================================================================
# pure-uav VIO online module
#
# 目标：把 IMU + 相机数据变成 pure_uav::VisionPose（给飞控通信模块）
#
# 依赖：Eigen3 / OpenCV4 / Boost(system,filesystem,thread,date_time) / Ceres
#       OpenVINS 核心（ov_core + ov_init + ov_msckf，ROS-free，已裁到本项目
#       third_party/open_vins）
#
# 不依赖 ROS。可视化/数据集回放可继续用你现有的 ~/vio_ws_ros1 环境做对照。
# ==============================================================================

if(CMAKE_SOURCE_DIR STREQUAL CMAKE_CURRENT_SOURCE_DIR)
    if(NOT CMAKE_BUILD_TYPE)
        set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
    endif()
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/lib)
endif()

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

# AUTO: 能找到 Orbbec SDK 和 MAVLink 头文件才编译实时程序
set(PURE_VIO_BUILD_LIVE "AUTO" CACHE STRING "Build live app: AUTO / ON / OFF")
option(PURE_VIO_BUILD_OPENVINS "Build vendored OpenVINS core" ON)

set(OPENVINS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/third_party/open_vins)

# ------------------------------------------------------------------------------
# 依赖
# ------------------------------------------------------------------------------
find_package(Eigen3 REQUIRED)
find_package(OpenCV 4 REQUIRED)
find_package(Boost 1.65 REQUIRED COMPONENTS system filesystem thread date_time)
find_package(Ceres REQUIRED)
find_package(Threads REQUIRED)

message(STATUS "Eigen:  ${Eigen3_VERSION}")
message(STATUS "OpenCV: ${OpenCV_VERSION}")
message(STATUS "Boost:  ${Boost_VERSION}")
message(STATUS "Ceres:  ${Ceres_VERSION}")

# ------------------------------------------------------------------------------
# OpenVINS 核心库（ROS-free）
# ------------------------------------------------------------------------------
if(PURE_VIO_BUILD_OPENVINS)
    file(GLOB_RECURSE OPENVINS_SOURCES
        ${OPENVINS_DIR}/ov_core/src/*.cpp
        ${OPENVINS_DIR}/ov_init/src/*.cpp
        ${OPENVINS_DIR}/ov_msckf/src/*.cpp
    )
    list(LENGTH OPENVINS_SOURCES OPENVINS_SOURCE_COUNT)

    add_library(openvins_core STATIC ${OPENVINS_SOURCES})
    set_target_properties(openvins_core PROPERTIES
        CXX_STANDARD 14
        CXX_STANDARD_REQUIRED ON
        POSITION_INDEPENDENT_CODE ON
    )
    target_compile_definitions(openvins_core PUBLIC
        ROS_AVAILABLE=0
        ENABLE_ARUCO_TAGS=0
    )
    target_compile_options(openvins_core PRIVATE
        -O3
        -fomit-frame-pointer
        -fno-signed-zeros
        -fno-math-errno
        -funroll-loops
        -Wno-unused-parameter
        -Wno-unused-variable
        -Wno-sign-compare
        -Wno-maybe-uninitialized
    )
    target_include_directories(openvins_core PUBLIC
        ${OPENVINS_DIR}/ov_core/src
        ${OPENVINS_DIR}/ov_init/src
        ${OPENVINS_DIR}/ov_msckf/src
        ${Eigen3_INCLUDE_DIRS}
    )
    target_link_libraries(openvins_core PUBLIC
        Eigen3::Eigen
        ${OpenCV_LIBS}
        Boost::system
        Boost::filesystem
        Boost::thread
        Boost::date_time
        Ceres::ceres
        Threads::Threads
    )
    message(STATUS "OpenVINS sources: ${OPENVINS_SOURCE_COUNT} (ROS-free)")
endif()

# ------------------------------------------------------------------------------
# pure_vio_core：估计器封装 + 坐标系转换 + 数据源/输出
# ------------------------------------------------------------------------------
add_library(pure_vio_core STATIC
    src/vio_estimator.cpp
    src/vio_health.cpp
    src/vision_pose_convert.cpp
    src/pose_logger.cpp
)
target_include_directories(pure_vio_core PUBLIC
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 如果仓库里已经有通信模块，直接复用它的 VisionPose 定义，避免同一个 struct 定义两份。
if(EXISTS "${CMAKE_SOURCE_DIR}/src/driver/communicate/include/communicator.hpp")
    # 仓库的 VisionPose 没有 cam_imu_dt 这个纯诊断字段（它归通讯模块维护）。
    # 用宏把它关掉，避免为一个诊断字段去改别人的公开头文件。
    target_compile_definitions(pure_vio_core PUBLIC PURE_UAV_VISIONPOSE_NO_CAM_IMU_DT)
    target_include_directories(pure_vio_core PUBLIC
        "${CMAKE_SOURCE_DIR}/src/driver/communicate/include")
    message(STATUS "VisionPose: 复用 src/driver/communicate/include/communicator.hpp")
endif()
target_link_libraries(pure_vio_core PUBLIC
    openvins_core
    Eigen3::Eigen
    ${OpenCV_LIBS}
    Boost::filesystem
    Threads::Threads
)

# 离线回放
add_executable(vio_offline apps/vio_offline_main.cpp src/offline_dataset.cpp)
target_include_directories(vio_offline PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
target_link_libraries(vio_offline PRIVATE pure_vio_core)

# 实时（Orbbec + MAVLink）
# 自动探测 pure-uav 仓库里的驱动；独立构建时回退到用户机器上的路径。
if(NOT PURE_UAV_DRIVER_DIR)
    if(EXISTS "${CMAKE_SOURCE_DIR}/src/driver/orbbec/include/libobsensor/ObSensor.hpp")
        set(PURE_UAV_DRIVER_DIR "${CMAKE_SOURCE_DIR}/src/driver")
    else()
        set(PURE_UAV_DRIVER_DIR "$ENV{HOME}/桌面/pure-uav/pure-uav-vio/src/driver")
    endif()
endif()

if(NOT PURE_UAV_MAVLINK_DIR)
    if(EXISTS "${CMAKE_SOURCE_DIR}/src/driver/communicate/third_party/mavlink/common/mavlink.h")
        set(PURE_UAV_MAVLINK_DIR "${CMAKE_SOURCE_DIR}/src/driver/communicate/third_party/mavlink")
    elseif(EXISTS "${CMAKE_SOURCE_DIR}/third_party/mavlink/common/mavlink.h")
        set(PURE_UAV_MAVLINK_DIR "${CMAKE_SOURCE_DIR}/third_party/mavlink")
    else()
        set(PURE_UAV_MAVLINK_DIR "$ENV{HOME}/桌面/pure-uav/pure-uav-communication/src/driver/communicate/third_party/mavlink")
    endif()
endif()

set(_have_live_prereqs FALSE)
if(EXISTS "${PURE_UAV_DRIVER_DIR}/orbbec/include/libobsensor/ObSensor.hpp" AND
   EXISTS "${PURE_UAV_MAVLINK_DIR}/common/mavlink.h")
    set(_have_live_prereqs TRUE)
endif()

set(_build_live FALSE)
if(PURE_VIO_BUILD_LIVE STREQUAL "ON")
    set(_build_live TRUE)
elseif(PURE_VIO_BUILD_LIVE STREQUAL "AUTO" AND _have_live_prereqs)
    set(_build_live TRUE)
endif()

if(PURE_VIO_BUILD_LIVE STREQUAL "ON" AND NOT _have_live_prereqs)
    message(FATAL_ERROR "PURE_VIO_BUILD_LIVE=ON，但找不到 Orbbec SDK 或 MAVLink 头文件：\n"
                        "  driver   = ${PURE_UAV_DRIVER_DIR}\n"
                        "  mavlink  = ${PURE_UAV_MAVLINK_DIR}")
endif()

if(_build_live)
    message(STATUS "实时程序: 启用 (driver=${PURE_UAV_DRIVER_DIR})")
    # 【2026-10-25】仓库里 SDK 放在 lib/<arch>/ 下（x86_64、aarch64），
    # 本地开发树是平铺在 lib/ 里。两个位置都找，按架构优先。
    find_library(ORBBEC_SDK_LIB NAMES OrbbecSDK
        PATHS "${PURE_UAV_DRIVER_DIR}/orbbec/lib/${CMAKE_SYSTEM_PROCESSOR}"
              "${PURE_UAV_DRIVER_DIR}/orbbec/lib/x86_64"
              "${PURE_UAV_DRIVER_DIR}/orbbec/lib"
        NO_DEFAULT_PATH)
    if(NOT ORBBEC_SDK_LIB)
        message(FATAL_ERROR "找不到 libOrbbecSDK.so（${PURE_UAV_DRIVER_DIR}/orbbec/lib）")
    endif()

    # 仓库里原有的 test_camera 直接 target_link_libraries(... OrbbecSDK)，
    # 但没有定义这个 target。这里补一个 imported target，让整个仓库能一次编过。
    if(NOT TARGET OrbbecSDK)
        add_library(OrbbecSDK SHARED IMPORTED GLOBAL)
        set_target_properties(OrbbecSDK PROPERTIES
            IMPORTED_LOCATION "${ORBBEC_SDK_LIB}"
            INTERFACE_INCLUDE_DIRECTORIES "${PURE_UAV_DRIVER_DIR}/orbbec/include")
    endif()

    add_executable(vio_live
        apps/vio_live_main.cpp
        src/orbbec_camera.cpp
        src/mavlink_link.cpp
        src/fcu_clock.cpp
    )
    target_include_directories(vio_live PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${PURE_UAV_DRIVER_DIR}/orbbec/include
    )
    target_include_directories(vio_live SYSTEM PRIVATE
        ${PURE_UAV_MAVLINK_DIR}
    )
    target_link_libraries(vio_live PRIVATE pure_vio_core OrbbecSDK)
    set_target_properties(vio_live PROPERTIES
        INSTALL_RPATH "${PURE_UAV_DRIVER_DIR}/orbbec/lib/${CMAKE_SYSTEM_PROCESSOR};${PURE_UAV_DRIVER_DIR}/orbbec/lib"
        BUILD_RPATH "${PURE_UAV_DRIVER_DIR}/orbbec/lib/${CMAKE_SYSTEM_PROCESSOR};${PURE_UAV_DRIVER_DIR}/orbbec/lib"
    )
else()
    message(STATUS "实时程序: 跳过（找不到 Orbbec SDK / MAVLink，可手动设 -DPURE_UAV_DRIVER_DIR / -DPURE_UAV_MAVLINK_DIR）")
endif()

# ------------------------------------------------------------------------------
# 单元测试（不依赖硬件）
# ------------------------------------------------------------------------------
add_executable(test_pose_convert tests/test_pose_convert.cpp)
target_link_libraries(test_pose_convert PRIVATE pure_vio_core)

# 离线"飞控时间基变化"恢复测试：需要已录数据集，没给 --dataset 时返回 77（ctest=skip）。
# 手动跑：
#   ./bin/test_timebase_recovery --dataset datasets/loop5 --config-dir config/gemini_1920x1080
add_executable(test_timebase_recovery
    tests/test_timebase_recovery_offline.cpp
    src/offline_dataset.cpp
)
target_include_directories(test_timebase_recovery PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)
target_link_libraries(test_timebase_recovery PRIVATE pure_vio_core)

add_executable(test_fcu_clock tests/test_fcu_clock.cpp src/fcu_clock.cpp)
target_include_directories(test_fcu_clock PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/src)
find_package(Threads REQUIRED)
target_link_libraries(test_fcu_clock PRIVATE Threads::Threads)

# ODOMETRY 打包/解析回归：不需要飞控、相机、串口，只需要 MAVLink 头文件。
# 它拦的是「字段顺序传反」这类只看发送返回值发现不了的 bug（2026-10-28 修过一次）。
if(EXISTS "${PURE_UAV_MAVLINK_DIR}/common/mavlink.h")
    add_executable(test_odometry_pack tests/test_odometry_pack.cpp src/mavlink_link.cpp)
    target_include_directories(test_odometry_pack PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${PURE_UAV_MAVLINK_DIR}
    )
    target_link_libraries(test_odometry_pack PRIVATE Threads::Threads)

    # MavlinkLink 的 PTY 线上回归：心跳来源筛选 + 解锁状态 + IMU 队列/清空
    add_executable(test_mavlink_link_wire tests/test_mavlink_link_wire.cpp src/mavlink_link.cpp)
    target_include_directories(test_mavlink_link_wire PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${PURE_UAV_MAVLINK_DIR}
    )
    target_link_libraries(test_mavlink_link_wire PRIVATE Threads::Threads)

    # 联调工具核心逻辑（合成轨迹 + ODOMETRY 统计）
    add_executable(test_odom_tool_core tests/test_odom_tool_core.cpp)
    target_include_directories(test_odom_tool_core PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/tools
        ${PURE_UAV_MAVLINK_DIR}
    )
    target_link_libraries(test_odom_tool_core PRIVATE Threads::Threads)

    # 台架工具：合成轨迹发送器（无相机/无 VIO 时单独测飞控接收与轴向）
    add_executable(odometry_sender tools/odometry_sender.cpp src/mavlink_link.cpp)
    target_include_directories(odometry_sender PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/src
        ${CMAKE_CURRENT_SOURCE_DIR}/include
        ${CMAKE_CURRENT_SOURCE_DIR}/tools
        ${PURE_UAV_MAVLINK_DIR}
    )
    target_link_libraries(odometry_sender PRIVATE Threads::Threads)

    # 台架工具：假飞控（PTY），没有飞机时把"上位机→飞控"整段跑通
    add_executable(fake_fcu tools/fake_fcu.cpp)
    target_include_directories(fake_fcu PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/tools
        ${PURE_UAV_MAVLINK_DIR}
    )
    target_link_libraries(fake_fcu PRIVATE Threads::Threads)

    # 台架工具：ODOMETRY 线上探针（从串口/原始字节文件解码核对）
    add_executable(odometry_probe tools/odometry_probe.cpp)
    target_include_directories(odometry_probe PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}/tools
        ${PURE_UAV_MAVLINK_DIR}
    )
    target_link_libraries(odometry_probe PRIVATE Threads::Threads)
else()
    message(STATUS "test_odometry_pack: 跳过（没找到 MAVLink 头文件: ${PURE_UAV_MAVLINK_DIR}）")
endif()

enable_testing()
add_test(NAME pose_convert COMMAND test_pose_convert)
add_test(NAME fcu_clock COMMAND test_fcu_clock)
if(TARGET test_odometry_pack)
    add_test(NAME odometry_pack COMMAND test_odometry_pack)
endif()
if(TARGET test_mavlink_link_wire)
    add_test(NAME mavlink_link_wire COMMAND test_mavlink_link_wire)
endif()
if(TARGET test_odom_tool_core)
    add_test(NAME odom_tool_core COMMAND test_odom_tool_core)
endif()

# 离线时间基恢复测试需要一份已录数据集。给了路径才加进 ctest；否则只保留可执行文件。
set(PURE_VIO_TEST_DATASET "" CACHE PATH "已录数据集目录（离线时间基恢复测试用；空则跳过）")
if(PURE_VIO_TEST_DATASET AND TARGET test_timebase_recovery)
    add_test(NAME timebase_recovery COMMAND test_timebase_recovery
             --dataset ${PURE_VIO_TEST_DATASET}
             --config-dir ${CMAKE_CURRENT_SOURCE_DIR}/config/gemini_1920x1080
             --warmup-frames 100
             --max-after 200
             --quiet)
    set_tests_properties(timebase_recovery PROPERTIES TIMEOUT 900)
endif()
