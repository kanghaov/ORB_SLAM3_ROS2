cmake_minimum_required(VERSION 3.5)
project(orbslam3)

set(ENV{PYTHONPATH} "/opt/ros/foxy/lib/python3.8/site-packages/")
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${CMAKE_CURRENT_SOURCE_DIR}/CMakeModules)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(message_filters REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(ORB_SLAM3 REQUIRED)

message(STATUS "ORB_SLAM3_INCLUDE_DIR = ${ORB_SLAM3_INCLUDE_DIR}")
message(STATUS "DBoW2_INCLUDE_DIR     = ${DBoW2_INCLUDE_DIR}")

if(NOT ORB_SLAM3_INCLUDE_DIR)
  message(FATAL_ERROR "ORB_SLAM3_INCLUDE_DIR is not set. Pass -DORB_SLAM3_INCLUDE_DIR=/home/hao/ORB_SLAM3/include")
endif()

# 从 include 推导 ORB_SLAM3 根目录（用于包含 Thirdparty/DBoW2/... 这种相对路径）
get_filename_component(ORB_SLAM3_ROOT_FROM_INCLUDE "${ORB_SLAM3_INCLUDE_DIR}/.." ABSOLUTE)

# 仅把本包 include 放到全局；其余路径在目标级别加
include_directories(include)

add_executable(mono
  src/monocular/mono.cpp
  src/monocular/monocular-slam-node.cpp
)
ament_target_dependencies(mono rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin)

add_executable(rgbd
  src/rgbd/rgbd.cpp
  src/rgbd/rgbd-slam-node.cpp
)
ament_target_dependencies(rgbd rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin)

add_executable(stereo
  src/stereo/stereo.cpp
  src/stereo/stereo-slam-node.cpp
)
ament_target_dependencies(stereo rclcpp sensor_msgs cv_bridge message_filters ORB_SLAM3 Pangolin)

add_executable(stereo-inertial
  src/stereo-inertial/stereo-inertial.cpp
  src/stereo-inertial/stereo-inertial-node.cpp
)
ament_target_dependencies(stereo-inertial rclcpp sensor_msgs cv_bridge ORB_SLAM3 Pangolin)

# 统一补齐头文件路径 + 链接第三方库
foreach(T mono rgbd stereo stereo-inertial)
  target_include_directories(${T} PRIVATE
    ${ORB_SLAM3_INCLUDE_DIR}
    ${ORB_SLAM3_INCLUDE_DIR}/CameraModels
    ${ORB_SLAM3_ROOT_FROM_INCLUDE}         # 让 "Thirdparty/DBoW2/..." 相对包含能找到
    ${DBoW2_INCLUDE_DIR}                   # 保险起见也加上
  )
  # ★★ 关键：使用“plain 签名”（不要写 PRIVATE/PUBLIC）
  target_link_libraries(${T}
    ${ORB_SLAM3_LIBRARY}
    ${DBoW2_LIBRARY}
    ${g2o_LIBRARY}
  )
endforeach()

install(TARGETS mono rgbd stereo stereo-inertial
  DESTINATION lib/${PROJECT_NAME})

ament_package()
