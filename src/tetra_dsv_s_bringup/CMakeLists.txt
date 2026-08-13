cmake_minimum_required(VERSION 3.8)
project(tetra_dsv_s_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_executable(tetra_drive_node
  src/tetra_drive_node.cpp
  src/drive_board.cpp)
target_include_directories(tetra_drive_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_compile_features(tetra_drive_node PUBLIC cxx_std_17)
ament_target_dependencies(tetra_drive_node
  rclcpp geometry_msgs nav_msgs sensor_msgs std_msgs)

install(TARGETS tetra_drive_node DESTINATION lib/${PROJECT_NAME})
# "config" now holds the real-robot configs written on 2026-08-13 (velodyne,
# imu_gv7, ekf, slam_toolbox), so it is installed again. It was left out while
# empty because git does not store empty directories and listing it broke every
# fresh clone. The sim copies still cannot be reused as they are - they carry
# use_sim_time: True.
install(DIRECTORY launch udev config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/probe_drive_board.py
                 scripts/check_imu_mounting.py
        DESTINATION lib/${PROJECT_NAME})

ament_package()
