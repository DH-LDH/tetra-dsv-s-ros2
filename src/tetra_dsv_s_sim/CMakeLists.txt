cmake_minimum_required(VERSION 3.8)
project(tetra_dsv_s_sim)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config worlds rviz maps
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
