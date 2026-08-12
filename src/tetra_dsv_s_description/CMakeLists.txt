cmake_minimum_required(VERSION 3.8)
project(tetra_dsv_s_description)

find_package(ament_cmake REQUIRED)

# Every directory listed here must exist AND contain at least one tracked file.
# git does not store empty directories, so an empty one survives on the machine
# that created it and vanishes on a fresh clone - where ament_cmake_symlink_install
# then fails with "can't find .../<dir>". That is exactly how this package broke
# on the Jetson while still building here. "meshes" was dropped rather than
# padded: the URDF is entirely primitives and references no mesh at all.
install(
  DIRECTORY urdf launch rviz
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
