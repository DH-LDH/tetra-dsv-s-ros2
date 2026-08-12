"""Simulation + slam_toolbox, for building a new map.

Mirrors scout_warehouse_sim/launch/slam.launch.py, plus a nav2 switch.

    # Map building by hand - recommended. Teleop drives /cmd_vel directly.
    ros2 launch tetra_dsv_s_sim slam.launch.py nav2:=false
    ros2 run teleop_twist_keyboard teleop_twist_keyboard

    # Mapping with Nav2 up, as the Scout version does. Teleop must then target
    # /cmd_vel_nav, and since velocity_smoother drops the command after
    # velocity_timeout (1 s) you have to HOLD each key rather than tap it.
    ros2 launch tetra_dsv_s_sim slam.launch.py
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
      --ros-args -r cmd_vel:=cmd_vel_nav

Save the finished map with:
    ros2 run nav2_map_server map_saver_cli -f /ws/src/tetra_dsv_s_sim/maps/room
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# ======================================================================
#  조정 파라미터 — 이 블록만 손대면 됩니다
#  스폰 포즈(x/y/z/yaw)는 여기가 아니라 simulation.launch.py 상단에 있습니다.
#  이 파일은 그 넷을 하위로 전달하지 않습니다.
# ======================================================================

DEFAULT_WORLD = "loop_room.sdf"
DEFAULT_ODOMETRY_SOURCE = "fused"
DEFAULT_NAV2 = "true"          # false = 매핑 전용. teleop 이 안 끊깁니다


def generate_launch_description():
    sim_share = get_package_share_directory("tetra_dsv_s_sim")
    nav2_share = get_package_share_directory("nav2_bringup")

    world_default = os.path.join(sim_share, "worlds", DEFAULT_WORLD)
    slam_params = os.path.join(sim_share, "config", "slam_toolbox.yaml")
    nav2_params = os.path.join(sim_share, "config", "nav2_params.yaml")

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(sim_share, "launch", "simulation.launch.py")
        ),
        launch_arguments={
            "world": LaunchConfiguration("world"),
            "odometry_source": LaunchConfiguration("odometry_source"),
            "gui": LaunchConfiguration("gui"),
            "rviz": LaunchConfiguration("rviz"),
        }.items(),
    )

    # Full Nav2 bringup, which starts slam_toolbox for us.
    slam_and_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "slam": "True",
            "map": "",
            "use_sim_time": "True",
            "autostart": "True",
            "params_file": nav2_params,
            "slam_params_file": slam_params,
            "use_composition": "False",
        }.items(),
        condition=IfCondition(LaunchConfiguration("nav2")),
    )

    # Mapping only. With no Nav2 there is no velocity_smoother sitting between
    # the keyboard and the robot, so /cmd_vel is driven directly and a held key
    # gives continuous motion instead of one-second bursts.
    slam_only = Node(
        package="slam_toolbox",
        executable="sync_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[slam_params, {"use_sim_time": True}],
        condition=UnlessCondition(LaunchConfiguration("nav2")),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=world_default),
        DeclareLaunchArgument("odometry_source",
                              default_value=DEFAULT_ODOMETRY_SOURCE,
                              choices=["fused", "wheel", "ground_truth"]),
        DeclareLaunchArgument("nav2", default_value=DEFAULT_NAV2,
                              choices=["true", "false"]),
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        simulation,
        slam_and_navigation,
        slam_only,
    ])
