"""Simulation + map server + AMCL + Nav2, for driving on a saved map.

Mirrors scout_warehouse_sim/launch/navigation.launch.py. Run this only after
slam.launch.py has been shut down and a map saved.

    ros2 launch tetra_dsv_s_sim navigation.launch.py \
      map:=$PWD/src/tetra_dsv_s_sim/maps/room.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

# ======================================================================
#  조정 파라미터 — 이 블록만 손대면 됩니다
#
#  WORLD 와 MAP 은 반드시 짝이 맞아야 합니다. 안 맞으면 AMCL 이 엉뚱한 벽에
#  스캔을 정합시키려 들고, 로봇 위치를 크게 틀리게 잡습니다.
#      loop_room.sdf  <-> room.yaml
#      empty_room.sdf <-> empty_room.yaml   (실제 공간과 같은 쪽)
# ======================================================================

DEFAULT_WORLD = "loop_room.sdf"
DEFAULT_MAP = "room.yaml"
DEFAULT_ODOMETRY_SOURCE = "fused"


def generate_launch_description():
    sim_share = get_package_share_directory("tetra_dsv_s_sim")
    nav2_share = get_package_share_directory("nav2_bringup")

    world_default = os.path.join(sim_share, "worlds", DEFAULT_WORLD)
    map_default = os.path.join(sim_share, "maps", DEFAULT_MAP)

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

    localization_and_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_share, "launch", "bringup_launch.py")
        ),
        launch_arguments={
            "slam": "False",
            "map": LaunchConfiguration("map"),
            "use_sim_time": "True",
            "autostart": "True",
            "params_file": os.path.join(sim_share, "config", "nav2_params.yaml"),
            "use_composition": "False",
        }.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=world_default),
        DeclareLaunchArgument("odometry_source",
                              default_value=DEFAULT_ODOMETRY_SOURCE,
                              choices=["fused", "wheel", "ground_truth"]),
        DeclareLaunchArgument("map", default_value=map_default),
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        simulation,
        localization_and_navigation,
    ])
