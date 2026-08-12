"""Gazebo Fortress + bridge + RViz for the TETRA-DSV-S.

Mirrors scout_warehouse_sim/launch/simulation.launch.py. Differences are only
where the Scout version handled hardware this robot does not have: no arm
trajectory bridge, no wrist camera, no workcell markers.

    ros2 launch tetra_dsv_s_sim simulation.launch.py
    ros2 launch tetra_dsv_s_sim simulation.launch.py world:=<path to .sdf>
    ros2 launch tetra_dsv_s_sim simulation.launch.py gui:=false rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import PythonExpression
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# ======================================================================
#  조정 파라미터 — 이 블록만 손대면 됩니다
#  전부 launch 인자로도 덮어쓸 수 있습니다. 예: x:=0.5 world:=/path/to.sdf
# ======================================================================

# 기본 world. loop_room 인 이유: 빈 방은 모든 벽이 항상 보여 drift 가 안 쌓이므로
# loop closure 를 검증할 수 없습니다. 실제 공간과 같은 건 empty_room.sdf 쪽.
DEFAULT_WORLD = "loop_room.sdf"

# 스폰 포즈. 기둥을 도는 첫 바퀴가 실제 순환이 되도록 중심에서 비켜 시작합니다.
# z 는 base_footprint 가 접지점이라 몇 cm 만 띄웁니다. 더 높이면 로봇이 떨어지며
# 튀고, 0 이면 바퀴가 지면에 파묻힌 채 시작합니다.
SPAWN_X = "-1.0"
SPAWN_Y = "0.0"
SPAWN_Z = "0.05"
SPAWN_YAW = "0.0"

# 오도메트리 소스 기본값.
#   fused         휠 vx + IMU yaw rate 를 EKF 로 융합. 실기에 가장 가까움
#   wheel         실제 엔코더처럼 drift 함. loop closure 를 관측하려면 이것
#   ground_truth  시뮬레이터의 정확한 위치. drift 가 0 이라 SLAM 이 이유 없이
#                 완벽해 보이고 loop closure 가 영영 발화하지 않습니다
DEFAULT_ODOMETRY_SOURCE = "fused"

# 노드 기동 지연(초). Gazebo 가 뜨기 전에 spawn 하면 조용히 실패하고,
# 로봇이 없는 상태에서 센서 노드가 뜨면 TF 를 못 찾습니다.
SPAWN_DELAY = 2.0
SENSOR_NODE_DELAY = 3.0


def generate_launch_description():
    package_share = get_package_share_directory("tetra_dsv_s_sim")
    description_share = get_package_share_directory("tetra_dsv_s_description")

    world = LaunchConfiguration("world")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")

    world_default = os.path.join(package_share, "worlds", DEFAULT_WORLD)
    robot_file = os.path.join(
        description_share, "urdf", "tetra_dsv_s.urdf.xacro"
    )
    bridge_file = os.path.join(package_share, "config", "ros_gz_bridge.yaml")
    scan_file = os.path.join(
        package_share, "config", "pointcloud_to_laserscan.yaml"
    )
    rviz_file = os.path.join(package_share, "rviz", "tetra_slam.rviz")
    ekf_file = os.path.join(package_share, "config", "ekf.yaml")

    use_ekf = IfCondition(
        PythonExpression(["'", LaunchConfiguration("odometry_source"), "' == 'fused'"])
    )

    robot_description = {
        "robot_description": ParameterValue(
            Command(
                [
                    FindExecutable(name="xacro"), " ", robot_file,
                    " use_gazebo:=true",
                    " odometry_source:=", LaunchConfiguration("odometry_source"),
                ]
            ),
            value_type=str,
        )
    }

    gazebo_gui = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", "--verbose", "2", world],
        output="screen",
        condition=IfCondition(gui),
    )
    gazebo_headless = ExecuteProcess(
        cmd=["ign", "gazebo", "-r", "-s", "--verbose", "2", world],
        output="screen",
        condition=UnlessCondition(gui),
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"use_sim_time": True}, robot_description],
        output="screen",
    )

    spawn_robot = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            "-name", "tetra_dsv_s",
            "-topic", "robot_description",
            "-x", LaunchConfiguration("x"),
            "-y", LaunchConfiguration("y"),
            "-z", LaunchConfiguration("z"),
            "-Y", LaunchConfiguration("yaw"),
        ],
        output="screen",
    )

    bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="tetra_gz_bridge",
        parameters=[{
            "config_file": bridge_file,
            "qos_overrides./tf_static.publisher.durability": "transient_local",
        }],
        output="screen",
    )

    pointcloud_to_scan = Node(
        package="pointcloud_to_laserscan",
        executable="pointcloud_to_laserscan_node",
        name="velodyne_to_scan",
        parameters=[scan_file, {"use_sim_time": True}],
        remappings=[("cloud_in", "/points"), ("scan", "/scan")],
        output="screen",
    )

    # Owns odom -> base_footprint in "fused" mode; in the other two modes a
    # Gazebo plugin owns it and this node is not started at all.
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        parameters=[ekf_file],
        remappings=[("odometry/filtered", "/odometry/filtered")],
        condition=use_ekf,
        output="screen",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_file],
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(rviz),
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("world", default_value=world_default),
        DeclareLaunchArgument("odometry_source",
                              default_value=DEFAULT_ODOMETRY_SOURCE,
                              choices=["fused", "wheel", "ground_truth"]),
        DeclareLaunchArgument("gui", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("rviz", default_value="true", choices=["true", "false"]),
        DeclareLaunchArgument("x", default_value=SPAWN_X),
        DeclareLaunchArgument("y", default_value=SPAWN_Y),
        DeclareLaunchArgument("z", default_value=SPAWN_Z),
        DeclareLaunchArgument("yaw", default_value=SPAWN_YAW),
        gazebo_gui,
        gazebo_headless,
        robot_state_publisher,
        bridge,
        TimerAction(period=SPAWN_DELAY, actions=[spawn_robot]),
        TimerAction(period=SENSOR_NODE_DELAY,
                    actions=[pointcloud_to_scan, ekf, rviz_node]),
    ])
