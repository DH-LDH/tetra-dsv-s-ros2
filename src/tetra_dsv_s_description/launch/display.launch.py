"""Show the TETRA-DSV-S model in RViz, with sliders for the drive wheels.

    ros2 launch tetra_dsv_s_description display.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")

    description_file = PathJoinSubstitution(
        [FindPackageShare("tetra_dsv_s_description"), "urdf", "tetra_dsv_s.urdf.xacro"]
    )
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", description_file]),
        value_type=str,
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "robot_description": robot_description,
            }
        ],
    )

    joint_state_publisher_gui = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(gui),
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=[
            "-d",
            PathJoinSubstitution(
                [FindPackageShare("tetra_dsv_s_description"), "rviz", "display.rviz"]
            ),
        ],
        parameters=[{"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="false"),
            DeclareLaunchArgument("gui", default_value="true"),
            robot_state_publisher,
            joint_state_publisher_gui,
            rviz,
        ]
    )
