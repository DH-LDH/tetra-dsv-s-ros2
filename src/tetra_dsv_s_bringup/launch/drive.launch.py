"""구동부만 띄웁니다 — 실기 첫 통전 때 쓰는 최소 구성.

센서(VLP-16, IMU)나 SLAM 은 여기 없습니다. 처음부터 전부 띄우면 뭐가 안 되는지
못 가리기 때문입니다. 전부 필요하면 slam.launch.py 를 쓰세요 — 구동부 + 라이다
+ IMU + EKF + slam_toolbox 를 한 번에 띄웁니다.
(IMU 는 MicroStrain 3DM-GV7-AHRS 입니다. 예전 문서의 iAHRS 는 오기였습니다.)

odom -> base_footprint 도 아직 아무도 발행하지 않습니다. 그 변환은 EKF 가
소유하고(시뮬과 동일), EKF 는 IMU 가 붙은 뒤에 들어옵니다. 그러니 이 단계에서
RViz 를 열면 로봇이 제자리에 서 있는 게 정상입니다 — 바퀴 관절만 움직입니다.
확인할 것은 /wheel_odometry 의 값이지 RViz 속 위치가 아닙니다.

    # 1. ROS 없이 시리얼부터 (권장)
    ros2 run tetra_dsv_s_bringup probe_drive_board.py --port /dev/ttyUSB0

    # 2. 구동부만
    ros2 launch tetra_dsv_s_bringup drive.launch.py

    # 3. 다른 터미널에서 조종. 바퀴를 띄운 채로 먼저 확인할 것
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
        --ros-args -p speed:=0.10 -p turn:=0.3
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

# ======================================================================
#  조정 파라미터 — 이 블록만 손대면 됩니다
# ======================================================================

# udev 규칙(udev/99-tetra-serial.rules)이 만드는 심볼릭 링크.
# 규칙을 아직 안 넣었으면 port:=/dev/ttyUSB0 으로 덮어쓰세요.
DEFAULT_PORT = "/dev/tetra_drive"

DEFAULT_RATE_HZ = 30.0        # 벤더 ROS 1 노드와 동일

# 실측 완료 (2026-08-12, 줄자): 좌우 바퀴 접지면 중심 간 거리 ~377mm.
# 벤더 드라이버 WHEEL_DISTANCE(0.377) 및 URDF 쪽과 일치. HANDOFF §2-(1) 참고.
DEFAULT_WHEEL_SEPARATION = 0.377
DEFAULT_WHEEL_RADIUS = 0.1015

# 첫 통전에는 낮게. 3 m 방에서 1.0 m/s 는 위험합니다.
DEFAULT_MAX_LINEAR = 0.5      # m/s
DEFAULT_MAX_ANGULAR = 1.0     # rad/s

# cmd_vel 이 끊기면 이 시간 뒤 정지. 실기에서 이걸 키우지 마세요.
DEFAULT_CMD_TIMEOUT = 0.5     # s


def generate_launch_description():
    description_share = get_package_share_directory("tetra_dsv_s_description")
    robot_file = os.path.join(description_share, "urdf", "tetra_dsv_s.urdf.xacro")

    # use_gazebo:=false — 실기에는 Gazebo 플러그인이 들어가면 안 됩니다.
    robot_description = {
        "robot_description": ParameterValue(
            Command([FindExecutable(name="xacro"), " ", robot_file,
                     " use_gazebo:=false"]),
            value_type=str,
        )
    }

    # joint_states 는 드라이버가 직접 냅니다. Gazebo 의 JointStatePublisher
    # 자리를 그대로 대체하므로 robot_state_publisher 설정은 시뮬과 동일합니다.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[robot_description, {"use_sim_time": False}],
        output="screen",
    )

    drive = Node(
        package="tetra_dsv_s_bringup",
        executable="tetra_drive_node",
        name="tetra_drive",
        parameters=[{
            "port": LaunchConfiguration("port"),
            "rate_hz": LaunchConfiguration("rate_hz"),
            "wheel_separation": LaunchConfiguration("wheel_separation"),
            "wheel_radius": LaunchConfiguration("wheel_radius"),
            "max_linear_velocity": LaunchConfiguration("max_linear_velocity"),
            "max_angular_velocity": LaunchConfiguration("max_angular_velocity"),
            "cmd_vel_timeout": LaunchConfiguration("cmd_vel_timeout"),
            "use_sim_time": False,
        }],
        output="screen",
    )

    return LaunchDescription([
        DeclareLaunchArgument("port", default_value=DEFAULT_PORT),
        DeclareLaunchArgument("rate_hz", default_value=str(DEFAULT_RATE_HZ)),
        DeclareLaunchArgument("wheel_separation",
                              default_value=str(DEFAULT_WHEEL_SEPARATION)),
        DeclareLaunchArgument("wheel_radius",
                              default_value=str(DEFAULT_WHEEL_RADIUS)),
        DeclareLaunchArgument("max_linear_velocity",
                              default_value=str(DEFAULT_MAX_LINEAR)),
        DeclareLaunchArgument("max_angular_velocity",
                              default_value=str(DEFAULT_MAX_ANGULAR)),
        DeclareLaunchArgument("cmd_vel_timeout",
                              default_value=str(DEFAULT_CMD_TIMEOUT)),
        robot_state_publisher,
        drive,
    ])
