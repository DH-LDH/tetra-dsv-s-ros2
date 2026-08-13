"""실기 SLAM 전체 스택 — 구동부 + VLP-16 + IMU + EKF + slam_toolbox.

drive.launch.py 가 구동부만 띄우는 최소 구성이라면, 이쪽은 지도를 만드는
데 필요한 전부입니다. 2026-08-13 젯슨에서 각 계통을 따로 살려본 뒤 하나로
묶은 것입니다.

TF 소유권 (여기서 어긋나면 SLAM 이 조용히 멈춥니다):
    map        -> odom             slam_toolbox
    odom       -> base_footprint   EKF (robot_localization)
    base_*     -> 나머지 링크       robot_state_publisher (URDF)
드라이브 노드는 TF 를 내지 않습니다. 시뮬과 같은 구조입니다.

    ros2 launch tetra_dsv_s_bringup slam.launch.py

    # 다른 터미널에서 조종
    ros2 run teleop_twist_keyboard teleop_twist_keyboard \
        --ros-args -p speed:=0.15 -p turn:=0.3

    # 다 돌았으면 지도 저장 (use_sim_time 을 붙이면 실패합니다)
    ros2 run nav2_map_server map_saver_cli -f ~/maps/my_map

사전 준비 — 매번 필요합니다:
  * 라이다 네트워크: 젯슨이 192.168.1.x 에 있어야 합니다.
        nmcli con up lidar
  * 구동보드 USB 지연: 재부팅/재연결 때마다 16 으로 돌아갑니다.
        sudo sh -c 'echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer'
    16 이면 BV 응답이 잘려 들어와 오도메트리가 끊깁니다.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("tetra_dsv_s_bringup")

    velodyne_params = os.path.join(bringup_share, "config", "velodyne_vlp16.yaml")
    imu_params = os.path.join(bringup_share, "config", "imu_gv7.yml")
    ekf_params = os.path.join(bringup_share, "config", "ekf.yaml")
    slam_params = os.path.join(bringup_share, "config", "slam_toolbox.yaml")

    # 구동부 + robot_state_publisher 는 이미 검증된 launch 를 그대로 씁니다.
    drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "drive.launch.py")),
    )

    # ---------------- VLP-16 ----------------
    # velodyne 패키지가 주는 launch 는 자기 config 를 물고 들어와서 frame_id
    # 를 덮어쓸 수 없습니다. 그래서 세 노드를 직접 띄웁니다.
    velodyne_driver = Node(
        package="velodyne_driver",
        executable="velodyne_driver_node",
        name="velodyne_driver_node",
        parameters=[velodyne_params],
        output="screen",
    )
    # calibration 은 절대 경로여야 합니다 - 파일명만 주면 노드가
    # "Failed to open calibration file" 로 즉시 죽습니다. velodyne 패키지의
    # 자체 launch 도 같은 이유로 여기서 경로를 만들어 넘깁니다.
    vlp16_calibration = os.path.join(
        get_package_share_directory("velodyne_pointcloud"),
        "params", "VLP16db.yaml")
    velodyne_transform = Node(
        package="velodyne_pointcloud",
        executable="velodyne_transform_node",
        name="velodyne_transform_node",
        parameters=[velodyne_params, {"calibration": vlp16_calibration}],
        output="screen",
    )
    velodyne_laserscan = Node(
        package="velodyne_laserscan",
        executable="velodyne_laserscan_node",
        name="velodyne_laserscan_node",
        parameters=[velodyne_params],
        output="screen",
    )

    # ---------------- IMU ----------------
    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("microstrain_inertial_driver"),
                "launch", "microstrain_launch.py")),
        launch_arguments={"params_file": imu_params}.items(),
    )

    # ---------------- EKF ----------------
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        parameters=[ekf_params],
        output="screen",
    )

    # ---------------- SLAM ----------------
    slam = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        parameters=[slam_params],
        output="screen",
        condition=IfCondition(LaunchConfiguration("slam")),
    )

    return LaunchDescription([
        # 센서만 확인하고 싶을 때 slam:=false 로 지도 작성을 뺄 수 있습니다.
        DeclareLaunchArgument("slam", default_value="true"),
        drive,
        velodyne_driver,
        velodyne_transform,
        velodyne_laserscan,
        imu,
        ekf,
        slam,
    ])
