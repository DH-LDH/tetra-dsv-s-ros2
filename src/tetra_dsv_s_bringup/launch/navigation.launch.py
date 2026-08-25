"""실기 Nav2 자율주행 — 구동부 + VLP-16 + IMU + EKF + map_server/AMCL/Nav2.

slam.launch.py 에서 slam_toolbox 를 빼고 map_server + AMCL + Nav2 스택
(controller/planner/behavior/bt_navigator/smoother/waypoint_follower/
velocity_smoother)을 더한 것입니다 (HANDOFF §10-18 2단계).

TF 소유권이 slam.launch.py 와 다릅니다 — 여기서 헷갈리면 로봇이 조용히
안 움직이거나 엉뚱한 델타로 움직입니다:
    map        -> odom             AMCL 이 냅니다 (slam_toolbox 대신)
    odom       -> base_footprint   EKF (robot_localization) — 동일
    base_*     -> 나머지 링크       robot_state_publisher (URDF) — 동일
slam_toolbox 와 AMCL 을 동시에 띄우면 map -> odom 의 부모가 둘이 되어
트리가 깨집니다. 반드시 하나만 켜세요.

속도 파이프라인 (nav2_bringup 표준과 동일):
    controller_server  -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel
tetra_drive_node 는 /cmd_vel 만 구독합니다. velocity_smoother 를 빼면 안
됩니다 — 가감속 제한이 사라져 좁은 방에서 급가속이 나옵니다.

    ros2 launch tetra_dsv_s_bringup navigation.launch.py

    # RViz 에서 반드시 먼저 할 것 — "2D Pose Estimate" 로 초기 위치 지정.
    # 안 하면 AMCL 파티클이 지도 전체에 흩어진 채로 시작합니다 (§9, §10-18).

사전 준비 — 매번 필요합니다 (slam.launch.py 와 동일):
  * 라이다 네트워크: nmcli con up lidar
  * 구동보드 USB 지연:
        sudo sh -c 'echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer'

첫 주행 안전 절차는 HANDOFF §10-18 3단계를 따르세요 — Nav2 Goal 은 1 m
앞부터, 비상정지에 손을 얹고, 바닥은 비운 상태로 시작합니다. 라이다가
바닥에서 1.048 m 라 책상다리·의자·박스는 지도에 없습니다 (§10-9).
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = get_package_share_directory("tetra_dsv_s_bringup")

    velodyne_params = os.path.join(bringup_share, "config", "velodyne_vlp16.yaml")
    imu_params = os.path.join(bringup_share, "config", "imu_gv7.yml")
    ekf_params = os.path.join(bringup_share, "config", "ekf.yaml")
    nav2_params = os.path.join(bringup_share, "config", "nav2_params.yaml")

    # 구동부 + robot_state_publisher — slam.launch.py 와 동일하게 검증된 것을 그대로 씁니다.
    drive = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_share, "launch", "drive.launch.py")),
    )

    # ---------------- VLP-16 (slam.launch.py 와 동일) ----------------
    velodyne_driver = Node(
        package="velodyne_driver",
        executable="velodyne_driver_node",
        name="velodyne_driver_node",
        parameters=[velodyne_params],
        output="screen",
    )
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

    # ---------------- IMU (slam.launch.py 와 동일) ----------------
    imu = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("microstrain_inertial_driver"),
                "launch", "microstrain_launch.py")),
        launch_arguments={"params_file": imu_params}.items(),
    )

    # ---------------- EKF (slam.launch.py 와 동일) ----------------
    ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="ekf_filter_node",
        parameters=[ekf_params],
        output="screen",
    )

    # ---------------- 지도 + AMCL ----------------
    # map -> odom 을 여기서부터 AMCL 이 소유합니다. yaml_filename 은
    # nav2_params.yaml 안에 tetra_lab_closed.yaml 로 하드코딩돼 있습니다
    # (0단계에서 철창 누수를 걷어낸 지도 — 원본 tetra_lab_main 아님, §10-17).
    map_server = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        parameters=[nav2_params],
        output="screen",
    )
    amcl = Node(
        package="nav2_amcl",
        executable="amcl",
        name="amcl",
        parameters=[nav2_params],
        output="screen",
    )
    lifecycle_manager_localization = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_localization",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": ["map_server", "amcl"],
            # Default 4.0s. §10-1/§10-11 already document wireless DDS
            # discovery jitter on the guest network; when a node's bond
            # takes longer than this to connect, the manager aborts bringup
            # on the spot and leaves every later node stuck 'inactive'
            # forever (HANDOFF §10-19, hit again 2026-08-24). 10s gives the
            # bond time to survive that jitter instead of needing a manual
            # `ros2 lifecycle set ... activate` recovery every session.
            "bond_timeout": 10.0,
        }],
    )

    # ---------------- Nav2 스택 ----------------
    # cmd_vel_nav -> velocity_smoother -> cmd_vel 파이프라인은 nav2_bringup
    # 의 navigation_launch.py 와 동일합니다. tetra_drive_node 는 cmd_vel
    # 만 구독하므로 이 리매핑을 빼면 가감속 제한 없이 원속도가 그대로 나갑니다.
    controller_server = Node(
        package="nav2_controller",
        executable="controller_server",
        name="controller_server",
        parameters=[nav2_params],
        output="screen",
        remappings=[("cmd_vel", "cmd_vel_nav")],
    )
    smoother_server = Node(
        package="nav2_smoother",
        executable="smoother_server",
        name="smoother_server",
        parameters=[nav2_params],
        output="screen",
    )
    planner_server = Node(
        package="nav2_planner",
        executable="planner_server",
        name="planner_server",
        parameters=[nav2_params],
        output="screen",
    )
    behavior_server = Node(
        package="nav2_behaviors",
        executable="behavior_server",
        name="behavior_server",
        parameters=[nav2_params],
        output="screen",
    )
    bt_navigator = Node(
        package="nav2_bt_navigator",
        executable="bt_navigator",
        name="bt_navigator",
        parameters=[nav2_params],
        output="screen",
    )
    waypoint_follower = Node(
        package="nav2_waypoint_follower",
        executable="waypoint_follower",
        name="waypoint_follower",
        parameters=[nav2_params],
        output="screen",
    )
    velocity_smoother = Node(
        package="nav2_velocity_smoother",
        executable="velocity_smoother",
        name="velocity_smoother",
        parameters=[nav2_params],
        output="screen",
        remappings=[("cmd_vel", "cmd_vel_nav"),
                   ("cmd_vel_smoothed", "cmd_vel")],
    )
    lifecycle_manager_navigation = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_navigation",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart": True,
            "node_names": ["controller_server", "smoother_server", "planner_server",
                          "behavior_server", "bt_navigator", "waypoint_follower",
                          "velocity_smoother"],
            # See lifecycle_manager_localization above - same wireless-jitter
            # bond timeout, same fix.
            "bond_timeout": 10.0,
        }],
    )

    return LaunchDescription([
        drive,
        velodyne_driver,
        velodyne_transform,
        velodyne_laserscan,
        imu,
        ekf,
        map_server,
        amcl,
        lifecycle_manager_localization,
        controller_server,
        smoother_server,
        planner_server,
        behavior_server,
        bt_navigator,
        waypoint_follower,
        velocity_smoother,
        lifecycle_manager_navigation,
    ])
