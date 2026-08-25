# TETRA-DSV-S ROS 2

휴림네트웍스 TETRA-DSV-S AMR 의 SLAM / 자율주행 스택입니다.

- `src/tetra_dsv_s_description` — URDF. 시뮬과 실기가 **공유**합니다
- `src/tetra_dsv_s_bringup` — 실기용. 구동보드 드라이버, 센서 설정, launch
- `src/tetra_dsv_s_sim` — Gazebo Fortress 시뮬. 젯슨에는 받지 않습니다

**작업 이력 · 검증 수치 · 함정 목록은 전부 [HANDOFF.md](HANDOFF.md) 에 있습니다.**
이 README 는 "어떻게 돌리는가"만 답합니다. 왜 그런지가 궁금하면 HANDOFF 를 보세요.

## 현재 상태 (2026-08-13)

실기에서 **SLAM 까지 동작**합니다.

| | 상태 |
|---|---|
| 구동부 (RS-232, CN20) | ✅ `/wheel_odometry` 30 Hz |
| 라이다 Velodyne VLP-16 | ✅ `/velodyne_points`, `/scan` 9.8 Hz |
| IMU MicroStrain 3DM-GV7-AHRS | ✅ `/imu/data` 100 Hz |
| EKF (`odom→base_footprint`) | ✅ |
| slam_toolbox | ✅ 지도 생성 확인 |
| Nav2 자율주행 | ✅ 실기 첫 자율주행 성공 (2026-08-19). 목표 7개 전부 도달. 후진 미지원 (HANDOFF §10-18~§10-20) |
| RealSense D455 | ⬜ 하드웨어는 붙어 있고 드라이버 미설치 |

---

# 젯슨 실행법

Ubuntu 22.04 / ROS 2 Humble.

## 매번 해야 하는 것 (재부팅하면 초기화됩니다)

```bash
# 1. 라이다 네트워크
sudo nmcli con up lidar

# 2. FTDI 지연 낮추기 — 빼먹으면 오도메트리가 끊깁니다
sudo sh -c 'echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer'

# 3. CPU 전 코어 최대 클럭 고정 — 자율주행 중 CPU 부하로 액션/서비스
#    타임아웃이 나는 걸 줄여줍니다 (HANDOFF §10-23). 출력 없으면 정상.
sudo jetson_clocks

# 4. 원격 RViz 를 쓸 때만, 게스트 와이파이인 경우만 (아래 "노트북에서 보기" 참고)
fast-discovery-server --server-id 0 --udp-port 11811 &
```

> 2번은 udev 규칙에 넣어뒀지만 실제로는 잘 안 먹습니다. USB 를 다시 꽂을
> 때마다 16 으로 돌아가므로 그때마다 다시 실행하세요. 이유는 HANDOFF §10-3.
> 젯슨 자체를 안 껐다면(AMR 본체 전원만 껐다 켠 경우) 이 값은 유지되니
> 매번 다시 안 해도 됩니다 — `cat` 으로 먼저 확인하세요.
>
> **개인 핫스팟에 물려 있다면 3번(discovery server)은 보통 필요 없습니다**
> — 핫스팟은 멀티캐스트를 막지 않는 경우가 많습니다(2026-08-24 확인,
> HANDOFF §10-22). `ros2 multicast send`/`receive` 로 양방향 먼저
> 테스트해보고, 그냥 되면 아래 어디에도 `ROS_DISCOVERY_SERVER`,
> `FASTRTPS_DEFAULT_PROFILES_FILE` export 안 해도 됩니다.

## SLAM 실행

```bash
cd ~/tetra_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DISCOVERY_SERVER="10.101.111.244:11811"   # 원격 RViz 쓸 때만

ros2 launch tetra_dsv_s_bringup slam.launch.py
```

구동부 + VLP-16 + IMU + EKF + slam_toolbox 가 한 번에 뜹니다.
센서만 확인하고 지도 작성은 빼려면 `slam:=false`.

다른 터미널에서 조종 — **속도를 낮게**:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p speed:=0.15 -p turn:=0.3
```

> `i` 전진 / `,` 후진 / `j` 좌회전 / `l` 우회전 / `k` 정지.
> 큰 바퀴 두 개가 있는 쪽이 **앞**이고, 작은 캐스터 쪽이 뒤입니다.
> 키를 떼면 0.5 초 안에 멈춰야 합니다 (cmd_vel 워치독).

지도 저장:

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/tetra_map
```

> `use_sim_time` 을 붙이면 실패합니다 (HANDOFF §6-9).

## Nav2 자율주행 실행

지도는 이미 만들어져 있고(`~/maps/tetra_lab_closed.{pgm,yaml}`) `config/nav2_params.yaml`,
`launch/navigation.launch.py` 도 이미 있습니다. 매번 아래 순서 그대로 하면 됩니다.
왜 이 순서인지는 HANDOFF §10-18~§10-20 참고.

**젯슨:**

```bash
# 1. 매번 필요한 것 (재부팅하면 초기화됨) — 맨 위 "매번 해야 하는 것" 참고
sudo sh -c 'echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer'
nmcli con up lidar   # 보통 자동 연결되지만 안 붙어 있으면

# 2. discovery server — 이 터미널을 계속 띄워둘 것
fast-discovery-server --server-id 0 --udp-port 11811
```

**다른 터미널에서 Nav2 실행 — `ROS_DISCOVERY_SERVER` 를 반드시 이 셸에도 걸 것:**

```bash
cd ~/tetra_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
export ROS_DISCOVERY_SERVER="10.101.111.244:11811"

ros2 launch tetra_dsv_s_bringup navigation.launch.py
```

> discovery server 만 띄워두고 이 export 를 빼먹으면, 로봇 노드끼리는 잘 통신해서
> 젯슨 로컬에서는 멀쩡해 보이는데 노트북에서는 영원히 안 보입니다 (§10-1 함정 3번).

**노트북 (원격 RViz):**

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=25
export ROS_LOCALHOST_ONLY=0
export ROS_DISCOVERY_SERVER="10.101.111.244:11811"
export FASTRTPS_DEFAULT_PROFILES_FILE=~/tetra_dds/super_client.xml

ros2 daemon stop
sleep 3
ros2 topic list   # /map, /scan, /tf 등이 보여야 정상

rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

> `slam.rviz` 에는 `2D Pose Estimate`/`Nav2 Goal` 도구가 없습니다. 위
> `nav2_default_view.rviz` (apt 로 이미 깔려 있음)를 쓰세요.

> **`ros2 topic list` 가 처음엔 비어 나오거나 몇 개만 보일 수 있습니다.**
> 무선 + discovery server 경유라 왕복이 느립니다 (§10-11). 몇 초 간격으로
> 2~3 번 다시 쳐보세요. 그래도 계속 `/parameter_events`, `/rosout` 둘뿐이면
> 그때는 진짜 문제입니다 — 젯슨 쪽 discovery server 랑 `ROS_DISCOVERY_SERVER`
> export 여부부터 확인하세요.

**RViz 에서:**

1. **로봇이 지금 실제로 있는 자리**(어디든 상관없음, 옮길 필요 없음)를 지도에서
   찾아서 그 위치·방향 그대로 `2D Pose Estimate` 로 찍기. 이전 세션이 어디서
   끝났는지 절대 가정하지 말 것 — 특히 teleop/자율주행 중간에 전원을 끈
   뒤라면 로봇이 원점이 아닌 곳에 있을 수 있습니다 (HANDOFF §10-23).
2. `Navigation 2` 패널의 `Localization` 이 `active` 로 바뀌는지 확인.
3. `Nav2 Goal` 로 목표 지정 — **1 m 안쪽부터, 비상정지에 손 얹고, 바닥 비운 채로.**
   짧은 목표로 몇 번 성공한 뒤에만 먼 목표로 늘리세요 — 먼 목표(약 10 m)는
   젯슨 CPU 부하로 액션/서비스 타임아웃이 나서 반복 실패한 사례가 있습니다
   (HANDOFF §10-23, `sudo jetson_clocks` 로 완화 시도).
4. 목표를 보낸 뒤엔 `Navigation 2` 패널의 `Feedback` 을 꼭 확인하세요.
   `aborted`/`failed` 인데 모르고 다시 목표를 찍으면, 그때마다 새로 spin/backup
   복구가 돌아서 "혼자 왔다갔다 회전한다"처럼 보입니다 — 실은 매번 새 시도입니다.

> 지금 설정은 후진을 못 냅니다 (`min_vel_x: 0.0`) — 목표가 뒤에 있으면 제자리
> 회전 후 전진합니다. 앞 공간을 미리 비워두세요.

## 구동부만 띄우기

문제를 가릴 때 씁니다. 센서 없이 주행만 확인합니다.

```bash
ros2 launch tetra_dsv_s_bringup drive.launch.py
```

## ROS 없이 시리얼만 확인

구동보드가 안 움직일 때 **가장 먼저** 이걸 하세요. ROS 문제인지 시리얼
문제인지 30초면 갈립니다.

```bash
python3 src/tetra_dsv_s_bringup/scripts/probe_drive_board.py --port /dev/tetra_drive
```

> `통신 정상` 이 나와야 합니다. `--spin 3.0` 을 붙이면 3초간 제자리 회전을
> 시도합니다 — **바퀴를 띄운 뒤에만** 쓰세요.

---

# 노트북에서 보기 (원격 RViz)

젯슨은 계산만 하고, 지도는 노트북 RViz 로 봅니다.

## ⚠ 사내 게스트 와이파이에서는 그냥은 안 됩니다

게스트망이 UDP 멀티캐스트를 막아서 ROS 2 디스커버리가 죽습니다. `ping` 은
되는데 `ros2 topic list` 에 `/parameter_events`, `/rosout` 둘만 보이면 이
증상입니다. Fast DDS Discovery Server 로 우회합니다. 자세한 진단·원리는
HANDOFF §10-1.

**노트북 준비 (한 번만)** — `~/tetra_dds/super_client.xml` 을 만듭니다:

```xml
<?xml version="1.0" encoding="UTF-8" ?>
<dds xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
  <profiles>
    <participant profile_name="super_client_profile" is_default_profile="true">
      <rtps><builtin><discovery_config>
        <discoveryProtocol>SUPER_CLIENT</discoveryProtocol>
        <discoveryServersList>
          <RemoteServer prefix="44.53.00.5f.45.50.52.4f.53.49.4d.41">
            <metatrafficUnicastLocatorList><locator><udpv4>
              <address>10.101.111.244</address><port>11811</port>
            </udpv4></locator></metatrafficUnicastLocatorList>
          </RemoteServer>
        </discoveryServersList>
      </discovery_config></builtin></rtps>
    </participant>
  </profiles>
</dds>
```

**RViz 실행** — 젯슨에서 discovery server 와 `slam.launch.py` 가 떠 있는 상태에서:

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=25
export ROS_LOCALHOST_ONLY=0
export ROS_DISCOVERY_SERVER="10.101.111.244:11811"
export FASTRTPS_DEFAULT_PROFILES_FILE=~/tetra_dds/super_client.xml

ros2 daemon stop      # ← 이걸 빼먹으면 위 설정이 하나도 반영되지 않습니다
sleep 3
ros2 topic list       # /map /scan /robot_description 이 보여야 정상

rviz2 -d ~/tetra_rviz/slam.rviz
```

> `ros2 daemon stop` 이 핵심입니다. 데몬이 이전 디스커버리 설정을 캐시하고
> 있어서, 환경변수만 바꾸면 아무 일도 일어나지 않습니다.
>
> RViz 설정은 `src/tetra_dsv_s_description/rviz/slam.rviz` 를 노트북으로
> 복사해 쓰면 됩니다. `/velodyne_points` 는 초당 30만 점이라 무선으로
> 구독하지 마세요 — 기본값으로 꺼져 있습니다.

---

# 처음 설치할 때

이미 젯슨에 구축돼 있습니다. 새 기기에 다시 올릴 때만 보세요.

## 클론

저장소가 private 이라 GitHub 인증이 먼저입니다.

```bash
sudo apt update && sudo apt install -y gh
gh auth login    # GitHub.com → HTTPS → "Authenticate Git with your credentials? Yes"
```

> `cli.github.com` apt 저장소를 추가하지 마세요 — 이 네트워크에서 차단됩니다
> (HANDOFF §1). 우분투 저장소의 gh 로 충분합니다.

시뮬 패키지는 받지 않습니다. Gazebo 의존성을 젯슨에 끌고 오지 않기 위해서입니다.

```bash
git clone --filter=blob:none --sparse \
  https://github.com/DH-LDH/tetra-dsv-s-ros2.git ~/tetra_ws
cd ~/tetra_ws
git sparse-checkout set src/tetra_dsv_s_description src/tetra_dsv_s_bringup \
  README.md HANDOFF.md CLAUDE.md
git config user.name "DH-LDH"
git config user.email "ekgks3451@gmail.com"
```

## 의존 패키지

```bash
sudo apt update
sudo apt install -y \
  ros-humble-velodyne \
  ros-humble-microstrain-inertial-driver \
  ros-humble-robot-localization \
  ros-humble-slam-toolbox \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-xacro \
  ros-humble-teleop-twist-keyboard \
  python3-serial python3-colcon-common-extensions
```

> IMU 는 **MicroStrain 3DM-GV7-AHRS** 입니다. 예전 문서의 `iahrs_driver_ros2`
> 는 오기였습니다 — clone 할 것 없이 위 apt 패키지로 끝납니다 (HANDOFF §10-2).
>
> `pointcloud_to_laserscan` 은 필요 없습니다. `velodyne_laserscan` 이 `/scan`
> 을 바로 냅니다.

## 시리얼 권한

```bash
sudo usermod -aG dialout $USER
newgrp dialout            # 재로그인 대신 지금 세션에 반영
```

## udev 규칙 — 포트 고정

`/dev/ttyUSB0` 번호는 부팅마다 바뀝니다. 구동보드를 `/dev/tetra_drive` 로
고정합니다.

```bash
sudo cp src/tetra_dsv_s_bringup/udev/99-tetra-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/tetra_drive
```

> 어댑터를 교체했다면 규칙 파일의 serial 값을 바꿔야 합니다:
> `udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|serial'`

## 라이다 네트워크

VLP-16 은 `192.168.1.201` 에서 브로드캐스트합니다. 젯슨이 같은 서브넷에
있어야 커널이 패킷을 올려줍니다.

```bash
sudo nmcli con add type ethernet ifname enP8p1s0 con-name lidar \
  ipv4.method manual ipv4.addresses 192.168.1.150/24 \
  ipv4.never-default yes ipv6.method ignore
```

> `ipv4.never-default yes` 가 중요합니다. 없으면 라이다망이 기본 경로를
> 가로채 인터넷이 끊깁니다.
>
> `ip addr add` 로 수동 지정하면 NetworkManager 가 곧바로 지웁니다. 반드시
> 위처럼 프로파일로 등록하세요.

## 빌드

```bash
cd ~/tetra_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

# 안 될 때

| 증상 | 먼저 볼 것 |
|---|---|
| `BV round trip failed: short BV reply` | ① 로봇 전원 ② 비상정지 ③ `latency_timer` — 이 순서. 프로토콜 코드는 마지막 (HANDOFF §10-3, §10-4) |
| `emergency stop engaged at the robot` | 대부분 버튼과 무관한 stale 에러 래치입니다 — `tetra_drive_node` 가 기동 시 `CG`(Error Reset) 를 자동으로 보내서 이제 보통 저절로 풀립니다 (HANDOFF §10-21). 그래도 계속 뜨면 그때는 진짜 버튼을 확인하세요 |
| 노트북에서 토픽이 2개만 보임 | 게스트 와이파이 멀티캐스트 차단. `ros2 daemon stop` 했는지 확인 (HANDOFF §10-1). 핫스팟이면 이 문제 자체가 없을 수 있음 (§10-22) |
| `/scan` 이 10 Hz 가 안 나옴 | 젯슨의 크롬·VSCode 를 닫으세요. CPU 부족입니다. `sudo jetson_clocks` 도 도움됨 |
| RViz 에 지도가 안 보임 | ① Map 디스플레이의 Durability 가 `Transient Local` 인지 ② TF 는 정상인데(`Global Status: Ok`) `Map` 만 `No map received` 면 `ros2 lifecycle set /map_server deactivate` 후 `activate` 로 재발행 유도 (HANDOFF §10-11) |
| SLAM 이 조용히 아무것도 안 함 | 라이다 `frame_id` 가 `velodyne_link` 인지 (`velodyne` 이면 TF 를 못 찾습니다) |
| `Nav2 Goal` 줘도 반응 없음, RViz 에 `Send goal call failed` | `ros2 lifecycle get /bt_navigator` 로 `inactive` 인지 확인. launch 중 bond 타임아웃으로 활성화가 중간에 멈춘 것. `bond_timeout` 을 10 초로 올려둬서 이제 잘 안 나지만(HANDOFF §10-22), 그래도 나면 §10-21 의 수동 activate 순서로 복구 |
| 먼 거리(≈10 m) Nav2 Goal 이 계속 spin/backup 반복하다 실패 | 젯슨 CPU 과부하 의심. `sudo jetson_clocks`, VSCode/Claude Code 종료 후 재시도 (HANDOFF §10-23) |
| RobotModel 에 `No transform from [rear_caster_link]` | 2026-08-24 에 고쳐짐 (`tetra_drive_node` 가 캐스터 조인트도 발행) — 여전히 뜨면 재빌드 안 된 것 |
