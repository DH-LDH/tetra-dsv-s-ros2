# TETRA-DSV-S ROS 2 인수인계

> **이 문서의 독자는 Claude입니다.** 사용자용 설명서가 아니라, 다른 맥락 없이
> 이 파일 하나로 작업을 이어받기 위한 브리핑입니다. 설명조로 늘이지 말고
> 검증된 수치 / 결정과 그 근거 / 무증상 함정을 밀도 있게 유지할 것.
> 작업을 끝낼 때 이 파일 갱신까지가 그 작업의 일부입니다. 틀린 내용을 발견하면
> 반드시 고칠 것 — 여기 남은 오류는 다음 세션을 그대로 오도합니다.
> 마지막 갱신: 2026-08-12
> — 문서를 실제 파일과 대조 검증 (§7 launch 인자, §8 iahrs/라이다 정정)
> — **맵 2개 작성·저장 + AMCL 운영 모드 검증 완료** (§4)
> — **local costmap clearing 결함 발견 후 수정 완료** (§8 맨 앞)
> — 실기 테스트 공간 정보 확보, 동적 장애물은 실기 이후로 (§8-C)

---

## 0. 이 프로젝트가 뭔가

`scout-ur3e-ros2`(AgileX Scout + UR3e) 워크스페이스를 베이스로, **모바일 로봇만
휴림네트웍스 TETRA-DSV-S로 교체**한 ROS 2 스택입니다. 전체 로직과 launch 구조는
원본을 미러링합니다.

**로봇팔은 다루지 않습니다.** TETRA-DSV-S에는 매니퓰레이터가 없으므로 UR3e /
Robotiq / D435i 서브트리를 애초에 가져오지 않았습니다. 주석 처리할 대상이 없습니다.

**최종 목표**: 젯슨 오린 나노(Ubuntu 22.04 / Humble)에 얹어 AMR SLAM + 자율주행.

---

## 1. 환경 (중요 — 여기서 여러 번 막혔음)

### 개발은 Docker 컨테이너 안에서

호스트 PC는 **Ubuntu 24.04 / ROS 2 Jazzy / Gazebo Harmonic**이지만,
배포 대상은 **젯슨 = Ubuntu 22.04 / Humble**입니다. Fortress와 Harmonic은
Gazebo 플러그인 이름이 다릅니다(`ignition-gazebo-*` vs `gz-sim-*`). 두 번 포팅하지
않으려고 **Humble + Fortress 컨테이너**에서 개발합니다.

```bash
cd ~/tetra_ws/tetra-dsv-s-ros2
./docker/run.sh                  # 대화형 셸
./docker/run.sh '명령'            # 한 번에 실행
```

이미지: `tetra-humble:dev` (`docker/Dockerfile`, `osrf/ros:humble-desktop` 기반)

### 이미 해결된 환경 이슈 — 다시 겪지 말 것

| 증상 | 원인 / 해결 |
|---|---|
| `docker` permission denied | 세션이 `docker` 그룹 추가 이전에 시작됨. `run.sh`가 `sg docker -c`로 자동 우회. 로그아웃/로그인하면 깔끔해짐 |
| `nvidia.github.io` 타임아웃 | **이 네트워크는 GitHub Pages(185.199.x.x) 대역이 차단됨.** `github.com` / `raw.githubusercontent.com`은 정상. GitHub Pages 기반 apt 저장소는 `raw.githubusercontent.com/<org>/<repo>/gh-pages/...`로 우회 |
| `gh` 설치가 `cli.github.com`에서 멈춤 | 같은 차단의 다른 얼굴. **공식 안내대로 `cli.github.com/packages` apt 저장소를 추가하지 말 것.** 우분투 자체 저장소에 gh 2.45.0이 있음 → `sudo apt install gh` 로 끝 |
| Gazebo가 느림 | `--gpus all`만으로는 OpenGL이 llvmpipe로 폴백. **`NVIDIA_DRIVER_CAPABILITIES=graphics,...` 필수.** `run.sh`에 반영됨. 확인: `./docker/run.sh 'glxinfo -B \| grep "OpenGL renderer"'` → NVIDIA 나와야 정상 |

---

## 2. 공식 제원 (매뉴얼 + 벤더 저장소)

**출처**
- `TETRA-DSV-S Operation Manual v231019` (사용자 보유 PDF)
- <https://github.com/Hyulim-Group/TETRA-DSV-S> — 로컬 사본 `~/tetra_ws/TETRA-DSV-S`

| 항목 | 값 | 출처 |
|---|---|---|
| 외형 | 485 × 430 × 380 mm | 표 1-1 |
| 지상고 | **32.5 mm** | 그림 2-2 |
| 회전반경 | 714 mm | 표 1-1 |
| 자중 / 적재 | 28 kg / 30 kg | 표 1-1 |
| 구동 | 2륜 차동 + 후방 캐스터, 100W PMSM | 표 1-1 |
| **바퀴 직경** | **203 mm** (반경 0.1015) | 표 4-3 Para4 |
| **바퀴간 거리** | **438 mm** | 표 4-3 Para5 |
| 최대속도 | 스펙 2.0 m/s, 펌웨어 Para8 = 1500 mm/s | 표 1-1 / 4-3 |
| 통신 | **RS-232C 115200bps** (CAN 아님!) | 표 3-5 |

### 반드시 알아야 할 두 가지

**(1) ⚠ 휠 트랙 — 이 결론은 재검토가 필요합니다 (2026-08-12)**

원래 판단은 "URDF의 0.378은 메쉬 장착 오프셋이고 펌웨어는 Para5 = 438로
변환하니 **0.438을 쓴다**"였고, 시뮬은 지금 0.438로 돌아가고 있습니다.
그런데 벤더 **실기 드라이버 소스**를 열어보니 근거가 흔들립니다.

| 출처 | 휠 트랙 | 휠 반경 |
|---|---|---|
| 매뉴얼 표 4-3 Para5 / Para4 | **0.438** | 0.1015 |
| 벤더 URDF (`wheel_offset_y` 0.189) | 0.378 | 0.100 |
| **벤더 드라이버 `tetraDS.cpp:34`** | **0.377** | **0.1015** |
| 우리 스택 (현재) | 0.438 | 0.1015 |

의심스러운 점 셋:

1. **드라이버는 반경만 매뉴얼을 따랐습니다.** `WHEEL_RADIUS 0.1015`는 Para4와
   정확히 일치하는데 `WHEEL_DISTANCE 0.377`은 Para5와 다릅니다. 매뉴얼을 보고
   쓴 사람이 트랙만 다른 값을 넣었다는 뜻이라, 0.377이 실측일 가능성이 큽니다.
2. **차체 폭이 430 mm입니다** (표 1-1). 트랙 438이면 바퀴가 양쪽으로 4 mm씩
   삐져나옵니다. 0.377/0.378은 안쪽에 들어옵니다.
3. **속도 제어에서 펌웨어는 트랙을 쓰지 않습니다.** `set_velocity(l, r)`는
   좌·우 바퀴 속도를 **mm/s로 직접** 받습니다. cmd_vel → 바퀴속도 변환은
   전부 호스트가 합니다(`tetraDS.cpp:435 speed_to_diwheel_rpm`). 즉 우리가
   쓰는 값은 **실제 물리 트랙**이어야 하고, Para5는 펌웨어 자체 오도메트리
   (`read_odometry`의 X/Y/deg)와 위치모드에만 관여합니다.

→ **실기가 오면 줄자로 좌우 바퀴 접지면 중심 간 거리를 재는 게 유일한 결론.**
0.377이 맞다면 지금 시뮬의 yaw rate는 16% 과대평가된 상태이고, 실기 드라이버에
0.438을 넣으면 로봇이 명령보다 **덜 회전**합니다.

측정 전까지: 시뮬은 0.438 유지(재튜닝 비용이 큼), **실기 드라이버는 벤더와
같은 0.377로 시작**하고 실측 후 양쪽을 일치시킬 것.

**(2) `base_link`는 차체 중심이 아니라 구동축에 있다**
차체는 x ∈ [−0.318, +0.1675] (기하 중심이 base_link보다 75.25 mm 뒤).
→ inscribed 0.1675 / circumscribed 0.3839, **비율 2.29배**.
이 비대칭이 Nav2 설정 전반에 영향을 줍니다 (§5 참고).

**(3) CAN 아님** — scout은 `can0` + `ugv_sdk`를 쓰지만 TETRA는 RS-232입니다.
`scout_bringup`의 `can_interface` 인자를 절대 그대로 가져오지 말 것.

---

## 3. 현재 파일 구조

```
~/tetra_ws/
├── tetra-dsv-s-ros2/          ← 우리 워크스페이스 (2.9M, git 미적용)
│   ├── docker/Dockerfile, run.sh
│   ├── HANDOFF.md             ← 이 파일
│   └── src/
│       ├── tetra_dsv_s_description/   URDF/xacro
│       └── tetra_dsv_s_sim/           Gazebo + SLAM + Nav2
├── TETRA-DSV-S/               ← 벤더 원본 (54M, 남겨둘 것)
│   └── tetraDS/src/dssp_rs232_drive_module.c, tetraDS.cpp
│                              ← 실기 드라이버 작성 시 유일한 레퍼런스
└── scout-ur3e-ros2/           ← 베이스 원본 (723M, 삭제 가능)
                                 남은 참조는 전부 주석뿐, 런타임 의존 0건
```

---

## 4. 검증 완료된 것

### URDF
- xacro 확장 / `check_urdf` / Fortress plugin `.so` 실존까지 확인
- 프레임 높이 실측 = 매뉴얼 일치: 마운트 바닥 380.0 mm, 차체 밑면 32.5 mm
- VLP-16 광학중심 **지면 위 1.0477 m**

### 센서 마스트 (사용자 지정)
- 크기 **전후 130 × 좌우 200 × 높이 630 mm**
- 위치 `base_link` 기준 x=0, y=0 (`sensor_mast.xacro`의 `mast_offset_x/y`로 조정 가능)
  - 주의: x=0은 구동축이라 **차체 기하 중심보다 75 mm 앞**. 중심에 두려면 −0.07525
- 마스트 중간 300 mm에 카메라 — **TF 프레임만** 정의, 드라이버/센서 없음

### SLAM (사용자 직접 확인 완료)
- 맵 150×120 @ 2 cm = **3.00 × 2.40 m** (방과 일치)
- 벽 두께 **2셀(4 cm)** — 이중벽 없음, 정합 양호
- 내부 폭 실측 2.420 m (실제 2.40)
- 원시 휠 오도메트리가 3바퀴에 1.95 m 틀어진 상태에서 나온 결과
- **맵 저장 완료 (2026-08-12).** 두 world 모두 Nav2 목표를 연속으로 보내
  두 바퀴 돌린 뒤 저장했습니다. 매핑 주행 목표 13개 전부 SUCCEEDED, abort 0회.

| 파일 | world | 크기 | origin |
|---|---|---|---|
| `maps/room.yaml` | loop_room | 151×122 @ 2 cm = 3.02 × 2.44 m | [−0.51, −1.21] |
| `maps/empty_room.yaml` | empty_room | 151×122 @ 2 cm = 3.02 × 2.44 m | [−0.51, −1.22] |

> `room.yaml`이 loop_room인 이유: `navigation.launch.py`의 `map` 기본값이
> `maps/room.yaml`이고 `world` 기본값이 loop_room이라, 인자 없이 띄웠을 때
> 짝이 맞아야 하기 때문입니다. 실제 방과 같은 건 `empty_room` 쪽입니다.

### AMCL 운영 모드 (2026-08-12 검증 완료)

`navigation.launch.py`로 저장된 맵 + AMCL 경로를 양쪽 world에서 확인했습니다.
`map_server` / `amcl` 둘 다 `active` 도달, 주행 목표 전부 SUCCEEDED.

| world | 위치추정 오차 (AMCL 추정 vs ground truth) | 목표 실제 도달 오차 |
|---|---|---|
| loop_room | **0.099 m** | **0.043 m** |
| empty_room | **0.085 m** | **0.064 m** |

§4 SLAM 모드의 fused 0.23 m보다 좋습니다. 맵이 이미 있으니 당연한 결과지만,
**빈 직사각형 방에서도 AMCL이 발산하지 않는다**는 건 확인해둘 값어치가 있었습니다
(특징 없는 대칭 공간은 AMCL이 어긋나기 쉬운 대표 사례).

- 초기 위치는 자동 설정되지 않습니다. `nav2_params.yaml`의 amcl에
  `set_initial_pose`가 없어서 `/initialpose`를 직접 쏴야 움직입니다.
  RViz의 "2D Pose Estimate" 또는 아래 명령:
  ```bash
  ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
    "{header: {frame_id: map}, pose: {pose: {position: {x: 0.0, y: 0.0, z: 0.0}, \
      orientation: {z: 0.0, w: 1.0}}}}"
  ```
- **map 프레임 = world 프레임 + (1.0, 0)** 입니다. map 원점이 로봇 스폰 위치
  (world −1.0, 0)에 잡히기 때문. 목표 좌표를 world 기준으로 생각했다면 x에 1.0을
  더해야 합니다. 위 오차 수치도 이 오프셋을 적용해 계산한 값이라 수 cm의
  불확실성이 섞여 있습니다.

### 자율주행
| | DWB (폐기) | **RPP (현재)** |
|---|---|---|
| 목표 도달 | 실패 (0.21 m 부족) | **SUCCEEDED 13~18초** |
| `Failed to make progress` | 15초마다 | **0회** |
| 기둥 최소 여유 | — | **+0.196 m** (inflation 0.39 기준) |

### EKF 융합 효과 (동일 조건 비교)
| | 최종 실제 위치 오차 | Nav2 판정 |
|---|---|---|
| `wheel` | **0.66 m** | SUCCEEDED ← **거짓 양성** |
| `fused` | **0.23 m** | 정직하게 보고 |

> 위치추정이 부정확하면 Nav2는 도착하지 않고도 도착했다고 보고합니다.
> 실기에서 훨씬 위험한 실패 양상이라 `fused`를 기본값으로 뒀습니다.

---

## 5. 현재 설정과 그 근거

> **어디를 만져야 하는지** (2026-08-12 정리): 조정 가능한 값은 전부 각 파일
> **상단의 `조정 파라미터` 블록**에 모아뒀습니다. 아래 표는 근거 설명이고,
> 실제로 바꿀 때는 그 블록을 보면 됩니다.
>
> | 파일 | 상단에 뭐가 있나 |
> |---|---|
> | `tetra_dsv_s_base.xacro` | 차체 치수, 휠, 캐스터, 질량, 지상고, 속도 한계 |
> | `sensor_mast.xacro` | 마스트 크기·위치, VLP-16, 카메라 높이 |
> | `tetra_dsv_s.gazebo.xacro` | 마찰, 가감속, 라이다/IMU 노이즈·발행률 |
> | `simulation.launch.py` | world, 스폰 포즈, odometry_source, 기동 지연 |
> | `slam.launch.py` / `navigation.launch.py` | world·map 기본값, nav2 on/off |
> | `nav2_params.yaml` / `slam_toolbox.yaml` | **색인만** (아래 참고) |
>
> YAML 두 개는 값을 상단으로 못 옮깁니다 — ROS 2 파라미터는
> `<노드>/ros__parameters/<키>` 구조가 고정이라 옮기면 로딩이 깨집니다.
> 대신 파일 맨 위에 **어느 키를 만져야 하는지 색인**을 달아뒀습니다.
> `inflation_radius`와 `footprint`는 local/global costmap **양쪽에 중복**되니
> 반드시 같이 고칠 것.

```
odometry_source   fused          (휠 vx + IMU yaw rate, robot_localization EKF)
컨트롤러          RegulatedPurePursuit
  lookahead_dist  0.28           (기본 0.6은 3m 방에서 코너를 잘라먹음)
  desired_linear_vel 0.22
  충돌검사 horizon 1.8초          (차체 길이 커버)
전역 플래너       NavfnPlanner
inflation_radius  0.39           (= circumscribed 0.3839)
footprint         [[0.1675,0.215],[0.1675,-0.215],[-0.318,-0.215],[-0.318,0.215]]
slam resolution   0.02
```

**`inflation_radius = 0.39`의 의미**: NavFn은 로봇을 점으로 취급하고 footprint를
전혀 보지 않습니다. 팽창 반경이 유일한 안전장치입니다. circumscribed 이상으로
잡으면 "비용 0 셀에 중심이 있으면 어떤 각도로도 충돌 불가"가 성립합니다.
(단 NavFn은 비용>0인 셀도 통과 가능하므로 절대 보장은 아님)

**시행착오 기록**: 0.22 → 0.20(너무 낮춰 **기둥에 −0.0002 m 충돌**) → 0.28(+0.099)
→ 0.39(+0.196). 0.20이 위험했던 이유는 비용 0 경계가 기둥 중심에서 0.35 m인데
직진 통과에만 0.365 m가 필요했기 때문.

---

## 6. 무증상 함정 모음 (다시 겪지 말 것)

전부 **에러 메시지 없이** 실패하던 것들입니다.

1. **fixed joint 병합이 `<gazebo reference>`를 삼킨다**
   sdformat URDF 임포터가 fixed joint 자식 링크를 부모에 병합 → 그 링크를 지목한
   `<gazebo reference="X">`가 조용히 버려짐. 캐스터 마찰 0.05가 이렇게 사라져
   기본값 1.0이 되어 **로봇이 전혀 회전하지 못했음**.
   확인법: `ign sdf -p robot.urdf | grep "<link name"` — 목록에 없으면 죽은 설정.
   해결: 해당 조인트를 non-fixed로.

2. **차체 collision이 지면에 닿으면 회전이 막힌다**
   직진은 되고 회전만 안 되는 증상이라 원인 파악이 어려움. 지상고 32.5 mm 필수.

3. **`gpu_lidar`는 토픽을 두 개 발행한다**
   `<topic>X` → `X`는 LaserScan, `X/points`가 PointCloudPacked.
   `X`를 PointCloud2로 브리지하면 타입 불일치로 **아무것도 안 나옴**.

4. **ROS 2 파라미터 시퀀스는 타입이 균질해야 한다**
   `ekf.yaml`의 covariance에 `1e-3`과 `0`을 섞었더니
   `Sequence should be of same type`로 **노드가 기동 즉시 사망**. 225개 전부 float.

5. **ground truth 오도메트리로는 loop closure를 검증할 수 없다**
   drift가 0이라 교정할 게 없음 → `map→odom` 고정 → loop closure 미발화.

6. **Gazebo IMU는 기본이 무노이즈**
   그대로 쓰면 EKF 결과가 실제 iAHRS보다 과도하게 좋게 나와 실기 오차를 숨김.
   현재 gyro yaw에 stddev 0.002 + bias 추가해 둠.

7. **teleop이 1초마다 끊긴다**
   ROS 2 `teleop_twist_keyboard`에는 `repeat_rate`가 **없음**(ROS 1엔 있었음).
   키 1회 = 메시지 1회인데 `velocity_smoother`의 `velocity_timeout: 1.0`이 걸림.
   → `nav2:=false`로 띄우면 smoother가 없어 해결. 또는 키를 꾹 누르기.

8. **Nav2 실행 중 `/cmd_vel`엔 퍼블리셔가 6개**
   직접 teleop 하지 말 것. `/cmd_vel_nav`로 보낼 것.

9. **`map_saver_cli`에 `use_sim_time:=true`를 붙이면 저장이 실패한다**
   `[ERROR] Failed to spin map subscription`만 뜨고 파일이 안 생김. 시뮬레이션인데
   sim time을 켜는 게 맞아 보여서 붙이기 쉬운데, 오히려 그것 때문에 2초 기본
   타임아웃이 흘러가지 않습니다. **§7 명령 그대로, 인자 없이** 쓸 것.
   (굳이 켜야 하면 `-p save_map_timeout:=20.0`을 같이 줘야 함)

10. **맵을 저장한 뒤 `colcon build`를 다시 돌려야 한다**
    `--symlink-install`은 빌드 시점에 존재하는 파일만 링크합니다. 저장 직후의
    `maps/room.yaml`은 `install/` 쪽에 없어서 `navigation.launch.py`가
    기본 경로로 찾지 못합니다. 빌드 한 번이면 링크가 생깁니다.

11. **`ros2 action send_goal`의 좌표를 따옴표로 감싸면 조용히 안 움직인다**
    `{position: {x: '1.9'}}`처럼 쓰면 문자열이 되어 목표가 서버까지 못 갑니다.
    에러가 눈에 안 띄고 로봇만 가만히 있어서 Nav2 문제로 오해하기 쉬움.
    따옴표 없이 `x: 1.9`.

---

## 7. 실행 방법

```bash
# 지도 작성 (권장 — teleop 이 끊기지 않음)
cd ~/tetra_ws/tetra-dsv-s-ros2
./docker/run.sh 'colcon build --symlink-install && source install/setup.bash && \
  ros2 launch tetra_dsv_s_sim slam.launch.py nav2:=false'

# 다른 터미널 — 조종 (speed 낮추는 게 중요, 기본 0.5는 3m 방엔 너무 빠름)
./docker/run.sh 'source install/setup.bash && \
  ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -p speed:=0.15 -p turn:=0.4'

# 자율주행 (Nav2 포함, RViz 에서 Nav2 Goal)
./docker/run.sh 'source install/setup.bash && ros2 launch tetra_dsv_s_sim slam.launch.py'

# 맵 저장 — 이 형태 그대로. use_sim_time 을 붙이면 실패한다 (§6-9 참고)
ros2 run nav2_map_server map_saver_cli -f /ws/src/tetra_dsv_s_sim/maps/room

# 저장된 맵으로 운영 (AMCL). 맵을 새로 저장했다면 colcon build 를 한 번 더 돌려야
# install/ 쪽 심볼릭 링크가 생긴다.
./docker/run.sh 'source install/setup.bash && ros2 launch tetra_dsv_s_sim navigation.launch.py'
# 실제 방(빈 방)으로 돌리려면
./docker/run.sh 'source install/setup.bash && ros2 launch tetra_dsv_s_sim navigation.launch.py \
  world:=/ws/src/tetra_dsv_s_sim/worlds/empty_room.sdf \
  map:=/ws/src/tetra_dsv_s_sim/maps/empty_room.yaml'
```

**launch 인자**
- `slam.launch.py` — `world`, `odometry_source`(fused / wheel / ground_truth),
  `nav2`(true/false), `gui`, `rviz`. **이 넷만 하위로 전달됩니다.**
- `world`는 **`.sdf` 전체 경로**여야 합니다. 컨테이너에 `IGN_GAZEBO_RESOURCE_PATH`가
  비어 있어서 `world:=empty_room` 같은 짧은 이름은 해석되지 않습니다.
- `navigation.launch.py`는 위에 더해 `map`(맵 yaml 전체 경로)을 받습니다.
  기본값은 `maps/room.yaml` + loop_room 조합입니다.
- `x/y/z/yaw`(스폰 포즈)는 `simulation.launch.py` 전용입니다. `slam.launch.py`에
  붙여도 스폰 위치가 안 바뀌고 기본값(−1.0, 0, 0.05, 0)으로 고정됩니다.
  포즈를 바꾸려면 `simulation.launch.py`를 직접 띄우거나 slam.launch.py의
  전달 목록에 추가하세요.

**world 두 개의 용도**
- `empty_room.sdf` — 3.0 × 2.4 m 빈 방. **사용자의 실제 공간과 동일**
- `loop_room.sdf` — 같은 방 + 중앙 기둥(0.3 m) + 비대칭 코너블록 2개.
  기둥은 **loop closure 테스트용 장치**이지 실제 환경이 아님. 빈 직사각형은
  모든 벽이 항상 보여 drift가 안 쌓이므로 loop closure를 검증할 수 없어서 추가함.

---

## 8. 내일 할 일

### ✅ 수정 완료 — local costmap이 장애물을 지우지 못하던 문제 (2026-08-12)

발견하고 같은 날 고쳤습니다. 아래는 재발 방지용 기록입니다.

| | 수정 전 | 수정 후 |
|---|---|---|
| raytrace 경고 (동일 주행) | **763회** | **0회** |
| 주행 목표 | SUCCEEDED | SUCCEEDED (변화 없음) |

`nav2_params.yaml`의 `local_costmap` → `voxel_layer` → **`z_resolution: 0.05 → 0.10`**.
근거 전문은 해당 위치 주석에 남겨뒀습니다.

**증상** — 실행 내내 5 Hz로 계속 뜨던 경고:

```
[local_costmap] Sensor origin at (0.00, -0.00 1.05) is out of map bounds
                (-1.98, -1.98, 0.00) to (2.01, 2.01, 0.78).
                The costmap cannot raytrace for it.
```

원인은 `nav2_params.yaml`의 `local_costmap` → `voxel_layer`:

```
origin_z: 0.0
z_resolution: 0.05
z_voxels: 16        ->  복셀 기둥 높이 = 0.05 × 16 = 0.80 m
```

그런데 **VLP-16 광학중심은 지면 위 1.0477 m** (§4)입니다. 센서 원점이 복셀 격자
바깥에 있으면 `raytraceFreespace`가 그냥 빠져나가므로, **local costmap은 한 번
찍힌 장애물을 레이트레이싱으로 지우지 못합니다.** marking만 되고 clearing이 안 됨.

정적인 빈 방에서는 증상이 안 드러나서 (지울 게 애초에 없음) 지금까지 통과했지만,
**사람이 지나다니는 실기에서는 유령 장애물이 쌓입니다.**

영향이 하나 더 있습니다: 기억 범위가 0.80 m까지라 **그보다 높은 곳에만 존재하는
정적 장애물도 local costmap이 못 봅니다** (예: 다리 가는 테이블의 상판).
동적 장애물만의 문제가 아닙니다.

**수정 (`z_voxels`는 손댈 수 없음 — 반드시 읽을 것)**

```
z_resolution: 0.05  ->  0.10      # z_voxels 는 16 그대로
```

`z_voxels`를 키우고 싶어지지만 **16이 구현상 상한입니다.** 복셀 기둥 하나를
`uint32_t` 하나에 담고 16비트씩 marked / unknown 으로 쪼개 쓰기 때문입니다.
`/opt/ros/humble/include/nav2_voxel_grid/voxel_grid.hpp` 에 명시돼 있습니다:

> `@param size_z The z size of the grid, only sizes <= 16 are supported`

그래서 칸 개수 대신 **칸 높이**를 키웁니다. 0.10 × 16 = **1.60 m**로 센서
1.0477 m를 품습니다. 메모리·CPU 변화 없음(칸 개수 동일), 수직 해상도만
5 cm → 10 cm로 떨어지는데 평면 주행 로봇이라 실질 영향 없습니다.

1.60 m라는 값은 임의로 고른 게 아니라 `pointcloud_to_laserscan.yaml`의
`max_height`와 같은 값입니다. 맞춰두면 local costmap의 수직 범위가 스택의
나머지가 쓰는 `/scan` 대역(0.30~1.60 m)과 일치합니다.

**부수 효과 (이게 지금 고친 이유)**: 기억 범위가 0.80 → 1.60 m로 넓어지면서
**그 사이 높이의 정적 장애물도 보이게 됩니다.** 사용자 실제 공간에 0.80 m보다
높은 가구가 있다는 게 확인돼서(아래 §8-C), 동적 장애물을 미루는 것과 별개로
지금 고쳐야 했습니다. 참고로 로봇 전고는 1.01 m(차체 0.38 + 마스트 0.63)라
0.75 m 테이블 밑으로 못 지나갑니다 — 상판이 반드시 장애물로 찍혀야 합니다.

### 결정된 것

**(C) 동적 장애물은 실기 검증 이후로 미룬다** — 2026-08-12 사용자 결정

순서: **정적 장애물로 실기에서 먼저 돌려본다 → 그 다음 동적 장애물.**
근거는 문제 분리입니다. 실기에서 뭐가 터질지 모르는 상태에서 시뮬의 동적
장애물까지 함께 붙들고 있으면, 실기 이슈를 만났을 때 원인 후보만 늘어납니다.

이 결정이 미루는 작업:
- 움직이는 장애물이 있는 테스트 world 작성
- RPP가 앞을 막힌 상황에서 실제로 어떻게 반응하는지 확인
  (예상: 비켜 돌아가지 않고 **멈춰 서서 전역 재계획을 기다림**. §8-A 참고)
- 필요시 컨트롤러 재검토

이 결정이 미루지 **않은** 것: 위의 `z_resolution` 수정. 동적 장애물 전용이
아니라 0.80 m 위쪽 정적 장애물에도 걸리는 문제라 **같은 날 적용했습니다.**

**실제 테스트 공간 (사용자 확인, 2026-08-12)**
- 3.0 × 2.4 m 방에 **0.80 m보다 높은 가구가 있고, 사용자가 옮길 수 있음**
- 그래서 실기 테스트는 **빈 방 구성 / 장애물 있는 구성 두 번** 진행 예정
- 시뮬의 `empty_room.sdf` / `loop_room.sdf` 두 world가 각각 여기에 대응됨.
  단 `loop_room`의 기둥·코너블록은 loop closure 검증용 배치라 실제 가구 위치와
  다릅니다. 실기 장애물 구성이 정해지면 그 배치를 world에 반영할지 판단할 것.

### 결정 대기 중인 것

**(A) Smac 플래너 도입 여부** — 오늘 여기서 멈춤
현재 NavFn은 로봇을 점으로 보고 팽창 반경에만 의존합니다. `SmacPlannerLattice`
+ 동봉된 `diff` 프리미티브를 쓰면 **자세마다 실제 footprint를 검사**하므로
근사가 사라지고, `inflation_radius`도 다시 낮춰 통행 폭을 되찾을 수 있습니다.
대가는 CPU(SE(2) 탐색, 각도 축 ×72).

- 설치 확인됨: `SmacPlanner` / `SmacPlannerHybrid` / `SmacPlannerLattice`
- `diff` 프리미티브: `0.5m_turning_radius/diff/output.json` 등
- **Hybrid보다 Lattice가 맞음** — Hybrid는 Dubins/RS라 제자리 회전을 표현 못 함
- **잠정 결론**: 실제 방이 빈 방이라 지금은 방법 A(inflation 0.39)로 충분.
  Smac은 젯슨에서 CPU 실측이 가능해진 뒤 판단하는 게 합리적.

**(B) 실기 드라이버 방식 — 2026-08-12 소스 확인으로 사실상 결론남: RS-232**

`dssp_rs232_drive_module.h`를 열어보니 고민할 여지가 별로 없습니다.

```c
int dssp_rs232_drv_module_set_velocity (int velocity_l, int velocity_r);
int dssp_rs232_drv_module_set_velocity2(int velocity_l, int velocity_r,
        int *Xpos_mm, int *Ypos_mm, int *deg, int *bumper, int *emg);
```

- **`set_velocity2`를 쓸 것.** 속도 명령을 보내면서 오도메트리 + 범퍼 + 비상정지를
  **한 번의 왕복으로** 같이 받아옵니다. 115200 bps에서 왕복 횟수가 곧 제어 주기
  상한이라, 명령/조회를 따로 하는 것보다 유리합니다.
- 단위는 좌·우 바퀴 **mm/s** 정수. cmd_vel 변환은 호스트 몫입니다
  (위 §2-(1) 휠 트랙 주의).
- 벤더 노드 루프는 **30 Hz** (`tetraDS.cpp:513`). 시작점으로 삼을 것.
- 범퍼/EMG가 같은 응답에 실려 오므로 안전 정지를 별도 폴링 없이 처리 가능.

그 외 API: `read_encoder`, `read_odometry`, `reset_odometry`, `set_servo`,
`set_velocitymode`, `read_bumper_emg`, `read_drive_err`, `set_parameter`.

TCP/IP 5100(매뉴얼 5장)은 `GO2` 같은 좌표이동 상위 명령이라 Nav2 아래에
넣기 부적합. **탈락.**

### `tetra_dsv_s_bringup` — 2026-08-12 착수, 구동부까지 작성됨

```
tetra_dsv_s_bringup/
├── include/.../drive_board.hpp   RS-232 프로토콜 (프레임/LRC/부호인코딩)
├── src/drive_board.cpp           termios 설정 + BV 왕복
├── src/tetra_drive_node.cpp      cmd_vel -> 바퀴속도, odom/joint_states 발행
├── launch/drive.launch.py        구동부만. 센서·Nav2 없음 (첫 통전용)
├── scripts/probe_drive_board.py  ROS 없이 시리얼만 확인 (pyserial)
└── udev/99-tetra-serial.rules    포트 고정. VID/PID/serial 은 비워둠
```

**프로토콜** (`drive_module.c` + `rs232_common.c`에서 역이식)

```
요청  STX 'B' 'V' Lhi Llo Rhi Rlo ETX LRC     9바이트
응답  [1] 0x30 정상 / 0x32 비상정지
      [2..5] X(mm)  [6..9] Y(mm)  [10..11] θ  [12] 범퍼
LRC   STX 다음부터 ETX 까지의 단순 XOR (체크섬/CRC 아님)
속도  바퀴별 mm/s 정수. **부호는 상위바이트 bit7, 2의 보수 아님**
      -500 -> 0x81F4 (0xFE0C 아님)
```

프레임 생성이 벤더 C 코드와 **바이트 단위로 일치**함을 검증했습니다:
`CZ11 -> 02 43 5a 31 31 03 1a`, `BV(-500,+500) -> 02 42 56 81 f4 01 f4 03 97`.

**설계 결정**
- `set_velocity2`(BV) 단일 왕복. 명령+오도메트리+범퍼+EMG를 한 번에 받습니다.
- **`odom -> base_footprint`를 발행하지 않습니다.** 시뮬과 똑같이 EKF가 소유해야
  시뮬↔실기 전환에 TF 구조가 안 바뀝니다. 여기서도 내면 부모가 둘이 됩니다.
- 부팅 시 보드는 **위치모드**입니다. `CZ11`(속도모드)을 먼저 안 보내면 BV가
  받아들여지고도 **아무것도 안 움직입니다.** 무증상 함정.
- cmd_vel 워치독 0.5초. 끊기면 정지. 실기에서 이 값을 키우지 말 것.

**검증된 것 / 안 된 것**
- `-Wall -Wextra -Wpedantic` 컴파일 통과, 포트 없을 때 FATAL 후 정상 종료 확인
- **실기 통신 미검증.** 개발 PC는 x86이고 시리얼 장치가 없습니다. 응답 프레임
  해석(특히 `theta_raw`의 0.1도 스케일)은 소스에서 읽은 것이지 실측이 아닙니다.

**다음**: 젯슨에서 `probe_drive_board.py`부터. 통신이 되면 launch로.
센서(VLP-16 / iAHRS)와 Nav2 wiring은 젯슨에 드라이버가 설치된 뒤 작성합니다 —
없는 패키지의 노드·토픽 이름을 추측해서 launch를 쓰면 안 되기 때문.

### 원래 계획 (참고)

`scout_bringup` 구조를 미러링하되 **CAN을 RS-232로 교체**.

젯슨에서 돌아야 하는 것:
```
시리얼 드라이버 ×2 (구동보드 CN20, 전원센서보드 CN24, 각 115200)
iahrs_driver_ros2                  ← 별도 clone 필요! (아래 주의 참고)
velodyne_driver + velodyne_pointcloud
pointcloud_to_laserscan
robot_state_publisher
robot_localization EKF
slam_toolbox (지도작성) 또는 amcl (운영)   ← 둘 중 하나만
Nav2
```

젯슨에서 **안** 돌리는 것: RViz(PC에서 원격), Gazebo, ros_gz_bridge

**실무 주의**: RViz를 WiFi로 붙일 때 `/points`를 구독하지 말 것 (VLP-16 초당 30만 점).
`/map`, `/scan`, `/tf`, `/plan`, costmap만.

**필요 작업**: USB-시리얼 포트 고정용 udev 규칙 (매뉴얼 6-6절에 IMU 예시 있음)

**`iahrs_driver_ros2` 주의**: 로컬 사본 `~/tetra_ws/TETRA-DSV-S`에는 **없습니다.**
그 저장소는 TETRA-DSV-S 하나뿐이고 IMU 드라이버는 Hyulim-Group 조직의 **별도
저장소**입니다. 따로 clone 해야 합니다. `github.com` 자체는 이 네트워크에서
정상이므로(차단은 GitHub Pages 대역뿐, §1 참고) 받는 데는 문제 없습니다.

**벤더 라이다는 우리 것과 다름**: 벤더 저장소의 라이다 패키지는 `lslidar_n301`
(2D 스캐너)입니다. 우리 스택의 VLP-16 + 센서 마스트는 사용자 추가 구성이므로,
bringup 작성 시 벤더 launch에서 라이다 부분은 참고하지 말고 통째로
`velodyne_driver` + `velodyne_pointcloud` + `pointcloud_to_laserscan`으로 대체할 것.

---

## 9. 아직 검증 안 된 것 (정직하게)

- **loop closure의 이산적 발화를 여전히 확인 못 함.** 2026-08-12에 loop_room을
  두 바퀴 돌며 다시 시도했지만 실패:
  - slam_toolbox는 loop closure를 로그로 남기지 않음 (Ceres 솔버 시작 줄만 나옴)
  - `/slam_toolbox/graph_visualization`은 노드 마커 127개만 담고 있고
    **constraint 엣지는 안 들어 있음** → 이 토픽으로는 비순차 constraint를 못 셈
  - 남은 방법: `serialize_map` 서비스로 pose graph를 떠서 직접 열어보거나,
    slam_toolbox를 디버그 로그 레벨로 올리는 것.
- **실기 하드웨어 일체 미검증.** 전부 시뮬레이션 결과.
- **젯슨 CPU 부하 미측정.**
- 기둥 여유 +0.196 m는 시뮬 기준. 실기는 위치추정 오차가 더 크므로 줄어들 수 있음.
- **AMCL의 kidnapped/전역 재추정은 미검증.** §4의 AMCL 결과는 전부 초기 위치를
  정확히 알려준 상태에서 나온 값입니다. 초기 위치 없이 또는 틀리게 준 경우
  (특히 빈 직사각형 방)에는 어떻게 되는지 확인 안 했습니다.
- **local costmap clearing이 실제로 지우는 걸 확인하진 못함.** §8의 수정으로
  경고는 763회 → 0회가 됐지만, 이건 "레이트레이싱이 이제 수행된다"까지만
  증명합니다. 장애물이 사라졌을 때 실제로 지워지는지는 **움직이는 장애물이
  있어야 검증 가능**하고, 그건 §8-C에 따라 실기 이후로 미뤄져 있습니다.
