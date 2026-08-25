# TETRA-DSV-S ROS 2 인수인계

> **이 문서의 독자는 Claude입니다.** 사용자용 설명서가 아니라, 다른 맥락 없이
> 이 파일 하나로 작업을 이어받기 위한 브리핑입니다. 설명조로 늘이지 말고
> 검증된 수치 / 결정과 그 근거 / 무증상 함정을 밀도 있게 유지할 것.
> 작업을 끝낼 때 이 파일 갱신까지가 그 작업의 일부입니다. 틀린 내용을 발견하면
> 반드시 고칠 것 — 여기 남은 오류는 다음 세션을 그대로 오도합니다.
> 마지막 갱신: 2026-08-13
> — **실기에서 SLAM 가동 성공. 지금까지 전부 시뮬이었던 것이 실기로 넘어감** (§10)
> — **IMU 는 iAHRS 가 아니라 MicroStrain 3DM-GV7-AHRS 였음.** 문서 전반의
>   "iAHRS" 기술은 전부 틀린 것이었습니다 (§10-2)
> — 라이다·IMU·구동부 실기 연결 완료, `slam.launch.py` 신규 (§10-5)
> — **사내 게스트 와이파이가 UDP 멀티캐스트를 막아 ROS 2 원격이 전면 불통.**
>   Fast DDS Discovery Server 로 우회 (§10-1) — 매번 겪을 문제라 꼭 읽을 것
> — FTDI `latency_timer` 기본 16ms 때문에 BV 응답이 잘려 들어오던 문제 (§10-3)
> — (같은 날 오전) 실기 바퀴 회전 시험 성공 (raw 프로토콜 레벨). 펌웨어 자체
>   통신두절 안전장치(Para30, 2000ms) 확인. 매뉴얼 6~10장은 다른 기종용이라
>   참고 대상 아님 (§8-B)
> — 2026-08-12 기록: 맵 2개 작성·저장 + AMCL 검증 (§4), local costmap
>   clearing 수정 (§8), 휠 트랙 실측 377mm (§2-(1)),
>   `probe_drive_board.py` 프레임 종료 판정 버그 수정

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
| **바퀴간 거리** | 438 mm (Para5, 펌웨어 내부 오도메트리 전용) / **377 mm (실측, 우리가 실제 쓰는 값)** | 표 4-3 Para5 / 아래 §2-(1) |
| 최대속도 | 스펙 2.0 m/s, 펌웨어 Para8 = 1500 mm/s | 표 1-1 / 4-3 |
| 통신 | **RS-232C 115200bps** (CAN 아님!) | 표 3-5 |

### 반드시 알아야 할 두 가지

**(1) ✅ 휠 트랙 확정 — 실측 완료 (2026-08-12)**

원래 판단은 "URDF의 0.378은 메쉬 장착 오프셋이고 펌웨어는 Para5 = 438로
변환하니 **0.438을 쓴다**"였고, 시뮬은 한동안 0.438로 돌아갔습니다.
그런데 벤더 **실기 드라이버 소스**를 열어보니 근거가 흔들려서 재검토했고,
실기가 온 뒤 줄자로 확인했습니다.

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

→ **실측 결과 (줄자, 2026-08-12): 좌우 바퀴 접지면 중심 간 거리 ~377mm.**
바퀴 외곽(타이어 바깥쪽 끝 대 끝) 실측은 ~410mm로, 377 + 바퀴폭(≈33mm)과
정합적이라 크로스체크도 통과. 위 세 정황 근거(반경만 매뉴얼값 일치, 438이면
430mm 차체 폭 밖으로 삐져나옴, 펌웨어가 속도제어엔 트랙을 안 씀) 그대로
확인된 셈이라 **438(Para5)은 폐기, 0.377을 kinematic wheel_separation으로
확정.** 0.438을 썼다면 로봇이 명령보다 16% 덜 회전했을 것.

`tetra_dsv_s_base.xacro`(시뮬 URDF)와 `drive.launch.py`(실기 기본값) 모두
0.377로 맞춰 반영 완료. 두 값이 이제 일치하므로 시뮬/실기 간 yaw rate 괴리
없음.

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
   그대로 쓰면 EKF 결과가 실기 IMU보다 과도하게 좋게 나와 실기 오차를 숨김.
   현재 gyro yaw에 stddev 0.002 + bias 추가해 둠.
   (실기 IMU는 MicroStrain 3DM-GV7-AHRS입니다 — §10-2. 이 노이즈 수치는
   일반 MEMS 기준으로 넣은 것이고 GV7 데이터시트와 대조하진 않았습니다.)

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
- **펌웨어 자체에도 통신두절 안전장치가 있습니다.** 매뉴얼 §4-4-1 Para30 —
  상위제어기 무응답 시 좌우모터 강제정지, 기본 **2000ms**. 우리 노드의 0.5초
  워치독 위에 얹힌 하드웨어 레벨 이중 안전망. 노드가 죽어도 최악 2초 내 정지.

**✅ 실기 통신 검증 완료 (2026-08-12, 젯슨)**
- `probe_drive_board.py`로 CN20 구동보드 통신 확인, 줄자로 휠 트랙 377mm 확정
  (§2-(1) 참고, 매뉴얼 Para5 438mm는 펌웨어 내부 오도메트리 전용으로 폐기)
- **바퀴 회전 시험 성공.** ROS 노드가 아니라 이 raw 프로토콜 레벨에서 확인.
- 과정에서 `probe_drive_board.py`의 프레임 종료 판정 버그를 발견해 수정
  (`0x03` 값이 페이로드 안에도 등장해 조기 절단되던 문제 — 커밋 3533ba5)
- ~~**아직 검증 안 됨**: `tetra_drive_node`(ROS 2 노드) 자체는 아직 실기로 안
  띄워봄~~ → **같은 날 오후 완료.** ROS 레이어(odom 발행, joint_states,
  cmd_vel 구독) 전부 실기에서 확인했습니다. `/wheel_odometry` 30 Hz 안정.
  단, 그 과정에서 FTDI `latency_timer` 문제를 만났습니다 — raw 프로브는
  되는데 ROS 노드만 안 되는 현상의 원인이었습니다 (§10-3).

**다음**: ~~`drive.launch.py`로 ROS 노드를 실기에서 띄워 확인~~
→ **2026-08-13 오후 완료.** 구동부·VLP-16·IMU·EKF·SLAM 전부 실기에서
가동했고 `slam.launch.py` 로 묶었습니다. **§10 이 실제 기록입니다.**
아래 "원래 계획"은 착수 전에 쓴 것이라 IMU 기종 등 틀린 내용이 있으니
§10 을 우선하세요.

**매뉴얼 6~10장 관련 — 참고하지 말 것**
Chapter 6(Software Setup)부터 10(TETRA APP)까지는 TETRA-DSV-S가 아니라
**TETRA-DS V**(도킹스테이션·컨베이어 달린 별도 완제품)의 벤더 레퍼런스
구현입니다. ROS 1 Melodic + Cartographer + move_base/TEB + SICK TIM571/
CygLiDAR D1 + RealSense D435 ×2 조합이라 우리 스택(ROS 2 Humble +
slam_toolbox + Nav2 + VLP-16)과 세 겹으로 다릅니다. §9의 `goto_cmd` 등
ROS 서비스는 벤더 ROS1 패키지(`tetraDS_2dnav`/`tetraDS_service`, 로컬
사본 `~/tetra_ws/TETRA-DSV-S`에 있음)가 제공하는 거라 우리 ROS2 코드에
이식 불가. **Chapter 4(프로토콜)까지만 유효한 레퍼런스.**

### 원래 계획 (참고)

`scout_bringup` 구조를 미러링하되 **CAN을 RS-232로 교체**.

젯슨에서 돌아야 하는 것:
```
시리얼 드라이버 ×2 (구동보드 CN20, 전원센서보드 CN24, 각 115200)
iahrs_driver_ros2                  ← ❌ 틀림. 실물은 MicroStrain 이고
                                      ros-humble-microstrain-inertial-driver
                                      를 apt 로 설치합니다 (§10-2)
velodyne_driver + velodyne_pointcloud
pointcloud_to_laserscan            ← 불필요. velodyne_laserscan 이 /scan 을
                                      바로 냅니다 (§10-5)
robot_state_publisher
robot_localization EKF
slam_toolbox (지도작성) 또는 amcl (운영)   ← 둘 중 하나만
Nav2
```

젯슨에서 **안** 돌리는 것: RViz(PC에서 원격), Gazebo, ros_gz_bridge

**실무 주의**: RViz를 WiFi로 붙일 때 `/points`를 구독하지 말 것 (VLP-16 초당 30만 점).
`/map`, `/scan`, `/tf`, `/plan`, costmap만.

**필요 작업**: USB-시리얼 포트 고정용 udev 규칙 (매뉴얼 6-6절에 IMU 예시 있음)

**~~`iahrs_driver_ros2` 주의~~ — 이 항목 전체가 무효입니다 (2026-08-13).**
실기에 달린 IMU는 iAHRS가 아니라 **LORD MicroStrain 3DM-GV7-AHRS** 였습니다.
clone 할 것 없이 `sudo apt install ros-humble-microstrain-inertial-driver`
로 끝납니다. 자세한 건 §10-2 (GV7 전용 파라미터 함정 두 개 포함).

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

---

## 10. 실기 브링업 (2026-08-13) — 젯슨에서 SLAM 가동까지

이 장 이전의 §4 검증 결과는 **전부 시뮬레이션**입니다. 이 장이 실기 첫 기록입니다.
하루에 걸쳐 구동부 → 라이다 → IMU → EKF → SLAM 순으로 뚫었고, 각 단계에서
막힌 지점과 원인을 아래에 남깁니다. **대부분 다음에도 똑같이 재발합니다.**

### 10-0. 하드웨어 실물 구성 (문서에 없던 것)

로봇은 다른 기관(공항산업기술연구원) 자산이고, 원래 **인텔 NUC** 이 제어
컴퓨터였습니다. AMR 내부에서 USB 5 가닥이 차체 위로 올라와 **USB 허브(UH508)**
에 모이고, 그 허브의 업스트림이 NUC 으로 갑니다. 별도로 **iPTIME H6005mini
(공유기 아니라 스위치)** 가 있고 여기에 AMR 내부 랜선 / NUC / 라이다(빨간 랜선,
기둥 위로) 가 물려 있습니다.

우리 젯슨을 붙이는 방법:

| 장치 | 방식 | 비고 |
|---|---|---|
| 구동보드 | USB-RS232(FTDI)를 허브에서 **뽑아 젯슨에** | 점대점이라 공유 불가 |
| IMU·RealSense·웹캠 | **USB 허브 업스트림을 통째로 젯슨에** | 5개를 한 번에 확보 |
| VLP-16 | 젯슨 이더넷 → **스위치 빈 포트에 추가** | 아무것도 안 뽑아도 됨 |

라이다를 뽑지 않아도 되는 이유: VLP-16 이 `255.255.255.255` 로 브로드캐스트하기
때문에 스위치에 물린 NUC 과 젯슨이 **동시에 같은 데이터를 받습니다.**
NUC 쪽 기존 시스템을 건드리지 않고 붙일 수 있는 유일한 경로라 이 방법을 씁니다.

**컴퓨터끼리 USB 로 연결하는 것은 답이 아닙니다** (실제로 검토했다가 기각).
USB 는 호스트 1 : 장치 N 구조라, 젯슨과 NUC 둘 다 호스트인 이상 서로의 장치를
볼 수 없습니다. USB-A 끼리 직결은 위험하기까지 합니다.

허브에서 새로 발견된 장치 (문서 어디에도 없던 것):
- **Intel RealSense D455** 뎁스 카메라 (ROS 패키지는 아직 미설치)
- USB2.0 UVC 웹캠
- PL2303 시리얼 어댑터 — 커널 드라이버가 안 올라와 포트가 안 생김. 용도 미상

### 10-1. ⚠ 사내 게스트 와이파이가 ROS 2 원격을 전면 차단 — 매번 겪습니다

**증상.** 노트북에서 `ros2 topic list` 하면 `/parameter_events`, `/rosout`
둘만 보입니다. 젯슨에서는 모든 토픽이 정상. `ROS_DOMAIN_ID`(25)와
`ROS_LOCALHOST_ONLY`(0) 를 양쪽 다 맞춰도 그대로입니다.

**진단.** 다음 순서로 30초면 확정됩니다.

```
ping <젯슨IP>                      # 성공  -> L3 유니캐스트는 살아있음
ros2 multicast receive   (한쪽)
ros2 multicast send      (다른쪽)  # 양방향 모두 실패
```

ping 은 되는데 멀티캐스트만 죽는 것이 결정적 단서입니다. ROS 2 기본 디스커버리
(SPDP)는 UDP 멀티캐스트를 쓰므로, 이게 막히면 서로를 **영원히** 못 찾습니다.

**원인.** 젯슨이 붙어 있던 SSID 가 `KITECH_Guest` — 게스트망입니다. 기업
게스트망은 무선 멀티캐스트 폭주 방지 목적으로 멀티캐스트/브로드캐스트를
선택적으로 드롭하는 것이 일반적입니다. (완전한 client isolation 은 아닙니다 —
그거였으면 ping 도 막힙니다.)

**우회 — Fast DDS Discovery Server.** 멀티캐스트 없이 유니캐스트로만
디스커버리합니다. 추가 설치 없이 Humble 에 이미 들어 있습니다.

```
# 젯슨: 서버 (계속 떠 있어야 함)
fast-discovery-server --server-id 0 --udp-port 11811

# 양쪽 공통 환경변수
export ROS_DISCOVERY_SERVER="10.101.111.244:11811"
export FASTRTPS_DEFAULT_PROFILES_FILE=~/tetra_dds/super_client.xml
```

`super_client.xml` 은 `<discoveryProtocol>SUPER_CLIENT</discoveryProtocol>` 과
서버 주소를 담습니다. 서버 GUID prefix 는 server-id 0 일 때
`44.53.00.5f.45.50.52.4f.53.49.4d.41` 로 고정입니다.

**무증상 함정 세 개 — 전부 실제로 당했습니다:**

1. **SUPER_CLIENT 프로파일 없이 환경변수만 설정하면** 노드끼리는 통신하지만
   `ros2 topic list` 같은 CLI 는 여전히 아무것도 못 봅니다. CLI 는 자기가
   구독하지 않는 토픽을 서버에 물어봐야 하는데, 평범한 CLIENT 는 그 권한이
   없습니다.
2. **`ros2 daemon stop` 을 안 하면 아무 설정도 반영되지 않습니다.** 데몬이
   이전 디스커버리 설정을 캐시한 채 살아 있습니다. 환경변수를 바꿨으면
   **반드시** 데몬을 죽이고 다시 조회해야 합니다. 이것 때문에 "설정이 틀렸나"
   하고 한참 헤맸습니다.
3. **젯슨 쪽에서 로봇 노드를 띄우는 셸에 `ROS_DISCOVERY_SERVER` 를 안
   걸어도, 젯슨 로컬에서는 아무 이상 없이 다 됩니다** (2026-08-19 실측).
   같은 호스트 안에서는 멀티캐스트가 안 막히므로 로봇 노드들끼리는 로컬
   멀티캐스트로 서로 잘 찾고, `ros2 topic echo`/`ros2 topic list` 도 젯슨
   로컬에서 돌리면 전부 정상으로 보입니다. 문제는 이 노드들이 discovery
   server 에는 **한 번도 등록되지 않았다는 것** — 서버 프로세스 자체는
   멀쩡히 떠 있어도(포트 11811 정상 리스닝) 노트북의 SUPER_CLIENT 는 그
   서버를 통해서만 디스커버리하므로 로봇 노드를 영원히 못 봅니다. 증상은
   "젯슨에서는 토픽이 다 보이는데 노트북에서는 `/parameter_events`,
   `/rosout` 둘뿐" — §10-1 처음에 적은 게스트망 증상과 **완전히 똑같이
   보입니다.** 구분법: 서버 프로세스(`ps aux | grep discovery`, 포트
   11811 `ss -uln`)와 로봇을 띄운 셸의 `env | grep ROS_DISCOVERY_SERVER`
   를 같이 확인하세요 — 서버는 떠 있는데 이 환경변수가 비어 있으면 이
   함정입니다. `slam.launch.py`/`navigation.launch.py` 를 실행하는 **바로 그
   셸**에서 `export ROS_DISCOVERY_SERVER="10.101.111.244:11811"` 를 먼저
   실행한 뒤 launch 해야 합니다 — 다른 터미널에서 서버만 띄워놓고 딴
   터미널에서 launch 하면 이 환경변수가 그 터미널에는 없다는 걸 잊기 쉽습니다.

**근본 해결.** 정규 사내망이나 별도 공유기로 옮기면 Discovery Server 없이
그냥 됩니다. 게스트망에 계속 있어야 한다면 위 구성을 유지하세요.

### 10-2. ⚠ IMU 는 iAHRS 가 아니라 MicroStrain 3DM-GV7-AHRS 였습니다

문서 전반(§2, §8, URDF 주석)이 "iAHRS" 라고 적고 있었지만 **실물이 다릅니다.**

```
/dev/ttyACM0   ID_VENDOR=Lord_Microstrain  ID_MODEL=Lord_Inertial_Sensor
Model Name: 3DMGV7-AHRS   S/N 6288.171025   FW 1.1.00
```

드라이버도 다릅니다: `ros-humble-microstrain-inertial-driver` (apt 설치).

**GV7 전용 함정 두 개 — 둘 다 노드가 그냥 죽습니다:**

| 파라미터 | 기본값 | 필요값 | 안 바꾸면 |
|---|---|---|---|
| `filter_pps_source` | 1 | **0** | `Failed to configure PPS source / Error(3)` 후 FATAL |
| `filter_declination_source` | 2 | **1** | `Failed to set declination source / Error(3)` 후 FATAL |

GV7 은 GNSS 가 없는 AHRS 라 PPS 입력도 자기편각 계산도 지원하지 않습니다.
드라이버 기본값은 GNSS 달린 상위 모델 기준입니다. 참고로 드라이버 자신의
`params.yml` 주석에도 "CV7 을 쓸 때는 declination 을 1 이나 3 으로 바꿔야
노드가 시작된다"고 적혀 있고, GV7 도 같은 7-시리즈라 동일하게 걸립니다.

설정은 `tetra_dsv_s_bringup/config/imu_gv7.yml`. 남는 에러
`Failed to set baudrate for port 0X13` 는 **무해합니다** — USB CDC 라 실제
보레이트 개념이 없어서 나는 것이고, 그 뒤에 `Node activated` 가 뜹니다.

**장착 방향 — 거꾸로 달려 있습니다.** 정지 상태에서 `az = -9.83 m/s^2`.
로봇을 159도 회전시켜 보니 IMU 의 yaw rate 가 휠 오도메트리와 **616 표본 중
611 개(99%)에서 부호 반대** — Z 축 반전이 확정입니다.

다만 **X 축으로 뒤집힌 것인지 Y 축으로 뒤집힌 것인지는 끝내 못 갈랐습니다.**
두 경우 모두 Z 를 똑같이 반전시키고, 구분하려면 깨끗한 전후 가속도 측정이
필요한데 실패했습니다:
- 정지→출발→정지 한 사이클의 가속도 총합은 **물리적으로 정확히 0** 입니다
  (속도가 0 으로 돌아오므로). 방향 부호를 곱해 누적하는 방식은 원리적으로 안 됩니다.
- 오도메트리 위치에서 가속도를 미분해 회귀하는 방식도 실패했습니다. 위치
  분해능이 1 mm, 발행 30 Hz 라 두 번 미분하면 양자화 잡음이 폭증합니다
  (최대 가속도가 19 m/s^2 로 나옴 — 중력의 2배, 명백한 잡음).

**그래서 모호성이 영향을 못 주게 설계했습니다:**
- URDF 는 `roll 180도` 를 씁니다 (`imu_mount_rpy`, 실기일 때만 적용)
- EKF 는 IMU 에서 **yaw rate 만** 받습니다. 절대 방위는 안 받습니다 —
  X/Y 모호성이 실제로 갈리는 유일한 값이 절대 방위이기 때문입니다.
  yaw **rate** 에 대해서는 두 경우가 완전히 같은 보정입니다.

평면 SLAM 에서는 이것으로 충분합니다. 지자기 기준 절대 방위 초기화 같은 게
필요해지면 그때 X/Y 를 확정해야 하고, 그때는 IMU 실물의 축 표시를 눈으로
보는 편이 빠릅니다 (다만 남의 장비 패널이라 사전 허가 필요).

측정 스크립트는 `scripts/check_imu_mounting.py` 에 남겨뒀습니다.

### 10-3. ⚠ FTDI `latency_timer` — 재부팅·재연결 때마다 되돌아옵니다

**증상.** `tetra_drive_node` 가 `BV round trip failed: short BV reply` 를
초당 한 번씩 뱉습니다. 응답 바이트가 1~3 개만 도착합니다 (정상은 16 바이트).

**원인.** FTDI 어댑터의 USB `latency_timer` 기본값이 **16 ms** 입니다. 칩이
수신 바이트를 모아뒀다 몰아서 호스트로 넘기는데, 그 사이 공백이 프레임 끝과
구분되지 않습니다. `probe_drive_board.py`(파이썬)는 `idle_gap` 을 20 ms 로
잡아서 우연히 이 문제를 피해 갔고, 그래서 **파이썬 프로브는 되는데 ROS 노드만
안 되는** 혼란스러운 상황이 나옵니다.

**해결.**
```
sudo sh -c 'echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer'
```

udev 규칙에 `ATTR{latency_timer}="1"` 을 넣어 뒀지만(`udev/99-tetra-serial.rules`),
**실제로는 잘 안 먹습니다.** 하루 동안 USB 를 재연결할 때마다 16 으로
돌아갔습니다. 그때마다 위 명령을 다시 쳐야 합니다. `slam.launch.py` 의
docstring 에도 사전 준비로 적어뒀습니다.

`drive_board.cpp` 의 `read_frame()` 도 ETX 값 매칭 대신 **바이트 간 공백**으로
프레임 끝을 판정하도록 고쳤습니다(`IDLE_GAP_MS = 5`, `poll()` 기반). 다만
이것만으로는 부족하고 `latency_timer` 를 낮춰야 합니다.

### 10-4. ⚠ "short BV reply" 가 사실은 비상정지였습니다

BV 응답이 `02 32 03 31` (4 바이트) 로 오면 그건 프로토콜 오류가 아니라
**비상정지가 걸린 상태**입니다. `rx[1] = 0x32` 가 E-stop 을 뜻하고, 이때
보드는 오도메트리 페이로드 없이 상태만 돌려줍니다.

기존 코드는 이걸 길이만 보고 `short BV reply` 로 뭉뚱그려서, 프레이밍 버그를
찾느라 시간을 썼습니다. `drive_board.cpp` 를 고쳐서 이제
`emergency stop engaged at the robot` 이라고 정확히 말합니다.

**진단 순서를 기억할 것:** `short BV reply` 가 나오면 ① 로봇 전원 ② 비상정지
③ `latency_timer` 순으로 확인하세요. 프로토콜 코드는 마지막입니다.

### 10-5. 실기 SLAM 구성 — `slam.launch.py`

```
ros2 launch tetra_dsv_s_bringup slam.launch.py
```

구동부 + VLP-16 + IMU + EKF + slam_toolbox 를 한 번에 띄웁니다.
`slam:=false` 로 지도 작성만 뺄 수 있습니다.

TF 소유권 (시뮬과 동일하게 유지):

```
map        -> odom             slam_toolbox
odom       -> base_footprint   EKF (robot_localization)
base_*     -> 나머지            robot_state_publisher (URDF)
```

드라이브 노드는 TF 를 내지 않습니다. 여기서 부모가 둘이 되면 트리가 깨집니다.

새로 만든 설정 (`tetra_dsv_s_bringup/config/`):

| 파일 | 핵심 |
|---|---|
| `velodyne_vlp16.yaml` | `frame_id: velodyne_link` — 드라이버 기본값 `velodyne` 이면 SLAM 이 TF 를 못 찾아 **조용히** 멈춥니다 |
| `imu_gv7.yml` | PPS·declination 비활성 (§10-2) |
| `ekf.yaml` | 휠에서 `vx,wz` / IMU 에서 yaw rate 만 |
| `slam_toolbox.yaml` | `max_laser_range: 20.0`, 이동·회전 문턱으로 젯슨 부하 억제 |

**설정에서 당한 함정 두 개:**

1. **`process_noise_covariance` 는 15x15 = 225 개**여야 합니다. 상태 벡터가
   15 차원(x y z, rpy, v, ω, a)이기 때문입니다. 6x6 = 36 개를 넣었더니 EKF 가
   즉시 `Critical Error, NaNs were detected` 를 초당 30 번씩 뱉었습니다.
   지금은 아예 안 쓰고 기본값에 맡깁니다 — 실측 없이 손으로 고치면 더 나빠집니다.
2. **velodyne `calibration` 은 절대 경로**여야 합니다. `VLP16db.yaml` 처럼
   파일명만 주면 `Failed to open calibration file` 로 SIGABRT 입니다.
   velodyne 패키지 자신의 launch 도 `os.path.join` 으로 경로를 만들어 넘깁니다.
   그래서 이 값은 yaml 이 아니라 `slam.launch.py` 에서 채웁니다.

### 10-6. 실기 검증 결과 (2026-08-13 기준)

| 계통 | 결과 |
|---|---|
| 구동부 | cmd_vel 주행 확인, `/wheel_odometry` **30 Hz 안정** |
| VLP-16 | `192.168.1.201`, 754 pkt/s (600 RPM), `/scan` **9.8 Hz** |
| IMU GV7 | `/imu/data` **100 Hz**, `frame_id: imu_link` |
| EKF | `odom -> base_footprint` 정상 |
| slam_toolbox | `/map` 2 초 주기, `map -> base_footprint` 체인 완성 |
| 원격 RViz | 노트북에서 지도·스캔·로봇모델 표시 확인 |

젯슨 부하: load average 2.6 (6 코어). **1 등이 VSCode(70%)** 였습니다 —
본격 매핑 때는 젯슨 데스크톱의 VSCode·크롬을 닫으세요. 닫기 전에는 `/scan` 이
4~6 Hz 까지 떨어졌습니다(정상 10 Hz).

네트워크 설정 (재부팅 후에도 유지되도록 NetworkManager 프로파일로):
```
sudo nmcli con add type ethernet ifname enP8p1s0 con-name lidar \
  ipv4.method manual ipv4.addresses 192.168.1.150/24 \
  ipv4.never-default yes ipv6.method ignore
```
`ipv4.never-default yes` 가 중요합니다. 없으면 라이다망이 기본 경로를 가로채
인터넷이 끊깁니다. 참고로 `ip addr add` 로 수동 지정하면 NetworkManager 가
곧바로 지워버립니다 — 반드시 프로파일로 등록해야 합니다.

### 10-8. ⚠ EKF yaw 드리프트 — IMU 자이로 바이어스 (2026-08-14)

**증상.** RViz 의 지도가 실제 공간과 어긋납니다. 로봇을 세워둔 채로 두면
지도 속 로봇만 계속 돌아갑니다.

**측정 (로봇 완전 정지, 젯슨 로컬).**

```
자이로 z 평균     -0.000444 rad/s = -0.0255 deg/s   (5840 표본, 편향)
자이로 z 표준편차  0.001773 rad/s                    (잡음 — 평균 내면 사라짐)
휠 오도메트리      정확히 0.0
EKF yaw 드리프트  +0.0240 deg/s = +86.3 deg/시간     (66초 실측)
```

**TF 를 쪼개면 범인이 바로 나옵니다. 이 진단을 먼저 하세요.**

```
map  -> odom            0.000 deg   <- slam_toolbox 는 보정을 전혀 안 하고 있었음
odom -> base_footprint  25.803 deg  <- EKF 가 혼자 돌고 있음
```

**원인.** 자이로에 상수 편향이 있는데 EKF 가 그걸 거의 그대로 적분했습니다.
왜 그렇게 됐는지가 핵심입니다 — **양쪽 기본 공분산이 우연히 그랬을 뿐,
근거가 없었습니다.**

| 센서 | yaw rate 공분산 | 가중치 |
|---|---|---|
| 휠 (`twist.covariance[35]`) | 0.05 | 17% |
| IMU (`angular_velocity_covariance[8]`) | 0.01 | **83%** |

드라이버 `params.yml` 354행이 대놓고 적고 있습니다 — *"Static IMU message
covariance values (**the device does not generate these**)"*. 즉 0.01 은
장치가 낸 값이 아니라 드라이버가 박아둔 상수입니다. 그런데 그 상수 때문에
EKF 가 IMU 를 휠보다 5배 신뢰했습니다. 예측 0.833 × 0.0255 = 0.0212 deg/s,
실측 0.0240 deg/s — 설명이 닫힙니다.

**실제 피해는 SLAM 에서 납니다.** `slam_toolbox.yaml` 의
`minimum_travel_heading` 이 0.2 rad(11.5도)입니다. 86 deg/시간이면 **가만히
서 있어도 8분마다 이 문턱을 넘습니다.** slam_toolbox 는 "로봇이 11.5도
돌았다"고 믿고 스캔을 새로 넣지만 실제 스캔은 안 돌았으므로 벽이 어긋나
겹칩니다.

**수정.** `config/imu_gv7.yml` 에 `imu_angular_cov` 를 1.0 으로 명시
(기본 0.01). 파일 안에 근거를 다 적어뒀습니다.

```
드리프트  86.3 -> 12.8 deg/시간   (6.7배 개선, 186초 실측)
문턱 도달  8분 -> 54분
수정 후 주행 중 map->odom = -7.4 도  <- 스캔매칭이 능동적으로 보정 중
```

**예측이 빗나간 점을 기록해 둡니다.** 공분산 비만으로 계산하면 IMU 가중치
4.8%, 드리프트 4.4 deg/시간이 나와야 하는데 실측은 12.8 이었습니다. 실효
가중치가 14% 라는 뜻이고, robot_localization 의 프로세스 노이즈가 추가로
작용하기 때문입니다. **공분산 비 = 가중치 비 라는 계산은 이 필터에서 대략적인
방향만 맞습니다.** 더 낮추려면 `imu_angular_cov` 를 5.0 으로 올리면 됩니다
(대략 3 deg/시간 예상, 미검증).

**근본 해결은 편향 자체를 빼는 것입니다.** 이 드라이버 버전에는 자이로 편향
캡처 서비스가 없습니다 (`ros2 service list` 에 파라미터 서비스 6개뿐).
보정 노드를 직접 써야 하고, 편향은 온도에 따라 변하므로 기동마다 재측정이
필요합니다.

**같은 날 오후에 1.0 으로도 부족해서 5.0 으로 다시 올렸습니다.** 최종 상태:

| `imu_angular_cov` | 드리프트 (정지, 실측) | 20분 누적 |
|---|---|---|
| 0.01 (드라이버 기본) | +86.3 deg/시간 | 28.8 deg |
| 1.0 | +12.8 / -13.8 deg/시간 | 4.6 deg |
| **5.0 (현재)** | **-3.2 deg/시간** | **1.1 deg** |

편향 자체는 -0.0249 ~ -0.0276 deg/s 로 계속 같습니다. 바뀐 건 EKF 가 그걸
믿는 비율뿐입니다. 5.0 에서는 예측(약 3 deg/시간)과 실측이 정확히 맞았습니다.

**주의 — 부호가 기동마다 다릅니다.** cov 1.0 일 때 오전은 +12.8, 오후는
-13.8 이었습니다. 크기는 같은데 부호가 반대입니다. 편향은 두 번 다 음수였는데
드리프트 부호가 갈린 이유를 못 찾았습니다. 크기만 보고 판단하세요.

### 10-9. 라이다는 바닥에서 1.048 m — 단일 링의 사각지대

**문서 어디에도 없던 값이라 실측했습니다.**

```
base_footprint -> base_link      0.102 m
base_link      -> velodyne_link  0.946 m   (기둥 위)
------------------------------------------
바닥에서 라이다까지              1.048 m
```

`/scan` 은 VLP-16 의 **수평에 가장 가까운 링 한 줄**입니다
(`velodyne_vlp16.yaml` 의 `ring: -1` = 자동 선택, 약 ±1도). 그래서 바닥과
나란한 **높이 1 m 짜리 평면 한 장**만 봅니다.

| 거리 | 스캔 높이 |
|---|---|
| 1 m | 1.03 ~ 1.07 m |
| 5 m | 0.96 ~ 1.14 m |
| 10 m | 0.87 ~ 1.22 m |
| 20 m (max_laser_range) | 0.70 ~ 1.40 m |

**따라오는 결론 두 가지:**

1. **사람은 확실히 보입니다.** 1 m 면 무릎이 아니라 몸통입니다. 매핑 중
   사람이 서 있으면 유령 벽이 남습니다. 특히 **로봇 옆에서 같이 걷는 사람이
   최악**입니다 — 시야에서 안 벗어나니 지워질 기회가 없습니다. 잠깐 지나가는
   사람은 레이트레이싱으로 대부분 지워집니다.
2. **1 m 아래는 지도에 아예 없습니다.** 책상(0.7~0.75 m), 의자, 박스,
   파렛트, 대차, 문턱, 바닥 케이블 — 전부 "빈 공간"으로 찍힙니다. **Nav2 를
   돌리면 로봇이 책상 다리로 그냥 들어갑니다.** §10-13 의 RealSense D455 가
   필요해지는 지점이 정확히 여기입니다.

참고: 다른 세션에서 이 값을 **0.3527 m 로 잘못 인용한 기록이 있었습니다.**
어느 프레임 쌍에서도 나오지 않는 값입니다. TF 로 직접 확인하세요.

### 10-10. 철창(메시 펜스) 환경 — Nav2 에서 터질 것

실험실이 철창으로 둘러싸여 있습니다. 라이다 관점에서 계산하면:

| | 값 |
|---|---|
| 방위각 분해능 (600 RPM) | 0.2도 → 5 m 에서 점 간격 **17 mm** |
| 빔 지름 (발산 약 3 mrad) | 5 m 에서 **15 mm** |

**틈이 2~3 cm 보다 크면 빔이 그냥 통과합니다.** 실측에서도 철창은 연속선이
아니라 **끊긴 점선**으로 찍혔습니다. 강철 자체는 905 nm 를 잘 되쏘므로 재질
문제는 아닙니다 (다만 아주 매끈한 스테인리스는 정반사로 안 돌아올 수 있음).

**투과량을 실측했습니다 (2026-08-14).** 빔 경로를 격자로 따라가, 도착점 전에
이미 점유 칸을 지났으면 '투과'로 판정하는 방식입니다. 3회 반복:

```
벽에서 50 cm 초과인 스캔 점        약 22%
  그중 뭔가를 뚫고 지나감 (투과)   53 ~ 55%
  그중 경로에 벽 없음 (미탐색)     45 ~ 47%
```

**스캔 점의 약 11% 가 항상 지도와 안 맞습니다.** 미탐색 영역은 주행하면
채워지지만 **투과분은 안 없어집니다** — 지도에 없는 철창 뒤 물체를 계속
보게 되기 때문입니다. 이만큼 스캔매칭의 근거가 상시로 약합니다.

**오늘 매핑에는 큰 문제가 없었지만 Nav2 에서 세 가지가 터집니다:**

1. **전역 planner 가 틈으로 경로를 냅니다.** 점 사이 빈 칸은 planner 눈에
   통과 가능 공간입니다. 가장 위험합니다.
2. **로컬 코스트맵이 철창을 스스로 지웁니다.** 틈으로 빠진 빔이 "이 경로는
   비어 있다"고 보고하면서 옆 막대 칸까지 지웁니다. 난간·펜스의 고전적 문제.
3. **AMCL 이 각도에 따라 흔들립니다.** 시차 때문에 막대 겹침 패턴이 바뀝니다.

**대책 (우선순위대로, 전부 미검증 — Nav2 단계에서 할 것):**

1. **저장한 PGM 을 편집해 철창을 실선으로 메우기.** 지도는 그냥 이미지
   파일입니다. 편법이 아니라 현장 표준입니다. 1번이 완전히 해결되고, AMCL 도
   **좋아집니다** — likelihood field 는 스캔 점에서 가장 가까운 점유 칸까지의
   거리로 채점하는데, 막대가 실선 위에 얹히면 거리 0 이 됩니다.
2. **Nav2 keepout 필터 (`costmap_filters`).** 원본 지도를 보존합니다.
3. **`inflation_radius` 를 틈보다 크게.** 2번 문제를 완화. 좁은 통로가 있으면
   같이 막히니 주의.
4. **`max_laser_range` 축소** (현재 20 m). 철창 너머가 지도에 안 들어오게.

**원본(점선 그대로)을 반드시 남겨두세요.** 편집본과 비교할 기준이 됩니다.

### 10-11. 무선 DDS 열화 — 노트북 화면만 나쁩니다

§10-1 의 Discovery Server 구성으로 원격 RViz 는 되지만, **노트북 수신 품질이
젯슨 로컬보다 크게 나쁩니다.**

```
젯슨 로컬 /scan   9.9 Hz
노트북 수신       1.8 Hz   (간격 0.038 ~ 1.622 s 로 매우 불규칙)
ICMP ping         0% 손실, RTT 평균 2.5 ms   <- 와이파이 자체는 건강
```

노트북 RViz 로그에 `Message Filter dropping message: frame 'velodyne_link'
... queue is full` 이 반복되고, `ros2 param list` 같은 **왕복 서비스 콜은
자주 타임아웃**합니다. pub/sub 은 되는데 서비스가 안 되는 게 특징입니다.
`ros2 topic list` 가 처음 몇 번 비어 나오다 나중에 채워지는 것도 같은 뿌리
입니다 — 데몬이 서버 응답을 받기까지 시간이 걸립니다. **안 나오면 몇 번 더
쳐보세요.**

**중요 — 이건 지도 품질과 무관합니다.** 지도는 젯슨에서 로컬 9.9 Hz 스캔으로
만들어지고 노트북은 완성품을 늦게 받아 그릴 뿐입니다. **화면이 실제 지도보다
나쁘게 보입니다.**

젯슨 쪽 slam_toolbox 의 같은 `queue is full` 메시지는 **기동 직후 TF 가 아직
안 올라왔을 때 1회**뿐이었습니다 (`Registering sensor` 보다 먼저 찍힘).
세션 전체 누적 8회. 젯슨은 정상입니다. **이 메시지를 노트북에서 봤다고 해서
SLAM 문제로 단정하지 마세요.** 2026-08-14 에 실제로 그렇게 오진한 기록이
있습니다 — 젯슨에 접근하지 않고 노트북 쪽만 보면 그렇게 결론납니다.

**갈림길은 하나입니다: 젯슨에서 TF 를 직접 쪼개 보세요** (§10-8 의
`map->odom` / `odom->base_footprint` 분해). 네트워크 아티팩트는 젯슨 자신의
TF 트리를 돌릴 수 없습니다.

**추가 (2026-08-24) — 노트북 RViz에서 "No map received".** TF는 정상(`Global
Status: Ok`)인데 `Map` 디스플레이만 계속 `Status: Warn / No map received`인
경우가 있었습니다. `map_server`는 지도를 **활성화 시점에 딱 한 번만**
transient_local로 발행합니다 — 그 순간 노트북 쪽 구독이 아직 안 맺어져
있었거나 무선 구간에서 그 패킷이 유실되면, 이후로는 재발행 기회 자체가
없습니다(§10-11의 노트북측 수신 열화가 원인 제공). **해결: RViz가 이미
구독을 맺은 상태에서 `map_server`를 한 번 더 activate 시켜 재발행을
유도.**

```bash
ros2 lifecycle set /map_server deactivate
ros2 lifecycle set /map_server activate
```

TF는 이미 살아있는데 유독 Map만 비어 있다면 이 순서로 의심할 것.

### 10-12. 2026-08-14 오전 결과

| 계통 | 결과 |
|---|---|
| 구동부 | `통신 정상`, `/wheel_odometry` 30.0 Hz |
| VLP-16 | `/scan` 9.8~9.9 Hz 유지 |
| IMU GV7 | `/imu/data` 100.0 Hz |
| EKF | 드리프트 86.3 → **12.8 deg/시간** (§10-8) |
| slam_toolbox | 주행 중 `map->odom` 능동 보정 확인 (-7.4도) |
| 원격 RViz | 연결됨. 수신 열화 있음 (§10-11) |
| 젯슨 부하 | load 2.1~3.4, `/scan` 10 Hz 유지 (VSCode 켠 채) |

저장한 지도 `~/maps/tetra_map_20260814_precleanup.{pgm,yaml}`:
629 × 447 @ 0.05 m = 31.5 × 22.4 m. **부분 지도입니다** — 점유 0.5%,
자유 6.8%(47.5 m^2), **미지 92.8%**. 실험실 정리 전에 찍은 것이라 오후
재매핑의 비교 기준으로만 씁니다.

**재부팅 없이 세션만 다시 여는 경우**, 라이다 프로파일과 `latency_timer` 는
살아 있습니다. discovery server 와 launch 만 다시 띄우면 됩니다. USB 를
뽑았거나 재부팅했다면 §10-3 대로 `latency_timer` 부터 다시 하세요.

### 10-13. 정렬을 눈이 아니라 숫자로 재는 법 — `check_scan_map_fit.py`

"RViz 에서 주황(스캔)이 검정(지도)에 안 붙는다" 를 눈으로 판단하면 **거의
항상 틀립니다.** 원격 RViz 는 실제보다 훨씬 나쁘게 보이기 때문입니다
(§10-11). 2026-08-14 에 이것 때문에 두 번 헛다리를 짚었습니다.

```
python3 src/tetra_dsv_s_bringup/scripts/check_scan_map_fit.py
```

스캔 점에서 가장 가까운 점유 칸까지의 거리 분포를 냅니다. **반드시 젯슨에서**
(네트워크를 안 건너고) 돌리세요.

**판정 기준: 중앙값 5.0 cm = 격자 한 칸 = 이보다 좋아질 수 없는 값입니다.**
10 cm 를 넘기 시작하면 그때가 진짜 문제입니다.

평균은 보지 마세요 — 철창 투과분(§10-10)과 미탐색 영역이 꼬리를 만들어
9~30 cm 사이를 오갑니다. 중앙값이 훨씬 안정적입니다.

**이 스크립트를 쓸 때 당한 함정:** TF 를 `rclpy.time.Time()`("가장 최근")으로
조회하면 안 됩니다. **스캔의 타임스탬프로** 조회해야 합니다. 회전 중에는
시간차가 그대로 오차가 됩니다 — 0.3 rad/s 에서 100 ms 만 어긋나도 1.7도,
5 m 거리에서 15 cm 입니다. 지금 스크립트는 고쳐져 있고, 스캔 시각의 TF 를
못 찾으면 경고를 찍습니다.

### 10-14. 회전 스케일 실측 — 휠은 정확, IMU 는 11% 부족

로봇을 제자리에서 50도 돌리며 네 값을 동시에 기록했습니다 (2026-08-14).

```
휠 오도메트리 (/wheel_odometry)   -50.00 deg
EKF (odom->base_footprint)        -49.98 deg
SLAM (map->base_footprint)        -49.98 deg   <- 참값 기준
IMU 자이로 적분 (원본 축)         +44.63 deg

휠  / SLAM = 1.0003   (오차 +0.03%)
EKF / SLAM = 1.0000   (오차  0.00%)
```

**결론 두 가지:**

1. **휠 오도메트리 회전 스케일은 정확합니다 (0.03%).** 트레드 폭 상수를
   의심할 필요가 없습니다. 이걸 확인해 두면 다음에 방위 오차가 나왔을 때
   후보 하나를 바로 지울 수 있습니다.
2. **IMU 는 회전량을 11% 적게 봅니다** (44.63 vs 50). 부호가 반대인 것은
   §10-2 의 `roll 180도` 장착대로라 정상입니다. 크기 부족은 별개 문제입니다.
   지금은 `imu_angular_cov: 5.0` 으로 IMU 비중이 낮아 EKF 가 SLAM 과 소수점
   까지 일치하므로 영향이 없습니다. **IMU 비중을 다시 올릴 일이 생기면
   이 11% 를 먼저 해결해야 합니다.**

### 10-15. 방위 오차 — 복귀하면 사라졌지만, 근거가 약합니다

**⚠ 이 절은 한 번 "해결"로 단정했다가 정정한 것입니다. 아래 한계를 먼저
읽으세요.**

**한계.** 이 결론을 낸 세션은 **pose graph 노드가 13개, 로봇 이동 범위가
1.32 x 0.33 m** 였습니다. 긴 경로를 돌아 원점에서 만나는 진짜 loop closure 가
아니라, **거의 안 움직인 로봇이 출발 자세로 돌아온 것**에 가깝습니다. 그
자세에서 스캔이 자기가 만든 지도와 맞는 것은 어느 정도 당연합니다. 정렬
0.0 cm 도 그래서 나온 값일 수 있습니다. 4~5도 오차가 났던 세션들은 훨씬 많이
주행했던 때이므로, **"많이 돌면 쌓이고 복귀하면 지워진다" 를 이 데이터로
확정할 수 없습니다.**

**그래도 복귀는 하세요.** 손해 볼 것이 없고 아래 수치가 실제로 좋아졌습니다.
다만 다음에 검증할 때는 **주행량을 충분히 확보한 세션에서** 재보세요
(pose graph 노드 수를 `/slam_toolbox/graph_visualization` 으로 확인 가능).

관측된 변화 (2026-08-14 17:30):

```
                     복귀 전        복귀 후
정렬 중앙값           5.0 cm    ->   0.0 cm
정렬 평균             6.5 cm    ->   1.8 cm
10 cm 이내           85.6%      ->  98.0%
자세 오차        yaw -5.25 deg  ->  yaw +0.00 deg, x +0.00, y +0.00
벽에서 먼 점         21.9%      ->   0.4% (3점, 전부 철창 투과)
지도의 점유 칸        1,854     ->     830
```

점유 칸이 절반 이하로 준 것은 손실이 아니라, 누적 오차로 **겹쳐 그려졌던
벽이 하나로 합쳐진 것**으로 보입니다.

**부호가 세션마다 바뀌는 것은 확실한 관측입니다.** 오후엔 +3.5 ~ +4.0도,
저녁엔 -5.25도. 라이다 장착 방위 같은 **고정 오프셋이었다면 부호가 늘 같아야
합니다.** 부호가 갈린다는 것은 고정 오차가 아니라 주행 중 생기는 것이라는
뜻입니다 — 이 부분은 위 한계와 무관하게 유효합니다.

**아래는 배제한 후보들입니다.** 다음에 비슷한 증상이 나오면 같은 순서로
지우면 됩니다 — 전부 실측으로 확인했습니다.

| 후보 | 배제 근거 |
|---|---|
| 자이로 편향 누적 | 드리프트를 13.8 -> 3.2 deg/시간으로 4배 줄였는데 4도는 그대로 |
| 휠 회전 스케일 | §10-14, 오차 0.03% |
| slam_toolbox 과부하 | CPU 5%, 스캔 드롭 0회, `/map` 정확히 2.0초 주기 |
| 측정 스크립트 시간차 결함 | §10-13 대로 고쳤는데 값 안 변함 |
| 라이다 장착 방위 오차 | 부호가 세션마다 바뀜 (고정 오프셋이면 불가능) |
| 원격 RViz 렌더링 지연 | 젯슨 로컬 측정에서도 동일하게 나옴 |

**주의 — 오전에 "13.8 deg/시간 x 18분 = 4.1도" 로 설명했던 것은 우연의
일치였습니다.** 오후에 드리프트를 4배 줄였는데도 같은 4도가 나오면서 그
설명이 깨졌습니다. 숫자가 맞아떨어진다고 원인이 확정된 게 아닙니다.

**남은 잔여 오차의 정체.** 복귀 후 벽에서 50 cm 넘게 떨어진 점이 0.4%(3점)
남았고 **셋 다 철창 투과**였습니다 (§10-10). 이건 원리상 안 없어집니다.

### 10-16. 2026-08-14 오후 결과 — 실험실 지도 완성

실험실을 정리한 뒤 새로 매핑했습니다.

**⚠ 먼저 알아야 할 것 — 실험실은 2.4 x 3.0 m = 7.2 m^2 밖에 안 됩니다.**
(시뮬의 `empty_room.sdf` 가 이 크기로 만들어진 이유입니다, §7)

이걸 모르고 하루 종일 지도를 키웠는데, **자유 공간의 대부분이 철창 투과로
생긴 유령이었습니다.** 발각된 경위와 수치는 §10-17 에 있습니다. 반드시
읽으세요.

**메인 지도는 `~/maps/tetra_lab_main.{pgm,yaml}` 입니다**
(= `tetra_lab_r4` 와 같은 내용, `max_laser_range: 4.0` 으로 찍은 것).

```
~/maps/tetra_lab_main.pgm / .yaml     <- 메인. Nav2 는 이걸 씁니다
     177 x 111 @ 0.05 m = 8.8 x 5.6 m
     점유(벽)      754 칸
     자유                    = 15.6 m^2   (실제 방 7.2 m^2 + 잔여 누수)
     정렬 중앙값 0.0 cm, 평균 3.0 cm, 10 cm 이내 93.8%
```

**아직 손볼 것이 남았습니다.** 자유 공간 15.6 m^2 중 절반가량이 여전히 철창
누수입니다 (오른쪽, 좌상 대각, 좌하 대각 방향). 방 경계를 검은 선으로 닫는
편집이 예정돼 있습니다 (아래 편집 주의 + §10-10).

비교용으로 남긴 것 (전부 `~/maps/`):
- `tetra_lab_r4_snapshot.*` — 같은 설정, 추가 주행 **전**. 자유 7.5 m^2 로
  실제 방과 가장 가깝지만 벽이 성깁니다(점유 287 칸). 추가 주행으로 벽은
  754 칸이 됐고 누수는 2배가 됐습니다 — **주행량과 누수량은 비례합니다**
- `tetra_lab_main_old_20m.*` — `max_laser_range: 20.0` 시절 메인.
  자유 29.2 m^2 (실제의 4배). **Nav2 에 쓰지 마세요.** 유령 자유 공간의
  표본으로만 두었습니다
- `tetra_lab_20260814_v2.*`, `tetra_lab_20260814.*` — 오후. 전부 20 m 시절
- `tetra_map_20260814_cov1p0.*` — `imu_angular_cov: 1.0` 시절
- `tetra_map_20260814_precleanup.*` — 오전, 실험실 정리 전

**저장된 지도의 픽셀값은 딱 3종입니다** (`mode: trinary`):
`0` = 점유 / `205` = 미지 / `254` = 자유.

**지도 손보기 (계획).** 철창이 점선으로 찍히므로 PGM 을 이미지 편집기로 열어
실선으로 메우기로 했습니다 (§10-10 의 1번 대책).

- **GIMP 의 연필(Pencil) 도구를 쓰세요. 붓(Paintbrush) 말고요.** 붓은
  안티앨리어싱으로 가장자리에 중간 회색을 만드는데, trinary 임계값
  (`occupied_thresh: 0.65`, `free_thresh: 0.25`) 때문에 그 중간값이 전부
  **미지**가 되어 벽 주변에 오히려 구멍이 생깁니다
- 순수 검정 `(0,0,0)`, 그레이스케일 8비트 유지
- **크기 변경·자르기 금지** — YAML 의 `origin` 이 이미지 크기 기준입니다
- **미지(회색)를 자유(흰색)로 칠하지 마세요.** 가본 적 없는 곳을 "안전하다"고
  표시하는 것이고, Nav2 가 거기로 경로를 냅니다. 자유 공간은 **주행으로만**
  만들 수 있습니다
- 편집본은 다른 이름으로 저장하고 YAML 을 복사해 `image:` 만 바꾸세요

### 10-17. ⚠⚠ 지도의 자유 공간 대부분이 유령이었습니다 — `max_laser_range`

**하루 작업 중 가장 큰 실수였습니다. 다음 사람은 이걸 먼저 확인하세요.**

**증상.** 지도가 잘 나왔다고 생각했는데, 사용자가 "실험실은 2.4 x 3.0 m" 라고
말해주고 나서야 이상함이 드러났습니다.

```
실제 방                       2.4 x 3.0 m =  7.2 m^2
지도의 자유 공간                            = 29.2 m^2   (4배)
가장 큰 연결 자유 영역        17.65 x 5.75 m
로봇이 실제로 주행한 범위      1.32 x 0.33 m   <- pose graph 13 노드
```

**원인.** `slam_toolbox.yaml` 의 `max_laser_range` 가 20.0 이었습니다. 방보다
7배 큰 값입니다. 철창 틈으로 빠져나간 빔이 **20 m 앞까지 "이 경로는 비어
있다" 고 보고**하고, slam_toolbox 는 그대로 자유 공간으로 칠했습니다.
RViz 에서 사방으로 뻗던 밝은 회색 부챗살이 전부 이것이었습니다.

**Nav2 관점에서 치명적입니다.** planner 는 자유 공간을 "갈 수 있는 곳" 으로
봅니다. 철창 너머 20 m 까지 자유이니 **로봇이 철창으로 돌진하는 경로를
냅니다.**

**주행량과 무관합니다. 오히려 많이 돌수록 심해집니다** — 보는 각도가 늘면
투과 방향도 늘어납니다. 실측: 추가 주행 후 자유 공간 7.5 -> 15.6 m^2
(벽은 287 -> 754 칸으로 좋아짐). 둘이 같이 늘어납니다.

**수정.** `max_laser_range: 20.0 -> 4.0`. 방 대각선이 3.84 m 이므로 방 전체는
다 보면서 그 너머는 안 그립니다. 효과:

```
              20.0 m           4.0 m
지도 캔버스   22.4 x 15.0 m    6.5 x 3.2 m
자유 공간     29.2 m^2         7.5 m^2      (실제 7.2 m^2)
```

**교훈 — 지도를 믿기 전에 두 가지를 대조하세요:**

1. **자유 공간 면적 vs 실제 공간 면적.** 크게 다르면 유령입니다.
2. **로봇이 실제로 주행한 범위.** `/slam_toolbox/graph_visualization` 의 노드
   위치로 바로 확인됩니다. 주행 범위보다 자유 공간이 훨씬 넓으면 전부
   레이캐스팅으로 칠해진 것이지 가본 곳이 아닙니다.

`max_laser_range` 는 **공간 크기에 맞춰야 하는 값**입니다. 공간이 바뀌면
반드시 다시 잡으세요.

### 10-18. 다음에 할 것 — 실기 Nav2 실행 계획

**Nav2 패키지는 이미 전부 설치돼 있습니다** (`ros-humble-nav2-*` 1.1.20,
amcl / bringup / controller / planner / mppi / collision-monitor 등 20여 개).
apt 설치 불필요. **없는 것은 이 로봇용 설정과 launch 뿐입니다.**

#### 먼저 알아야 할 치수 (URDF 실측, 2026-08-14)

```
로봇 본체     0.485 x 0.430 m   base_link 기준 x -0.318 ~ +0.167, y +-0.215
외접 반지름   0.384 m           제자리 회전에 지름 0.77 m 필요
바퀴          radius 0.1015, separation 0.377
방            2.40 x 3.00 m
```

**방 짧은 변(2.4 m)이 로봇 회전 지름의 3.1배뿐입니다.** Nav2 기본값을 그대로
쓰면 안 됩니다. 기본 `inflation_radius: 0.55` 면 양쪽 벽에서 1.1 m 가 고비용
영역이 되어 planner 가 경로를 못 찾습니다.

#### 0단계 — 지도 경계 닫기 (선행 필수)

`tetra_lab_main` 의 자유 공간 15.6 m^2 중 절반이 철창 누수입니다
(실제 방 7.2 m^2). **닫기 전에는 Nav2 를 켜지 마세요** — planner 가 철창을
뚫는 경로를 냅니다 (§10-17, §10-10).

GIMP **연필(Pencil)** 로 방 경계에 검은 선을 긋습니다(붓 금지 — 안티앨리어싱).
그 뒤 "방 안에서 도달 못 하는 자유 공간을 미지(205)로 되돌리는" 정리는
연결영역 채우기로 자동화 가능합니다. **검증 기준: 자유 공간이 7.2 m^2 근처로
떨어질 것.**

#### 1단계 — `config/nav2_params.yaml` 신규 작성

시뮬 config 는 못 씁니다: ① `use_sim_time: True` ② 시뮬 패키지가 젯슨에
sparse-checkout 제외라 파일 자체가 없습니다 (§1). `nav2_bringup` 기본값에서
시작하세요.

**좁은 방 때문에 반드시 고칠 값:**

| 항목 | 기본값 | 이 방에서 |
|---|---|---|
| `footprint` | 원형 0.22 | `[[0.167,0.215],[0.167,-0.215],[-0.318,-0.215],[-0.318,0.215]]` |
| `robot_radius` | 0.22 | **쓰지 말 것** — 앞뒤 비대칭이라 footprint 로 |
| `inflation_radius` | 0.55 | **0.20 ~ 0.25** |
| `local_costmap` width/height | 3.0 | **1.5 ~ 2.0** (방보다 크면 무의미) |
| `max_vel_x` | 0.26 | **0.15** (teleop 에서 쓰던 값) |
| `xy_goal_tolerance` | 0.25 | **0.10** |
| AMCL `laser_max_range` | 100 | **4.0** (`max_laser_range` 와 맞출 것, §10-17) |

#### 2단계 — `launch/navigation.launch.py` 신규 작성

`slam.launch.py` 에서 slam_toolbox 만 빼고 셋을 더하면 됩니다.

```
slam.launch.py            navigation.launch.py
----------------------    ----------------------
drive.launch.py      ->   그대로
velodyne 3노드       ->   그대로
imu                  ->   그대로
ekf                  ->   그대로
slam_toolbox         ->   제거
                          + map_server (tetra_lab_main.yaml)
                          + amcl
                          + nav2 스택 (planner/controller/bt_navigator)
```

**TF 소유권이 바뀝니다 — `map -> odom` 을 slam_toolbox 대신 AMCL 이 냅니다.**
둘을 동시에 띄우면 부모가 둘이 되어 트리가 깨집니다 (§10-5).

#### 3단계 — 첫 주행 안전 절차

1. **RViz 의 `2D Pose Estimate` 로 초기 위치를 정확히 지정.** §9 에 "AMCL 의
   초기 위치 없는 전역 재추정은 미검증" 이라고 적혀 있고, 빈 직사각형 방은
   특히 불리합니다.
2. **`Nav2 Goal` 은 1 m 앞부터.** 방이 3 m 입니다.
3. **비상정지에 손을 얹고 하세요.** 라이다가 1.048 m 높이라 **책상·의자·박스가
   지도에 없습니다** (§10-9). 자율주행에서는 로봇이 스스로 그리로 갑니다.
   바닥에 아무것도 없는 상태로 시작하고, 장애물 회피 시험은 **높이 1 m 넘는
   물체**로 하세요.

**RealSense D455 (이미 장착, 드라이버 미설치) 가 1 m 사각지대의 유일한 근본
대책입니다.** 이번에 붙일지 결정이 필요합니다.

#### 그 밖에 남은 것

- **방위 오차 (§10-15).** "복귀하면 사라진다" 는 관측은 있으나 **13 노드짜리
  데이터라 근거가 약합니다.** 주행량이 충분한 세션에서 재검증하세요.
- **IMU 회전량 11% 부족 (§10-14).** 지금은 IMU 비중이 낮아 무해하지만,
  비중을 올릴 일이 생기면 먼저 해결해야 합니다.
- **드리프트 부호가 기동마다 갈리는 것 (§10-8).** 크기는 일정한데 부호가 다릅니다.
- **IMU X/Y 축 확정.** 지금은 회피 설계로 넘어갔습니다 (§10-2).
- **PL2303 시리얼의 정체.** 커널 드라이버가 안 올라와 포트가 안 생깁니다.
  전원/센서 보드(CN24)일 가능성이 있습니다.
- **NUC 원상복구 절차.** 지금 USB 허브를 젯슨이 가져간 상태입니다. 업체에
  돌려줘야 할 때를 대비해 배선 사진을 남겨두는 게 좋습니다.

**진단 도구 (전부 젯슨에서 돌릴 것 — 네트워크를 건너면 값이 왜곡됩니다):**

| 스크립트 | 용도 |
|---|---|
| `probe_drive_board.py` | 구동보드 통신. ROS 문제인지 시리얼 문제인지 30초에 판정 |
| `check_scan_map_fit.py` | 스캔-지도 정렬. **중앙값 5 cm 가 정상** (§10-13) |
| `check_pose_error.py` | 자세가 얼마나 틀어졌는지 (yaw/x/y 로 분해) |
| `check_scan_through.py` | 먼 스캔 점이 철창 투과인지 미탐색인지 (§10-10) |
| `check_imu_mounting.py` | IMU 장착 축 (§10-2) |

### 10-19. 2026-08-19 — 0/1/2단계 완료, 3단계(첫 주행) 직전

§10-18 계획대로 0~2단계를 마쳤습니다. 3단계(첫 주행)만 남았고, 이건 로봇이
실제로 움직이므로 사용자 확인 없이 진행하지 않습니다.

**0단계 — 지도 경계 닫기.** `~/maps/tetra_lab_closed.{pgm,yaml}` 신규 생성
(원본 `tetra_lab_main` 은 그대로 둠). 벽 네 변을 기존 점유칸 중앙값 위치에
실선으로 닫고, 도달 불가능한 자유 공간은 전부 미지로 되돌렸습니다(반대
방향은 한 번도 안 함 — 미지를 자유로 칠한 적 없음). 사용자 확인 결과 위/아래
회색 덩어리가 실제 장애물이라 사각형으로 채우고 벽까지 붙였습니다.

```
자유 공간   15.59 -> 7.37 m^2   (실제 방 7.2 m^2 근접, 검증 통과)
```

방 짧은 변이 지도에서 2.70 m 로 나온 것(§10-16 은 2.4 m 기록) — **사용자
확인 결과 §10-16 의 2.4 m 가 오기였습니다.** 지도 값(2.70 m)이 맞습니다.
문이 사방 중 한 곳 막혀 있는 것도 의도된 것입니다(실험실 안에서만 주행).

**1단계 — `config/nav2_params.yaml` 신규 작성.** `nav2_bringup` 기본값에서
시작해 §10-18 표의 6개 값을 반영했고, 표에 없던 수정도 했습니다:

- `robot_base_frame` 을 전부 `base_link` -> `base_footprint` 로 고침. 기본값
  그대로면 TF 를 못 찾아 조용히 멈췄을 것 (§10-5 와 동일한 함정).
- `odom_topic` 을 `/wheel_odometry` 가 아니라 EKF 가 실제로 내는
  `/odometry/filtered` 로.
- `map_server.yaml_filename` 을 `tetra_lab_closed.yaml` 로 하드코딩.
- `planner_server.allow_unknown: true -> false`. 0단계 원칙("미지는 갈 수
  있는 곳이 아니다")을 planner 에도 적용.
- `local_costmap`/`global_costmap` 의 `obstacle_layer.scan.raytrace_max_range`
  `/obstacle_max_range` 를 기본 3.0/2.5 에서 3.5/3.0 으로. §10-10 의 "로컬
  코스트맵이 철창을 스스로 지운다"는 문제가 정적 지도(0단계로 막음)뿐 아니라
  **실시간 코스트맵에도 똑같이 적용됩니다** — 미검증, 실주행에서 벽이
  지워지면 더 줄일 것.
- 선속도 0.05 m/s, 각속도 0.2 rad/s (사용자 지정, 2026-08-19). `max_vel_x`,
  `max_speed_xy`, `velocity_smoother` 의 선속도, `behavior_server` 의
  `max_rotational_vel`/`min_rotational_vel` 까지 전부 일관되게 맞췄습니다
  (behavior_server 기본값 1.0/0.4 를 그대로 두면 min > max 가 되어 Spin
  복구가 아예 실패했을 것).

**2단계 — `launch/navigation.launch.py` 신규 작성.** `slam.launch.py` 에서
slam_toolbox 를 빼고 map_server + AMCL + Nav2 스택(controller/smoother/
planner/behavior_server/bt_navigator/waypoint_follower/velocity_smoother)과
lifecycle_manager 두 개(localization, navigation, 둘 다 autostart: true)를
더했습니다. 속도 파이프라인은 `nav2_bringup` 표준과 동일하게
`controller_server -> /cmd_vel_nav -> velocity_smoother -> /cmd_vel` 로
리매핑 — 이걸 빼먹으면 가감속 제한 없이 원속도가 그대로 나갑니다.

**젯슨 실기 검증 중 잡은 버그 두 개 — 둘 다 코드 리뷰로는 못 잡고 실제로
띄워봐야 나왔습니다:**

1. **`local_costmap.width/height` 는 정수여야 합니다.** §10-18 표가 권장한
   `1.5` 를 그대로 넣었더니 `controller_server` 가 기동 직후
   `InvalidParameterTypeException`(`height` is of type integer, setting it
   to double is not allowed)으로 죽었습니다. 이 Nav2 빌드(1.1.20)가
   width/height 를 정수로 선언해 둔 탓입니다 — `nav2_bringup` 기본값이
   `3.0` 이 아니라 `3` 인 이유가 이것이었습니다. 표 범위(1.5~2.0)의 정수
   상한인 **2** 로 바꿔서 해결.
2. **`voxel_layer` 는 이 로봇에 안 맞습니다.** `local_costmap` 기본
   플러그인은 3D `voxel_layer` 인데, `z_voxels(16) x z_resolution(0.05)
   = 0.8 m` 높이까지만 격자를 만듭니다. 라이다 실측 장착 높이가 1.048 m
   (§10-9)라 센서 원점 자체가 격자 천장보다 높아서 `Sensor origin ... out
   of map bounds` 경고와 함께 raytrace 가 전혀 안 됐습니다. `/scan` 이
   VLP-16 한 링짜리 2D `LaserScan` 이라 애초에 3D voxel 이 필요 없어서,
   `global_costmap` 이 이미 쓰던 `obstacle_layer`(2D)로 통일해 해결.

**검증 결과 (젯슨에서 직접 launch, cmd_vel 은 안 보냄):**

```
노드 20개 전부 활성화 (lifecycle_manager_localization/navigation 둘 다
  "Managed nodes are active")
odom -> base_footprint         정상 (EKF)
base_footprint -> velodyne_link  [0, 0, 1.048]  <- §10-9 실측치와 정확히 일치
/wheel_odometry, /scan, /imu/data, /odometry/filtered, /map  전부 발행 확인
/cmd_vel_nav: controller_server -> velocity_smoother  리매핑 정상
/cmd_vel: 구독자 tetra_drive 하나뿐  정상
map -> odom  없음 (정상 — AMCL 이 초기 위치 대기 중, §10-18 3단계에서 사람이
  RViz "2D Pose Estimate" 로 직접 지정해야 함)
```

**남은 것 — 3단계(첫 주행).** RViz 2D Pose Estimate 로 초기 위치 지정 후
Nav2 Goal 은 1 m 앞부터, 비상정지에 손을 얹고, 바닥 비운 채로 시작
(§10-18 절차 그대로). 노트북 RViz 는 discovery server 우회가 필요합니다
(§10-1) — 아직 안 띄웠습니다.

### 10-20. 2026-08-19 오후 — 3단계: 첫 자율주행 성공

점심 먹고 돌아와 젯슨을 재부팅한 상태에서 이어갔습니다. §10-1 세 번째 함정
(`ROS_DISCOVERY_SERVER` 안 걸고 launch)을 실제로 한 번 더 겪었고, 그걸 고친
뒤로는 순조로웠습니다.

**속도.** 실행 중인 노드에서 직접 확인 (`ros2 param get`):

```
FollowPath.max_vel_x       0.05 m/s
FollowPath.max_vel_theta   0.2  rad/s
velocity_smoother          [0.05, 0.0, 0.2] / [-0.05, 0.0, -0.2]
```

**⚠ `FollowPath.min_vel_x` 가 0.0 입니다 — 후진을 아예 못 냅니다.** 목표가
로봇 뒤쪽에 있으면 후진이 아니라 **제자리 회전 후 전진**으로 도달합니다.
오늘은 이 상태로 진행했고(앞 공간을 비우는 쪽을 택함), **후진이 실제로
필요해지면 `min_vel_x` 를 음수로 바꿔야 합니다** — 아직 안 했습니다.

**RViz 도구.** `slam.rviz` (매핑용으로 만든 기존 설정)에는 `2D Pose
Estimate`/`Nav2 Goal` 도구가 없습니다. apt 로 이미 깔려 있는
`/opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz` 를 그대로
썼습니다 — `SetInitialPose`, `GoalTool`, `Navigation 2` 패널이 다 들어있고
토픽 이름도 저희 스택과 그대로 맞아서 수정 없이 됩니다. 노트북에도 같은
버전의 nav2_bringup 이 apt 로 깔려 있으면 같은 경로에 있습니다.

**초기 위치.** 로봇을 매핑 시작했던 자리(대략 지도 원점)로 손으로 옮긴 뒤
`2D Pose Estimate` 로 지정. 처음엔 `Failed to transform initial pose in
time (extrapolation)` 경고가 떴지만 무해했습니다 — 그 직후 `Setting pose`
로 이어졌고 전체 스택이 정상 활성화됐습니다. **정렬 검증**
(`check_scan_map_fit.py`, §10-13):

```
중앙값 거리   5.0 cm   (§10-13 기준 정상치와 정확히 일치)
평균   거리   8.0 cm
10 cm 이내   73.9%
20 cm 이내   92.3%
```

**주행 기록.** 목표 7개, 전부 `Reached the goal!` / `Goal succeeded`.
아무 것도 aborted/failed 없었고, `/emergency_stop` 은 계속 `false` 였습니다
(비상정지 안 걸림). 첫 목표는 8.3초에 0.36 m 이동, 평균 속도
0.044 m/s — 설정한 0.05 m/s 와 일치. 도중에 목표를 새로 주면 진행 중이던
경로가 preempt 되고 즉시 새 경로로 바뀌는 것도 확인했습니다.

**젯슨 쪽 CLI 진단 시 주의.** `ros2 node list` 는 discovery server 참가자
정보만으로도 되지만, `ros2 topic list`/`echo`/`tf2_echo` 는 SUPER_CLIENT
프로파일이 없으면 젯슨 로컬에서도 안 됩니다 (§10-1 함정 1번이 노트북뿐
아니라 젯슨 자신에게도 적용됨). 젯슨에서 직접 진단할 때도
`~/tetra_dds/super_client.xml` 을 걸어야 합니다. `tf2_echo` 는 SUPER_CLIENT
를 걸어도 무선 지연 때문에 자주 응답 없이 "Terminated" 로 끊겼습니다 —
이때는 `/amcl_pose`, `/tf` 를 `echo --once` 로 직접 찍어 확인하는 편이
더 안정적이었습니다.

**남은 것.**
- 후진 허용 (`min_vel_x` 음수화) — 앞 공간이 좁은 상황이 다시 나오면 필요.
- 오늘은 좁은 초기 구간(원점 근처)에서만 주행. 방 전체를 가로지르는 긴
  주행, 장애물(§10-9 사각지대) 회피 시험은 아직 안 함.
- RealSense D455 미장착 상태 그대로 — 1 m 아래 장애물은 여전히 안 보임.

### 10-21. 2026-08-19 저녁 — ⚠ Nav2 Goal 이 아무 반응 없던 것: bond 타임아웃으로 절반만 켜진 스택

박사님이 시연 보러 오시는 상황에서 실제로 겪었습니다. `2D Pose Estimate`
는 계속 잘 됐는데(`initialPoseReceived` 가 매번 찍힘) **`Nav2 Goal` 을 몇 번을
줘도 `bt_navigator` 로그에 목표를 받았다는 줄이 단 한 줄도 없었습니다.**
RViz 쪽에는 `Send goal call failed` 가 떴습니다.

**원인.** launch 시작 시점 로그를 보니:

```
Activating controller_server -> connected with bond
Activating smoother_server   -> connected with bond
Activating planner_server    -> (184초 뒤) ERROR: Server planner_server was
                                 unable to be reached after 4.00s by bond.
                                 Failed to bring up all requested nodes.
                                 Aborting bringup.
```

`lifecycle_manager_navigation` 은 노드를 순서대로 하나씩 활성화하면서 각
노드의 **bond**(생존 확인용 하트비트)가 **4초 안에** 연결되길 기다립니다.
무선 + discovery server 경유 지연(§10-1, §10-11 에 이미 기록된 것과 같은
현상) 때문에 `planner_server` 의 bond 연결이 4초를 넘겼고, 매니저는
**그 자리에서 전체 기동 절차를 포기**했습니다. `controller_server` 와
`smoother_server` 만 `active` 로 남고, **`planner_server` 뒤 순서였던
`behavior_server`, `bt_navigator`, `waypoint_follower`, `velocity_smoother`
는 시도조차 되지 않고 영원히 `inactive` 로 남았습니다.**

**증상이 헷갈리는 이유.** `ros2 topic list`, `ros2 action list` 로 보면
`/navigate_to_pose` 액션 서버가 정상적으로 보입니다 (`bt_navigator` 프로세스
자체는 살아있고 `configure` 까지는 됐으니까요) — 그래서 "서버는 있는데 왜
반응이 없지"로 헷갈리기 쉽습니다. **`ros2 lifecycle get <node>` 로 상태를
직접 봐야 `inactive` 인 게 드러납니다.**

**응급 복구 — 전체 재기동 없이, AMCL 위치추정을 안 잃고 고쳤습니다:**

```bash
ros2 lifecycle set /behavior_server activate
ros2 lifecycle set /bt_navigator activate
ros2 lifecycle set /waypoint_follower activate
ros2 lifecycle set /velocity_smoother activate
```

순서대로 하나씩 (lifecycle_manager 가 원래 하려던 순서와 동일). 넷 다
`Transitioning successful` 나오면 `ros2 lifecycle get` 으로 7개 노드
(`controller_server`, `smoother_server`, `planner_server`, `behavior_server`,
`bt_navigator`, `waypoint_follower`, `velocity_smoother`) 전부 `active` 인지
재확인. 이후 목표 재전송 정상 동작 확인.

**주의 — `velocity_smoother` 도 같이 꺼져 있었습니다.** 이건 특히 위험한
조합입니다 — `controller_server` 가 살아서 경로 계산은 되는데
`velocity_smoother` 가 죽어 있으면 `cmd_vel_nav -> cmd_vel` 리매핑 마지막
단계가 없어서 **최종 `/cmd_vel` 이 아예 안 나갑니다.** "명령을 계산하는데
로봇이 안 움직인다"는 증상이 나올 수 있는 또 다른 경로입니다.

**앞으로 이 실기 환경(게스트망+discovery server)에서는 매번 launch 직후
`ros2 lifecycle get` 으로 7개 노드 상태를 확인하는 걸 습관으로 삼아야
합니다.** "`Managed nodes are active` 로그가 찍혔다"만 믿으면 안 됩니다 —
그 로그도 사실 이번엔 아예 안 찍혔는데(Aborting bringup 으로 끝났으므로)
한참 뒤에 무심코 넘어갈 뻔했습니다.

---

### 10-20. 2026-08-24 — 다른 로봇이 매핑한 지도로 교체, local_costmap 에 static_layer 추가 (미검증)

**아직 실기에서 안 돌려봤습니다.** 아래는 설정 변경 내역과 그 근거만 남깁니다.

**지도 교체.** `/home/han/maps/map_20260821_144451_final.yaml`(+`.pgm`)을
받아서 `map_server.yaml_filename` 을 여기로 바꿨습니다
(`nav2_params.yaml` §13). 기존 `tetra_lab_closed`(3~4 m 방)와 달리
**38.85 x 61.95 m, 건물 한 층 전체** 규모 — 사용자 확인상 tetra 가 실제로
놓일 공간과 같은 건물이 맞습니다. `negate:0`/픽셀 관례는 정상(회색=미지,
흰색=자유, 검은 얇은 선=벽)이라 yaml 값 자체는 손대지 않았습니다.

이 지도는 **tetra보다 낮게 장착된 라이다를 쓰는 다른 로봇**이 만든 것이라,
tetra 라이다(장착고 1.048 m, §10-9)가 못 보는 낮은 장애물(책상다리·의자
등)이 지도에는 점유로 찍혀 있을 수 있습니다. 사용자가 원하는 동작은 —
tetra 스스로는 그 장애물을 실시간으로 볼 수 없어도, **이 지도에 있는 걸
그대로 신뢰해서 피해가는 것.**

**local_costmap 에 static_layer 추가.** 기존엔 `global_costmap`
(`plugins: [static_layer, obstacle_layer, inflation_layer]`)만 정적 지도를
썼고, `local_costmap` 은 `plugins: [obstacle_layer, inflation_layer]`뿐이라
`static_layer` 설정 블록이 있는데도 목록에 안 들어가 죽어있었습니다(아마
nav2_bringup 기본 템플릿을 그대로 남긴 흔적). 그 상태면 global costmap
경로계획은 저상 장애물을 피해가도 DWB 로컬 컨트롤러(`controller_server`)는
실시간 스캔(`obstacle_layer`)만 보고 판단하므로 좁은 구간에서 스쳐 지나갈
위험이 있었습니다. `local_costmap.plugins` 에 `static_layer` 를 맨 앞에
추가해 이 격차를 없앴습니다. `obstacle_layer` 의 실시간 clearing 은 자기
레이어만 지우고 `static_layer` 의 lethal 표시엔 영향을 못 주므로, tetra가
그 장애물을 계속 못 보고 있어도 지워지지 않습니다 — 의도한 동작.

**실기 전 확인할 것:**
- RViz 에서 local/global costmap 레이어를 직접 보고 저상 장애물 지점이
  실제로 lethal(보라/검정)로 남아있는지 확인.
- `inflation_radius: 0.22`(로컬), `0.22`(글로벌)가 이 건물 규모의 복도
  폭에서도 여전히 적절한지 — 기존 튜닝은 3~4 m 방 기준이었음.
- `laser_max_range: 4.0`(AMCL), `raytrace_max_range: 3.5` /
  `obstacle_max_range: 3.0`(costmap) 은 전부 작은 방 기준으로 잡은 값.
  건물 규모에서 correctness 를 깨진 않지만(그냥 보수적으로 짧은 범위),
  긴 복도에서 장애물 감지 거리가 짧다는 점은 인지하고 시작할 것.
- `check_scan_map_fit.py` 로 매칭률 확인 — 저상 장애물 지점은 매칭률이
  낮게 나오는 게 정상(§10-13 참고, tetra 라이다가 그 높이를 못 보므로).

---

### 10-21. ⚠ 로봇 본체 전원을 껐다 켜면 구동보드가 "EMG"로 걸린 채 뜹니다 — 버튼과 무관

**증상.** 로봇 본체(젯슨 아님, 모터·구동보드·라이다 쪽 배전) 전원을 껐다
켰더니 `tetra_drive_node`가 `BV round trip failed: emergency stop engaged
at the robot`를 계속 뱉었습니다. **비상정지 버튼은 눌려 있지 않았습니다**
(사용자가 직접 확인, 후면 버튼도 돌려서 재확인).

**원인.** `drive_board.cpp`의 기존 주석은 "rx[1]==0x32 는 E-stop"이라고
2026-08-13 실측을 근거로 적어뒀는데, 매뉴얼 표4-4(구동보드 모터 에러코드
표)를 다시 보니 실제로는:

| 코드 | 의미 |
|---|---|
| 0x30 ('0') | 정상 |
| 0x31 ('1') | Emergency Stop (버튼) |
| 0x32 ('2') | **모터 홀센서/모터라인 에러** |

즉 우리 코드의 `last_error_ = "emergency stop engaged"` 문구는 **오해를
유발하는 라벨**이었을 가능성이 있습니다. 다만 이게 진짜 홀센서/모터라인
단선인지, 아니면 전원 인가 순서 문제로 걸린 stale 에러 래치인지는 직접
확인이 필요했습니다.

**진단 (`probe_cg.py`, 바퀴 안 돌림 — BV(0,0) 조회 + CG만 전송):**

```
[0] BV 정지명령 (CG 이전)    : raw 02 32 03 31   rx[1]=0x32
[1] Error Reset (CG)                              rx[1]=0x30
[2] CZ11                                          rx[1]=0x30
[3] DB11                                           rx[1]=0x30
[4] BV 정지명령 (CG 이후)    : 16바이트 정상 오도메트리, rx[1]=0x30
```

**`CG`(Error Reset) 한 번으로 완전히 풀렸습니다.** 버튼을 만지지 않았는데
풀렸다는 것 자체가 "진짜 살아있는 EMG 신호"가 아니라 **전원 인가 시점에
구동보드가 부팅하면서 걸어두는 stale 에러 래치**라는 뜻입니다 — 매뉴얼
4-4-9절이 정확히 이 용도("구동 보드에 발생한 모든 Error를 초기화")로
`CG`를 문서화하고 있습니다.

**고침.** `DriveBoard::reset_error()`(`CG` 전송) 를 추가하고,
`tetra_drive_node`의 초기화 시퀀스 맨 앞 — `set_velocity_mode()`/
`set_servo()`보다 먼저 — 에서 항상 호출하도록 바꿨습니다. 이제 로봇
본체를 껐다 켜도 `navigation.launch.py`/`slam.launch.py` 기동 시 자동으로
클리어됩니다. 재빌드 필요 (`colcon build --packages-select
tetra_dsv_s_bringup`).

**남은 의문.** `0x32`가 매뉴얼 표4-4의 "모터 홀센서/모터라인 에러"와 같은
코드인지, 아니면 BV 응답의 FLAG 바이트가 그 표와 무관한 별도 enum(예:
"에러 있음"을 뭉뚱그려 나타내는 값)인지는 확정 못 했습니다. 실제 홀센서/
모터라인 단선이라면 `CG`로 안 풀렸어야 하는데 풀렸으니, 최소한 이번
사례는 stale 래치였던 게 맞습니다. 진짜 하드웨어 결함이 있을 때 `CG` 이후
에도 다시 `0x32`로 돌아오는지는 미검증 — 다음에 이 에러를 보면 `CG` 이후
몇 초 안에 재발하는지 확인할 것.

---

### 10-22. 2026-08-24 — 같은 날 나머지: bond_timeout, 캐스터 조인트, 노트북 RViz 실전 기록

**`lifecycle_manager` bond_timeout 기본값(4.0s)이 §10-19 멈춤의 진짜 근본
원인이었습니다.** 오늘도 같은 증상이 재발했습니다 — `map_server`/`amcl`이
`Configuring` 이후 `Activating` 로그도 없이 멈췄고, 그 여파로
`planner_server`의 `global_costmap`이 `map` TF를 무한 대기하며 멈춰
`lifecycle_manager_navigation`도 같이 아보트. §10-1/§10-11이 이미 기록한
무선 discovery 지연을 감안하면 4초는 너무 짧습니다. `navigation.launch.py`
의 두 `lifecycle_manager` 노드 파라미터에 `bond_timeout: 10.0`을 추가했습니다
— launch 파일은 symlink install이라 재빌드 불필요, 다음 실행부터 바로
반영됩니다. **그래도 멈추면** 응급 복구는 여전히 유효합니다:

```bash
ros2 lifecycle set /map_server activate
ros2 lifecycle set /amcl activate
# 그래도 planner_server 이후가 inactive면 순서대로:
ros2 lifecycle set /behavior_server activate
ros2 lifecycle set /bt_navigator activate
ros2 lifecycle set /waypoint_follower activate
ros2 lifecycle set /velocity_smoother activate
```

**캐스터 링크 RobotModel 에러.** `rear_caster_joint`가 URDF에서
`continuous`(Gazebo 마찰 유지 목적, §URDF 주석)인데 `tetra_drive_node`가
`left_wheel_joint`/`right_wheel_joint`만 `/joint_states`에 발행해서
`robot_state_publisher`가 `rear_caster_link`의 TF를 못 만들었습니다
(RViz: "No transform from [rear_caster_link] to [map]"). 캐스터는 무센서
자유회전 바퀴라 실제 각도를 잴 방법이 없으니, 고정값 0.0을 같이
발행하도록 고쳤습니다(`tetra_drive_node.cpp`) — 순수 시각화 문제였고 Nav2
동작(코스트맵 등)엔 원래 영향 없었습니다. 재빌드함.

**노트북 RViz — `nav2_default_view.rviz` 쓸 것.** 맨 `rviz2`로 빈 설정에서
디스플레이를 하나씩 추가하는 건 이번에 처음 해봐서 오래 걸렸습니다.
`nav2_bringup` 패키지에 이미 완성된 설정이 있습니다 — `RobotModel`, `TF`,
`LaserScan`, `Map`, `AMCL Particle Swarm`(=ParticleCloud), 코스트맵,
`Navigation 2` 컨트롤 패널까지 한 번에 뜹니다:

```bash
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

`Bumper Hit`/`Realsense` 항목은 scout-ur3e 템플릿 잔재라 TETRA엔 안 맞음 —
체크 해제하고 무시. `Navigation 2` 패널의 `Localization/Navigation: unknown`
은 위 수동 lifecycle 복구를 거치면(매니저 자신의 bond 신호를 못 받아서)
뜰 수 있는데, 개별 노드가 실제로 `active`면 기능상 문제 아님.

**`/map`이 TF는 살아있는데 안 뜨는 경우.** §10-11 "추가 (2026-08-24)"
항목 참고 — `map_server` 재activate로 재발행 유도.

**다음에 할 일 — Wi-Fi를 핫스팟으로.** §10-1의 사내 게스트망 멀티캐스트
차단이 오늘 겪은 문제 상당수(discovery 지연, bond timeout, `/map` 수신
유실)의 근본 원인입니다. 다음 전원 재기동부터는 개인 핫스팟으로 전환
예정 — 이러면 Discovery Server 우회 자체가 필요 없어질 수 있습니다(핫스팟이
멀티캐스트를 막지 않는다면). **단, 일부 핸드폰 핫스팟은 클라이언트 격리
(AP isolation)를 켜두어 기기 간 통신 자체를 막기도 하니** 완전히 안심할
수는 없음 — 전환 후 plain discovery(Discovery Server 없이 `ROS_DOMAIN_ID`
만 맞추고 `ros2 topic list`)부터 먼저 시험해보고, 안 되면 §10-1 그대로
Discovery Server로 복귀. 핫스팟 IP 대역은 게스트망(10.101.111.244)과
다를 거의 확실하니 `super_client.xml`의 서버 주소와 양쪽
`ROS_DISCOVERY_SERVER` 값도 새 IP로 갱신해야 함.

---

### 10-23. 2026-08-24 — 첫 실주행 Nav2 Goal 시도, 결과와 미해결 사항

**핫스팟 전환은 완전히 성공.** 게스트망 문제(§10-1) 없이 멀티캐스트
양방향 확인됐고(젯슨 172.20.10.6 ↔ 노트북 172.20.10.2), discovery server
자체가 필요 없어짐. `bond_timeout: 10.0`(§10-22) 효과까지 겹쳐서
`navigation.launch.py` 9개 노드가 수동 복구 없이 전부 한 번에 `active`가
됐음 — 이번 세션 최초.

**짧은 목표(~2m)는 성공.** Spin 복구 1회 후 `Goal succeeded`.

**긴 목표(~9.6~10m)는 반복 실패.** `bt_navigator`가
`compute_path_to_pose` 액션 서버 ack 타임아웃 + `global_costmap
clear_entirely` 서비스 타임아웃을 겪었고, backup↔spin 복구를 여러 차례
반복하다 매번 `Goal failed`로 끝남. **원인으로 보이는 것: 젯슨 CPU 과부하.**
그 시점 `uptime` load average **4.18** — 이 Claude Code 세션(VSCode `code`
프로세스 2개)이 각각 35%/29%를 먹고 있었음. 짧은 목표는 이 지연 창을
안 만나고 끝나서 성공했을 가능성. **다음 실주행 전엔 이 세션(또는 무거운
프로세스)을 내리고 젯슨 부하를 낮춘 뒤 긴 목표를 재시도해서, CPU 과부하
가설이 맞는지 검증할 것.**

**사용자 반응 — 실패마다 Nav2 Goal을 다시 찍은 것으로 보임.** 로그의
목표 좌표가 매번 조금씩 다름(11.59→10.21→8.50→11.02, 전부 비슷한 방향) —
로봇이 "혼자 앞뒤로 왔다갔다, 왼쪽으로 90도 회전"하는 것처럼 보인 건 실은
"실패→정지 후 재전송" 사이클이 여러 번 반복된 것. RViz Navigation2
패널의 `Feedback: aborted/failed` 표시를 매번 확인하는 습관이 필요함.

**Teleop 시도 실패, 원인 미확인.** 원점으로 되돌리려고
`teleop_twist_keyboard`(속도 0.1/0.3로 낮춰서)를 노트북에서 실행 안내했지만,
`ros2 node list`에도 안 잡히고 `/cmd_vel`도 5초간 0건 — teleop 노드
자체가 안 뜬 것으로 보임(노트북 쪽 `source /opt/ros/humble/setup.bash`
누락 또는 `ROS_DOMAIN_ID` 불일치 의심, **확인 전에 사용자가 AMR 전원을
통째로 꺼서 미해결로 남음**).

**⚠⚠ 다음 세션 시작 시 반드시 알아야 할 것: 로봇이 원점(초기 2D Pose
Estimate 위치)에 있지 않습니다.** 마지막 기록된 위치는 대략 map 좌표
(7~11, -1 부근)이고, 정확한 최종 위치는 모릅니다(teleop 복귀 실패,
AMR 전원 통째로 끔). **다음에 켤 때 로봇이 실제로 어디 있는지 눈으로
먼저 확인하고, 그 위치 기준으로 "2D Pose Estimate"를 새로 찍을 것** —
이전 세션 좌표를 절대 재사용하지 말 것.

**속도 상향 기록.** `nav2_params.yaml`의 `FollowPath.max_vel_x/max_speed_xy`
와 `velocity_smoother.max_velocity/min_velocity`를 0.05 → **0.15 m/s**로
올림(사용자 지정, 2026-08-24). 후진(`min_vel_x`)은 여전히 0.0 — §의 후진
불안정 이슈와 무관.

---

### 10-24. 2026-08-24 — planner_server 가 큰 지도에서 너무 느림(Dijkstra), 게스트망 RViz 는 결국 세그폴트

**§10-23 의 "CPU 과부하가 원인" 진단은 절반만 맞았습니다.** VSCode 를 끄고
`jetson_clocks` 까지 적용해 load average 를 4.18 → 0.98 로 낮춘 뒤에도,
**짧은 목표(1.9m)에서조차 같은 spin/backup 반복 실패가 재현**됐습니다.
로그를 다시 보니 진짜 원인은 따로 있었습니다:

```
[bt_navigator]: Begin navigating from current location (1.17, -0.15) to (2.86, -0.58)
[planner_server]: Planner loop missed its desired rate of 20.0000 Hz. Current loop rate is 19.0656 Hz
[planner_server]: Planner loop missed its desired rate of 20.0000 Hz. Current loop rate is 17.5176 Hz
[bt_navigator]: Timed out while waiting for action server to acknowledge goal request for compute_path_to_pose
[planner_server]: Planner loop missed its desired rate of 20.0000 Hz. Current loop rate is 11.3831 Hz
...
[planner_server]: Planner loop missed its desired rate of 20.0000 Hz. Current loop rate is 3.2969 Hz
```

목표가 시작되자마자 `planner_server` 자신의 내부 루프 속도가 19Hz에서
**3Hz까지 떨어지면서** `compute_path_to_pose` 액션 서버 ack 타임아웃이
납니다. **젯슨 CPU 전체 부하는 낮은데(0.98) planner_server 만 특정
순간에 느려지는 패턴** — 이건 전역 CPU 경합이 아니라 **경로계획 알고리즘
자체가 이 지도 크기에서 무거운 것**이라는 뜻입니다.

**원인.** `nav2_params.yaml`의 `planner_server.GridBased.use_astar: false`
(Dijkstra, 목표 방향과 무관하게 도달 가능한 공간을 넓게 탐색) — 옆 주석
"기본 true -> false"는 사실 바로 아래 `allow_unknown` 에 대한 설명이었고,
`use_astar` 자체는 그냥 Nav2 기본값이 손 안 댄 채 남아있던 것뿐입니다.
3~4 m 방(수천 칸)에선 Dijkstra 든 A* 든 체감 차이가 없었지만, 오늘 지도는
777x1239 = **약 96만 칸**이라 Dijkstra 의 전역 탐색 비용이 실제로 문제가
됩니다.

**고침.** `use_astar: true` 로 변경. 재시작 없이
`ros2 param set /planner_server GridBased.use_astar true` 로 라이브
반영도 검증(AMCL 위치추정 안 날아감). **아직 A* 적용 후 재시도 결과는
검증 못 함** — 다음에 이어서 확인할 것.

**게스트망 RViz 가 결국 세그폴트로 죽음.** §10-11 에 이미 기록된
`Message Filter dropping message ... discarding message because the
queue is full` (frame `odom`/`velodyne_link`) 증상이 몇 분간 계속되다
`Segmentation fault (core dumped)`로 끝남. 이전엔 "화면만 나쁘다"였는데
이번엔 진짜로 죽었습니다 — 지도가 커지고(777x1239) 디스플레이도
늘어서(costmap x2 + particle cloud + laser scan + robot model) 노트북
쪽 메시지 동기화 부담이 커진 탓으로 보입니다. **핫스팟(§10-22)에서는
이 큐 오버플로우 자체가 안 생겼음** — 게스트망을 계속 써야 한다면
디스플레이 개수를 줄이거나(특히 ParticleCloud), 가능하면 핫스팟으로
돌아갈 것을 권함.

---

### 10-25. ⚠ 2026-08-25 — 원인 미확정: 모든 설정이 맞는데도 토픽이 2개만 보임

**§10-1 의 게스트망 증상과 똑같아 보이지만 원인이 다릅니다.** §10-1 의
세 가지 함정을 **전부 배제하고도** 재현됐습니다.

**증상.** `navigation.launch.py` 기동 후 `ros2 topic list` 에
`/parameter_events`, `/rosout` 둘만 보임. **노트북뿐 아니라 젯슨 자신에서
`--no-daemon` 으로 조회해도 똑같이 2개** — 이게 §10-1 과 결정적으로
다른 점입니다(§10-1 은 젯슨 로컬에서는 정상으로 보임).

**배제한 것 (전부 확인함, 문제 없었음):**

| 확인 항목 | 방법 | 결과 |
|---|---|---|
| discovery server 가 launch 보다 먼저 떴는가 | `ps -o lstart` | 13:29:54 vs 13:31:01 — 순서 맞음 |
| launch 셸에 `ROS_DISCOVERY_SERVER` 가 걸렸는가 (§10-1 함정 3) | `/proc/<launch pid>/environ` | 걸려 있었음 |
| 개별 노드가 그 변수를 상속받았는가 | `/proc/<map_server pid>/environ` 등 | 받았음 |
| 노드들이 실제로 살아있는가 | `ps --ppid` | 18개 전부 정상 실행 |
| 노드가 소켓을 열었는가 | `ss -unap` | 열려 있음 |
| CLI 쪽 SUPER_CLIENT 프로파일 (§10-1 함정 1) | 명시 export 후 재조회 | 여전히 2개 |
| ros2 데몬 캐시 (§10-1 함정 2) | `--no-daemon` | 여전히 2개 |
| discovery server 자체 | GUID prefix `44.53...41`, 포트 11811 listening | 정상 |

**해결 — 같은 명령으로 그냥 재시작.** launch 와 discovery server 를 둘 다
죽이고 **완전히 동일한 순서·동일한 명령으로** 다시 띄우니 토픽 50개로
정상화. **설정은 아무것도 안 바꿨습니다.**

**원인 미확정.** 등록 핸드셰이크가 한 번 어긋난 것으로 보이나 이유는
모릅니다. 짚이는 것은 젯슨에 네트워크 인터페이스가 3개(무선, 라이다 유선
192.168.1.150, docker0)라 Fast DDS 가 locator announce 할 때 타이밍에 따라
꼬일 수 있다는 정도인데 **근거 약함 — 검증 안 됨.** 재현 조건을 모르므로
현장에서 다시 날 수 있습니다.

**실용 대처 — launch 직후 매번 이걸 확인할 것:**

```bash
ros2 topic list | wc -l     # 젯슨에서
```

- **50 근처** → 정상. 노트북으로 진행.
- **2** → 이 버그. launch + discovery server 둘 다 죽이고 재시작.
  (2026-08-25 에는 한 번에 해결됐음)

> 핫스팟(§10-22)에서는 discovery server 자체를 안 쓰므로 이 문제의
> 표면적이 줄어들 가능성이 있으나, 이 역시 검증 안 됨.

---

### 10-26. ⚠⚠ 2026-08-25 — spin/backup 반복의 진짜 원인은 `default_server_timeout: 20`(ms) 이었습니다

**§10-23 의 "CPU 과부하" 진단, §10-24 의 "planner 가 느려서" 진단 모두
빗나갔습니다.** 진짜 원인은 `bt_navigator` 의 타임아웃 설정 하나였습니다.

**결정적 로그.** 목표 시작 직후 1초 구간을 정밀하게 보면:

```
929.200  bt_navigator: Begin navigating from (6.23, -0.88) to (11.24, -1.18)
929.251  controller_server: Received a goal, begin computing control effort.
930.302  controller_server: Passing new path to controller.      <- 경로 계산 성공!
931.311  bt_navigator: Timed out while waiting for action server to
                       acknowledge goal request for compute_path_to_pose
931.332  bt_navigator: Node timed out while executing service call to
                       global_costmap/clear_entirely_global_costmap.
931.374  behavior_server: Running spin      <- 복구 동작 시작
```

**경로 계획은 성공하고 있었습니다** (`Passing new path to controller`).
문제는 그 다음 — BT 가 planner 에게 다음 요청을 보내고 **"받았다"는 ACK 를
기다리는 시간이 `default_server_timeout: 20`, 즉 20 밀리초**라는 것.
planner_server 가 이 지도(96만 칸)에서 경로 하나 계산하는 데 ~1초가
걸리는데, 계산 중이라 20ms 안에 ACK 를 못 하면 BT 는 **"서버가 죽었다"**
로 판정하고 곧장 복구 동작(spin → backup)으로 넘어갑니다. 그게 사용자가
본 "왼쪽으로 90도 돌았다가 후진했다가 전진" 입니다.

**왜 §10-18 첫 자율주행 때는 안 드러났나.** 그때는 3~4 m 방(수천 칸)이라
경로 계산이 수십 ms 에 끝나서 20ms 를 아슬아슬하게 맞췄습니다. 지도가
커지자마자 즉시 터진 것.

**고침 (`nav2_params.yaml`):**

| 파라미터 | 기존 | 변경 | 이유 |
|---|---|---|---|
| `bt_navigator.default_server_timeout` | 20 (ms) | **1000** | 위 원인. 핵심 수정 |
| `bt_navigator.bt_loop_duration` | 10 (ms) | **20** | "BT tick rate 100.00 exceeded" 해소 |
| `planner_server.expected_planner_frequency` | 20.0 | **1.0** | 달성 불가한 목표치. 경고 스팸이 진짜 문제를 가렸음 |

**✅ 실주행 검증 완료 (2026-08-25).** 수정 후 같은 지도에서:

| 항목 | 수정 전 | 수정 후 |
|---|---|---|
| `acknowledge goal request` 타임아웃 | 목표마다 반복 | **0 건** |
| 목표 성공 | 짧은 것 1회뿐 | **3회 연속 성공** |
| 복구 동작(spin/backup) | 목표당 4~6회 | 전체 2회 |

성공한 목표 중에는 **6.3 m** 짜리도 포함됩니다
(`(4.69,-0.43) → (10.93,-1.38)`, 50초). 어제 이 거리대는 전부 실패했으므로,
"짧은 목표만 된다"가 아니라 중거리도 정상화된 것이 확인됐습니다.

**교훈 — 로그를 시간순으로 좁게 보세요.** §10-23/§10-24 에서 오진한 이유는
`Planner loop missed its desired rate` 경고가 로그를 뒤덮어서 그것만 보고
"planner 가 느리다"로 단정했기 때문입니다. 정작 바로 아래 줄의
`Passing new path to controller`(=성공) 와 `Timed out ... acknowledge`
(=ACK 실패) 조합을 못 봤습니다. **경고가 많이 찍히는 것과 그게 원인인
것은 다릅니다.**

---

### 10-27. 노트북 RViz 세그폴트 — 게스트망 탓이 아니었음, 원인 미확정

**§10-24 에서 "게스트망 때문"이라고 적은 것은 틀렸습니다.** 2026-08-25 에
**핫스팟에서도 동일하게 재현**됐습니다:

```
[rviz]: Message Filter dropping message: frame 'velodyne_link' ...
        for reason 'discarding message because the queue is full'
[rviz]: Message Filter dropping message: frame 'odom' ... (동일)
Segmentation fault (core dumped)
```

**시도했지만 효과 없었던 것.** 전송량이 원인이라고 보고
`always_send_full_costmap` 을 `True → False` 로 바꿨습니다(전역 코스트맵이
777x1239 = ~960 KB 를 1Hz 로 통째로 재전송하고 있었음). 대역폭은 확실히
줄었지만 **세그폴트는 그대로 재현**됐습니다. 그래도 이 변경 자체는
합리적이라 되돌리지 않았습니다.

**남은 단서.** 큐가 차는 프레임이 항상 둘입니다:
- `velodyne_link` → `LaserScan` 디스플레이
- `odom` → `Controller` 그룹의 local plan

젯슨 쪽 TF 발행 주기는 과하지 않습니다(EKF 30Hz, drive node 30Hz). 즉
RViz 가 무선 구간의 TF 지터를 못 따라가 tf2 message filter 큐가 계속
넘치고, 그 상태가 지속되면 RViz 가 죽는 것으로 보입니다. **RViz/tf2 쪽
버그로 추정하나 확정 못 했습니다.**

**실용 대처 (검증 안 됨, 다음에 확인할 것):**
1. 노트북 RViz 에서 `LaserScan` 과 `Controller` 체크 해제 — 위 두 프레임의
   message filter 를 아예 없앰. 주행 성공 여부(`Recoveries: 0`) 확인에는
   이 둘이 필요 없음. 장애물 회피 테스트할 때만 `LaserScan` 을 잠깐 켤 것.
2. 그래도 죽으면 젯슨 로컬 화면에서 RViz 실행(무선 미경유). 단 젯슨 CPU 를
   쓰므로 차선책.

**⚠ 안전.** RViz 가 죽어도 **로봇은 계속 자율주행합니다.** RViz 의 Pause
버튼도 같이 사라집니다. 목표를 준 상태에서는 **반드시 비상정지 버튼에 손이
닿는 위치**를 유지할 것. RViz 없이 멈추는 방법:

```bash
pkill -INT -f "ros2 launch tetra_dsv_s_bringup"   # 젯슨에서, 확실함
```

---

### 10-28. 내 CLI 조회를 믿지 마세요 — 로그를 보세요

2026-08-25 에 `ros2 topic list`/`node list`/`lifecycle get`/`tf2_echo` 가
**젯슨 로컬에서조차** 비어 나오거나 "Node not found" 를 뱉는 일이 반복됐고,
그 때문에 두 번 오진했습니다:

- `map_server` 가 안 떴다고 판단 → 실제로는 정상 (재조회하니 나옴)
- AMCL 이 `map` TF 를 안 낸다고 판단 → **실제로는 정상적으로 내고 있었음**

**확실한 판별법은 launch 로그의 에러 문구 변화입니다:**

| 로그 | 의미 |
|---|---|
| `Invalid frame ID "map" ... frame does not exist` | `map` 프레임 **없음** (초기 위치 미설정) |
| `Extrapolation Error ... earliest data is at <시각>` | `map` 프레임 **있음** (시간대만 안 맞음) |

두 번째로 바뀌었으면 AMCL 은 정상입니다. §10-25 의 discovery 불안정이
CLI 조회에도 똑같이 영향을 줍니다.

**부수 발견 — planner_server 가 오래된 목표에 갇힐 수 있음.**

```
[transformPoseInTargetFrame]: Extrapolation Error looking up target frame:
Requested time 1787635026.25 but the earliest data is at time 1787635382.37
```

6분 전 타임스탬프의 목표를 계속 재시도하는데 TF 버퍼는 10초치뿐이라 영원히
실패합니다. 새 목표를 보내도 안 풀렸고, **launch 재시작으로만 해결**했습니다.
재현 조건 미확정.
