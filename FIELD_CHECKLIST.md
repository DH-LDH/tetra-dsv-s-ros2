# 현장 체크리스트 — 새 공간에서 매핑 후 자율주행

2026-08-26 인천국제공항 작업용으로 작성했습니다. 이후 다른 현장에서도
같은 순서로 쓸 수 있습니다.

- **실행 명령의 근거·배경**은 [README.md](README.md)
- **왜 그런지·과거에 뭘 겪었는지**는 [HANDOFF.md](HANDOFF.md)
- 이 문서는 **현장에서 순서대로 따라가는 것**만 담습니다

---

## ⚠ 출발 전에 반드시 읽을 것

### 이 설정들은 전부 "3~4 m 실험실 방" 기준입니다

새 공간에서 그대로 쓰면 **매핑이 안 되거나 지도가 쓸모없게 나옵니다.**
공간 크기에 맞춰 다시 잡아야 하는 값들:

| 파일 | 파라미터 | 현재 | 왜 바꿔야 하나 |
|---|---|---|---|
| `slam_toolbox.yaml` | `max_laser_range` | **4.0** | ⚠⚠ 가장 중요. 아래 참고 |
| `slam_toolbox.yaml` | `loop_search_maximum_distance` | 3.0 | 넓은 공간에서 루프 클로저가 안 걸림 |
| `slam_toolbox.yaml` | `resolution` | 0.05 | 터미널 규모면 지도가 거대해짐 |
| `nav2_params.yaml` | `amcl.laser_max_range` | 4.0 | **`max_laser_range` 와 반드시 같은 값** |
| `nav2_params.yaml` | `obstacle_max_range` | 3.0 | 실시간 장애물 감지 거리 |
| `nav2_params.yaml` | `raytrace_max_range` | 3.5 | 위와 함께 |

### `max_laser_range` — 이것부터 정하세요

VLP-16 은 100 m 를 봅니다. 이 값은 **"이 거리까지의 빔만 믿겠다"** 는 뜻입니다.

- **너무 크면**: 유리·틈새를 통과한 빔이 그 너머까지 "비어 있다"고 보고하고,
  slam_toolbox 가 자유 공간으로 칠합니다. Nav2 는 자유 공간을 "갈 수 있는
  곳"으로 보므로 **로봇이 유리벽으로 돌진하는 경로를 냅니다.**
  실험실에서 실제로 당했습니다 (HANDOFF §10-17: 20.0 → 실제 방의 4배 크기로
  유령 자유공간 생성).
- **너무 작으면**: 벽에서 그 거리 안으로 들어가야만 벽이 그려집니다. 넓은
  터미널을 4 m 로 매핑하려면 모든 벽 4 m 이내를 다 훑어야 하고, 루프 클로저도
  안 걸립니다.

**정하는 법**: 그 공간에서 **한눈에 보이는 가장 긴 직선 거리**(예: 복도
끝에서 끝)를 눈대중으로 재고, 그보다 조금 크게 잡습니다. 터미널이면
보통 15~30 m 대일 텐데, **공항 유리벽 때문에 처음부터 크게 잡는 건
위험합니다.** 10 m 정도로 시작해서 지도를 보고 늘리는 쪽을 권합니다.

**⚠ 공항 유리벽 = 실험실 철창과 같은 문제입니다.** 라이다 빔이 유리를
통과하거나 엉뚱하게 반사됩니다 (HANDOFF §10-10, §10-17). **유리가 많은
구간에서 지도가 어떻게 나오는지 초반에 반드시 확인하세요.**

### 지도 해상도

`resolution: 0.05` 로 넓은 터미널을 뜨면 칸 수가 폭발합니다. 참고로
38 x 62 m 지도가 이미 **96만 칸**이고, 그 크기에서 경로 계산 하나에
**약 1초**가 걸립니다 (HANDOFF §10-26). 터미널이 그보다 훨씬 넓으면
`0.1` 로 낮추는 것을 고려하세요 — 칸 수가 1/4 이 됩니다. 대신 5 cm 단위
정밀도는 포기합니다.

### 준비물

- 젯슨, 노트북, AMR (충전 확인)
- **개인 핫스팟** — 게스트망보다 훨씬 안정적입니다 (HANDOFF §10-22)
- 회피 테스트용 박스 하나

---

## 현장 절차

### 0. 전원

1. AMR 본체 메인 스위치 ON
2. **라이다 스핀업 30초~1분 대기**
3. 젯슨 부팅

### 1. 젯슨 — 매번 필요 (재부팅하면 초기화됨)

```bash
sudo nmcli con up lidar
sudo sh -c 'echo 1 > /sys/bus/usb-serial/devices/ttyUSB0/latency_timer'
sudo jetson_clocks
```

> `jetson_clocks` 는 출력이 없는 게 정상입니다.
> 젯슨을 안 껐다면(AMR 전원만 껐다 켠 경우) `latency_timer` 는 유지되니
> `cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer` 로 먼저 확인하세요.

### 2. 네트워크 — 핫스팟 권장

젯슨과 노트북을 **같은 핫스팟**에 연결한 뒤, 젯슨에서 IP 확인:

```bash
hostname -I    # 172.20.10.x 같은 핫스팟 IP를 적어둘 것
```

멀티캐스트가 되는지 확인 (되면 discovery server 불필요):

```bash
# 노트북에서
ros2 multicast receive
# 젯슨에서
ros2 multicast send
```

양쪽에서 `Hello World!` 가 보이면 **discovery server 없이 진행**합니다.
안 되면 게스트망처럼 막힌 것이니 README 의 Discovery Server 절차를 따르세요.

**양쪽 공통 환경변수** (매 터미널마다):

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=25
export ROS_LOCALHOST_ONLY=0
unset ROS_DISCOVERY_SERVER
unset FASTRTPS_DEFAULT_PROFILES_FILE
```

### 3. 매핑 파라미터 조정

**launch 전에** `src/tetra_dsv_s_bringup/config/slam_toolbox.yaml` 을 열어
위 표대로 수정합니다.

```bash
nano ~/tetra_ws/src/tetra_dsv_s_bringup/config/slam_toolbox.yaml
```

> **재빌드 불필요합니다.** config 는 symlink install 이라 파일만 고치면
> 다음 launch 부터 바로 반영됩니다.

### 4. 매핑 실행

```bash
cd ~/tetra_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch tetra_dsv_s_bringup slam.launch.py
```

**노트북에서 RViz:**

```bash
rviz2 -d ~/tetra_rviz/slam.rviz
```

**teleop (노트북 별도 터미널):**

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p speed:=0.15 -p turn:=0.3
```

> `i` 전진 / `,` 후진 / `j` 좌회전 / `l` 우회전 / `k` 정지.
> 큰 바퀴 두 개가 있는 쪽이 **앞**입니다.

**매핑 중 계속 확인할 것:**

- 벽이 그려지는 곳이 **실제로 벽이 있는 곳인가**
- 유리벽 근처에서 **부챗살 모양으로 자유 공간이 뻗어 나가지 않는가**
  ← 나가면 `max_laser_range` 를 줄이고 처음부터 다시
- 지나온 길이 실제 경로와 맞는가 (드리프트 확인)

### 5. 지도 저장

```bash
mkdir -p ~/maps
ros2 run nav2_map_server map_saver_cli -f ~/maps/airport_20260826
```

> `use_sim_time` 을 붙이면 실패합니다 (HANDOFF §6-9).

**저장 직후 반드시 검증하세요 (HANDOFF §10-17 의 교훈):**

1. **자유 공간 면적 vs 실제 공간 면적** — 크게 다르면 유령입니다
2. **로봇이 실제로 주행한 범위 vs 자유 공간 범위** — 안 가본 곳이 자유로
   칠해져 있으면 레이캐스팅으로 만들어진 가짜입니다

pgm 파일을 이미지 뷰어로 열어서 눈으로 보는 게 가장 빠릅니다.

### 6. 지도를 Nav2 에 반영

```bash
nano ~/tetra_ws/src/tetra_dsv_s_bringup/config/nav2_params.yaml
```

- `map_server.yaml_filename` → `/home/han/maps/airport_20260826.yaml`
- `amcl.laser_max_range` → **`slam_toolbox.yaml` 의 `max_laser_range` 와 동일하게**
- `obstacle_max_range` / `raytrace_max_range` 도 공간에 맞게

> 재빌드 불필요 (symlink install).

### 7. 자율주행

**slam 을 반드시 먼저 끄세요** — `slam_toolbox` 와 `amcl` 이 동시에 뜨면
`map → odom` 의 부모가 둘이 되어 TF 트리가 깨집니다.

```bash
# slam.launch.py 를 Ctrl-C 로 종료한 뒤
cd ~/tetra_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch tetra_dsv_s_bringup navigation.launch.py
```

**⚠ launch 직후 젯슨에서 이걸 먼저 확인 (HANDOFF §10-25):**

```bash
ros2 topic list | wc -l
```

- **50 근처** → 정상
- **2** → 등록이 어긋난 것. launch(+discovery server 쓴다면 그것도) 둘 다
  죽이고 같은 명령으로 재시작하면 해결됩니다

**노트북 RViz:**

```bash
rviz2 -d /opt/ros/humble/share/nav2_bringup/rviz/nav2_default_view.rviz
```

> **`LaserScan` 과 `Controller` 체크를 꺼두세요.** 켜두면 RViz 가
> `queue is full` 을 반복하다 **세그폴트로 죽습니다** (HANDOFF §10-27).
> 장애물 회피를 눈으로 확인할 때만 잠깐 켜세요.
> `Bumper Hit`, `Realsense` 는 이 로봇에 없는 항목이니 무시.

**RViz 에서:**

1. **로봇이 지금 실제로 있는 자리**에 `2D Pose Estimate` 로 위치·방향 지정
2. `Nav2 Goal` 로 목표 — **1 m 안쪽부터 시작해서 점점 늘릴 것**
3. 목표마다 `Navigation 2` 패널의 **`Feedback`** 과 **`Recoveries`** 확인

---

## 유의할 점

### 안전

- **비상정지 버튼에 손이 닿는 위치를 항상 유지하세요.** 이게 가장 확실합니다.
- **RViz 가 죽어도 로봇은 계속 주행합니다.** Pause 버튼도 같이 사라집니다.
  RViz 없이 멈추는 방법:
  ```bash
  pkill -INT -f "ros2 launch tetra_dsv_s_bringup"   # 젯슨에서
  ```
- 현재 설정은 **후진을 안 합니다** (`min_vel_x: 0.0`). 목표가 뒤에 있으면
  제자리 회전 후 전진하니 **앞 공간을 비워두세요.**
- 속도는 **0.15 m/s** 입니다 (사람 걷는 속도의 약 1/10).

### 증상이 헷갈리는 것들

| 보이는 것 | 실제 의미 |
|---|---|
| `AMCL cannot publish a pose... set the initial pose` | **정상.** 2D Pose Estimate 를 아직 안 찍은 것 |
| RobotModel 의 모든 링크가 `No transform` | **정상.** 위와 같은 이유 (`map` 프레임이 아직 없음) |
| `Navigation 2` 패널의 `unknown` / `inactive` | **표시 오류인 경우가 많습니다.** 로그와 실제 거동으로 판단하세요 |
| 지도가 안 뜸 (TF 는 정상인데 `No map received`) | `ros2 lifecycle set /map_server deactivate` → `activate` 로 재발행 유도 |
| `ros2 node list` / `lifecycle get` 이 비어 나옴 | **CLI 조회가 불안정한 것.** 노드는 멀쩡한 경우가 많습니다 (HANDOFF §10-28). 로그를 보세요 |
| 로봇이 혼자 돌고 후진/전진 반복 | 복구 동작(spin/backup). 목표가 실패한 것이니 `Feedback` 을 확인하세요 |

### `map` 프레임이 살아있는지 판별하는 법 (HANDOFF §10-28)

launch 로그의 에러 **문구**로 구분합니다:

- `Invalid frame ID "map" ... frame does not exist` → 프레임 **없음**
- `Extrapolation Error ... earliest data is at <시각>` → 프레임 **있음**

### 아직 검증 안 된 것 — 현장에서 처음 겪게 됩니다

- **움직이는 사람 회피.** 정적 장애물 인식(로컬 코스트맵에 즉시 마킹됨)까지는
  확인했지만, 실제로 피해 가는 것은 미검증입니다.
- **좁은 통로.** `inflation_radius: 0.35` 라 **폭 0.7 m 미만 통로는 경로를
  못 찾습니다.** 게이트·검색대처럼 좁은 곳에서 경로가 안 나오면 이 값을
  먼저 의심하세요 (HANDOFF §10-29).
- **유리벽에서의 라이다 거동.**
- **터미널 규모 지도에서의 경로 계산 시간.** 96만 칸에서 이미 ~1초입니다.
  더 커지면 `resolution` 을 0.1 로 낮추는 것을 고려하세요.

### 회피 여유 참고 (HANDOFF §10-29)

`inflation_radius: 0.35` 기준 실제 몸체 여유:

| 방향 | 여유 |
|---|---|
| 좌우 | 13.5 cm |
| 앞 | 18.3 cm |
| 뒤 | 3.2 cm (후진을 안 하므로 제자리 회전·backup 때만 해당) |

더 넉넉하게 하려면 값을 올리되, **폭 `inflation_radius x 2` 미만 통로는
막힌다**는 대가를 기억하세요.

---

## 하루 마무리

```bash
pkill -INT -f "ros2 launch tetra_dsv_s_bringup"
sleep 10
pkill -9 -f fast-discovery-server     # 썼다면
```

지도 파일(`~/maps/`)과 그날 바꾼 설정을 **커밋해두세요.** 현장에서 맞춘
값을 잃으면 다음에 처음부터 다시 찾아야 합니다.
