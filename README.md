# TETRA-DSV-S ROS 2

휴림네트웍스 TETRA-DSV-S AMR의 SLAM / 자율주행 스택입니다.

- `src/tetra_dsv_s_description` — URDF. 시뮬과 실기가 **공유**합니다
- `src/tetra_dsv_s_bringup` — 실기용. RS-232 구동보드 드라이버
- `src/tetra_dsv_s_sim` — Gazebo Fortress 시뮬. 젯슨에는 받지 않습니다

작업 이력·검증 수치·함정 목록은 [HANDOFF.md](HANDOFF.md)에 있습니다.

---

# 젯슨에서 할 일

Ubuntu 22.04 / ROS 2 Humble 기준. **위에서부터 순서대로** 하시면 됩니다.
각 단계마다 "이게 나와야 정상"을 적어뒀으니 다르면 거기서 멈추고 알려주세요.

## 0. ROS 2 Humble 확인

```bash
source /opt/ros/humble/setup.bash
printenv ROS_DISTRO
```

> `humble` 이 나와야 합니다. 아무것도 안 나오면 ROS 2가 안 깔린 겁니다.

## 1. 클론

시뮬 패키지는 받지 않습니다. Gazebo 의존성을 젯슨에 끌고 오지 않기 위해서입니다.

```bash
git clone --filter=blob:none --sparse \
  https://github.com/DH-LDH/tetra-dsv-s-ros2.git ~/tetra_ws
cd ~/tetra_ws
git sparse-checkout set src/tetra_dsv_s_description src/tetra_dsv_s_bringup
ls src/
```

> `tetra_dsv_s_bringup` 과 `tetra_dsv_s_description` **둘만** 보여야 정상입니다.

## 2. 의존 패키지 설치

```bash
sudo apt update
sudo apt install -y \
  ros-humble-velodyne \
  ros-humble-pointcloud-to-laserscan \
  ros-humble-robot-localization \
  ros-humble-slam-toolbox \
  ros-humble-nav2-bringup \
  ros-humble-navigation2 \
  ros-humble-teleop-twist-keyboard \
  python3-serial python3-colcon-common-extensions
```

> `ros-humble-velodyne` 는 메타패키지라 `velodyne_driver` + `velodyne_pointcloud`
> + `velodyne_msgs` 가 같이 들어옵니다.
>
> `iahrs_driver_ros2`(IMU)는 apt에 **없습니다.** Hyulim-Group GitHub에서 따로
> clone하고 foxy → humble 포팅이 필요합니다. 지금은 안 해도 됩니다.

## 3. 시리얼 포트 권한

**이걸 빼먹으면 다음 단계가 `Permission denied`로 막힙니다.**

```bash
sudo usermod -aG dialout $USER
```

```bash
# 그룹 반영에는 재로그인이 필요합니다. 지금 세션에서 바로 쓰려면:
newgrp dialout
groups | grep dialout
```

> `dialout` 이 보여야 정상입니다.

## 4. 빌드

```bash
cd ~/tetra_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

> `2 packages finished` 가 나와야 정상입니다.

## 5. 시리얼 통신 확인 — ROS 없이

**가장 중요한 단계입니다.** ROS를 걷어내고 시리얼만 봅니다. 여기서 되면 이후에
안 움직일 때 원인이 ROS 쪽이라고 바로 좁혀집니다.

```bash
cd ~/tetra_ws
python3 src/tetra_dsv_s_bringup/scripts/probe_drive_board.py --list
```

> 시리얼 장치 목록이 나옵니다. 보통 `/dev/ttyUSB0`, `/dev/ttyUSB1` …
> 하나도 안 나오면 USB 케이블 / 로봇 전원을 확인하세요.

```bash
python3 src/tetra_dsv_s_bringup/scripts/probe_drive_board.py --port /dev/ttyUSB0
```

> 마지막에 `통신 정상` 이 나와야 합니다.
>
> - **포트가 여러 개면 하나씩** 시도하세요. 구동보드(CN20)가 아니면
>   `프레임 해석 실패` 라고 알려줍니다. 전원/센서보드(CN24)와 IMU도 같은
>   USB-시리얼로 보이기 때문에 눈으로는 구분이 안 됩니다.
> - `⚠ 비상정지가 걸려 있습니다` 가 뜨면 로봇의 빨간 버튼을 풀어주세요.

## 6. 바퀴 회전 시험

> ### ⚠ 로봇을 들어올려 바퀴를 띄운 뒤에 하세요.
> 제자리 회전 명령입니다. 바닥에 둔 채로 하면 실제로 돕니다.

```bash
python3 src/tetra_dsv_s_bringup/scripts/probe_drive_board.py \
  --port /dev/ttyUSB0 --spin 3.0 --speed 100
```

> 3초간 좌우 바퀴가 반대로 돌고, 매 주기 `x= y= th=` 값이 흐릅니다.
> `th` 가 계속 변하면 엔코더와 오도메트리가 살아 있는 겁니다.
> Ctrl-C로 즉시 멈추고, 끝나면 정지 명령을 3번 보냅니다.

## 7. 휠 트랙 실측 — 줄자

코드가 아니라 자로 재는 일입니다. **좌우 바퀴 접지면 중심 사이 거리**를
mm 단위로 재주세요.

> 왜 필요한가: 자료마다 값이 다릅니다. 매뉴얼 Para5는 438 mm, 벤더 드라이버는
> 377 mm, 벤더 URDF는 378 mm입니다. 차체 폭이 430 mm이니 438이면 바퀴가 밖으로
> 튀어나와야 합니다. 이 값이 틀리면 로봇이 명령한 만큼 회전하지 않습니다.
> 자세한 내용은 HANDOFF.md §2-(1).

## 8. udev 규칙 — 포트 고정

`/dev/ttyUSB0` 번호는 부팅할 때마다 바뀝니다. 5단계에서 찾은 포트를 고정합니다.

```bash
# 구동보드로 확인된 포트의 식별자를 확인
udevadm info -a -n /dev/ttyUSB0 | grep -E 'idVendor|idProduct|\{serial\}' | head -5
```

출력의 `idVendor` / `idProduct` / `serial` 값을
`src/tetra_dsv_s_bringup/udev/99-tetra-serial.rules` 의 `CHANGEME` 자리에 넣은 뒤:

```bash
sudo cp src/tetra_dsv_s_bringup/udev/99-tetra-serial.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules && sudo udevadm trigger
ls -l /dev/tetra_drive
```

> `/dev/tetra_drive -> ttyUSB0` 같은 심볼릭 링크가 보여야 정상입니다.
>
> 어댑터에 고유 serial이 없으면(싸구려 CH340에서 흔합니다) 규칙 파일 주석대로
> USB 포트 경로(`KERNELS==`)로 대신 묶으세요.

## 9. ROS 드라이버 기동

> ### ⚠ 바퀴는 계속 띄워둔 채로 하세요.

```bash
cd ~/tetra_ws && source install/setup.bash
ros2 launch tetra_dsv_s_bringup drive.launch.py port:=/dev/ttyUSB0
```

> udev 규칙을 넣었으면 `port:=` 없이 그냥 실행하면 됩니다.
> `tetra_drive up on ... (track 0.3770 m, r 0.1015 m)` 가 나와야 정상입니다.

다른 터미널에서 확인:

```bash
cd ~/tetra_ws && source install/setup.bash
ros2 topic hz /wheel_odometry     # 30 Hz 근처가 나와야 정상
ros2 topic echo /emergency_stop --once
ros2 topic echo /bumper --once
```

또 다른 터미널에서 조종 (**속도를 낮게**):

```bash
cd ~/tetra_ws && source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard \
  --ros-args -p speed:=0.10 -p turn:=0.3
```

> 키를 누르면 바퀴가 돌고, 떼면 **0.5초 안에** 멈춰야 합니다.
> 안 멈추면 즉시 Ctrl-C 하고 알려주세요 — 워치독이 안 도는 겁니다.

---

# 결과 알려주실 것

이 네 가지면 다음 단계로 넘어갈 수 있습니다.

1. **5단계 출력 전체** — `raw:` 로 시작하는 바이트 줄 포함. 실패해도 그대로
2. **6단계에서 `th` 값이 변했는지**
3. **7단계 줄자 실측값** (mm)
4. **9단계 `ros2 topic hz /wheel_odometry` 결과**

막히면 막힌 단계 번호와 터미널 출력을 그대로 주세요.
