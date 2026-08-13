#!/usr/bin/env python3
"""IMU 장착 방향을 로봇을 움직여서 알아냅니다 — 분해 없이.

배경: 3DM-GV7 을 젯슨에 붙여보니 정지 상태에서 중력이 z = -9.8 로 읽힙니다.
IMU 가 뒤집혀 달렸다는 뜻인데, X 축으로 180도 돈 것인지 Y 축으로 180도 돈
것인지는 정지 데이터만으로는 구분할 수 없습니다 (둘 다 z 를 뒤집으므로).

    X축 180도 뒤집힘 -> y 와 z 가 반전, x 는 그대로
    Y축 180도 뒤집힘 -> x 와 z 가 반전, y 는 그대로

측정 설계에서 조심할 점 세 가지 (앞선 두 판에서 여기서 틀렸습니다):

  1. `/wheel_odometry` 의 twist 는 실측이 아니라 **cmd_vel 명령값**입니다
     (tetra_drive_node.cpp: publish_odometry 가 vx, wz 를 그대로 실음).
     teleop 키를 떼면 즉시 0 이 되므로 기준으로 못 씁니다. 대신 **pose 의
     방위각/위치**를 씁니다 — 펌웨어가 엔코더로 적분한 실측값입니다.

  2. **좌회전과 우회전을 둘 다 하면 단순 합산은 상쇄되어 0 이 됩니다.**
     전진·후진도 같습니다. 그래서 합산이 아니라 **상관(곱의 누적)** 을
     봅니다. 부호가 늘 같으면 양수로, 늘 반대면 음수로 쌓이므로 좌우를
     섞어 움직일수록 오히려 표본이 좋아집니다.

  3. 차체가 조금만 기울어도 중력이 가속도계 x/y 로 샙니다. 0.6도면
     9.81*sin(0.6도) = 0.10 m/s^2 로 실제 주행 가속도와 비슷합니다.
     그래서 **정지 중 편향을 먼저 재서 빼고** 씁니다.

사용법:
    python3 check_imu_mounting.py

화면 안내대로 (1) 잠시 정지 (2) 좌우로 제자리 회전 (3) 전진·후진 을
편하게 섞어 움직이면 됩니다.
"""

import math

import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from sensor_msgs.msg import Imu


def yaw_of(q):
    """쿼터니언에서 yaw 만 뽑습니다 (평면 주행이라 z, w 면 충분)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class ImuMountingCheck(Node):

    STILL_GYRO = 0.02          # rad/s — 이보다 조용하면 정지로 봅니다
    BIAS_SAMPLES = 200         # 100Hz 기준 약 2초

    def __init__(self):
        super().__init__("imu_mounting_check")

        self.imu = None
        self.odom_yaw = None
        self.odom_xy = None

        # 1단계: 정지 편향
        self.b_ax = self.b_wz = 0.0
        self.b_n = 0
        self.bias_done = False

        # 2단계: 회전 상관
        self.rot_corr = 0.0        # sum(d_yaw_wheel * wz_imu)
        self.rot_mag = 0.0         # sum(|d_yaw_wheel|)  진행도
        self.rot_agree = 0         # 부호 일치 표본 수
        self.rot_disagree = 0

        # 3단계: 전후진 — 오도메트리에서 구한 실제 가속도와 회귀 비교
        self.prev_speed = None
        self.prev_t = None
        self.speed_lp = 0.0        # 저역통과한 속도 (1mm 분해능 잡음 억제)
        self.reg_num = 0.0         # sum(a_odom * ax_imu)
        self.reg_den = 0.0         # sum(a_odom^2)
        self.acc_n = 0
        self.dist_mag = 0.0        # sum(|이동거리|)  진행도
        self.max_a_odom = 0.0      # 가속이 충분했는지 보여주는 지표

        self.create_subscription(Imu, "/imu/data", self._on_imu, 50)
        self.create_subscription(Odometry, "/wheel_odometry", self._on_odom, 50)
        self.create_timer(0.4, self._report)

    # ------------------------------------------------------------------
    def _on_imu(self, msg):
        self.imu = msg
        if self.bias_done:
            return
        # 정지 중일 때만 편향을 모읍니다.
        if abs(msg.angular_velocity.z) < self.STILL_GYRO:
            self.b_ax += msg.linear_acceleration.x
            self.b_wz += msg.angular_velocity.z
            self.b_n += 1
            if self.b_n >= self.BIAS_SAMPLES:
                self.b_ax /= self.b_n
                self.b_wz /= self.b_n
                self.bias_done = True

    def _on_odom(self, msg):
        y = yaw_of(msg.pose.pose.orientation)
        p = (msg.pose.pose.position.x, msg.pose.pose.position.y)

        if self.odom_yaw is None or self.imu is None or not self.bias_done:
            self.odom_yaw, self.odom_xy = y, p
            return

        d_yaw = wrap(y - self.odom_yaw)
        dx = p[0] - self.odom_xy[0]
        dy = p[1] - self.odom_xy[1]
        self.odom_yaw, self.odom_xy = y, p

        wz = self.imu.angular_velocity.z - self.b_wz
        ax = self.imu.linear_acceleration.x - self.b_ax

        # --- 회전: 부호 상관 ---
        # 좌우를 섞어 돌려도 상쇄되지 않도록 곱을 누적합니다.
        if abs(d_yaw) > 0.0015:            # 잡음 문턱
            self.rot_corr += d_yaw * wz
            self.rot_mag += abs(d_yaw)
            if d_yaw * wz > 0:
                self.rot_agree += 1
            else:
                self.rot_disagree += 1

        # --- 전후진: 오도메트리 가속도와 IMU 가속도의 회귀 ---
        # 방향 부호를 곱하는 방식은 못 씁니다. 정지에서 출발해 정지로 끝나면
        # 가속(+)과 감속(-)이 정확히 상쇄되어 총합이 물리적으로 0 이 되기
        # 때문입니다. 대신 매 순간의 실제 가속도 a_odom 을 오도메트리에서
        # 구해 IMU 의 ax 와 회귀시킵니다. 기울기의 부호가 답입니다.
        #     기울기 > 0  ->  ax 가 로봇 전진축과 같은 방향  ->  X축 뒤집힘
        #     기울기 < 0  ->  반대 방향                      ->  Y축 뒤집힘
        along = dx * math.cos(y) + dy * math.sin(y)
        now = (msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9)
        if self.prev_t is not None and abs(d_yaw) < 0.01:
            dt = now - self.prev_t
            if 0.0 < dt < 0.2:
                speed = along / dt
                # 위치 분해능이 1mm 라 속도 미분이 거칠습니다. 가볍게 평활화.
                self.speed_lp += 0.35 * (speed - self.speed_lp)
                if self.prev_speed is not None:
                    a_odom = (self.speed_lp - self.prev_speed) / dt
                    if abs(a_odom) > 0.05:      # 가속 구간만
                        self.reg_num += a_odom * ax
                        self.reg_den += a_odom * a_odom
                        self.acc_n += 1
                        self.max_a_odom = max(self.max_a_odom, abs(a_odom))
                self.prev_speed = self.speed_lp
            self.dist_mag += abs(along)
        self.prev_t = now

    # ------------------------------------------------------------------
    def _report(self):
        if self.imu is None or self.odom_yaw is None:
            miss = []
            if self.imu is None:
                miss.append("/imu/data")
            if self.odom_yaw is None:
                miss.append("/wheel_odometry")
            print(f"\r수신 대기: {', '.join(miss)}", end="", flush=True)
            return

        print("\033[2J\033[H", end="")
        print("=" * 64)
        print(" IMU 장착 방향 확인   (좌/우, 전/후 섞어서 움직이세요)")
        print("=" * 64)

        az = self.imu.linear_acceleration.z
        print(f"  중력 az = {az:+7.2f}   "
              f"{'뒤집힘 (z축 아래)' if az < 0 else '정상 (z축 위)'}")
        print()

        if not self.bias_done:
            print(f"  [1] 정지 편향 측정 중 {self.b_n}/{self.BIAS_SAMPLES}"
                  "  — 로봇을 잠깐 가만히 두세요")
            print("=" * 64)
            return
        print(f"  [1] 편향 완료  ax {self.b_ax:+.3f}  wz {self.b_wz:+.4f}")

        # ---- 회전 ----
        print()
        deg = math.degrees(self.rot_mag)
        if deg < 25.0:
            print(f"  [2] 회전 — 제자리 회전(j/l) 시켜 주세요")
            print(f"      누적 회전량 {deg:5.1f}도 / 25도 필요")
        else:
            total = self.rot_agree + self.rot_disagree
            pct = 100.0 * self.rot_disagree / max(total, 1)
            print(f"  [2] 회전 {deg:.0f}도 측정  "
                  f"(부호 반대 {self.rot_disagree}/{total} = {pct:.0f}%)")
            if pct > 80:
                print("      -> z축 반전 확정 (IMU 뒤집힘 맞음)")
            elif pct < 20:
                print("      -> z축 정상! 뒤집힘 가정이 틀렸습니다")
            else:
                print("      -> 일관성 부족. 더 크게 회전시켜 주세요")

        # ---- 전후진 ----
        print()
        if self.acc_n < 40:
            print("  [3] 전후진 — 앞뒤로(i / ,) **급하게 출발·정지** 반복")
            print(f"      가속 표본 {self.acc_n}/40  "
                  f"(최대 가속도 {self.max_a_odom:.2f} m/s^2)")
            print("      속도를 0.15~0.2 로 올리고 키를 짧게 끊어 치세요")
        else:
            slope = self.reg_num / max(self.reg_den, 1e-9)
            print(f"  [3] 가속 표본 {self.acc_n}개, "
                  f"최대 가속 {self.max_a_odom:.2f} m/s^2")
            print(f"      회귀 기울기 {slope:+.2f} "
                  f"(정상 장착이면 +1 부근, 반전이면 -1 부근)")
            if abs(slope) < 0.25:
                print("      -> 기울기가 0 에 가까움. 가속이 약해 판정 보류")
            elif slope > 0:
                print("      -> 양수: X축 180도 뒤집힘")
                print("         URDF imu_link 에 rpy=\"3.14159 0 0\"")
            else:
                print("      -> 음수: Y축 180도 뒤집힘")
                print("         URDF imu_link 에 rpy=\"0 3.14159 0\"")

        print("=" * 64)


def main():
    rclpy.init()
    node = ImuMountingCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n측정 종료")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
