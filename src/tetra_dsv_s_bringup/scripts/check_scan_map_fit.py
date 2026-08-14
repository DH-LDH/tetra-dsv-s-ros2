#!/usr/bin/env python3
"""스캔 점이 지도의 벽에서 얼마나 떨어져 있는지 젯슨에서 직접 잰다.

RViz 에서 "주황이 검정에 안 붙는다" 를 눈으로 판단하는 대신 수치로 만든다.
무선 렌더링 지연과 진짜 위치추정 오차를 가르는 것이 목적이라, 반드시
젯슨에서 (네트워크를 안 건너고) 돌려야 한다.
"""
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy, QoSHistoryPolicy
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


class Fit(Node):
    def __init__(self):
        super().__init__("scan_map_fit")
        self.map = None
        self.scan = None

        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        scan_qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(OccupancyGrid, "/map", self.on_map, map_qos)
        self.create_subscription(LaserScan, "/scan", self.on_scan, scan_qos)

        self.buf = Buffer()
        self.listener = TransformListener(self.buf, self)

    def on_map(self, m):
        self.map = m

    def on_scan(self, s):
        self.scan = s


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    rclpy.init()
    node = Fit()

    # 지도와 스캔, TF 가 다 모일 때까지 기다린다
    deadline = node.get_clock().now().nanoseconds + 20e9
    while rclpy.ok():
        rclpy.spin_once(node, timeout_sec=0.2)
        if node.map is not None and node.scan is not None:
            if node.buf.can_transform("map", node.scan.header.frame_id,
                                      rclpy.time.Time()):
                break
        if node.get_clock().now().nanoseconds > deadline:
            print("타임아웃: map/scan/TF 를 다 못 받았습니다")
            print(f"  map={node.map is not None} scan={node.scan is not None}")
            return 1

    m, s = node.map, node.scan

    # TF 는 반드시 **스캔의 타임스탬프로** 조회한다. "가장 최근"(Time())으로
    # 받으면 회전 중에 시간차가 그대로 오차로 들어온다 — 0.3 rad/s 에서
    # 100 ms 만 어긋나도 1.7도, 5 m 거리의 벽에서 15 cm 다. 2026-08-14 에
    # 이것 때문에 정렬이 5 cm 에서 20 cm 로 나빠진 것처럼 보였다.
    stamp = rclpy.time.Time.from_msg(s.header.stamp)
    try:
        tf = node.buf.lookup_transform("map", s.header.frame_id, stamp)
        exact_time = True
    except Exception as e:
        tf = node.buf.lookup_transform("map", s.header.frame_id, rclpy.time.Time())
        exact_time = False
        print(f"주의: 스캔 시각의 TF 를 못 찾아 최근 값으로 대체했습니다 ({e})")
        print("      로봇이 회전 중이면 아래 수치가 실제보다 나쁘게 나옵니다")

    tx = tf.transform.translation.x
    ty = tf.transform.translation.y
    th = yaw_of(tf.transform.rotation)
    if exact_time:
        print("TF 조회: 스캔 타임스탬프 기준 (회전 중에도 유효)")

    grid = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    occ = grid >= 65
    res = m.info.resolution
    ox = m.info.origin.position.x
    oy = m.info.origin.position.y

    # 각 자유/미지 칸에서 가장 가까운 점유 칸까지의 거리 (칸 단위)
    try:
        from scipy.ndimage import distance_transform_edt
        dist = distance_transform_edt(~occ) * res
        exact = True
    except ImportError:
        dist = None
        exact = False

    rng = np.array(s.ranges, dtype=np.float64)
    ang = s.angle_min + np.arange(len(rng)) * s.angle_increment
    good = np.isfinite(rng) & (rng > s.range_min) & (rng < min(s.range_max, 20.0))
    rng, ang = rng[good], ang[good]

    # 스캔 프레임 -> map
    px = tx + rng * np.cos(ang + th)
    py = ty + rng * np.sin(ang + th)
    ix = ((px - ox) / res).astype(int)
    iy = ((py - oy) / res).astype(int)
    inside = (ix >= 0) & (ix < m.info.width) & (iy >= 0) & (iy < m.info.height)
    ix, iy = ix[inside], iy[inside]

    n = len(ix)
    print(f"지도   {m.info.width} x {m.info.height} @ {res} m,  점유 칸 {occ.sum()}")
    print(f"스캔   유효 {len(rng)} 점, 지도 범위 안 {n} 점")
    if n == 0:
        print("지도 범위 안에 스캔 점이 없습니다 — 위치추정이 크게 틀어졌습니다")
        return 2

    if exact:
        d = dist[iy, ix]
        for t in (0.05, 0.10, 0.20, 0.50):
            print(f"  벽에서 {t*100:>4.0f} cm 이내 : {(d <= t).sum():5d} 점  {(d<=t).mean()*100:5.1f}%")
        print(f"  중앙값 거리 : {np.median(d)*100:.1f} cm")
        print(f"  평균   거리 : {d.mean()*100:.1f} cm")
    else:
        hit = occ[iy, ix]
        print(f"  점유 칸에 정확히 얹힌 점 : {hit.sum()} / {n}  ({hit.mean()*100:.1f}%)")
        print("  (scipy 없음 — 거리 분포는 못 냅니다)")

    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
