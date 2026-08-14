#!/usr/bin/env python3
"""벽에서 먼 스캔 점이 '철창 투과'인지 '미탐색 영역'인지 가른다.

판정: 센서에서 점까지 빔 경로를 격자로 따라가, 도착점 직전에 이미 점유 칸을
지났으면 '투과'(뭔가를 뚫고 지나감), 아니면 '미탐색'.
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


class N(Node):
    def __init__(self):
        super().__init__("why_far")
        self.map = None
        self.scan = None
        mq = QoSProfile(depth=1, reliability=QoSReliabilityPolicy.RELIABLE,
                        durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                        history=QoSHistoryPolicy.KEEP_LAST)
        sq = QoSProfile(depth=5, reliability=QoSReliabilityPolicy.BEST_EFFORT,
                        history=QoSHistoryPolicy.KEEP_LAST)
        self.create_subscription(OccupancyGrid, "/map", lambda m: setattr(self, "map", m), mq)
        self.create_subscription(LaserScan, "/scan", lambda s: setattr(self, "scan", s), sq)
        self.buf = Buffer()
        self.tl = TransformListener(self.buf, self)


def yaw_of(q):
    return math.atan2(2*(q.w*q.z + q.x*q.y), 1 - 2*(q.y*q.y + q.z*q.z))


def main():
    rclpy.init()
    n = N()
    dl = n.get_clock().now().nanoseconds + 20e9
    while rclpy.ok():
        rclpy.spin_once(n, timeout_sec=0.2)
        if n.map and n.scan and n.buf.can_transform("map", n.scan.header.frame_id, rclpy.time.Time()):
            break
        if n.get_clock().now().nanoseconds > dl:
            print("타임아웃"); return 1

    m, s = n.map, n.scan
    tf = n.buf.lookup_transform("map", s.header.frame_id, rclpy.time.Time())
    tx, ty = tf.transform.translation.x, tf.transform.translation.y
    th = yaw_of(tf.transform.rotation)

    g = np.array(m.data, dtype=np.int16).reshape(m.info.height, m.info.width)
    occ = g >= 65
    res, ox, oy = m.info.resolution, m.info.origin.position.x, m.info.origin.position.y
    W, H = m.info.width, m.info.height

    from scipy.ndimage import distance_transform_edt
    dist = distance_transform_edt(~occ) * res

    r = np.array(s.ranges, dtype=np.float64)
    a = s.angle_min + np.arange(len(r)) * s.angle_increment
    ok = np.isfinite(r) & (r > s.range_min) & (r < min(s.range_max, 20.0))
    r, a = r[ok], a[ok]
    px, py = tx + r*np.cos(a+th), ty + r*np.sin(a+th)

    sx, sy = (tx-ox)/res, (ty-oy)/res          # 센서 위치 (칸)
    ex, ey = (px-ox)/res, (py-oy)/res          # 도착점 (칸)

    far = []
    for k in range(len(r)):
        ix, iy = int(ex[k]), int(ey[k])
        if not (0 <= ix < W and 0 <= iy < H):
            continue
        if dist[iy, ix] > 0.50:                 # 벽에서 50 cm 초과 = '먼 점'
            far.append(k)

    through = 0
    unexplored = 0
    for k in far:
        steps = int(max(abs(ex[k]-sx), abs(ey[k]-sy)))
        if steps < 4:
            continue
        t = np.linspace(0.0, 1.0, steps)
        cx = (sx + (ex[k]-sx)*t).astype(int)
        cy = (sy + (ey[k]-sy)*t).astype(int)
        inb = (cx >= 0) & (cx < W) & (cy >= 0) & (cy < H)
        cx, cy = cx[inb], cy[inb]
        if len(cx) < 5:
            continue
        # 도착점 근처 3칸은 제외 (자기 자신이 만든 벽일 수 있음)
        hit = occ[cy[:-3], cx[:-3]].any()
        if hit:
            through += 1
        else:
            unexplored += 1

    tot = through + unexplored
    print(f"전체 유효 스캔 점        {len(r)}")
    print(f"벽에서 50 cm 초과인 점   {len(far)}  ({len(far)/len(r)*100:.1f}%)")
    if tot == 0:
        print("판정할 먼 점이 없습니다"); return 0
    print(f"  그중 판정 가능          {tot}")
    print(f"  ├ 뭔가를 뚫고 지나감 (철창 투과)  {through:4d}  {through/tot*100:5.1f}%")
    print(f"  └ 경로에 벽 없음 (미탐색 영역)    {unexplored:4d}  {unexplored/tot*100:5.1f}%")
    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
