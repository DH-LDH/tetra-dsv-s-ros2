#!/usr/bin/env python3
"""현재 스캔을 지도에 가장 잘 맞추는 보정량을 찾아, 추정 자세가 얼마나
틀어져 있는지 수치로 낸다. yaw 를 먼저 훑고, 그 yaw 에서 x/y 를 훑는다.
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
        super().__init__("pose_error")
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


def score(dist, res, ox, oy, W, H, px, py):
    ix = ((px - ox) / res).astype(np.int32)
    iy = ((py - oy) / res).astype(np.int32)
    ok = (ix >= 0) & (ix < W) & (iy >= 0) & (iy < H)
    if ok.sum() < 20:
        return 9.99
    d = dist[iy[ok], ix[ok]]
    return float(np.median(d))


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
    try:
        tf = n.buf.lookup_transform("map", s.header.frame_id,
                                    rclpy.time.Time.from_msg(s.header.stamp))
    except Exception:
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

    base = score(dist, res, ox, oy, W, H,
                 tx + r*np.cos(a+th), ty + r*np.sin(a+th))
    print(f"현재 자세  x={tx:+.3f} y={ty:+.3f} yaw={math.degrees(th):+.2f} deg")
    print(f"현재 중앙값 오차 : {base*100:.1f} cm")

    # 1단계: yaw 만 훑는다
    yaws = np.arange(-25.0, 25.01, 0.25)
    best_y, best_s = 0.0, base
    for dy_deg in yaws:
        t2 = th + math.radians(dy_deg)
        v = score(dist, res, ox, oy, W, H,
                  tx + r*np.cos(a+t2), ty + r*np.sin(a+t2))
        if v < best_s:
            best_s, best_y = v, dy_deg

    # 2단계: 그 yaw 에서 x/y
    t2 = th + math.radians(best_y)
    cx, cy = np.cos(a+t2), np.sin(a+t2)
    best_dx = best_dy = 0.0
    for dx in np.arange(-0.6, 0.601, 0.05):
        for dyy in np.arange(-0.6, 0.601, 0.05):
            v = score(dist, res, ox, oy, W, H, tx+dx + r*cx, ty+dyy + r*cy)
            if v < best_s:
                best_s, best_dx, best_dy = v, dx, dyy

    print()
    print(f"가장 잘 맞는 보정 : yaw {best_y:+.2f} deg,  x {best_dx:+.2f} m,  y {best_dy:+.2f} m")
    print(f"보정 후 중앙값 오차 : {best_s*100:.1f} cm   (현재 {base*100:.1f} cm)")
    if abs(best_y) < 1.0 and abs(best_dx) < 0.1 and abs(best_dy) < 0.1:
        print("=> 자세는 사실상 맞습니다. 오차의 원인은 자세가 아닙니다")
    elif abs(best_y) >= 2.0:
        print(f"=> 방위가 {best_y:+.1f} deg 틀어져 있습니다 (회전 오차)")
    else:
        print("=> 주로 위치(평행이동) 오차입니다")

    n.destroy_node(); rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
