#!/usr/bin/env python3
import argparse
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class PoleFinder(Node):
    def __init__(self, topic: str, threshold: float):
        super().__init__('pole_finder')
        self.topic = topic
        self.threshold = threshold
        self.done = False
        self.sub = self.create_subscription(LaserScan, topic, self.cb, 10)
        self.get_logger().info(f'Listening on {topic}, threshold={threshold} m ...')

    def cb(self, msg: LaserScan):
        if self.done:
            return
        self.done = True

        clusters = []
        current = []

        for i, r in enumerate(msg.ranges):
            is_close = (not math.isinf(r)) and (not math.isnan(r)) and (r < self.threshold) and (r >= msg.range_min)
            if is_close:
                current.append(i)
            else:
                if current:
                    clusters.append(current)
                    current = []
        if current:
            clusters.append(current)

        if clusters and clusters[0][0] == 0 and clusters[-1][-1] == len(msg.ranges) - 1 and len(clusters) > 1:
            clusters[0] = clusters[-1] + clusters[0]
            clusters.pop()

        print(f'\nTotal points: {len(msg.ranges)}')
        print(f'angle_min={msg.angle_min:.6f}  angle_max={msg.angle_max:.6f}  angle_increment={msg.angle_increment:.6f}')
        print(f'Found {len(clusters)} close-range cluster(s) (< {self.threshold} m):\n')

        for n, c in enumerate(clusters, 1):
            i_start, i_end = c[0], c[-1]
            a_start = msg.angle_min + i_start * msg.angle_increment
            a_end = msg.angle_min + i_end * msg.angle_increment
            lo, hi = sorted([a_start, a_end])
            sample_vals = [msg.ranges[i] for i in c[:5]]
            print(f'  Cluster {n}: index {i_start}-{i_end}  '
                  f'({len(c)} pts)  ranges~{[round(v,3) for v in sample_vals]}')
            print(f'    angle: {lo:.4f} to {hi:.4f} rad   '
                  f'({math.degrees(lo):.1f} to {math.degrees(hi):.1f} deg)')
            print(f'    -> YAML: lower_angle: {lo - 0.03:.3f}   upper_angle: {hi + 0.03:.3f}\n')

        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--threshold', type=float, default=0.35)
    parser.add_argument('--topic', type=str, default='/scan')
    args = parser.parse_args()

    rclpy.init()
    node = PoleFinder(args.topic, args.threshold)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
