#!/usr/bin/env python3
"""
scan_self_check.py
--------------------
Find the EXACT angular ranges where the robot sees its own structure
(wheels/brackets), so the laser_filters angular bound filters can be set
tight instead of guessed too wide (too-wide bands cut out real walls too).

HOW TO USE:
1. Move the robot to an open area with NOTHING real closer than ~0.6m in
   any direction (open floor space, nothing near it at all).
2. Run this (needs bringup / rplidar_node already running and publishing
   /scan):
       source ~/ros_environment.bash
       python3 scan_self_check.py
3. It grabs ONE scan, finds every angular range where range < threshold
   (0.6m by default - only self-detections should trigger this since
   nothing real is that close), groups consecutive hits, and prints each
   group's angle range in both radians and degrees.
4. Use those printed ranges (with maybe +/-0.5 deg margin) as the new
   lower_angle/upper_angle values in laser_config.yaml - tight instead of
   the current guessed-too-wide bands.

Adjust --threshold if 0.6m isn't appropriate (e.g. if the open area you
have isn't perfectly clear that far, lower it - but keep it clearly above
where the self-detections show up, which earlier data suggested is
~0.19-0.2m).
"""

import argparse
import math

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan


class ScanAnalyzer(Node):
    def __init__(self, threshold):
        super().__init__('scan_self_check')
        self.threshold = threshold
        self.done = False
        self.sub = self.create_subscription(LaserScan, '/scan_filtered', self.callback, 10)

    def callback(self, msg):
        if self.done:
            return
        self.done = True

        print(f"angle_min={msg.angle_min:.4f} rad, angle_increment={msg.angle_increment:.6f} rad")
        print(f"Looking for ranges < {self.threshold} m (potential self-detections)\n")

        groups = []
        current = []
        for i, r in enumerate(msg.ranges):
            if 0.0 < r < self.threshold:
                angle = msg.angle_min + i * msg.angle_increment
                current.append((i, angle, r))
            else:
                if current:
                    groups.append(current)
                    current = []
        if current:
            groups.append(current)

        if not groups:
            print("No close ranges found - either the robot is clear of anything "
                  "within threshold, or threshold is too low.")
        else:
            print(f"Found {len(groups)} group(s) of close readings:\n")
            for gi, g in enumerate(groups):
                idxs = [x[0] for x in g]
                angles = [x[1] for x in g]
                ranges = [x[2] for x in g]
                lo_a, hi_a = min(angles), max(angles)
                print(f"--- Group {gi+1} ---")
                print(f"  index range   : {min(idxs)} to {max(idxs)}  ({len(g)} points)")
                print(f"  angle range   : {lo_a:.4f} to {hi_a:.4f} rad "
                      f"({math.degrees(lo_a):.2f} deg to {math.degrees(hi_a):.2f} deg)")
                print(f"  range values  : {min(ranges):.3f} to {max(ranges):.3f} m")
                print(f"  suggested filter bounds (rad, +/- small margin):")
                print(f"    lower_angle: {lo_a - 0.01:.4f}")
                print(f"    upper_angle: {hi_a + 0.01:.4f}\n")

        rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--threshold', type=float, default=0.6,
                         help='Range threshold in meters below which a reading is '
                              'treated as a possible self-detection (default 0.6)')
    args = parser.parse_args()

    rclpy.init()
    node = ScanAnalyzer(args.threshold)
    rclpy.spin(node)


if __name__ == '__main__':
    main()
