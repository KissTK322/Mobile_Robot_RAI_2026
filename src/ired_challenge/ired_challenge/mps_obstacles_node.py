#!/usr/bin/env python3
"""Puts the MPS stations into the Nav2 costmaps as obstacles.

The LiDAR scans above the 30 cm boxes, so Nav2 cannot see them. This node
publishes each box footprint as a PointCloud2 on /mps_obstacles (map frame),
for a separate `mps_layer` in the costmaps (marking only, so LiDAR clearing in
the normal obstacle layer cannot erase the boxes). Sources:
- /refbox/machine_info: zone + rotation of every machine (navigation challenge)
- /challenge/mps_estimates: JSON list [{"x","y","rotation"}, ...] from the
  exploration program once it has seen a box with the camera
Same idea as Carologistics' mps-map-gen."""
import json

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from refbox_msgs.msg import MachineInfoData
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, String

from ired_challenge.field import Field, mps_footprint


class MpsObstacles(Node):
    def __init__(self):
        super().__init__('mps_obstacles')
        p = self.declare_parameters('', [
            ('zone_size', 1.0), ('origin_x', 0.0), ('origin_y', 0.0), ('margin', 0.03),
            ('rate', 2.0), ('frame', 'map'),
        ])
        g = {x.name: x.value for x in p}
        self.cfg = g
        self.field = Field(g['zone_size'], (g['origin_x'], g['origin_y']))
        self.from_refbox = {}        # machine name -> (x, y, rotation)
        self.from_camera = {}
        self.create_subscription(MachineInfoData, '/refbox/machine_info', self.on_machines, 10)
        self.create_subscription(String, '/challenge/mps_estimates', self.on_estimates, 10)
        self.pub = self.create_publisher(PointCloud2, '/mps_obstacles',
                                         QoSProfile(depth=1, durability=DurabilityPolicy.VOLATILE))
        self.create_timer(1.0 / g['rate'], self.publish)

    def on_machines(self, msg):
        boxes = {}
        for m in msg.machines:
            try:
                cx, cy = self.field.center(m.zone)
            except ValueError:
                continue                         # no zone yet, or not on this field
            boxes[m.name] = (cx, cy, float(m.rotation))
        self.from_refbox = boxes

    def on_estimates(self, msg):
        try:
            items = json.loads(msg.data)
            self.from_camera = {str(i.get('machine', n)): (float(i['x']), float(i['y']), float(i.get('rotation', 0)))
                                for n, i in enumerate(items)}
        except (ValueError, KeyError, TypeError):
            self.get_logger().warn('Bad /challenge/mps_estimates message')

    def publish(self):
        boxes = {**self.from_camera, **self.from_refbox}   # the refbox wins when both know a machine
        pts = [(x, y, 0.15) for bx, by, rot in boxes.values()
               for x, y in mps_footprint((bx, by), rot, margin=self.cfg['margin'])]
        header = Header(frame_id=self.cfg['frame'], stamp=self.get_clock().now().to_msg())
        self.pub.publish(create_cloud_xyz32(header, pts))


def main(args=None):
    rclpy.init(args=args)
    node = MpsObstacles()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
