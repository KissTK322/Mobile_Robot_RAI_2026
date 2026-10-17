#!/usr/bin/env python3
"""Puts the MPS stations into the Nav2 costmaps as obstacles.

The LiDAR scans above the 30 cm boxes, so Nav2 cannot see them. This node
publishes each box footprint as a PointCloud2 on /mps_obstacles, for a
separate `mps_layer` in the costmaps (marking only, so LiDAR clearing in
the normal obstacle layer cannot erase the boxes). Sources:
- /refbox/machine_info: zone + rotation of every machine (navigation challenge)
- /challenge/mps_estimates: JSON list [{"x","y","rotation"}, ...] from the
  exploration program once it has seen a box with the camera
Same idea as Carologistics' mps-map-gen.

The points are sent in the robot frame: Nav2 takes the cloud's frame origin as
the sensor position and drops the whole cloud if that is outside the local
costmap window, which the map origin usually is."""
import json
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from refbox_msgs.msg import MachineInfoData
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import create_cloud_xyz32
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import euler_from_quaternion

from ired_challenge.field import Field, mps_footprint


class MpsObstacles(Node):
    def __init__(self):
        super().__init__('mps_obstacles')
        p = self.declare_parameters('', [
            ('zone_size', 1.0), ('origin_x', 0.0), ('origin_y', 0.0), ('margin', 0.03),
            ('rate', 2.0), ('map_frame', 'map'), ('robot_frame', 'base_footprint'),
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
        self.tf = Buffer()
        TransformListener(self.tf, self)
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
        try:
            t = self.tf.lookup_transform(self.cfg['map_frame'], self.cfg['robot_frame'], rclpy.time.Time())
        except TransformException:
            return                               # no localisation yet
        rx, ry = t.transform.translation.x, t.transform.translation.y
        q = t.transform.rotation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        c, s = math.cos(-yaw), math.sin(-yaw)
        pts = []
        for bx, by, rot in boxes.values():
            for x, y in mps_footprint((bx, by), rot, margin=self.cfg['margin']):
                dx, dy = x - rx, y - ry              # map -> robot frame
                pts.append((c * dx - s * dy, s * dx + c * dy, 0.15))
        header = Header(frame_id=self.cfg['robot_frame'], stamp=t.header.stamp)
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
