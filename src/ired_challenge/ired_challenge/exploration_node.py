#!/usr/bin/env python3
"""Exploration challenge node: ArUco detections + robot pose -> ExplorationTask.

Needs ired_bringup (with the ArUco node publishing /aruco/detections), ired_navigation
(Nav2 + AMCL on the field map) and ired_refbox (mqtt_refbox, /refbox/machine_report).
Drives along the zone borders with Nav2's controller (followPath, no Nav2 planning),
turns in place with its own small controller on /cmd_vel, and reports each machine
once through /refbox/machine_report. State as JSON on /challenge/exploration/state."""
import json
import math
import threading
import time

import numpy as np
import rclpy
from geometry_msgs.msg import TwistStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
from nav_msgs.msg import OccupancyGrid
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from rclpy.time import Time
from refbox_msgs.msg import GamePlay
from refbox_msgs.srv import MachineReport
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import euler_from_quaternion

from ired_challenge.exploration import MachineEstimator
from ired_challenge.exploration_task import ExplorationTask
from ired_challenge.field import Field
from ired_challenge.grid_planner import seg_point_dist
from ired_challenge.navigation_node import Nav2Adapter, wait_nav2_active


class Mover(Nav2Adapter):
    """Nav2 controller for the border routes, plus turning in place on /cmd_vel."""

    def __init__(self, navigator, node):
        super().__init__(navigator)
        self.node = node
        self.turn_goal = None

    def turn_to(self, yaw):
        self.cancel()
        self.turn_goal = yaw
        self.last = 'active'

    def cancel(self):
        if self.turn_goal is not None:
            self.turn_goal = None
            self.node.send_cmd(0.0)
        super().cancel()

    def state(self):
        if self.turn_goal is None:
            return super().state()
        pose = self.node.pose()
        if pose is None:
            return 'active'
        err = math.atan2(math.sin(self.turn_goal - pose[2]), math.cos(self.turn_goal - pose[2]))
        if abs(err) < math.radians(4):
            self.turn_goal = None
            self.node.send_cmd(0.0)
            self.last = 'succeeded'
            return 'succeeded'
        w = max(-0.8, min(0.8, 1.6 * err))
        if abs(w) < 0.25:
            w = math.copysign(0.25, w)            # enough to overcome friction
        self.node.send_cmd(w)
        return 'active'


class ExplorationNode(Node):
    def __init__(self):
        super().__init__('exploration_challenge')
        p = self.declare_parameters('', [
            ('zone_size', 1.0), ('origin_x', 0.0), ('origin_y', 0.0), ('red_zone', 'C_Z32'),
            ('expected', ['C-BS', 'C-CS1']), ('skip_zones', ['C_Z31', 'C_Z41', 'C_Z51', 'C_Z32']),
            ('look_time', 1.5), ('wall_clearance', 0.30), ('rate', 10.0),
            ('min_sightings', 10), ('min_views', 2), ('max_range', 2.0), ('refbox_rules', False),
            ('map_frame', 'map'), ('base_frame', 'base_footprint'), ('cmd_vel_topic', '/cmd_vel'),
        ])
        g = {x.name: x.value for x in p}
        self.cfg = g
        self.field = Field(g['zone_size'], (g['origin_x'], g['origin_y']))
        self.game_on = False
        self.walls = np.zeros((0, 2))
        self.est = MachineEstimator(self.field, min_sightings=g['min_sightings'], min_views=g['min_views'],
                                    max_range=g['max_range'], refbox_rules=g['refbox_rules'])
        self.tf = Buffer()
        TransformListener(self.tf, self)
        self.create_subscription(GamePlay, '/refbox/game_play', lambda m: setattr(self, 'game_on', bool(m.status)), 10)
        self.create_subscription(String, '/aruco/detections', self.on_detections, 10)
        self.create_subscription(OccupancyGrid, '/map', self.on_map,
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.cmd_pub = self.create_publisher(TwistStamped, g['cmd_vel_topic'], 10)
        self.state_pub = self.create_publisher(String, '/challenge/exploration/state', 10)
        self.report_cli = self.create_client(MachineReport, '/refbox/machine_report')

        self.navigator = BasicNavigator(node_name='exploration_challenge_navigator')
        self.get_logger().info('Waiting for Nav2 to be active...')
        wait_nav2_active(self.navigator, self.get_logger().info)
        skip = set(g['skip_zones'])
        self.task = ExplorationTask(
            Mover(self.navigator, self), self.field, self.est, self.report, log=self.get_logger().info,
            expected=g['expected'], red_zone=g['red_zone'], look_time=g['look_time'],
            candidate=lambda z: z not in skip, free=self.free)
        self.get_logger().info(f"Exploration challenge ready, looking for {g['expected']}")

    # ------------------------------------------------------------ inputs
    def on_map(self, msg):
        w, res = msg.info.width, msg.info.resolution
        ox, oy = msg.info.origin.position.x, msg.info.origin.position.y
        self.walls = np.array([(ox + (i % w + 0.5) * res, oy + (i // w + 0.5) * res)
                               for i, v in enumerate(msg.data) if v >= 65]).reshape(-1, 2)

    def free(self, p):
        return seg_point_dist(p, p, self.walls) >= self.cfg['wall_clearance']

    def on_detections(self, msg):
        pose = self.pose()
        if pose is None:
            return
        try:
            data = json.loads(msg.data)
        except ValueError:
            return
        for m in data.get('markers', []):
            self.est.add(pose, int(m.get('id', -1)), m.get('position_base_link_m'), m.get('marker_yaw_deg'))

    def pose(self):
        try:
            t = self.tf.lookup_transform(self.cfg['map_frame'], self.cfg['base_frame'], Time())
        except TransformException:
            return None
        q = t.transform.rotation
        return (t.transform.translation.x, t.transform.translation.y, euler_from_quaternion([q.x, q.y, q.z, q.w])[2])

    # ------------------------------------------------------------ outputs
    def send_cmd(self, w):
        c = TwistStamped()
        c.header.stamp = self.get_clock().now().to_msg()
        c.header.frame_id = self.cfg['base_frame']
        c.twist.angular.z = float(w)
        self.cmd_pub.publish(c)

    def report(self, machine, x, y, rotation_deg):
        """One machine report; the refbox bridge wants yaw in degrees 0..359."""
        if not self.report_cli.service_is_ready():
            self.get_logger().warn('/refbox/machine_report not available yet')
            return False
        req = MachineReport.Request()
        req.machine, req.x, req.y, req.yaw = machine, float(x), float(y), float(rotation_deg % 360)
        fut = self.report_cli.call_async(req)
        t_end = time.monotonic() + 2.0
        while not fut.done() and time.monotonic() < t_end:
            time.sleep(0.02)
        ok = fut.done() and fut.result() is not None and fut.result().status
        self.get_logger().info(f'Report {machine} x={x:.2f} y={y:.2f} yaw={rotation_deg % 360:.0f}: '
                               + ('sent' if ok else 'FAILED'))
        return bool(ok)

    def step(self):
        pose = self.pose()
        if pose is None:
            return
        info = self.task.update(self.get_clock().now().nanoseconds / 1e9, self.game_on, pose)
        info['pose'] = [round(v, 3) for v in pose]
        self.state_pub.publish(String(data=json.dumps(info, default=str)))


def main(args=None):
    rclpy.init(args=args)
    node = ExplorationNode()
    # same threading as the navigation node: callbacks in a thread, the task in this one
    ex = SingleThreadedExecutor()
    ex.add_node(node)
    threading.Thread(target=ex.spin, daemon=True).start()
    period = 1.0 / node.cfg['rate']
    try:
        while rclpy.ok():
            t = time.monotonic()
            node.step()
            time.sleep(max(0.0, period - (time.monotonic() - t)))
    except KeyboardInterrupt:
        pass
    finally:
        node.task.nav.cancel()
        node.navigator.destroy_node()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
