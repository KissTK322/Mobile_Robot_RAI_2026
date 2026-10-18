#!/usr/bin/env python3
"""Navigation challenge node: refbox topics + robot pose -> NavigationTask -> Nav2.

Needs ired_bringup, ired_navigation (Nav2 + AMCL on the field map) and
ired_refbox (mqtt_refbox) running. Publishes its state as JSON on
/challenge/navigation/state for the web app and for logging."""
import json
import math
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TwistStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from nav_msgs.msg import OccupancyGrid
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from refbox_msgs.msg import GamePlay, MachineInfoData, NavigationRoutesData
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from ired_challenge.escape import escape_direction, nearest
from ired_challenge.field import Field, mps_footprint
from ired_challenge.navigation_task import NavigationTask


class Nav2Adapter:
    """The small interface NavigationTask expects, on top of nav2_simple_commander."""

    def __init__(self, navigator, escape=None):
        self.n = navigator
        self.active = False
        self.last = 'succeeded'
        self._escape = escape

    def go_to(self, x, y, yaw):
        p = PoseStamped()
        p.header.frame_id = 'map'
        p.header.stamp = self.n.get_clock().now().to_msg()
        p.pose.position.x, p.pose.position.y = x, y
        q = quaternion_from_euler(0.0, 0.0, yaw)
        p.pose.orientation.x, p.pose.orientation.y, p.pose.orientation.z, p.pose.orientation.w = q
        self.active = self.n.goToPose(p)
        self.last = 'active' if self.active else 'failed'

    def state(self):
        if self.active and self.n.isTaskComplete():
            self.active = False
            self.last = 'succeeded' if self.n.getResult() == TaskResult.SUCCEEDED else 'failed'
        return self.last

    def cancel(self):
        if self.active:
            self.n.cancelTask()
            # Wait for the old goal to end before a new one is sent: a late cancel
            # otherwise reaches bt_navigator after the new goal and aborts that one.
            deadline = time.monotonic() + 2.0
            while not self.n.isTaskComplete() and time.monotonic() < deadline:
                pass
        self.active = False
        self.last = 'failed'

    def clear_costmaps(self):
        self.n.clearAllCostmaps()

    def escape(self):
        if self._escape:
            self._escape()


class NavigationChallengeNode(Node):
    def __init__(self):
        super().__init__('navigation_challenge')
        p = self.declare_parameters('', [
            ('zone_size', 1.0), ('origin_x', 0.0), ('origin_y', 0.0), ('red_zone', 'C_Z32'),
            ('dwell', 5.5), ('confirm_timeout', 4.0), ('drive_timeout', 40.0), ('retries', 2),
            ('order', 'nearest'), ('avoid_mps_zones', True), ('rate', 10.0),
            ('map_frame', 'map'), ('base_frame', 'base_footprint'),
            ('escape_speed', 0.06), ('escape_time', 3.0), ('escape_clearance', 0.30),
            ('scan_topic', '/scan_filtered'), ('cmd_vel_topic', '/cmd_vel'),
            ('stuck_time', 8.0), ('stuck_dist', 0.10),
        ])
        g = {x.name: x.value for x in p}
        self.cfg = g
        self.field = Field(g['zone_size'], (g['origin_x'], g['origin_y']))
        self.game_on = False
        self.remaining = None
        self.had_routes = False
        self.mps_zones = []
        self.mps_boxes = []          # (x, y, rotation) of each machine, map frame
        self.scan = None
        self.costmap = None          # the planner's view: walls, boxes and LiDAR, inflated

        self.create_subscription(GamePlay, '/refbox/game_play', self.on_game, 10)
        self.create_subscription(NavigationRoutesData, '/refbox/navigation_routes', self.on_routes, 10)
        self.create_subscription(MachineInfoData, '/refbox/machine_info', self.on_machines, 10)
        self.create_subscription(LaserScan, g['scan_topic'], lambda m: setattr(self, 'scan', m), qos_profile_sensor_data)
        self.cmd_pub = self.create_publisher(TwistStamped, g['cmd_vel_topic'], 10)
        self.create_subscription(OccupancyGrid, '/global_costmap/costmap', lambda m: setattr(self, 'costmap', m),
                                 QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                            durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.state_pub = self.create_publisher(String, '/challenge/navigation/state', 10)
        self.tf = Buffer()
        TransformListener(self.tf, self)

        self.navigator = BasicNavigator(node_name='navigation_challenge_navigator')
        self.get_logger().info('Waiting for Nav2 to be active...')
        # localizer='amcl' would make BasicNavigator publish its default initial pose (0, 0)
        # until AMCL answers, which throws AMCL out of the field. AMCL gets its start pose from
        # navigation.yaml (or the web app), so wait only for the lifecycle nodes to be active.
        self.navigator.waitUntilNav2Active(localizer='controller_server')
        self.task = NavigationTask(
            Nav2Adapter(self.navigator, self.escape), self.field, log=self.get_logger().info,
            red_zone=g['red_zone'], dwell=g['dwell'], confirm_timeout=g['confirm_timeout'],
            drive_timeout=g['drive_timeout'], retries=g['retries'], order=g['order'],
            stuck_time=g['stuck_time'], stuck_dist=g['stuck_dist'])
        self.get_logger().info(f"Navigation challenge ready (order={g['order']}, red zone {g['red_zone']})")

    def on_game(self, msg):
        self.game_on = bool(msg.status)

    def on_routes(self, msg):
        zones = [pt.zone for r in msg.routes for pt in r.remaining]
        if msg.routes and any(r.route for r in msg.routes):
            self.had_routes = True
        # An empty list only means "all done" after routes have been received
        self.remaining = zones if self.had_routes else None

    def on_machines(self, msg):
        zones, boxes = [], []
        for m in msg.machines:
            try:
                cx, cy = self.field.center(m.zone)
            except ValueError:
                continue
            zones.append(m.zone)
            boxes.append((cx, cy, float(m.rotation)))
        self.mps_zones = zones
        self.mps_boxes = boxes

    def obstacle_points(self, pose):
        """Nearby obstacle points in the robot frame: LiDAR (laser assumed at the robot
        centre, as on the iRED) plus the MPS boxes the LiDAR scans over."""
        pts = []
        sc = self.scan
        if sc is not None:
            try:
                t = self.tf.lookup_transform(self.cfg['base_frame'], sc.header.frame_id, Time())
                q = t.transform.rotation
                lyaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
                lx, ly = t.transform.translation.x, t.transform.translation.y
            except TransformException:
                lyaw, lx, ly = 0.0, 0.0, 0.0
            for i, r in enumerate(sc.ranges):
                if sc.range_min < r < 1.0:
                    a = sc.angle_min + i * sc.angle_increment + lyaw
                    pts.append((lx + r * math.cos(a), ly + r * math.sin(a)))
        x, y, yaw = pose
        c, s = math.cos(-yaw), math.sin(-yaw)
        cm = self.costmap
        if cm is not None:
            # lethal cells (100) are the obstacles themselves: map walls, MPS boxes, LiDAR hits
            res, ox, oy, w = cm.info.resolution, cm.info.origin.position.x, cm.info.origin.position.y, cm.info.width
            r_cells = int(0.6 / res)
            ci, cj = int((x - ox) / res), int((y - oy) / res)
            for j in range(max(0, cj - r_cells), min(cm.info.height, cj + r_cells + 1)):
                for i in range(max(0, ci - r_cells), min(w, ci + r_cells + 1)):
                    if cm.data[j * w + i] >= 100:
                        dx, dy = ox + (i + 0.5) * res - x, oy + (j + 0.5) * res - y
                        pts.append((c * dx - s * dy, s * dx + c * dy))
        for bx, by, rot in self.mps_boxes:
            for px, py in mps_footprint((bx, by), rot, margin=0.0, step=0.05):
                dx, dy = px - x, py - y
                pts.append((c * dx - s * dy, s * dx + c * dy))
        return pts

    def escape(self):
        """Slide slowly away from nearby obstacles until clear (or escape_time)."""
        g = self.cfg
        t_end = time.monotonic() + g['escape_time']
        moved = False
        while time.monotonic() < t_end and rclpy.ok():
            pose = self.pose()
            if pose is None:
                break
            pts = self.obstacle_points(pose)
            if nearest(pts) >= g['escape_clearance']:
                break
            d = escape_direction(pts, radius=g['escape_clearance'] + 0.15)
            if d is None:
                break
            cmd = TwistStamped()
            cmd.header.stamp = self.get_clock().now().to_msg()
            cmd.header.frame_id = g['base_frame']
            cmd.twist.linear.x, cmd.twist.linear.y = d[0] * g['escape_speed'], d[1] * g['escape_speed']
            self.cmd_pub.publish(cmd)
            moved = True
            time.sleep(0.1)
        stop = TwistStamped()
        stop.header.stamp = self.get_clock().now().to_msg()
        stop.header.frame_id = g['base_frame']
        self.cmd_pub.publish(stop)
        pose = self.pose()
        if pose:
            self.get_logger().info(f'Escape {"done" if moved else "not needed"}: at ({pose[0]:.2f}, {pose[1]:.2f}), '
                                   f'nearest obstacle {nearest(self.obstacle_points(pose)):.2f} m')

    def pose(self):
        try:
            t = self.tf.lookup_transform(self.cfg['map_frame'], self.cfg['base_frame'], Time())
        except TransformException:
            return None
        q = t.transform.rotation
        yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])[2]
        return (t.transform.translation.x, t.transform.translation.y, yaw)

    def step(self):
        pose = self.pose()
        if pose is None:
            return
        if self.cfg['avoid_mps_zones']:
            self.task.blocked = self.mps_zones
        now = self.get_clock().now().nanoseconds / 1e9
        info = self.task.update(now, self.game_on, self.remaining, pose)
        info.update(game_on=self.game_on, pose=[round(v, 3) for v in pose],
                    zone_now=self.field.zone_at(pose[0], pose[1]), mps_zones=self.mps_zones)
        self.state_pub.publish(String(data=json.dumps(info)))


def main(args=None):
    rclpy.init(args=args)
    node = NavigationChallengeNode()
    # step() runs in this thread, not as a timer callback: BasicNavigator spins its own node
    # while it sends a goal, and doing that inside a callback raises "Executor is already
    # spinning". The subscriptions (refbox, TF) are served by their own executor in a thread,
    # so the pose stays current while step() waits on Nav2.
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    threading.Thread(target=executor.spin, daemon=True).start()
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
