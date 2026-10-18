#!/usr/bin/env python3
"""Navigation challenge node: refbox topics + robot pose -> NavigationTask -> Nav2.

Needs ired_bringup, ired_navigation (Nav2 + AMCL on the field map) and
ired_refbox (mqtt_refbox) running. Publishes its state as JSON on
/challenge/navigation/state for the web app and for logging."""
import json
import threading
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.time import Time
from refbox_msgs.msg import GamePlay, MachineInfoData, NavigationRoutesData
from std_msgs.msg import String
from tf2_ros import Buffer, TransformException, TransformListener
from tf_transformations import euler_from_quaternion, quaternion_from_euler

from ired_challenge.field import Field
from ired_challenge.navigation_task import NavigationTask


class Nav2Adapter:
    """The small interface NavigationTask expects, on top of nav2_simple_commander."""

    def __init__(self, navigator):
        self.n = navigator
        self.active = False
        self.last = 'succeeded'

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


class NavigationChallengeNode(Node):
    def __init__(self):
        super().__init__('navigation_challenge')
        p = self.declare_parameters('', [
            ('zone_size', 1.0), ('origin_x', 0.0), ('origin_y', 0.0), ('red_zone', 'C_Z32'),
            ('dwell', 5.5), ('confirm_timeout', 4.0), ('drive_timeout', 40.0), ('retries', 2),
            ('order', 'nearest'), ('avoid_mps_zones', True), ('rate', 10.0),
            ('map_frame', 'map'), ('base_frame', 'base_footprint'),
        ])
        g = {x.name: x.value for x in p}
        self.cfg = g
        self.field = Field(g['zone_size'], (g['origin_x'], g['origin_y']))
        self.game_on = False
        self.remaining = None
        self.had_routes = False
        self.mps_zones = []

        self.create_subscription(GamePlay, '/refbox/game_play', self.on_game, 10)
        self.create_subscription(NavigationRoutesData, '/refbox/navigation_routes', self.on_routes, 10)
        self.create_subscription(MachineInfoData, '/refbox/machine_info', self.on_machines, 10)
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
            Nav2Adapter(self.navigator), self.field, log=self.get_logger().info,
            red_zone=g['red_zone'], dwell=g['dwell'], confirm_timeout=g['confirm_timeout'],
            drive_timeout=g['drive_timeout'], retries=g['retries'], order=g['order'])
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
        zones = []
        for m in msg.machines:
            try:
                self.field.parse(m.zone)
                zones.append(m.zone)
            except ValueError:
                pass
        self.mps_zones = zones

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
