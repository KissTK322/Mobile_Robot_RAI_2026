#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys, tty, termios, select

# Constants
MAX_LINEAR_SPEED = 0.35
MAX_ANGULAR_SPEED = 0.6
STEP_LINEAR = 0.05
STEP_ANGULAR = 0.2

instructions = """
Control Your Robot!
---------------------------
Moving:
   w    
a  s  d
   x

W/X : Increase/Decrease linear X
A/D : Increase/Decrease linear Y
a/d : Increase/Decrease angular Z
space/s : stop
CTRL-C : quit
"""

def get_key():
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    key = sys.stdin.read(1) if rlist else ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def clamp(val, min_val, max_val):
    return max(min(val, max_val), min_val)

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        self.publisher = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0

    def publish_cmd(self):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = self.target_linear_x
        msg.twist.linear.y = self.target_linear_y
        msg.twist.angular.z = self.target_angular_z
        self.publisher.publish(msg)

    def stop_robot(self):
        self.target_linear_x = 0.0
        self.target_linear_y = 0.0
        self.target_angular_z = 0.0
        self.publish_cmd()

def main():
    global settings
    settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = TeleopNode()
    print(instructions)

    try:
        while True:
            key = get_key()
            if key == 'w':
                node.target_linear_x = clamp(node.target_linear_x + STEP_LINEAR, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED)
            elif key == 'x':
                node.target_linear_x = clamp(node.target_linear_x - STEP_LINEAR, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED)
            elif key == 'a':
                node.target_angular_z = clamp(node.target_angular_z + STEP_ANGULAR, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
            elif key == 'd':
                node.target_angular_z = clamp(node.target_angular_z - STEP_ANGULAR, -MAX_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
            elif key == 'W':
                node.target_linear_x = clamp(node.target_linear_x + STEP_LINEAR, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED)
            elif key == 'X':
                node.target_linear_x = clamp(node.target_linear_x - STEP_LINEAR, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED)
            elif key == 'A':
                node.target_linear_y = clamp(node.target_linear_y + STEP_LINEAR, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED)
            elif key == 'D':
                node.target_linear_y = clamp(node.target_linear_y - STEP_LINEAR, -MAX_LINEAR_SPEED, MAX_LINEAR_SPEED)
            elif key in [' ', 's', 'S']:
                node.stop_robot()
            elif key == '\x03':  # CTRL-C
                break

            node.publish_cmd()
            print(f"Linear: x = {node.target_linear_x:.2f} m/s, y = {node.target_linear_y:.2f} m/s, Angular: theta = {node.target_angular_z:.2f} rad/s" , end='\r')

    except Exception as e:
        print("\nError:", e)

    finally:
        node.stop_robot()
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        node.destroy_node()
        rclpy.shutdown()
        print("\nShutting down teleop keyboard.")

if __name__ == '__main__':
    main()
