#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from ired_msgs.msg import MotorData

class FORWARDKINEMATICS(Node):
    def __init__(self):
        super().__init__('forward_kinematics_node')
        
        # Initialise Variables and Parameters
        self.initial_parameters()
        self.initial_variables()
        
        # Initialise ROS Publisher and Subscriber
        self.robot_speed_pub = self.create_publisher(TwistStamped, self.forward_kinematics_topic, 10)
        self.create_subscription(MotorData, self.motor_data_topic, self.fetchMotorSpeedCallback, 10)
        
        # Initialise ROS Timers
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.update_callback)
        
        # ROS Logger
        self.get_logger().info(f"Forward : ROS publisher on {self.forward_kinematics_topic} [geometry_msgs/TwistStamped]")
        self.get_logger().info(f"Forward : ROS subscriber on {self.motor_data_topic} [ired_msgs/MotorData]")
    
    def initial_parameters(self):
        # Declare parameters
        self.declare_parameter('motor_data_topic', '/ired/motor/data')
        self.declare_parameter('robot_speedtopic', '/ired/speed')
        self.declare_parameter('wheel_seperation', 0.22)
        self.declare_parameter('rpm_to_mps', 0.00680678)
        
        # ROS Parameters
        self.motor_data_topic = self.get_parameter('motor_data_topic').get_parameter_value().string_value
        self.forward_kinematics_topic = self.get_parameter('robot_speedtopic').get_parameter_value().string_value
        self.wheel_seperation = self.get_parameter('wheel_seperation').get_parameter_value().double_value
        self.rpm_to_mps = self.get_parameter('rpm_to_mps').get_parameter_value().double_value
    
    def initial_variables(self):
        self.wheel_speed_fb = [0.0, 0.0, 0.0, 0.0]
        self.robot_speed = TwistStamped()
        self.robot_speed.twist.linear.x = 0.0
        self.robot_speed.twist.linear.y = 0.0
        self.robot_speed.twist.linear.z = 0.0
        self.robot_speed.twist.angular.x = 0.0
        self.robot_speed.twist.angular.y = 0.0
        self.robot_speed.twist.angular.z = 0.0
        self.FLEFT = 0
        self.FRIGHT = 1
        self.RLEFT = 2
        self.RRIGHT = 3
        self.last_time_speed_fb = self.get_clock().now()
    
    def fetchMotorSpeedCallback(self, msg):
        self.last_time_speed_fb = self.get_clock().now()
        self.wheel_speed_fb = list(msg.speed_fb)
    
    def calculateForwardKinematics(self):
        w_fl = self.wheel_speed_fb[self.FLEFT]
        w_fr = self.wheel_speed_fb[self.FRIGHT]
        w_rl = self.wheel_speed_fb[self.RLEFT]
        w_rr = self.wheel_speed_fb[self.RRIGHT]

        w_fl_mps = w_fl * self.rpm_to_mps
        w_fr_mps = w_fr * self.rpm_to_mps
        w_rl_mps = w_rl * self.rpm_to_mps
        w_rr_mps = w_rr * self.rpm_to_mps

        R = self.wheel_seperation

        v_x = 0.5 * (w_fl_mps - w_fr_mps + w_rl_mps - w_rr_mps) * -1.0
        v_y = 0.5 * (-w_fl_mps - w_fr_mps + w_rl_mps + w_rr_mps) * -1.0
        omega = (-1.0 / (4.0 * R)) * (w_fl_mps + w_fr_mps + w_rl_mps + w_rr_mps) * -1.0

        self.robot_speed.twist.linear.x = v_x
        self.robot_speed.twist.linear.y = v_y
        self.robot_speed.twist.angular.z = omega

    def update_callback(self):
        time_now = self.get_clock().now()

        if (time_now - self.last_time_speed_fb).nanoseconds / 1e9 > 1.0:
            self.wheel_speed_fb = [0.0] * 4

        self.calculateForwardKinematics()
        self.robot_speed.header.stamp = self.get_clock().now().to_msg()
        self.robot_speed_pub.publish(self.robot_speed)

def main(args=None):
    rclpy.init(args=args)
    forward_kinematics = FORWARDKINEMATICS()

    rclpy.spin(forward_kinematics)

    forward_kinematics.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
