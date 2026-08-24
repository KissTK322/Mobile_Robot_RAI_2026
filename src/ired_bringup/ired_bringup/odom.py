#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped, TransformStamped
from ired_msgs.msg import IMUData, MotorData
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from tf2_msgs.msg import TFMessage
from tf_transformations import quaternion_from_euler
from math import sin, cos, isnan

class ODOM(Node):
    def __init__(self):
        super().__init__('odom_node')
        
        # Initialise Variables and Parameters
        self.initial_parameters()
        self.initial_variables()
        
        # Initialise ROS Publisher and Subscriber
        self.odom_data_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.joint_states_pub = self.create_publisher(JointState, self.joint_state_topic, 10)
        self.tf_pub = self.create_publisher(TFMessage, '/tf', 10)
        self.create_subscription(MotorData, self.motor_data_topic, self.fetchMotorDataCallback, 10)
        self.create_subscription(IMUData, self.rpy_topic, self.fetchIMUCallback, 10)
        self.create_subscription(TwistStamped, self.robot_speed_topic, self.fetchRobotSpeedCallback, 10)
        
        # Initialise ROS Timers
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.update_callback)
        
        # ROS Logger
        self.get_logger().info(f"ODOM : ROS publisher on {self.odom_topic} [nav_msgs/Odometry]")
        self.get_logger().info(f"ODOM : ROS publisher on {self.joint_state_topic} [sensor_msgs/JointState]")
        self.get_logger().info(f"ODOM : ROS subscriber on {self.motor_data_topic} [ired_msgs/MotorData]")
        self.get_logger().info(f"ODOM : ROS subscriber on {self.rpy_topic} [ired_msgs/IMUData]")
        self.get_logger().info(f"ODOM : ROS subscriber on {self.robot_speed_topic} [geometry_msgs/TwistStamped]")

    def initial_parameters(self):
        # Declare parameters
        self.declare_parameter("odom_topic", "/odom")
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("motor_data_topic", "/ired/motor/data")
        self.declare_parameter("rpy_topic", "/ired/imu/rollpitchyaw")
        self.declare_parameter("robot_speed_topic", "/ired/speed")
        self.declare_parameter("odom_tf_publish", True)
        self.declare_parameter("rpm_to_mps", 0.003560471674)
        self.declare_parameter("wheel_radius", 0.34)
        
        # ROS Parameters
        self.odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self.joint_state_topic = self.get_parameter('joint_state_topic').get_parameter_value().string_value
        self.motor_data_topic = self.get_parameter('motor_data_topic').get_parameter_value().string_value
        self.rpy_topic = self.get_parameter('rpy_topic').get_parameter_value().string_value
        self.robot_speed_topic = self.get_parameter('robot_speed_topic').get_parameter_value().string_value
        self.odom_tf_publish = self.get_parameter('odom_tf_publish').get_parameter_value().bool_value
        self.rpm_to_mps = self.get_parameter('rpm_to_mps').get_parameter_value().double_value
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
    
    def initial_variables(self):
        pcov = [ 0.1, 0,   0,   0,   0, 0,
                0, 0.1,  0,   0,   0, 0,
                0,   0, 1e6,  0,   0, 0,
                0,   0,   0, 1e6,  0, 0,
                0,   0,   0,   0, 1e6, 0,
                0,   0,   0,   0,   0, 0.2 ]
        self.odom_data = Odometry()
        self.odom_data.header.frame_id  = "odom"
        self.odom_data.child_frame_id = "base_footprint"
        self.odom_data.pose.covariance = pcov
        self.odom_data.twist.covariance = pcov
        self.joint_states = JointState()
        self.joint_states.header.frame_id = "base_footprint"

        self.joint_states.name = [
            "wheel_front_left_joint", 
            "wheel_front_right_joint", 
            "wheel_rear_left_joint", 
            "wheel_rear_right_joint"
        ]
        self.joint_states.position = [0.0, 0.0, 0.0, 0.0]
        self.joint_states.velocity = [0.0, 0.0, 0.0, 0.0]
        self.joint_states.effort = [0.0, 0.0, 0.0, 0.0]

        self.prev_update_time = self.get_clock().now()
        self.last_time_motor_data = self.get_clock().now()
        self.last_time_imu = self.get_clock().now()
        self.last_time_robot_speed = self.get_clock().now()
        self.wheel_speed_cmd = [0.0, 0.0, 0.0, 0.0]
        self.rpy_data = IMUData()
        self.linear_x = 0.0
        self.linear_y = 0.0
        self.angular_z = 0.0
        self.odom_pose = [0.0, 0.0, 0.0]

        self.last_position = [0.0, 0.0, 0.0, 0.0]
        self.last_velocity = [0.0, 0.0, 0.0, 0.0]

        self.X, self.Y, self.Z = 0, 1, 2
        self.FLEFT, self.FRIGHT, self.RLEFT, self.RRIGHT = 0, 1, 2, 3

    def fetchMotorDataCallback(self, msg):
        self.last_time_motor_data = self.get_clock().now()

        self.wheel_speed_cmd[self.FLEFT] = msg.speed_fb[self.FLEFT]
        self.wheel_speed_cmd[self.FRIGHT] = msg.speed_fb[self.FRIGHT]
        self.wheel_speed_cmd[self.RLEFT] = msg.speed_fb[self.RLEFT]
        self.wheel_speed_cmd[self.RRIGHT] = msg.speed_fb[self.RRIGHT]

    def fetchIMUCallback(self, msg):
        self.last_time_imu = self.get_clock().now()
        self.rpy_data = msg
    
    def fetchRobotSpeedCallback(self, msg):
        self.last_time_robot_speed = self.get_clock().now()
        self.linear_x = msg.twist.linear.x
        self.linear_y = msg.twist.linear.y
        self.angular_z = msg.twist.angular.z

    def updateOdometry(self, diff_time):
        v, w = [0.0, 0.0], [0.0, 0.0]
        step_time = diff_time.nanoseconds / 1e9
        theta = self.rpy_data.yaw
        delta_x = ((self.linear_x * cos(theta)) - (self.linear_y * sin(theta))) * step_time
        delta_y = ((self.linear_x * sin(theta)) + (self.linear_y * cos(theta))) * step_time
        self.odom_pose[self.X] += delta_x
        self.odom_pose[self.Y] += delta_y
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.rpy_data.yaw)
        self.odom_data.pose.pose.position.x = self.odom_pose[self.X]
        self.odom_data.pose.pose.position.y = self.odom_pose[self.Y]
        self.odom_data.pose.pose.position.z = 0.0
        self.odom_data.pose.pose.orientation.x = qx
        self.odom_data.pose.pose.orientation.y = qy
        self.odom_data.pose.pose.orientation.z = qz
        self.odom_data.pose.pose.orientation.w = qw
        self.odom_data.twist.twist.linear.x  = self.linear_x
        self.odom_data.twist.twist.linear.y  = self.linear_y
        self.odom_data.twist.twist.angular.z = self.angular_z

        v = [0.0, 0.0, 0.0, 0.0]
        w = [0.0, 0.0, 0.0, 0.0]

        for i in range(4):
            v[i] = self.wheel_speed_cmd[i] * self.rpm_to_mps
            w[i] = v[i] / self.wheel_radius

            wheel_delta = w[i] * step_time
            if isnan(wheel_delta):
                wheel_delta = 0.0
            self.last_velocity[i] = w[i]
            self.last_position[i] += wheel_delta

    def updateJoint(self):
        self.joint_states.position = self.last_position
        self.joint_states.velocity = self.last_velocity

    def updateTF(self, odom_tf):
        odom_tf.header = self.odom_data.header
        odom_tf.child_frame_id = self.odom_data.child_frame_id
        odom_tf.transform.translation.x = self.odom_data.pose.pose.position.x
        odom_tf.transform.translation.y = self.odom_data.pose.pose.position.y
        odom_tf.transform.translation.z = self.odom_data.pose.pose.position.z
        odom_tf.transform.rotation = self.odom_data.pose.pose.orientation

    def update_callback(self):
        time_now = self.get_clock().now()
        duration = time_now - self.prev_update_time
        self.prev_update_time = time_now

        if (time_now - self.last_time_motor_data).nanoseconds / 1e9 > 1.0:
            self.wheel_speed_cmd[self.FLEFT] = 0.0
            self.wheel_speed_cmd[self.FRIGHT] = 0.0
            self.wheel_speed_cmd[self.RLEFT] = 0.0
            self.wheel_speed_cmd[self.RRIGHT] = 0.0

        if (time_now - self.last_time_imu).nanoseconds / 1e9 > 1.0:
            self.rpy_data.roll = 0.0
            self.rpy_data.pitch = 0.0
            self.rpy_data.yaw = 0.0

        if (time_now - self.last_time_robot_speed).nanoseconds / 1e9 > 1.0:
            self.linear_x = 0.0
            self.linear_y = 0.0
            self.angular_z = 0.0

        self.updateOdometry(duration)
        self.odom_data.header.stamp = time_now.to_msg()
        self.odom_data_pub.publish(self.odom_data)

        self.updateJoint()
        self.joint_states.header.stamp = time_now.to_msg()
        self.joint_states_pub.publish(self.joint_states)

        if self.odom_tf_publish:
            odom_tf = TransformStamped()
            self.updateTF(odom_tf)
            odom_tf_msg = TFMessage()
            odom_tf_msg.transforms.append(odom_tf)
            self.tf_pub.publish(odom_tf_msg)

def main(args=None):
    rclpy.init(args=args)
    odom_node = ODOM()
    
    rclpy.spin(odom_node)
    
    odom_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
