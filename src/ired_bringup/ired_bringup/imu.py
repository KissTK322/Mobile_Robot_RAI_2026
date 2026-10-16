#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from ired_msgs.msg import IMUData
from sensor_msgs.msg import Imu
from tf_transformations import quaternion_from_euler

class IMU(Node):
    def __init__(self):
        super().__init__('imu_node')
        
        # Initialise Variables and Parameters
        self.initial_parameters()
        self.initial_variables()
        
        # Initialise ROS Publisher and Subscriber
        self.imu_data_pub = self.create_publisher(Imu, self.imu_data_topic, 10)
        self.rpy_pub = self.create_publisher(IMUData, self.rpy_topic, 10)
        self.create_subscription(Twist, self.imu_raw_topic, self.fetchIMUCallback, 10)
        
        # Initialise ROS Timers
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.update_callback)
        
        # ROS Logger
        self.get_logger().info(f"IMU : ROS publisher on {self.imu_data_topic} [sensor_msgs/Imu]")
        self.get_logger().info(f"IMU : ROS publisher on {self.rpy_topic} [ired_msgs/IMUData]")
        self.get_logger().info(f"IMU : ROS subscriber on {self.imu_raw_topic} [geometry_msgs/Twist]")

    def initial_parameters(self):
        # Declare parameters
        self.declare_parameter("imu_raw_topic", "/ired/imu/raw")
        self.declare_parameter("imu_data_topic", "/ired/imu/data")
        self.declare_parameter("rpy_topic", "/ired/imu/rollpitchyaw")
        
        # ROS Parameters
        self.imu_raw_topic = self.get_parameter('imu_raw_topic').get_parameter_value().string_value
        self.imu_data_topic = self.get_parameter('imu_data_topic').get_parameter_value().string_value
        self.rpy_topic = self.get_parameter('rpy_topic').get_parameter_value().string_value
    
    def DEG2RAD(self, degree):
        return degree * 0.01745329252
    
    def initial_variables(self):
        self.imu_data = Imu()
        self.imu_data.header.frame_id = "imu_link"
        # Covariances for the EKF: only yaw / yaw rate are trusted, roll and pitch are not measured
        self.imu_data.orientation_covariance = [1e6, 0.0, 0.0,
                                                0.0, 1e6, 0.0,
                                                0.0, 0.0, 0.01]
        self.imu_data.angular_velocity_covariance = [1e6, 0.0, 0.0,
                                                     0.0, 1e6, 0.0,
                                                     0.0, 0.0, 0.001]
        self.imu_data.linear_acceleration_covariance = [0.1, 0.0, 0.0,
                                                        0.0, 0.1, 0.0,
                                                        0.0, 0.0, 0.1]
        self.rpy_data = IMUData()
        self.rpy_data.roll = 0.0
        self.rpy_data.pitch = 0.0
        self.rpy_data.yaw = 0.0
        self.angular_velocity = [0.0, 0.0, 0.0]
        self.linear_acceleration = [0.0, 0.0, 0.0]
        self.rpy = [0.0, 0.0, 0.0]
        self.prev_update_time = self.get_clock().now()
        self.last_time_imu = self.get_clock().now()
        self.X, self.Y, self.Z = 0, 1, 2
        self.ROLL, self.PITCH, self.YAW = 0, 1, 2

    def fetchIMUCallback(self, msg):
        self.last_time_imu = self.get_clock().now()
        self.angular_velocity[self.X] = msg.angular.x
        self.angular_velocity[self.Y] = msg.angular.y
        self.angular_velocity[self.Z] = msg.angular.z
        self.linear_acceleration[self.X] = msg.linear.x
        self.linear_acceleration[self.Y] = msg.linear.y
        self.linear_acceleration[self.Z] = msg.linear.z
        
    def calculateRPY(self, diff_time):
        self.rpy_data.roll += self.DEG2RAD((self.angular_velocity[self.X] * diff_time.nanoseconds / 1e9))
        self.rpy_data.pitch += self.DEG2RAD((self.angular_velocity[self.Y] * diff_time.nanoseconds / 1e9))
        self.rpy_data.yaw += self.DEG2RAD((self.angular_velocity[self.Z] * diff_time.nanoseconds / 1e9))
    
    def calculateIMU(self):
        qx, qy, qz, qw = quaternion_from_euler(0.0, 0.0, self.rpy_data.yaw)
        self.imu_data.orientation.x = qx
        self.imu_data.orientation.y = qy
        self.imu_data.orientation.z = qz
        self.imu_data.orientation.w = qw
        self.imu_data.angular_velocity.x = self.DEG2RAD(self.angular_velocity[self.X])
        self.imu_data.angular_velocity.y = self.DEG2RAD(self.angular_velocity[self.Y])
        self.imu_data.angular_velocity.z = self.DEG2RAD(self.angular_velocity[self.Z])
        self.imu_data.linear_acceleration.x = self.linear_acceleration[self.X]
        self.imu_data.linear_acceleration.y = self.linear_acceleration[self.Y]
        self.imu_data.linear_acceleration.z = self.linear_acceleration[self.Z]

    def update_callback(self):
        time_now = self.get_clock().now()
        duration = time_now - self.prev_update_time
        self.prev_update_time = time_now

        if (time_now - self.last_time_imu).nanoseconds / 1e9 > 1.0:
            self.angular_velocity[self.X] = 0.0
            self.angular_velocity[self.Y] = 0.0
            self.angular_velocity[self.Z] = 0.0
            self.linear_acceleration[self.X] = 0.0
            self.linear_acceleration[self.Y] = 0.0
            self.linear_acceleration[self.Z] = 0.0

        self.calculateRPY(duration)
        self.rpy_pub.publish(self.rpy_data)

        self.calculateIMU()
        self.imu_data.header.stamp = time_now.to_msg()
        self.imu_data_pub.publish(self.imu_data)

def main(args=None):
    rclpy.init(args=args)
    imu_node = IMU()
    
    rclpy.spin(imu_node)
    
    imu_node.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
