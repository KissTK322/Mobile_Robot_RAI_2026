#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from ired_msgs.msg import IMUData, MotorData
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
        self.create_subscription(MotorData, self.motor_data_topic, self.fetchMotorDataCallback, 10)
        
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
        self.declare_parameter("gyro_z_scale", 1.0)
        self.declare_parameter("motor_data_topic", "/ired/motor/data")
        self.declare_parameter("still_rpm", 1.0)
        self.declare_parameter("still_time", 0.5)
        self.declare_parameter("bias_alpha", 0.05)

        # ROS Parameters
        self.imu_raw_topic = self.get_parameter('imu_raw_topic').get_parameter_value().string_value
        self.imu_data_topic = self.get_parameter('imu_data_topic').get_parameter_value().string_value
        self.rpy_topic = self.get_parameter('rpy_topic').get_parameter_value().string_value
        self.gyro_z_scale = self.get_parameter('gyro_z_scale').get_parameter_value().double_value
        self.motor_data_topic = self.get_parameter('motor_data_topic').get_parameter_value().string_value
        self.still_rpm = self.get_parameter('still_rpm').get_parameter_value().double_value
        self.still_time = self.get_parameter('still_time').get_parameter_value().double_value
        self.bias_alpha = self.get_parameter('bias_alpha').get_parameter_value().double_value
    
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
        # Gyro bias is learned whenever all wheels are stopped (zero-velocity update),
        # because it keeps changing for a while after the controller starts
        self.gyro_bias = [0.0, 0.0, 0.0]
        self.bias_ready = False
        self.last_time_moving = self.get_clock().now()
        self.last_time_motor_data = None
        self.prev_imu_time = None

    def fetchMotorDataCallback(self, msg):
        self.last_time_motor_data = self.get_clock().now()
        if any(abs(rpm) > self.still_rpm for rpm in msg.speed_fb):
            self.last_time_moving = self.last_time_motor_data

    def isStill(self, time_now):
        # Without fresh wheel data we cannot tell, so assume moving
        if self.last_time_motor_data is None or (time_now - self.last_time_motor_data).nanoseconds / 1e9 > 0.5:
            return False
        return (time_now - self.last_time_moving).nanoseconds / 1e9 > self.still_time

    def fetchIMUCallback(self, msg):
        time_now = self.get_clock().now()
        self.last_time_imu = time_now
        raw = [msg.angular.x, msg.angular.y, msg.angular.z]
        self.linear_acceleration[self.X] = msg.linear.x
        self.linear_acceleration[self.Y] = msg.linear.y
        self.linear_acceleration[self.Z] = msg.linear.z

        still = self.isStill(time_now)
        if still:
            for i in range(3):
                if self.bias_ready:
                    self.gyro_bias[i] += self.bias_alpha * (raw[i] - self.gyro_bias[i])
                else:
                    self.gyro_bias[i] = raw[i]
            if not self.bias_ready:
                self.bias_ready = True
                self.get_logger().info(f"IMU : zero-velocity bias update on, z scale {self.gyro_z_scale}")

        if still:
            # Wheels stopped: the robot cannot be turning, so do not let noise build up yaw
            self.angular_velocity = [0.0, 0.0, 0.0]
        else:
            self.angular_velocity[self.X] = raw[self.X] - self.gyro_bias[self.X]
            self.angular_velocity[self.Y] = raw[self.Y] - self.gyro_bias[self.Y]
            self.angular_velocity[self.Z] = (raw[self.Z] - self.gyro_bias[self.Z]) * self.gyro_z_scale

        # Integrate on the real time between IMU messages, not the publish timer
        if self.prev_imu_time is None:
            self.prev_imu_time = time_now
        diff_time = time_now - self.prev_imu_time
        self.prev_imu_time = time_now
        if diff_time.nanoseconds / 1e9 < 0.5:
            self.calculateRPY(diff_time)

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

        if (time_now - self.last_time_imu).nanoseconds / 1e9 > 1.0:
            self.angular_velocity[self.X] = 0.0
            self.angular_velocity[self.Y] = 0.0
            self.angular_velocity[self.Z] = 0.0
            self.linear_acceleration[self.X] = 0.0
            self.linear_acceleration[self.Y] = 0.0
            self.linear_acceleration[self.Z] = 0.0

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
