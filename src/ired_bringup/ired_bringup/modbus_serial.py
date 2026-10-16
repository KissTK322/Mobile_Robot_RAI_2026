#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from ired_msgs.msg import MotorData, MotorSpeed
from ired_msgs.srv import PIDTuning
from std_srvs.srv import Empty
from std_msgs.msg import String

import serial
import struct
import crcmod

class IREDController(Node):
    def __init__(self):
        super().__init__('iredcr_modbus_node')
        
        # Initialise Variables and Parameters
        self.initial_parameters()
        self.initial_variables()
        
        # Initialise ROS Publisher and Subscriber
        self.imu_raw_pub = self.create_publisher(Twist, self.imu_raw_topic, 10)
        self.motor_data_pub = self.create_publisher(MotorData, self.motor_data_topic, 10)
        self.create_subscription(MotorSpeed, self.motor_control_topic, self.commandMotorSpeedCallback, 10)
        
        # Initialise ROS Service server
        self.create_service(Empty, '/ired/imu/reset', self.commandIMUResetCallback)
        self.create_service(Empty, '/ired/pid/reset', self.commandPIDResetCallback)
        self.create_service(PIDTuning, '/ired/pid/tuning', self.commandPIDTuningCallback)
        
        # Initialise ROS Timers
        timer_period = 0.1 # seconds
        self.timer = self.create_timer(timer_period, self.update_callback)
        
        # ROS Logger
        self.get_logger().info(f"iRED Controller : Setup publisher on {self.imu_raw_topic} [geometry_msgs/Twist]")
        self.get_logger().info(f"iRED Controller : Setup publisher on {self.motor_data_topic} [ired_msgs/MotorData]")
        self.get_logger().info(f"iRED Controller : Setup subscriber on {self.motor_control_topic} [ired_msgs/MotorSpeed]")
        self.get_logger().info(f"iRED Controller : Setup service server on /ired/imu/reset [std_srvs/Empty]")
        self.get_logger().info(f"iRED Controller : Setup service server on /ired/pid/reset [std_srvs/Empty]")
        self.get_logger().info(f"iRED Controller : Setup service server on /ired/pid/tuning [ired_msgs/PIDTuning]")
        
        # Serial Modbus
        self.initial_modbus()
    
    def initial_parameters(self):
        # Declare parameters
        self.declare_parameter('imu_raw_topic', '/ired/imu/raw')
        self.declare_parameter('motor_control_topic', '/ired/motor/speed_control')
        self.declare_parameter('motor_data_topic', '/ired/motor/data')
        self.declare_parameter('port', '/dev/ttyMicro')
        self.declare_parameter('baud', 115200)
        # Wheel PID gains [kp, ki, kd] written to the controller at startup; all zeros = keep the controller's own
        self.declare_parameter('pid_fl', [0.0, 0.0, 0.0])
        self.declare_parameter('pid_fr', [0.0, 0.0, 0.0])
        self.declare_parameter('pid_rl', [0.0, 0.0, 0.0])
        self.declare_parameter('pid_rr', [0.0, 0.0, 0.0])
        
        # ROS Parameters
        self.imu_raw_topic = self.get_parameter('imu_raw_topic').get_parameter_value().string_value
        self.motor_control_topic = self.get_parameter('motor_control_topic').get_parameter_value().string_value
        self.motor_data_topic = self.get_parameter('motor_data_topic').get_parameter_value().string_value
        self.serial_port = self.get_parameter('port').get_parameter_value().string_value
        self.serial_baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.pid_startup = {
            "FL": list(self.get_parameter('pid_fl').get_parameter_value().double_array_value),
            "FR": list(self.get_parameter('pid_fr').get_parameter_value().double_array_value),
            "RL": list(self.get_parameter('pid_rl').get_parameter_value().double_array_value),
            "RR": list(self.get_parameter('pid_rr').get_parameter_value().double_array_value),
        }
    
    def initial_variables(self):
        self.imu_raw_msg = Twist()
        self.motor_data_msg = MotorData()
        self.pid_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        self.last_time_cmd_vel = self.get_clock().now()
        self.imu_reset_check = False
        self.pid_reset_check = False
        self.pid_update_check = False
        self.pid_tuning_check = False
        self.pid_tuning_motor = ""
        self.pid_tuning_value = [0.0, 0.0, 0.0]
        # Startup PID gains from parameters: sent one wheel per cycle once the controller
        # has finished its IMU calibration, then checked against the read-back values
        self.start_time = self.get_clock().now()
        self.pid_startup_queue = [m for m, g in self.pid_startup.items() if len(g) == 3 and any(g)]
        self.pid_startup_tries = {m: 0 for m in self.pid_startup_queue}
    
    def commandIMUResetCallback(self, request, response):
        self.get_logger().info("iRED Controller : Start Calibration of MP9250...")
        self.imu_reset_check = True

        return response

    def commandPIDResetCallback(self, request, response):
        self.get_logger().info("iRED Controller : Reset PID Data...")
        self.pid_reset_check = True

        return response

    def commandPIDTuningCallback(self, request, response):
        if request.motor in ["FL", "FR", "RL", "RR"]:
            self.get_logger().info(f"iRED Controller : PID Tuning motor {request.motor}")
            self.pid_tuning_check = True
            self.pid_tuning_motor = request.motor
            self.pid_tuning_value = [request.kp, request.ki, request.kd]
            response.status = True
        else:
            self.get_logger().warn("Wrong Motor: FL-FrontLeft FR-FrontRight RL-RearLeft RR-RearRight")
            response.status = False
        return response
    
    def commandMotorSpeedCallback(self, msg):
        self.last_time_cmd_vel = self.get_clock().now()
        self.motor_data_msg.speed_sp = list(msg.speed)
    
    def initial_modbus(self):
        self.get_logger().info(f"iRED Controller : Connect to port {self.serial_port} on baudrate {self.serial_baud}")
        
        # Serial Parameter
        self.crc16 = crcmod.predefined.mkCrcFun('modbus')
        self.serial_communication = serial.Serial(port=self.serial_port, baudrate=self.serial_baud, bytesize=8, parity='N', stopbits=1, timeout=1)
        self.modbus_slave_id = 0x01
        self.modbus_function_read_holdingregs = 0x03
        self.modbus_function_read_inputregs = 0x04
        self.modbus_function_write_multiregs = 0x10
        
        # Start initialise data
        self.get_logger().info("iRED Controller : Start Calibration of MP9250...")
        self.modbusSendData([1], 0x0022, 1)
        self.get_logger().info("iRED Controller : Calibration End")
        
        if not self.modbusReadData(self.modbus_function_read_holdingregs, 0x0008, 12):
            self.get_logger().warn("Cannot read data from iRED Controller...")
        else:
            self.setupPIDData()

    def modbusBuildSendData(self, datas, start_addr, write_data):
        modbus_start_addr = start_addr
        modbus_write_data = write_data
        modbus_write_regs = modbus_write_data * 2
        frame = bytearray()
        frame.append(self.modbus_slave_id)
        frame.append(self.modbus_function_write_multiregs)
        frame += modbus_start_addr.to_bytes(2, 'big')
        frame += modbus_write_regs.to_bytes(2, 'big')
        frame.append(modbus_write_regs * 2)
        for val in datas:
            frame += struct.pack('>f', val)
        crc = self.crc16(frame)
        frame += crc.to_bytes(2, byteorder='little')
        
        return frame

    def modbusBuildReadData(self, function, start_addr, read_data):
        modbus_start_addr = start_addr
        modbus_read_data = read_data
        modbus_read_regs_ = modbus_read_data * 2
        frame = bytearray()
        frame.append(self.modbus_slave_id)
        frame.append(function)
        frame += modbus_start_addr.to_bytes(2, 'big')
        frame += modbus_read_regs_.to_bytes(2, 'big')
        crc = self.crc16(frame)
        frame += crc.to_bytes(2, byteorder='little')

        return frame

    def modbusSendData(self, datas, start_addr, write_data):
        write_frame = self.modbusBuildSendData(datas, start_addr, write_data)
        self.serial_communication.write(write_frame)

        write_resp = self.serial_communication.read(8)
        if len(write_resp) == 8:
            return True
        else:
            self.get_logger().warn("Write: No or incomplete response")
            return False
    
    def modbusReadData(self, function, start_addr, read_data):
        read_frame = self.modbusBuildReadData(function, start_addr, read_data)
        self.serial_communication.write(read_frame)
        expected_bytes = 3 + (read_data * 2) * 2 + 2
        read_resp = self.serial_communication.read(expected_bytes)

        if len(read_resp) == expected_bytes:
            if read_resp[-2:] != self.crc16(read_resp[:-2]).to_bytes(2, 'little'):
                self.get_logger().warn("Read: CRC mismatch")
                return False
            else:
                data_bytes = read_resp[3:-2]
                self.modbus_read_data = []
                for i in range(0, len(data_bytes), 4):
                    chunk = data_bytes[i:i+4]
                    val = struct.unpack('>f', chunk)[0]
                    self.modbus_read_data.append(val)
                return True
        else:
            self.get_logger().warn("Read: No or incomplete respons")
            return False
    
    def setupPIDData(self):
        self.motor_data_msg.pid_motor_front_left[0] = self.modbus_read_data[0]
        self.motor_data_msg.pid_motor_front_left[1] = self.modbus_read_data[1]
        self.motor_data_msg.pid_motor_front_left[2] = self.modbus_read_data[2]
        self.motor_data_msg.pid_motor_front_right[0] = self.modbus_read_data[3]
        self.motor_data_msg.pid_motor_front_right[1] = self.modbus_read_data[4]
        self.motor_data_msg.pid_motor_front_right[2] = self.modbus_read_data[5]
        self.motor_data_msg.pid_motor_rear_left[0] = self.modbus_read_data[6]
        self.motor_data_msg.pid_motor_rear_left[1] = self.modbus_read_data[7]
        self.motor_data_msg.pid_motor_rear_left[2] = self.modbus_read_data[8]
        self.motor_data_msg.pid_motor_rear_right[0] = self.modbus_read_data[9]
        self.motor_data_msg.pid_motor_rear_right[1] = self.modbus_read_data[10]
        self.motor_data_msg.pid_motor_rear_right[2] = self.modbus_read_data[11]
        
    def readInputRegs(self, received_data):
        if received_data:
            self.motor_data_msg.speed_fb[0] = self.modbus_read_data[0]
            self.motor_data_msg.speed_fb[1] = self.modbus_read_data[1]
            self.motor_data_msg.speed_fb[2] = self.modbus_read_data[2]
            self.motor_data_msg.speed_fb[3] = self.modbus_read_data[3]
            self.imu_raw_msg.linear.x = self.modbus_read_data[4]
            self.imu_raw_msg.linear.y = self.modbus_read_data[5]
            self.imu_raw_msg.linear.z = self.modbus_read_data[6]
            self.imu_raw_msg.angular.x = self.modbus_read_data[7]
            self.imu_raw_msg.angular.y = self.modbus_read_data[8]
            self.imu_raw_msg.angular.z = self.modbus_read_data[9]
        else:
            self.motor_data_msg.speed_fb[0] = 0.0
            self.motor_data_msg.speed_fb[1] = 0.0
            self.motor_data_msg.speed_fb[2] = 0.0
            self.motor_data_msg.speed_fb[3] = 0.0
            self.imu_raw_msg.linear.x = 0.0
            self.imu_raw_msg.linear.y = 0.0
            self.imu_raw_msg.linear.z = 0.0
            self.imu_raw_msg.angular.x = 0.0
            self.imu_raw_msg.angular.y = 0.0
            self.imu_raw_msg.angular.z = 0.0

    def PIDTuningModbus(self):
        if self.pid_tuning_motor == "FL":
            pid_starting_addr = 0x0008
            self.motor_data_msg.pid_motor_front_left[0] = self.pid_tuning_value[0]
            self.motor_data_msg.pid_motor_front_left[1] = self.pid_tuning_value[1]
            self.motor_data_msg.pid_motor_front_left[2] = self.pid_tuning_value[2]
        elif self.pid_tuning_motor == "FR":
            pid_starting_addr = 0x000E
            self.motor_data_msg.pid_motor_front_right[0] = self.pid_tuning_value[0]
            self.motor_data_msg.pid_motor_front_right[1] = self.pid_tuning_value[1]
            self.motor_data_msg.pid_motor_front_right[2] = self.pid_tuning_value[2]
        elif self.pid_tuning_motor == "RL":
            pid_starting_addr = 0x0014
            self.motor_data_msg.pid_motor_rear_left[0] = self.pid_tuning_value[0]
            self.motor_data_msg.pid_motor_rear_left[1] = self.pid_tuning_value[1]
            self.motor_data_msg.pid_motor_rear_left[2] = self.pid_tuning_value[2]
        else:
            pid_starting_addr = 0x001A
            self.motor_data_msg.pid_motor_rear_right[0] = self.pid_tuning_value[0]
            self.motor_data_msg.pid_motor_rear_right[1] = self.pid_tuning_value[1]
            self.motor_data_msg.pid_motor_rear_right[2] = self.pid_tuning_value[2]

        return self.modbusSendData(self.pid_tuning_value, pid_starting_addr, 3)

    def startupPIDStep(self, time_now):
        if not self.pid_startup_queue or self.pid_tuning_check or not self.pid_update_check:
            return
        if (time_now - self.start_time).nanoseconds / 1e9 < 3.0:
            return
        current = {"FL": self.motor_data_msg.pid_motor_front_left, "FR": self.motor_data_msg.pid_motor_front_right,
                   "RL": self.motor_data_msg.pid_motor_rear_left, "RR": self.motor_data_msg.pid_motor_rear_right}
        motor = self.pid_startup_queue[0]
        gains = self.pid_startup[motor]
        if all(abs(a - b) < 1e-3 for a, b in zip(current[motor], gains)):
            # Already right (each write stalls the controller ~1 s, so skip it when possible)
            self.pid_startup_queue.pop(0)
            state = "set from parameters" if self.pid_startup_tries[motor] else "already matches parameters"
            self.get_logger().info(f"iRED Controller : PID {motor} {state} Kp: {gains[0]} Ki: {gains[1]} Kd: {gains[2]}")
        elif self.pid_startup_tries[motor] >= 3:
            self.pid_startup_queue.pop(0)
            self.get_logger().warn(f"iRED Controller : Cannot set PID {motor} from parameters")
        else:
            self.pid_startup_tries[motor] += 1
            self.pid_tuning_motor = motor
            self.pid_tuning_value = list(gains)
            self.pid_tuning_check = True

    def update_callback(self):
        time_now = self.get_clock().now()
        self.startupPIDStep(time_now)
        
        # Zero if time out
        if (time_now - self.last_time_cmd_vel).nanoseconds / 1e9 > 1.0:
            self.motor_data_msg.speed_sp = [0.0] * 4

        if not self.modbusSendData(self.motor_data_msg.speed_sp, 0x0000, 4):
            self.get_logger().warn("Cannot send data to iRED Controller...")
        
        if not self.modbusReadData(self.modbus_function_read_inputregs, 0x0000, 10) or self.modbus_read_data is None:
            self.readInputRegs(False)
            self.get_logger().warn("Cannot read data from iRED Controller...")
        else:
            self.readInputRegs(True)

        self.imu_raw_pub.publish(self.imu_raw_msg)
        self.motor_data_pub.publish(self.motor_data_msg)
        
        if self.pid_reset_check:
            self.modbusSendData([1], 0x0020, 1)
            self.pid_reset_check = False
            self.pid_update_check = False
            self.get_logger().info("iRED Controller : Completed Reset PID Data.")
        
        if not self.pid_update_check:
            if not self.modbusReadData(self.modbus_function_read_holdingregs, 0x0008, 12):
                self.get_logger().info("iRED Controller : Cannot update new PID Dta...")
            else:
                self.setupPIDData()
                self.get_logger().warn("iRED Controller : New PID data updated.")
                self.pid_update_check = True
            
        if self.imu_reset_check:
            self.modbusSendData([1], 0x0022, 1)
            self.imu_reset_check = False
            self.get_logger().info("iRED Controller : Calibration End")

        if self.pid_tuning_check:
            self.PIDTuningModbus()
            self.pid_tuning_check = False
            self.pid_update_check = False  # read the gains back from the controller next cycle
            self.get_logger().info(
                f"PID {self.pid_tuning_motor} Kp: {self.pid_tuning_value[0]} Ki: {self.pid_tuning_value[1]} Kd: {self.pid_tuning_value[2]}"
            )
        
def main(args=None):
    rclpy.init(args=args)
    ired_controller_ = IREDController()
    
    rclpy.spin(ired_controller_)
    
    ired_controller_.destroy_node()
    rclpy.shutdown()
    
if __name__ == '__main__':
    main()
