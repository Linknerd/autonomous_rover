#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import threading
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, Range
from std_msgs.msg import Float32MultiArray

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.get_logger().info('Serial Bridge Node Started')
        
        # Serial connection setup (adjust port as needed for Raspberry Pi, usually /dev/ttyUSB0 or /dev/ttyACM0)
        self.port = '/dev/ttyACM0'
        self.baudrate = 9600
        
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
            self.get_logger().info(f'Connected to Arduino on {self.port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to Arduino: {e}')
            return

        # Publishers
        self.imu_pub = self.create_publisher(Imu, 'imu/data_raw', 10)
        self.scd30_pub = self.create_publisher(Float32MultiArray, 'scd30/data', 10) 
        self.sharp_pub = self.create_publisher(Float32MultiArray, 'sharp_sensors/distances', 10)
        self.odom_pub = self.create_publisher(Twist, 'odom/velocity', 10) # Simple Twist representation for now

        # Subscriber for movement commands
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10)

        # Start a thread to read from serial continuously
        self.read_thread = threading.Thread(target=self.read_serial_loop)
        self.read_thread.daemon = True
        self.read_thread.start()

    def cmd_vel_callback(self, msg):
        # Convert cmd_vel Twist to differential drive PWMs
        # This is a very basic conversion and requires tuning for your specific motors
        linear = msg.linear.x
        angular = msg.angular.z
        
        # Basic diff drive kinematics (dummy scaling factors applied)
        left_speed = (linear - angular) * 200 
        right_speed = (linear + angular) * 200
        
        # Clamp to PWM limits [-255, 255]
        left_pwm = max(min(int(left_speed), 255), -255)
        right_pwm = max(min(int(right_speed), 255), -255)
        
        command = f"M,{left_pwm},{right_pwm}\n"
        try:
            self.ser.write(command.encode('utf-8'))
        except Exception as e:
            self.get_logger().warn(f'Failed to write to serial: {e}')

    def read_serial_loop(self):
        while rclpy.ok():
            try:
                if self.ser.in_waiting > 0:
                    line = self.ser.readline().decode('utf-8').strip()
                    self.parse_line(line)
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')

    def parse_line(self, line):
        if not line:
            return
            
        parts = line.split(',')
        prefix = parts[0]
        
        try:
            if prefix == 'I' and len(parts) == 7: # IMU
                imu_msg = Imu()
                imu_msg.header.stamp = self.get_clock().now().to_msg()
                imu_msg.header.frame_id = 'imu_link'
                imu_msg.linear_acceleration.x = float(parts[1]) * 9.81 # convert g to m/s^2
                imu_msg.linear_acceleration.y = float(parts[2]) * 9.81
                imu_msg.linear_acceleration.z = float(parts[3]) * 9.81
                imu_msg.angular_velocity.x = float(parts[4]) * 0.0174533 # convert deg/s to rad/s
                imu_msg.angular_velocity.y = float(parts[5]) * 0.0174533
                imu_msg.angular_velocity.z = float(parts[6]) * 0.0174533
                self.imu_pub.publish(imu_msg)
                
            elif prefix == 'C' and len(parts) == 4: # SCD30
                scd_msg = Float32MultiArray()
                # Contains: [Temp(C), Humidity(%), CO2(ppm)]
                scd_msg.data = [float(parts[1]), float(parts[2]), float(parts[3])]
                self.scd30_pub.publish(scd_msg)
                
            elif prefix == 'S' and len(parts) >= 2: # Sharp Sensors
                sharp_msg = Float32MultiArray()
                # Contains array of distances in cm
                sharp_msg.data = [float(p) for p in parts[1:]]
                self.sharp_pub.publish(sharp_msg)
                
            elif prefix == 'O' and len(parts) == 3: # Odometry (Speed, Rotation)
                odom_msg = Twist()
                odom_msg.linear.x = float(parts[1])
                odom_msg.angular.z = float(parts[2])
                self.odom_pub.publish(odom_msg)
                
            elif prefix == 'E':
                self.get_logger().warn(f'Arduino Error: {line[2:]}')
        except ValueError as e:
            self.get_logger().debug(f'Failed to parse float from line: {line}')

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
