#!/usr/bin/env python3
"""
serial_bridge.py — ROS2 Humble
Bridges the Arduino (robot_controller) to ROS2 topics.

Serial protocol (Arduino → Pi):
  I,ax,ay,az,gx,gy,gz        IMU (accel in g, gyro in deg/s)
  O,linear_m_s,angular_rad_s  Odometry velocities (computed on Arduino, sent every T ms)
  C,temp,humidity,co2         SCD30 environmental sensor
  S,d0,d1,d2                  Sharp IR distances (cm)
  E,message                   Arduino error string

Serial protocol (Pi → Arduino):
  V,vd,wd\n                   Desired linear [m/s] and angular [rad/s] velocity
  S\n                         Stop
"""

import math
import threading

import rclpy
from rclpy.node import Node

import serial

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster


class SerialBridge(Node):

    def __init__(self):
        super().__init__('serial_bridge')
        self.get_logger().info('Serial Bridge Node started')

        # ── Serial ───────────────────────────────────────────────────────────
        self.port     = '/dev/ttyACM0'
        self.baudrate = 115200
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
            self.get_logger().info(f'Connected to Arduino on {self.port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        # ── Odometry state ───────────────────────────────────────────────────
        self.x     = 0.0
        self.y     = 0.0
        self.theta = 0.0
        self.last_odom_time: float | None = None

        # ── TF broadcaster ───────────────────────────────────────────────────
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── Publishers ───────────────────────────────────────────────────────
        self.odom_pub  = self.create_publisher(Odometry,          'odom',                    10)
        self.imu_pub   = self.create_publisher(Imu,               'imu',                     10)
        self.scd30_pub = self.create_publisher(Float32MultiArray, 'scd30/data',              10)
        self.sharp_pub = self.create_publisher(Float32MultiArray, 'sharp_sensors/distances', 10)

        # ── Subscriber ───────────────────────────────────────────────────────
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # ── Serial reader thread ──────────────────────────────────────────────
        self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.read_thread.start()

    # ── cmd_vel → V,vd,wd ────────────────────────────────────────────────────

    def cmd_vel_callback(self, msg: Twist):
        """
        Forward linear and angular velocity directly to the Arduino.
        The Arduino PI controller handles converting these to motor PWM.
        """
        vd = msg.linear.x
        wd = msg.angular.z
        command = f'V,{vd:.4f},{wd:.4f}\n'
        try:
            self.ser.write(command.encode('utf-8'))
        except Exception as e:
            self.get_logger().warn(f'Serial write error: {e}')

    # ── Serial reader ─────────────────────────────────────────────────────────

    def read_serial_loop(self):
        while rclpy.ok():
            try:
                if self.ser.in_waiting > 0:
                    raw  = self.ser.readline()
                    line = raw.decode('utf-8', errors='replace').strip()
                    if line:
                        self.parse_line(line)
            except Exception as e:
                self.get_logger().error(f'Serial read error: {e}')

    # ── Line parser ───────────────────────────────────────────────────────────

    def parse_line(self, line: str):
        parts  = line.split(',')
        prefix = parts[0]

        try:
            # ── IMU ──────────────────────────────────────────────────────────
            if prefix == 'I' and len(parts) == 7:
                msg = Imu()
                msg.header.stamp    = self.get_clock().now().to_msg()
                msg.header.frame_id = 'imu_link'
                msg.linear_acceleration.x = float(parts[1]) * 9.81
                msg.linear_acceleration.y = float(parts[2]) * 9.81
                msg.linear_acceleration.z = float(parts[3]) * 9.81
                msg.angular_velocity.x    = float(parts[4]) * 0.017453
                msg.angular_velocity.y    = float(parts[5]) * 0.017453
                msg.angular_velocity.z    = float(parts[6]) * 0.017453
                msg.orientation_covariance[0]         = -1.0
                msg.linear_acceleration_covariance[0] = 0.01
                msg.linear_acceleration_covariance[4] = 0.01
                msg.linear_acceleration_covariance[8] = 0.01
                msg.angular_velocity_covariance[0]    = 0.005
                msg.angular_velocity_covariance[4]    = 0.005
                msg.angular_velocity_covariance[8]    = 0.005
                self.imu_pub.publish(msg)

            # ── Odometry ─────────────────────────────────────────────────────
            elif prefix == 'O' and len(parts) == 3:
                v = float(parts[1])
                w = float(parts[2])
                self.publish_odometry(v, w)

            # ── SCD30 ─────────────────────────────────────────────────────────
            elif prefix == 'C' and len(parts) == 4:
                msg      = Float32MultiArray()
                msg.data = [float(parts[1]), float(parts[2]), float(parts[3])]
                self.scd30_pub.publish(msg)

            # ── Sharp IR ─────────────────────────────────────────────────────
            elif prefix == 'S' and len(parts) >= 2:
                msg      = Float32MultiArray()
                msg.data = [float(p) for p in parts[1:]]
                self.sharp_pub.publish(msg)

            # ── Arduino error ─────────────────────────────────────────────────
            elif prefix == 'E':
                self.get_logger().warn(f'Arduino: {line[2:]}')

        except (ValueError, IndexError) as e:
            self.get_logger().debug(f'Parse error on line "{line}": {e}')

    # ── Odometry integration ──────────────────────────────────────────────────

    def publish_odometry(self, v: float, w: float):
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        if self.last_odom_time is None:
            self.last_odom_time = now_sec
            return

        dt = now_sec - self.last_odom_time
        self.last_odom_time = now_sec

        if dt <= 0.0 or dt > 5.0:
            return

        self.x     += v * math.cos(self.theta) * dt
        self.y     += v * math.sin(self.theta) * dt
        self.theta += w * dt

        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        stamp = self.get_clock().now().to_msg()

        # TF: odom → base_footprint
        tf = TransformStamped()
        tf.header.stamp            = stamp
        tf.header.frame_id         = 'odom'
        tf.child_frame_id          = 'base_footprint'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.z    = qz
        tf.transform.rotation.w    = qw
        self.tf_broadcaster.sendTransform(tf)

        # /odom topic
        odom = Odometry()
        odom.header.stamp            = stamp
        odom.header.frame_id         = 'odom'
        odom.child_frame_id          = 'base_footprint'
        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x    = v
        odom.twist.twist.angular.z   = w

        pc = [0.0] * 36
        pc[0]  = 0.05
        pc[7]  = 0.05
        pc[35] = 0.1
        odom.pose.covariance = pc

        tc = [0.0] * 36
        tc[0]  = 0.01
        tc[35] = 0.05
        odom.twist.covariance = tc

        self.odom_pub.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
