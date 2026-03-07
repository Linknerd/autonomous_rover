#!/usr/bin/env python3
"""
serial_bridge.py — ROS2 Humble
Bridges the Arduino (robot_controller) to ROS2 topics.

Serial protocol (Arduino → Pi):
  I,ax,ay,az,gx,gy,gz        IMU (accel in g, gyro in deg/s)
  O,linear_m_s,angular_rad_s  Odometry velocities (computed on Arduino, sent every 1 s)
  C,temp,humidity,co2         SCD30 environmental sensor
  S,d0,d1,d2                  Sharp IR distances (cm)
  E,message                   Arduino error string

Serial protocol (Pi → Arduino):
  M,left_pwm,right_pwm\n      Motor command  (-255 to 255)
  S\n                          Stop
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.time import Time

import serial

from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray
from tf2_ros import TransformBroadcaster


# ── Robot physical constants (must match sensors.cpp) ────────────────────────
WHEEL_RADIUS   = 0.0625   # RHO  [m]
WHEEL_BASE     = 0.2775   # ELL  [m]  (centre-to-centre wheel separation)

# ── PWM scaling ───────────────────────────────────────────────────────────────
# Tune MAX_SPEED_MS to the free-running wheel speed at PWM=255.
# Measure: put rover on blocks, send M,255,255, measure actual m/s from /odom.
MAX_SPEED_MS   = 0.5      # [m/s] at full PWM — TUNE THIS
MAX_PWM        = 255


def speed_to_pwm(speed_ms: float) -> int:
    """Convert a desired wheel speed [m/s] to a PWM value [-255, 255]."""
    pwm = int((speed_ms / MAX_SPEED_MS) * MAX_PWM)
    return max(-MAX_PWM, min(MAX_PWM, pwm))


class SerialBridge(Node):

    def __init__(self):
        super().__init__('serial_bridge')
        self.get_logger().info('Serial Bridge Node started')

        # ── Serial ───────────────────────────────────────────────────────────
        self.port     = '/dev/ttyACM0'
        self.baudrate = 9600
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=1.0)
            self.get_logger().info(f'Connected to Arduino on {self.port}')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise

        # ── Odometry state ───────────────────────────────────────────────────
        self.x     = 0.0   # [m]
        self.y     = 0.0   # [m]
        self.theta = 0.0   # [rad]
        self.last_odom_time: float | None = None   # seconds (clock)

        # ── TF broadcaster ───────────────────────────────────────────────────
        self.tf_broadcaster = TransformBroadcaster(self)

        # ── Publishers ───────────────────────────────────────────────────────
        self.odom_pub   = self.create_publisher(Odometry,          'odom',                    10)
        self.imu_pub    = self.create_publisher(Imu,               'imu',                     10)
        self.scd30_pub  = self.create_publisher(Float32MultiArray, 'scd30/data',              10)
        self.sharp_pub  = self.create_publisher(Float32MultiArray, 'sharp_sensors/distances', 10)

        # ── Subscriber ───────────────────────────────────────────────────────
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        # ── Serial reader thread ──────────────────────────────────────────────
        self.read_thread = threading.Thread(target=self.read_serial_loop, daemon=True)
        self.read_thread.start()

    # ── cmd_vel → PWM ────────────────────────────────────────────────────────

    def cmd_vel_callback(self, msg: Twist):
        """
        Convert a Twist (linear.x, angular.z) to differential drive PWMs.

        v  = (v_R + v_L) / 2
        ω  = (v_R - v_L) / ELL

        → v_L = v - ω * ELL/2
          v_R = v + ω * ELL/2
        """
        v   = msg.linear.x
        w   = msg.angular.z
        v_L = v - w * (WHEEL_BASE / 2.0)
        v_R = v + w * (WHEEL_BASE / 2.0)

        left_pwm  = speed_to_pwm(v_L)
        right_pwm = speed_to_pwm(v_R)

        command = f'M,{left_pwm},{right_pwm}\n'
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

                # Arduino already applies bias offsets before sending
                msg.linear_acceleration.x = float(parts[1]) * 9.81   # g → m/s²
                msg.linear_acceleration.y = float(parts[2]) * 9.81
                msg.linear_acceleration.z = float(parts[3]) * 9.81
                msg.angular_velocity.x    = float(parts[4]) * 0.017453  # deg/s → rad/s
                msg.angular_velocity.y    = float(parts[5]) * 0.017453
                msg.angular_velocity.z    = float(parts[6]) * 0.017453

                # Covariance: -1 means "unknown orientation" (we don't have a magnetometer)
                msg.orientation_covariance[0] = -1.0

                # Diagonal covariance for accel and gyro (tune if needed)
                msg.linear_acceleration_covariance[0] = 0.01
                msg.linear_acceleration_covariance[4] = 0.01
                msg.linear_acceleration_covariance[8] = 0.01
                msg.angular_velocity_covariance[0]    = 0.005
                msg.angular_velocity_covariance[4]    = 0.005
                msg.angular_velocity_covariance[8]    = 0.005

                self.imu_pub.publish(msg)

            # ── Odometry ─────────────────────────────────────────────────────
            elif prefix == 'O' and len(parts) == 3:
                v = float(parts[1])   # linear  [m/s]
                w = float(parts[2])   # angular [rad/s]
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
        """
        Integrate v and w into a 2-D pose and publish nav_msgs/Odometry
        + the odom → base_footprint TF transform.

        The Arduino sends velocities every T=1000 ms, so dt ≈ 1.0 s.
        We use the actual wall-clock delta so timing jitter is handled.
        """
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        if self.last_odom_time is None:
            self.last_odom_time = now_sec
            return

        dt = now_sec - self.last_odom_time
        self.last_odom_time = now_sec

        # Guard against absurd dt values (e.g. first tick after long pause)
        if dt <= 0.0 or dt > 5.0:
            return

        # Integrate pose (Euler method — good enough at 1 Hz for a slow rover)
        delta_x     =  v * math.cos(self.theta) * dt
        delta_y     =  v * math.sin(self.theta) * dt
        delta_theta =  w * dt

        self.x     += delta_x
        self.y     += delta_y
        self.theta += delta_theta

        # Build quaternion from yaw
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        stamp = self.get_clock().now().to_msg()

        # ── Publish odom → base_footprint TF ─────────────────────────────
        tf = TransformStamped()
        tf.header.stamp            = stamp
        tf.header.frame_id         = 'odom'
        tf.child_frame_id          = 'base_footprint'
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.translation.z = 0.0
        tf.transform.rotation.x    = 0.0
        tf.transform.rotation.y    = 0.0
        tf.transform.rotation.z    = qz
        tf.transform.rotation.w    = qw
        self.tf_broadcaster.sendTransform(tf)

        # ── Publish /odom ─────────────────────────────────────────────────
        odom = Odometry()
        odom.header.stamp            = stamp
        odom.header.frame_id         = 'odom'
        odom.child_frame_id          = 'base_footprint'

        odom.pose.pose.position.x    = self.x
        odom.pose.pose.position.y    = self.y
        odom.pose.pose.position.z    = 0.0
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        odom.twist.twist.linear.x    = v
        odom.twist.twist.angular.z   = w

        # Pose covariance (diagonal): higher uncertainty because of 1 Hz update
        pc = [0.0] * 36
        pc[0]  = 0.05   # x
        pc[7]  = 0.05   # y
        pc[35] = 0.1    # yaw
        odom.pose.covariance = pc

        # Twist covariance
        tc = [0.0] * 36
        tc[0]  = 0.01   # vx
        tc[35] = 0.05   # vyaw
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
