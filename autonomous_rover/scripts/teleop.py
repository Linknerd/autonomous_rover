"""
teleop.py  —  ROS 2 Node
Reads Logitech F710 (D-mode) via pygame and publishes geometry_msgs/Twist to /cmd_vel.
"""

import sys
import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ═══════════════════════════════════════════════════════════════════════════════
#  CONFIGURATION
# ═══════════════════════════════════════════════════════════════════════════════

MAX_VD = 1.0   # [m/s]   linear  — ~0.3–0.5 is typical for a small robot
MAX_WD = 2.5   # [rad/s] angular — ~1.5–2.5 is typical

VD_SLEW = 0.8   # [m/s  per second]
WD_SLEW = 4.0   # [rad/s per second]
DEADZONE     = 0.12   # Fraction of full stick travel to ignore
CONTROL_HZ   = 30     # Loop rate [Hz]

# Gamepad axis indices (Logitech F710, D-mode, pygame)
AXIS_LEFT_Y  = 1      # Forward/back
AXIS_RIGHT_X = 2      # Turn left/right
BTN_ESTOP    = 6      # Left trigger
BTN_QUIT_A   = 0      # A
BTN_QUIT_X   = 2      # X

# ═══════════════════════════════════════════════════════════════════════════════
#  HELPERS
# ═══════════════════════════════════════════════════════════════════════════════

def apply_deadzone(value: float, dz: float) -> float:
    """Rescale axis output so there is no velocity discontinuity at the edge."""
    if abs(value) < dz:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - dz) / (1.0 - dz)

def slew(current: float, target: float, rate: float, dt: float) -> float:
    """Move current toward target at most rate * dt per step."""
    delta = target - current
    max_delta = rate * dt
    if abs(delta) <= max_delta:
        return target
    return current + (max_delta if delta > 0 else -max_delta)

# ═══════════════════════════════════════════════════════════════════════════════
#  NODE
# ═══════════════════════════════════════════════════════════════════════════════

class TeleopNode(Node):
    def __init__(self):
        super().__init__('teleop_joystick')

        # Publisher for velocity commands
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No joystick found. Connect your Logitech F710 and retry.")
            sys.exit(1)

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"Controller connected: {self.joystick.get_name()}")
        self.get_logger().info(f"Publishing cmd_vel at {CONTROL_HZ} Hz. Press A/X to quit.")

        self.axes = {}
        self.e_stop = False

        # Ramp state
        self.cur_vd = 0.0
        self.cur_wd = 0.0

        # ROS 2 Timer for polling and publishing
        self.timer = self.create_timer(1.0 / CONTROL_HZ, self.timer_callback)

    def timer_callback(self):
        dt = 1.0 / CONTROL_HZ

        # Process pygame events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                self.get_logger().info("QUIT event received, ignoring...")
            elif event.type == pygame.JOYAXISMOTION:
                self.axes[event.axis] = event.value
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == BTN_ESTOP:
                    self.e_stop = True
            elif event.type == pygame.JOYBUTTONUP:
                if event.button == BTN_ESTOP:
                    self.e_stop = False

        # Compute velocities
        if self.e_stop:
            self.cur_vd = 0.0
            self.cur_wd = 0.0
        else:
            ly = apply_deadzone(self.axes.get(AXIS_LEFT_Y, 0.0), DEADZONE)
            rx = apply_deadzone(self.axes.get(AXIS_RIGHT_X, 0.0), DEADZONE)

            tgt_vd = -ly * MAX_VD
            tgt_wd = rx * MAX_WD

            self.cur_vd = slew(self.cur_vd, tgt_vd, VD_SLEW, dt)
            self.cur_wd = slew(self.cur_wd, tgt_wd, WD_SLEW, dt)

        # Publish Twist message
        msg = Twist()
        msg.linear.x = float(self.cur_vd)
        msg.angular.z = float(self.cur_wd)
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = TeleopNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Exiting manually via Ctrl+C...")
    except SystemExit:
        print("Exiting due to controller quit command...")
    except Exception as e:
        print(f"FATAL ERROR: {e}")
    finally:
        pygame.quit()
        if 'node' in locals():
            stop_msg = Twist()
            node.cmd_pub.publish(stop_msg)
            node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()

if __name__ == "__main__":
    main()
