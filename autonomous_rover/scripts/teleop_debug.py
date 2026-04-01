#!/usr/bin/env python3
"""
teleop_debug.py  —  ROS 2 Node (Strict Hardware Governor)
Safely caps output velocity at 0.4 m/s to prevent Arduino interrupt
starvation when using ultra-high 12,000 TPR decoding.
"""

import sys
import pygame
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# ═══════════════════════════════════════════════════════════════════════════════
#  CONFIGURATION (12,000 TPR Safety Specs)
# ═══════════════════════════════════════════════════════════════════════════════

MAX_VD = 0.40   # [m/s] CAPPED. Produces max ~12,000 interrupts/sec. CPU SAFE.
MAX_WD = 1.00   # [rad/s] CAPPED.

VD_SLEW    = 0.6    # [m/s ^ 2]
WD_SLEW    = 3.0    # [rad/s ^ 2]
DEADZONE   = 0.15   # High threshold to completely ignore stick drift
CONTROL_HZ = 30     # Loop rate [Hz]

# D-Mode Button Map
AXIS_TURN     = 0   # Let's use Left-Stick Horizontal for turning.
AXIS_DRIVE    = 1   # Left-Stick Vertical for normal driving.
BTN_BRAKE     = 6   # Left Trigger (L2) - Kills momentum
BTN_FORWARD   = 7   # Right Trigger (R2) - Locks to max forward
BTN_QUIT      = 2   # 'X' Button

# ═══════════════════════════════════════════════════════════════════════════════

def apply_deadzone(value: float, dz: float) -> float:
    if abs(value) < dz:
        return 0.0
    sign = 1.0 if value > 0 else -1.0
    return sign * (abs(value) - dz) / (1.0 - dz)

def slew(current: float, target: float, rate: float, dt: float) -> float:
    delta = target - current
    max_delta = rate * dt
    if abs(delta) <= max_delta:
        return target
    return current + (max_delta if delta > 0 else -max_delta)

# ═══════════════════════════════════════════════════════════════════════════════

class TeleopDebugNode(Node):
    def __init__(self):
        super().__init__('teleop_debug_12k')
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        pygame.init()
        pygame.joystick.init()

        if pygame.joystick.get_count() == 0:
            self.get_logger().error("No joystick found. Check USB receiver.")
            sys.exit(1)

        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        self.get_logger().info(f"Connected: {self.joystick.get_name()}")
        self.get_logger().info("CPU SAFE GOVERNOR ACTIVE (Max V: 0.40 m/s)")
        
        self.axes = {}
        self.btn_brake = False
        self.btn_fwd = False

        self.cur_vd = 0.0
        self.cur_wd = 0.0

        self.timer = self.create_timer(1.0 / CONTROL_HZ, self.timer_callback)

    def timer_callback(self):
        dt = 1.0 / CONTROL_HZ

        for event in pygame.event.get():
            if event.type == pygame.JOYAXISMOTION:
                self.axes[event.axis] = event.value
            elif event.type == pygame.JOYBUTTONDOWN:
                if event.button == BTN_BRAKE: self.btn_brake = True
                if event.button == BTN_FORWARD: self.btn_fwd = True
                if event.button == BTN_QUIT: 
                    self.get_logger().warn("Quit requested.")
                    sys.exit(0)
            elif event.type == pygame.JOYBUTTONUP:
                if event.button == BTN_BRAKE: self.btn_brake = False
                if event.button == BTN_FORWARD: self.btn_fwd = False

        tgt_vd = 0.0
        tgt_wd = 0.0

        if self.btn_brake:
            # Active emergency braking
            tgt_vd = 0.0
            tgt_wd = 0.0
            self.cur_vd = 0.0
            self.cur_wd = 0.0
        else:
            # Turning
            rx = apply_deadzone(self.axes.get(AXIS_TURN, 0.0), DEADZONE)
            
            # Driving (Stick takes priority if not zero, else trigger is checked)
            ly = apply_deadzone(self.axes.get(AXIS_DRIVE, 0.0), DEADZONE)
            
            if abs(ly) > 0.01:
                tgt_vd = -ly * MAX_VD  # Invert Y stick so UP is Forward
            elif self.btn_fwd:
                tgt_vd = MAX_VD

            tgt_wd = -rx * MAX_WD  # Often inverted to make Left-push turn left (CCW) 

            # Smoothly ramp speeds
            self.cur_vd = slew(self.cur_vd, tgt_vd, VD_SLEW, dt)
            self.cur_wd = slew(self.cur_wd, tgt_wd, WD_SLEW, dt)

        # Force mathematically perfect zero to allow Arduino to reset integrators
        if abs(self.cur_vd) < 0.005: self.cur_vd = 0.0
        if abs(self.cur_wd) < 0.005: self.cur_wd = 0.0

        # PRINT TO CONSOLE SO YOU CAN VERIFY YOUR SPRING DRIFT
        print(f"\rCmd: V={self.cur_vd:+.3f}  W={self.cur_wd:+.3f}   [Brake: {self.btn_brake}]", end="")

        msg = Twist()
        msg.linear.x = float(self.cur_vd)
        msg.angular.z = float(self.cur_wd)
        self.cmd_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    try:
        node = TeleopDebugNode()
        rclpy.spin(node)
    except SystemExit:
        pass
    except KeyboardInterrupt:
        pass
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
