#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class RobotLogic(Node):
    def __init__(self):
        super().__init__('robot_logic')
        self.get_logger().info('Robot Logic Node Started')

def main(args=None):
    rclpy.init(args=args)
    node = RobotLogic()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
