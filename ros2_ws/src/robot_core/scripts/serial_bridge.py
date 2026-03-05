#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
# import serial

class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')
        self.get_logger().info('Serial Bridge Node Started')
        # Setup serial connection here

def main(args=None):
    rclpy.init(args=args)
    node = SerialBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
