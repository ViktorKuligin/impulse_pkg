#!/usr/bin/env python3
import rclpy
from rclpy.node import Node

class HelloROS(Node):
    def __init__(self):
        super().__init__('node_hello_ROS')
        self.get_logger().info("Hello ROS")

def main(args=None):
    rclpy.init(args=args)
    node = HelloROS()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()