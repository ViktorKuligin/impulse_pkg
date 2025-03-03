#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class Publisher(Node):
    def __init__(self):
        super().__init__('node_publisher')
        self.get_logger().warn("publisher start")

        self.count = 0

        self.pub = self.create_publisher(String, "topic_string", 10)
        self.tim = self.create_timer(1.0, self.my_timers)

    def my_timers(self):
        msg = String()
        msg.data = 'text ' + str(self.count)
        self.count += 1
        self.pub.publish(msg)
        self.get_logger().info(msg.data)

def main(args=None):
    rclpy.init(args=args)
    node = Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()