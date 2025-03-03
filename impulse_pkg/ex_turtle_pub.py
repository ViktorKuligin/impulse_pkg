#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtlePub(Node):
    def __init__(self):
        super().__init__('node_turtle_simple')
        self.get_logger().warn("turtle simple start")

        self.count = 0

        self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.tim = self.create_timer(1.0, self.timer)

    def timer(self):
        msg = Twist()
        msg.linear.x = 1.0
        msg.angular.z = 1.0
        self.pub.publish(msg)
        txt = f'linear: x = {msg.linear.x} angular: z = {msg.angular.z}'
        self.get_logger().info(txt)


def main(args=None):
    rclpy.init(args=args)
    node = TurtlePub()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ =='__main__':
    main()