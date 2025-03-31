#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Point


class converter(Node):

    def __init__(self):

        super().__init__('converter_key')
        self.get_logger().warn("converter key start")

        self.declare_parameter('velocity', 0.01)
        self.declare_parameter('screen', False)

        self.velocity = self.get_parameter('velocity').get_parameter_value().double_value
        self.screen = self.get_parameter('screen').get_parameter_value().bool_value

        self.xx = 0.0
        self.yy = 0.0

        self.sub = self.create_subscription(Twist, "/turtle1/cmd_vel", self.twist_cb, 10)
        self.pub = self.create_publisher(Point, "point_xy", 10)

        self.get_logger().info(f'velocity={self.velocity}')
        self.get_logger().info(f'screen={self.screen}')

    def twist_cb(self, msg: Twist):

        x = msg.linear.x
        y = msg.angular.z

        msg_out = Point()
        self.xx = round(self.xx + x * self.velocity, 2)
        self.yy = round(self.yy + y * self.velocity, 2)
        msg_out.x = self.xx
        msg_out.y = self.yy
        self.pub.publish(msg_out)

        if self.screen:
            self.get_logger().info(f'x={self.xx}, y={self.yy}')


def main(args=None):
    rclpy.init(args=args)
    node = converter()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()