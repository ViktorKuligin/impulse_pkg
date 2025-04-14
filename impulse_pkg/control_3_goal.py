#!/usr/bin/env python3
import rclpy

from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist, Point, Vector3


class Main_node(Node):

    def __init__(self):

        super().__init__('main_node')
        self.get_logger().warn("main node start")

        self.declare_parameter('velocity', 0.01)
        self.declare_parameter('screen', True)

        self.velocity = self.get_parameter('velocity').get_parameter_value().double_value
        self.screen = self.get_parameter('screen').get_parameter_value().bool_value

        self.xx = 0.0
        self.yy = 0.0

        self.sub = self.create_subscription(Vector3, 'direction', self.twist_cb, 10)
        self.pub = self.create_publisher(Point, "point_obj", 10)

        self.get_logger().info(f'velocity={self.velocity}')
        self.get_logger().info(f'screen={self.screen}')

    def twist_cb(self, msg: Vector3):

        x = msg.x
        y = msg.y

        msg_point = Point()
        self.xx = round(self.xx + x * self.velocity, 2)
        self.yy = round(self.yy + y * self.velocity, 2)
        msg_point.x = self.xx
        msg_point.y = self.yy
        self.pub.publish(msg_point)

        if self.screen:
            self.get_logger().info(f'x={self.xx}, y={self.yy}')


def main(args=None):
    rclpy.init(args=args)
    node = Main_node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()