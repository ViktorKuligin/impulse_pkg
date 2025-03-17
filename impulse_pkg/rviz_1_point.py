#!/usr/bin/env python3
import rclpy
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import PointStamped

class RvizPoint(Node):
    def __init__(self):
        super().__init__('point_node')
        self.get_logger().warn("point node start")

        self.degree = 0.0
        self.radius = 1.5
        self.center_x = 2.0
        self.center_y = 3.0

        self.pub_static = self.create_publisher(PointStamped, "point_static", 10)
        self.pub_dynamic = self.create_publisher(PointStamped, "point_dynamic", 10)
        self.tim = self.create_timer(0.01, self.point_timer)

    def point_timer(self):
        point_static = PointStamped()
        point_static.header.stamp = self.get_clock().now().to_msg()
        point_static.header.frame_id = 'map'
        point_static.point.x = 2.0
        point_static.point.y = 4.0
        point_static.point.z = 0.0
        self.pub_static.publish(point_static)

        point_dynamic = PointStamped()
        point_dynamic.header.stamp = self.get_clock().now().to_msg()
        point_dynamic.header.frame_id = 'map'
        point_dynamic.point.x = np.cos(self.degree * np.pi / 180) * self.radius + self.center_x
        point_dynamic.point.y = np.sin(self.degree * np.pi / 180) * self.radius + self.center_y
        point_dynamic.point.z = 0.0

        self.pub_dynamic.publish(point_dynamic)

        self.degree += 1
        if self.degree == 360:
            self.degree = 0

def main(args=None):
    rclpy.init(args=args)
    node = RvizPoint()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()