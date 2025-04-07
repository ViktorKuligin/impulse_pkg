#!/usr/bin/env python3
import rclpy
import numpy as np
from tf_transformations import quaternion_from_euler             # sudo apt install ros-humble-tf-transformations

from rclpy.node import Node
from geometry_msgs.msg import Vector3, Point, Quaternion
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

color = {
    # r - red, g - green, b - blue, a - transparency
    'red': ColorRGBA(r = 1.0, g = 0.0, b = 0.0, a = 1.0),
    'blue': ColorRGBA(r = 0.0, g = 0.0, b = 1.0, a = 1.0),
    'green': ColorRGBA(r = 0.0, g = 1.0, b = 0.0, a = 1.0),
    'white': ColorRGBA(r = 1.0, g = 1.0, b = 1.0, a = 1.0),
}
geom = {
    "cube": Marker.CUBE,
    "sphere": Marker.SPHERE,
    "cylinder": Marker.CYLINDER,
    "arrow": Marker.ARROW,
}

class rviz_xy(Node):

    def __init__(self):

        super().__init__('rviz_xy')
        self.get_logger().warn("rviz xy start")

        self.declare_parameter('object_color', 'red')
        self.declare_parameter('object_type', 'sphere')
        self.declare_parameter('object_scale', 0.5)
        self.declare_parameter('screen', True)

        self.object_color = self.get_parameter('object_color').get_parameter_value().string_value
        self.object_type = self.get_parameter('object_type').get_parameter_value().string_value
        self.object_scale = self.get_parameter('object_scale').get_parameter_value().double_value
        self.screen = self.get_parameter('screen').get_parameter_value().bool_value

        self.sub = self.create_subscription(Point, "point_xy", self.point_cb, 10)
        
        self.pub = self.create_publisher(Marker, "marker_xy", 10)

        self.get_logger().info(f'object_color={self.object_color}')
        self.get_logger().info(f'screen={self.screen}')

    def point_cb(self, msg: Point):

        msg_out = Marker()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.header.frame_id = 'map'
        msg_out.type = geom.get(self.object_type)
        # marker type:
        # 1) ARROW = 0
        # 2) CUBE = 1
        # 3) SPHERE = 2
        # 4) CYLINDER = 3
        # 5) TEXT_VIEW_FACING = 9
        msg_out.pose.position = Point(x = msg.x, y = msg.y, z = 0.0)
        msg_out.pose.orientation = Quaternion(x = 0.0, y = 0.0, z = 0.0, w = 1.0)
        msg_out.scale = Vector3(x = self.object_scale, y = self.object_scale, z = self.object_scale)
        msg_out.color = color.get(self.object_color)
        self.pub.publish(msg_out)

        if self.screen:
            self.get_logger().info(f'x={msg_out.pose.position.x}, y={msg_out.pose.position.y}')


def main(args=None):
    rclpy.init(args=args)
    node = rviz_xy()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()