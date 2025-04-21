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

class goal(Node):

    def __init__(self):

        super().__init__('goal')
        self.get_logger().warn("goal start")

        self.declare_parameter('goal_color_on', 'green')
        self.declare_parameter('goal_color_off', 'red')
        self.declare_parameter('goal_type', 'sphere')
        self.declare_parameter('goal_scale', 0.5)
        self.declare_parameter('screen', True)
        self.declare_parameter('location_x', 4.0)
        self.declare_parameter('location_y', 4.0)

        self.goal_color_on = self.get_parameter('goal_color_on').get_parameter_value().string_value
        self.goal_color_off = self.get_parameter('goal_color_off').get_parameter_value().string_value
        self.goal_type = self.get_parameter('goal_type').get_parameter_value().string_value
        self.goal_scale = self.get_parameter('goal_scale').get_parameter_value().double_value
        self.screen = self.get_parameter('screen').get_parameter_value().bool_value
        self.location_x = self.get_parameter('location_x').get_parameter_value().double_value
        self.location_y = self.get_parameter('location_y').get_parameter_value().double_value

        self.sub = self.create_subscription(Point, "point_obj", self.point_cb, 10)
        self.pub = self.create_publisher(Marker, "marker_goal", 10)

        self.get_logger().info(f'object_color={self.goal_color_on}')
        self.get_logger().info(f'screen={self.screen}')

    def point_cb(self, msg: Point):

        color_now = color.get(self.goal_color_off)

        if (self.location_x == msg.x) and (self.location_y == msg.y):
            color_now = color.get(self.goal_color_on)

        msg_out = Marker()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.header.frame_id = 'map'
        msg_out.type = geom.get(self.goal_type) # ARROW = 0 / CUBE = 1 / SPHERE = 2 / CYLINDER = 3 / TEXT_VIEW_FACING = 9
        msg_out.pose.position = Point(x = self.location_x, y = self.location_y, z = 0.0)
        msg_out.pose.orientation = Quaternion(x = 0.0, y = 0.0, z = 0.0, w = 1.0)
        msg_out.scale = Vector3(x = self.goal_scale, y = self.goal_scale, z = self.goal_scale)
        msg_out.color = color_now
        self.pub.publish(msg_out)

        if self.screen:
            self.get_logger().info(f'x={msg_out.pose.position.x}, y={msg_out.pose.position.y}')


def main(args=None):
    rclpy.init(args=args)
    node = goal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()