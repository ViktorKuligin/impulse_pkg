#!/usr/bin/env python3
import rclpy
import numpy as np
from tf_transformations import quaternion_from_euler       # sudo apt install ros-humble-tf-transformation

from rclpy.node import Node
from geometry_msgs.msg import Vector3, Point, Quaternion
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA

class RvizMarkerText(Node):
    def __init__(self):
        super().__init__('marker_text_node')
        self.get_logger().warn("marker text node start")

        self.pub = self.create_publisher(Marker, "marker_text", 10)
        self.tim = self.create_timer(0.01, self.marker_timers)

    def marker_timers(self):
        msg = Marker()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.type = Marker.TEXT_VIEW_FACING
        # marker type:
        # 1) ARROW = 0
        # 2) CUBE = 1
        # 3) SPHERE = 2
        # 4) CYLINDER = 3
        # 5) TEXT_VIEW_FACING = 9
        msg.pose.position = Point(x = 1.0, y = 1.0, z = 0.0)
        msg.pose.orientation = Quaternion(x = 0.0, y = 0.0, z = 0.0, w = 1.0)
        msg.scale = Vector3(x = 1.0, y = 1.0, z = 1.0)
        msg.color = ColorRGBA(r = 1.0, g = 1.0, b = 0.0, a = 1.0)
        msg.text = 'IMPULSE'
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RvizMarkerText()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()