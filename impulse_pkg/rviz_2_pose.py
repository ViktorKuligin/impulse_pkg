#!/usr/bin/env python3
import rclpy
import numpy as np

from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Point, Quaternion
from tf_transformations import quaternion_from_euler

class RvizPose(Node):
    def __init__(self):
        super().__init__('pose_node')
        self.get_logger().warn("pose node start")

        self.degree = 0.0

        self.pub_static = self.create_publisher(PoseStamped, "pose_static", 10)
        self.pub_dynamic = self.create_publisher(PoseStamped, "pose_dynamic", 10)

        self.tim_static = self.create_timer(0.01, self.timers_static)
        self.tim_dynamic = self.create_timer(0.01, self.timers_dynamic)

    def timers_static(self):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position = Point(x=3.0, y=3.0, z=0.0)
        msg.pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        self.pub_static.publish(msg)

    def timers_dynamic(self):
        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position = Point(x=-3.0, y=-3.0, z=0.0)

        q = quaternion_from_euler(0.0, 0.0, self.degree * np.pi / 180)
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]

        self.pub_dynamic.publish(msg)

        self.degree += 1
        if self.degree == 360:
            self.degree = 0


def main(args=None):
    rclpy.init(args=args)
    node = RvizPose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()