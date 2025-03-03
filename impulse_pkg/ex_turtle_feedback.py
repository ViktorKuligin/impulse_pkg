#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class TurtleFeedback(Node):
    def __init__(self):
        super().__init__('node_turtle_rad2deg')
        self.get_logger().warn("turtle rad2deg start")

        self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_cb, 10)

    def pose_cb(self, msg: Pose):
        pose_x = msg.x
        pose_y = msg.y
        pose_theta = msg.theta
        msg_linear_vel = msg.linear_velocity
        msg_angular_vel = msg.angular_velocity

        msg = Twist()

        if pose_x > 9.0:
            msg.linear.x = 0.5
            msg.angular.z = 0.5
        elif pose_x < 2.0:
            msg.linear.x = 0.5
            msg.angular.z = 0.5
        else:
            msg.linear.x = 40.0
            msg.angular.z = 0.0

        self.pub.publish(msg)

        txt = f"pose = ({round(pose_x, 2), round(pose_y, 2)})"
        self.get_logger().info(txt)

def main(args=None):
    rclpy.init(args=args)
    node = TurtleFeedback()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()