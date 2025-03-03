#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

class Turtle_pose(Node):
    def __init__(self):
        super().__init__('node_turtle_pose')
        self.get_logger().warn("turtle pose start")

        self.sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_cb, 10)

    def pose_cb(self, msg: Pose):
        msg_x = msg.x
        msg_y = msg.y
        msg_theta = msg.theta
        msg_linear_vel = msg.linear_velocity
        msg_angular_vel = msg.angular_velocity

        txt_pose = f"pose = ({round(msg_x, 2), round(msg_y, 2)})"
        txt_angle = f" angle = {round(msg_theta, 2)}"

        self.get_logger().info(txt_pose + txt_angle)

def main(args=None):
    rclpy.init(args=args)
    node = Turtle_pose()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()