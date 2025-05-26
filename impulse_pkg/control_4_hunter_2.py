#!/usr/bin/env python3
import rclpy
import numpy as np
from tf_transformations import quaternion_from_euler             # sudo apt install ros-humble-tf-transformations

from rclpy.node import Node
from geometry_msgs.msg import Vector3, Point, Quaternion, PointStamped
from visualization_msgs.msg import Marker
from std_msgs.msg import ColorRGBA, String, Int8

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
        self.declare_parameter('screen', False)
        self.declare_parameter('hunter_x', 0.0)
        self.declare_parameter('hunter_y', 0.0)

        self.goal_color_on = self.get_parameter('goal_color_on').get_parameter_value().string_value
        self.goal_color_off = self.get_parameter('goal_color_off').get_parameter_value().string_value
        self.goal_type = self.get_parameter('goal_type').get_parameter_value().string_value
        self.goal_scale = self.get_parameter('goal_scale').get_parameter_value().double_value
        self.screen = self.get_parameter('screen').get_parameter_value().bool_value
        self.hunter_x = self.get_parameter('hunter_x').get_parameter_value().double_value
        self.hunter_y = self.get_parameter('hunter_y').get_parameter_value().double_value

        self.goal_x = -1.0
        self.goal_y = -4.0
        self.radius = 0.5

        self.goal_angle = 0
        self.hunter_angle = 0

        self.hunter_mode = 'differential drive'
        self.mode = {
            0: 'omni/mecanum drive',
            1: 'differential drive',
            2: 'Ackerman drive',
        }
        self.hunter_velocity = 1

        self.delta_x = 0.01
        self.delta_y = 0.01

        # self.sub = self.create_subscription(Point, "point_goal", self.point_cb, 10)
        # self.sub = self.create_subscription(String, "hunter_mode", self.mode_cb, 10)
        self.sub = self.create_subscription(Int8, "hunter_velocity", self.velocity_cb, 10)
        self.pub_m = self.create_publisher(Marker, "marker_hunter", 10)
        self.pub_p = self.create_publisher(PointStamped, "point_goal", 10)

        self.get_logger().info(f'object_color={self.goal_color_on}')
        self.get_logger().info(f'screen={self.screen}')

        self.keyboard = self.create_timer(0.1, self.hunter_timer)

    def hunter_timer(self):
        if self.hunter_mode == 'omni/mecanum drive':

            if self.hunter_x > self.goal_x:
                self.hunter_x -= 0.1
            elif self.hunter_x < self.goal_x:
                self.hunter_x += 0.1

            if self.hunter_y > self.goal_y:
                self.hunter_y -= 0.1
            elif self.hunter_y < self.goal_y:
                self.hunter_y += 0.1

        elif self.hunter_mode == 'differential drive':

            self.delta_x = abs(self.hunter_x - self.goal_x)
            self.delta_y = abs(self.hunter_y - self.goal_y)
            self.goal_angle = np.arctan(self.delta_y / self.delta_x) * 180 / np.pi
            if (self.goal_x > 0) and (self.goal_y > 0):
                self.goal_angle = self.goal_angle
            elif (self.goal_x < 0) and (self.goal_y > 0):
                self.goal_angle = 180 - self.goal_angle
            elif (self.goal_x > 0) and (self.goal_y < 0):
                self.goal_angle = -self.goal_angle
            elif (self.goal_x < 0) and (self.goal_y < 0):
                self.goal_angle = 180 + self.goal_angle

            # self.
            if self.hunter_angle > self.goal_angle + 0.5:
                self.hunter_angle -= 0.5
            elif self.hunter_angle < self.goal_angle - 0.5:
                self.hunter_angle += 0.5
            else:
                if self.hunter_x > self.goal_x + 0.1:
                    self.hunter_x += 0.1 * np.cos(self.hunter_angle * np.pi / 180)
                elif self.hunter_x < self.goal_x - 0.1:
                    self.hunter_x += 0.1 * np.cos(self.hunter_angle * np.pi / 180)

                if self.hunter_y > self.goal_y + 0.1:
                    self.hunter_y += 0.1 * np.sin(self.hunter_angle * np.pi / 180)
                elif self.hunter_y < self.goal_y - 0.1:
                    self.hunter_y += 0.1 * np.sin(self.hunter_angle * np.pi / 180)

            self.get_logger().info(f'goal={round(self.goal_angle,2)}, hunter={round(self.hunter_angle,2)}, x = {round(self.hunter_x,2)}, y = {round(self.hunter_y,2)}')

            # self.goal_angle

            q = quaternion_from_euler(0.0, 0.0, self.hunter_angle * np.pi / 180)

            self.car_angle_z = q[2]
            self.car_angle_w = q[3]


        elif self.hunter_mode == 'Ackerman drive':
            pass
        else: 
            self.hunter_mode = 'omni/mecanum drive'

        color_now = color.get(self.goal_color_off)

        msg_out = Marker()
        msg_out.header.stamp = self.get_clock().now().to_msg()
        msg_out.header.frame_id = 'map'
        msg_out.type = Marker.ARROW #geom.get(self.goal_type) # ARROW = 0 / CUBE = 1 / SPHERE = 2 / CYLINDER = 3 / TEXT_VIEW_FACING = 9
        msg_out.pose.position = Point(x = self.hunter_x, y = self.hunter_y, z = 0.0)

        msg_out.pose.orientation = Quaternion(x = 0.0, y = 0.0, z = q[2], w = q[3])

        msg_out.scale = Vector3(x = self.goal_scale * 2, y = self.goal_scale, z = self.goal_scale)
        msg_out.color = color_now
        self.pub_m.publish(msg_out)
        
        point_static = PointStamped()
        point_static.header.stamp = self.get_clock().now().to_msg()
        point_static.header.frame_id = 'map'
        point_static.point.x = self.goal_x
        point_static.point.y = self.goal_y
        point_static.point.z = 0.0
        self.pub_p.publish(point_static)

        if self.screen:
            self.get_logger().info(f'x={msg_out.pose.position.x}, y={msg_out.pose.position.y}')

    
    def point_cb(self, msg: Point):

        self.goal_x = msg.x
        self.goal_y = msg.y
    
    def mode_cb(self, msg: String):

        self.mode = msg.data

    def velocity_cb(self, msg: Int8):

        self.velocity = msg.data



def main(args=None):
    rclpy.init(args=args)
    node = goal()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()