#!/usr/bin/env python3
import rclpy
import numpy as np
import pygame as pg      # pip3 install pygame

from sensor_msgs.msg import Joy    # ros2 launch teleop_twist_joy teleop-launch.py
from rclpy.node import Node
from geometry_msgs.msg import Twist, Vector3, Pose
from tf_transformations import quaternion_from_euler # sudo apt install ros-humble-tf-transformations

pg.init()
pg.display.set_caption('Direction')
screen = pg.display.set_mode((600,400))
screen.fill((0,0,0))
pg.draw.rect(screen, (255,255,0), (0,0,600,400), 3)
pg.display.update()

font = pg.font.Font(None, 50)

wb = 70
thickness = 5

button_color = {
    'on': (0, 255, 0),
    'off': (255, 0, 0)
}
button_location = {
    '1': (100-int(wb/2), 300-int(wb/2), wb, wb),
    '2': (200-int(wb/2), 300-int(wb/2), wb, wb),
    '3': (300-int(wb/2), 300-int(wb/2), wb, wb),
    '4': (100-int(wb/2), 200-int(wb/2), wb, wb),
    '5': (200-int(wb/2), 200-int(wb/2), wb, wb),
    '6': (300-int(wb/2), 200-int(wb/2), wb, wb),
    '7': (100-int(wb/2), 100-int(wb/2), wb, wb),
    '8': (200-int(wb/2), 100-int(wb/2), wb, wb),
    '9': (300-int(wb/2), 100-int(wb/2), wb, wb),
}
button_value = {
    '1': (-0.7071, 0.7071),
    '2': (-1.0, 0.0),
    '3': (-0.7071, -0.7071),
    '4': (0.0, 1.0),
    '5': (0.0, 0.0),
    '6': (0.0, -1.0),
    '7': (0.7071, 0.7071),
    '8': (1.0, 0.0),
    '9': (0.7071, -0.7071),
}
button_value_1 = {
    # '1': (-0.7071, 0.7071),
    '2': (-1.0, 0.0),
    # '3': (-0.7071, -0.7071),
    '4': (0.0, 1.0),
    '5': (0.0, 0.0),
    '6': (0.0, -1.0),
    # '7': (0.7071, 0.7071),
    '8': (1.0, 0.0),
    # '9': (0.7071, -0.7071),
}
button_scancode_0 = {
    89: '1',
    90: '2',
    91: '3',
    92: '4',
    93: '5',
    94: '6',
    95: '7',
    96: '8',
    97: '9',
}
button_scancode_1 = {
    # 89: '1',
    90: '2',
    # 91: '3',
    92: '4',
    93: '5',
    94: '6',
    # 95: '7',
    96: '8',
    # 97: '9',
}
button_scancode_2 = {
    # 89: '1',
    # 90: '2',
    # 91: '3',
    # 92: '4',
    93: '5',
    # 94: '6',
    # 95: '7',
    # 96: '8',
    # 97: '9',
}
button_setup = {
    16 : 'mode',
}
button_mode = {
    'status': 0,
    0: 'omni/mecanum drive',
    1: 'differential drive',
    2: 'Ackerman drive',
}

def button_draw(button_name, status):
    color = button_color.get(status)
    array = button_location.get(button_name)
    pg.draw.rect(screen, color, array, thickness)
    img = font.render(button_name, True, color)
    screen.blit(img, (array[0] + 10, array[1] + 20))

# def button_mode():


class JoyGreen(Node):
    def __init__(self):
        super().__init__('joy_green')
        self.get_logger().warn("joy green start")

        self.vel = 0.5
        self.button_name = ''

        self.but_mode = 0

        self.lin_xx = 0.0
        self.lin_yy = 0.0
        self.ang_zz = 0.0
        
        self.car_xx = 0.0
        self.car_yy = 0.0
        self.car_angle_z = 0.0
        self.car_angle_w = 1.0

        self.pub = self.create_publisher(Vector3, 'direction', 10)
        self.pub_mode = self.create_publisher(Pose, 'direction_mode', 10)
        self.keyboard = self.create_timer(0.01, self.keyboard_timer)

        for i in range(1,10):
            button_draw(str(i), 'off')

        array = (10, 20, 580, 40)
        pg.draw.rect(screen, (0,0,0), array)
        txt = f'Mode: {button_mode.get(self.but_mode)}'
        img = font.render(txt, True, (0, 0, 255))
        screen.blit(img, (10, 20))

        pg.display.update()

    def keyboard_timer(self):
        for event in pg.event.get():
            if event.type == pg.QUIT:
                pg.quit()
            if event.type == pg.KEYDOWN:
                if event.scancode == 16:
                    self.but_mode += 1
                    if self.but_mode == 3: self.but_mode = 0
                    array = (10, 20, 580, 40)
                    pg.draw.rect(screen, (0,0,0), array)
                    txt = f'Mode: {button_mode.get(self.but_mode)}'
                    img = font.render(txt, True, (0, 0, 255))
                    screen.blit(img, (10, 20))

                if self.but_mode == 0:
                    if event.scancode in button_scancode_0:
                        self.button_name = button_scancode_0.get(event.scancode)
                        button_draw(self.button_name, 'on')
                        
                        # msg.x = button_value.get(self.button_name)[0] * self.vel
                        # msg.y = button_value.get(self.button_name)[1] * self.vel

                        self.lin_xx = self.lin_xx + button_value.get(self.button_name)[0] * self.vel
                        self.lin_yy = self.lin_yy + button_value.get(self.button_name)[1] * self.vel

                        self.get_logger().info(f'x={self.lin_xx}, y={self.lin_yy}')
                        
                        # q = quaternion_from_euler(0.0, 0.0, self.car_angle * np.pi / 180)
                        # msg_body.pose.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])
                        
                        # self.car_angle = self.car_angle + msg.angular.z * 0.5

                        # self.car_xx = self.car_xx + msg.linear.x * 0.01 * np.cos(self.car_angle * np.pi / 180)
                        # self.car_yy = self.car_yy + msg.linear.x * 0.01 * np.sin(self.car_angle * np.pi / 180)

                        # self.get_logger().info(f'x={msg.x}, y={msg.y}')
                        # self.pub.publish(msg)
                        msg_mode = Pose()
                        msg_mode.position.x = self.lin_xx
                        msg_mode.position.y = self.lin_yy
                        msg_mode.position.z = 0.0
                        msg_mode.orientation.x = 0.0
                        msg_mode.orientation.y = 0.0
                        msg_mode.orientation.z = self.car_angle_z
                        msg_mode.orientation.w = self.car_angle_w
                        self.pub_mode.publish(msg_mode)

                elif self.but_mode == 1:
                    if event.scancode in button_scancode_1:
                        self.button_name = button_scancode_1.get(event.scancode)
                        button_draw(self.button_name, 'on')
                        # msg = Vector3()
                        # msg.x = button_value.get(self.button_name)[0] * self.vel
                        # msg.y = button_value.get(self.button_name)[1] * self.vel
                        # self.get_logger().info(f'x={msg.x}, y={msg.y}')
                        # self.pub.publish(msg)

                        # q = quaternion_from_euler(0.0, 0.0, self.car_angle * np.pi / 180)
                        # msg_body.pose.orientation = Quaternion(x = q[0], y = q[1], z = q[2], w = q[3])

                        # self.car_angle_z

                        # self.car_angle = self.car_angle + msg.angular.z * 0.5

                        # self.lin_xx = self.lin_xx + button_value.get(self.button_name)[0] * self.vel
                        self.ang_zz = self.ang_zz + button_value.get(self.button_name)[1] * 2.0
                        q = quaternion_from_euler(0.0, 0.0, self.ang_zz * np.pi / 180)

                        self.car_angle_z = q[2]
                        self.car_angle_w = q[3]

                        self.lin_xx = self.lin_xx + button_value.get(self.button_name)[0] * self.vel * np.cos(self.ang_zz * np.pi / 180)
                        self.lin_yy = self.lin_yy + button_value.get(self.button_name)[0] * self.vel * np.sin(self.ang_zz * np.pi / 180)

                        msg_mode = Pose()
                        msg_mode.position.x = self.lin_xx
                        msg_mode.position.y = self.lin_yy
                        msg_mode.position.z = 0.0
                        msg_mode.orientation.x = 0.0
                        msg_mode.orientation.y = 0.0
                        msg_mode.orientation.z = self.car_angle_z
                        msg_mode.orientation.w = self.car_angle_w
                        # msg_mode.linear.x = button_value.get(self.button_name)[0] * self.vel
                        # msg_mode.angular.z = button_value.get(self.button_name)[1] * self.vel
                        self.pub_mode.publish(msg_mode)
                
                elif self.but_mode == 2:
                    if event.scancode in button_scancode_2:
                        self.button_name = button_scancode_2.get(event.scancode)
                        button_draw(self.button_name, 'on')
                        msg = Vector3()
                        msg.x = button_value.get(self.button_name)[0] * self.vel
                        msg.y = button_value.get(self.button_name)[1] * self.vel
                        self.get_logger().info(f'x={msg.x}, y={msg.y}')
                        self.pub.publish(msg)




            if event.type == pg.KEYUP:
                for i in range(1,10):
                    button_draw(str(i), 'off')

        pg.display.update()

def main(args=None):
    rclpy.init(args=args)
    node = JoyGreen()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ =='__main__':
    main()