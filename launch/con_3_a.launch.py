import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

# ros2 run turtlesim turtle_teleop_key

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', "/home/vi/impulse_ws/src/impulse_pkg/config/imp_config.rviz"],
            output='screen'
        ),
        Node(
            package= 'impulse_pkg',
            executable= 'con_3_main_node',
            name= 'main_node',
            # parameters=[{
            #     "velocity": 0.02,
            #     "screen": False,
            # }]
        ),
        Node(
            package= 'impulse_pkg',
            executable= 'con_3_object_node',
            name= 'object',
            # parameters=[{
            #     "object_color": "green",
            #     "object_type": "cylinder",
            #     "object_scale": 1.0,
            #     "screen": True,
            # }]
        )
    ])