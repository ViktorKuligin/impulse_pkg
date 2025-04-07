import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node

# from launch import LaunchDescription
# from launch_ros.actions import Node

# ros2 run turtlesim turtle_teleop_key

# def generate_launch_description():
#     return LaunchDescription([
#         Node(
#             package='rviz2',
#             executable='rviz2',
#             name='rviz2',
#             arguments=['-d', "/home/vi/rviz_save_ws/src/rviz_save_pkg/config/save_1.rviz"],
#             output='screen'
#         ),
#     ])

def generate_launch_description():

    ld = LaunchDescription()

    convert_node = Node(

        package= 'impulse_pkg',
        executable= 'con_1_convert_node',
        name= 'convert',

        parameters=[{
            "velocity": 0.02,
            "screen": False,
        }]

    )

    rviz_node = Node(

        package= 'impulse_pkg',
        executable= 'con_1_rviz_node',
        name= 'rviz_marker',

        parameters=[{
            "object_color": "green",
            "object_type": "cylinder",
            "object_scale": 1.0,
            "screen": True,
        }]

    )

    rviz_config = Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', "/home/vi/impulse_ws/src/impulse_pkg/config/imp_config.rviz"],
            output='screen'
        ),

    ld.add_action(convert_node)
    ld.add_action(rviz_node)
    # ld.add_action(rviz_config)
    # ld.add_entity(rviz_config)

    return ld