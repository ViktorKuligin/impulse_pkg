from launch import LaunchDescription
from launch_ros.actions import Node

# ros2 run turtlesim turtle_teleop_key

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

        # parameters=[{
        #     "object_color": "green",
        #     "screen": True,
        # }]

    )

    ld.add_action(convert_node)
    ld.add_action(rviz_node)

    return ld