from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    tx_node = Node(
        package= 'impulse_pkg',
        executable= 'ex_publisher_node',     # name node in pkg writen in setup.py
        name= 'tx',                                # new node name
        # remappings= [                              # change topic name
        #     ('/msg', '/topic_msg'),
        #     # ('/topic_name_1','/new_topic_name_1'),
        #     # ('/topic_name_2','/new_topic_name_2'),
        # ]
        parameters=[{
            # "topic_name":"message",
            # "timer_period": 0.2,
            # "print_available": True,
            # "fix_data": 45,
        }]
    )

    rx_node = Node(
        package= 'impulse_pkg',
        executable= 'ex_subscriber_node',
        name= 'rx',
        # remappings= [
        #     ('/msg', '/topic_msg'),
        #     # ('/topic_name_1','/new_topic_name_1'),
        #     # ('/topic_name_2','/new_topic_name_2'),
        # ]

    )

    ld.add_action(tx_node)
    ld.add_action(rx_node)
    return ld