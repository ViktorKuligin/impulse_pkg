from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    ld = LaunchDescription()

    tx_node = Node(
        package= 'impulse_pkg',
        executable= 'ex_publisher_node',     # name node in pkg writen in setup.py
        name= 'tx',                          # new node name
    )

    rx_node = Node(
        package= 'impulse_pkg',
        executable= 'ex_subscriber_node',
        name= 'rx',
    )

    ld.add_action(tx_node)
    ld.add_action(rx_node)
    return ld