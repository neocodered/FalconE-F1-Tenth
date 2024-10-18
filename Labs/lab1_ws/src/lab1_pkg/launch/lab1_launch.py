from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Node for talker
    talker_node = Node(
        package='lab1_pkg',
        executable='talker',
        name='talker_node',
        parameters=[
            {'v': 1.0},  # Set the 'v' parameter
            {'d': 0.5},  # Set the 'd' parameter
        ],
        output='screen'  # Prints node output to terminal
    )

    # Node for relay
    relay_node = Node(
        package='lab1_pkg',
        executable='relay',
        name='relay_node',
        output='screen'
    )

    return LaunchDescription([
        talker_node,
        relay_node
    ])
