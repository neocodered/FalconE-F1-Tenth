from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    return LaunchDescription([

        # Launch the talker node with parameters 'v' and 'd'
        Node(
            package = 'lab1_pkg',
            executable = 'talker',
            parameters=[{'v': 1.0, 'd': 0.5}]
        ),

        # Launch the 'relay' node
        Node(
            package = 'lab1_pkg',
            executable = 'relay',
            name = 'relay',
            outputs = 'screen'
        )
    ])