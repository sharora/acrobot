from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='acrobot',
            namespace='acrobot',
            executable='controller',
            name='controller'
        ),
        Node(
            package='acrobot',
            namespace='acrobot',
            executable='acrobot',
            name='acrobot'
        ),
    ])

