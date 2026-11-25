from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='roarm_driver',
            executable='roarm_driver',
            output='screen',
        ),
    ])

