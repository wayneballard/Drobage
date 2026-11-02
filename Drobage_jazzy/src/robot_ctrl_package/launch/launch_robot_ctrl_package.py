from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource 
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription, TimerAction

def generate_launch_description():
    Node(
        package='robot_ctrl_package',
        executable='control_node',
        name='control'
    )
   

    return LaunchDescription([
        control
    ])
