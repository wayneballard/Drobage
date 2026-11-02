from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource 
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():

    demo = IncludeLaunchDescription(PythonLaunchDescriptionSource('/robot_eblan/src/roarm_moveit_config/launch/demo.launch.py'))

    xyz_control = IncludeLaunchDescription(PythonLaunchDescriptionSource('/robot_eblan/src/roarm_moveit_config/launch/xyz_control.launch.py'))

    driver = IncludeLaunchDescription(PythonLaunchDescriptionSource('/robot_eblan/src/roarm_moveit_config/launch/driver.launch.py'))

    

    return LaunchDescription([
        demo,
        xyz_control,
        driver
    ])

