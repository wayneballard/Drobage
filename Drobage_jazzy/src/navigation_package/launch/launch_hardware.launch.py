from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource 
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():

    lidar = IncludeLaunchDescription(PythonLaunchDescriptionSource('/ldlidar_ros2_binning/src/ldlidar_stl_ros2/launch/ld19.launch.py'))

    odometry_node = Node(package='my_yolo_package', executable='odom_node')

    

    return LaunchDescription([
        lidar,
        odometry_node,
    ])
