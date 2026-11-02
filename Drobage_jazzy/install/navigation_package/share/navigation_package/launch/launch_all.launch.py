from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource 
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():

    imu = ExecuteProcess(
        cmd=['python3', '/joanna/bno085_node.py'],
        output='screen'
    )

    hardware = IncludeLaunchDescription(PythonLaunchDescriptionSource('/Drobage_jazzy/src/navigation_package/launch/launch_hardware.launch.py'))

#    arm = IncludeLaunchDescription(PythonLaunchDescriptionSource('/robot_eblan/src/roarm_moveit_config/launch/launch_arm.launch.py'))

    localization = TimerAction(period=5.0, actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource('/Drobage_jazzy/src/navigation_package/launch/launch_lidar.launch.py'))])

    nav = TimerAction(period=10.0, actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource('/Drobage_jazzy/src/navigation_package/launch/launch_nav2.launch.py'))])

    control = Node(
        package='robot_ctrl_package',
        executable='control_node',
        name='control'
    )

    delayed_control = TimerAction(period=30.0, actions=[control])

    manager = Node(
        package='mission_manager',
        executable='mission',
        name='mission'
    )

    delayed_manager = TimerAction(period=35.0, actions=[manager]) 

#    delayed_arm = TimerAction(period=45.0, actions=[arm])

    return LaunchDescription([
        imu,
        hardware,
        localization,
        nav,
        delayed_control,
        delayed_manager,
 #       delayed_arm,
    ])
