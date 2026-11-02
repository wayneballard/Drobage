from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource 
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnProcessStart

def generate_launch_description():

    slam_node = IncludeLaunchDescription(PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),  launch_arguments={'slam_params_file': '/opt/ros/jazzy/share/slam_toolbox/config/mapper_params_online_async.yaml'}.items())

#    set_param = TimerAction(period=10.0, actions=[ExecuteProcess(cmd=['ros2', 'param', 'set', 'slam_toolbox', 'use_sim_time', 'false'], output='screen')])

    static_tf = Node(package = 'tf2_ros', executable='static_transform_publisher', arguments=['0.1', '-0.085', '0.22', '0', '0', '0', 'base_link', 'imu_link'], output='screen')

    ekf =  Node(package = 'robot_localization', executable='ekf_node', parameters=['/Drobage_jazzy/src/navigation_package/share/config/ekf_node.yaml', {'use_sim_time': False}], output='screen')

    set_param = TimerAction(period=15.0, actions=[ExecuteProcess(cmd=['ros2', 'param', 'set', 'slam_toolbox', 'use_sim_time', 'false'], output='screen')])

    return LaunchDescription([
        slam_node,
        static_tf,
#        set_param,
        ekf,
        set_param,
    ])
