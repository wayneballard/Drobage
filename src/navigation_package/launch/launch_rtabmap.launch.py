from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource 
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    static_tf_node_one = Node(
       package='tf2_ros',
       executable='static_transform_publisher',
       arguments=['0.05', '0', '0.1', '0', '0', '0', 'base_link', 'camera_link'])

    static_tf_node_two = Node(
       package='tf2_ros',
       executable='static_transform_publisher',
       arguments=['0', '0', '0', '-1.5708', '0', '-1.5708', 'camera_link', 'camera_optical_frame'])

    static_tf_node_three = Node(
       package='tf2_ros',
       executable='static_transform_publisher',
       arguments=['0', '0', '0', '0', '0', '0', 'camera_link', 'imu_link'])

    rtabmap_launch_dir = get_package_share_directory('rtabmap_launch')

    rtabmap_node = IncludeLaunchDescription(
                      PythonLaunchDescriptionSource(os.path.join(rtabmap_launch_dir, 'launch', 'rtabmap.launch.py')),
                      launch_arguments={'args':"--delete_db_on_start --Rtabmap/DetectionRate 8 --Vis/MaxFeatures 800 --Vis/MinInliers 20 --RGBD/LoopClosureReextractFeatures true --Grid/NoiseFilteringRadius 0.05 --Grid/NoiseFilteringMinNeighbors 5",
                                        'rgb_topic':'/image_raw',
                                        'depth_topic':'/image_depth',
                                        'camera_info_topic':'/camera_info',
                                        'imu_topic':'/imu',
                                        'frame_id':'base_link',
                                        'approx_sync':'true', 
                                        'wait_imu_to_init':'false', 
                                        'approx_sync_max_interval':'0.1',
                                        'odom_topic':'/odom',
                                        'vo_frame_id':'odom'}.items()
                   )            

    delayed_rtabmap = TimerAction(period=3.0, actions=[rtabmap_node])
    return LaunchDescription([
        static_tf_node_one,
        static_tf_node_two,
        static_tf_node_three,
        delayed_rtabmap
    ])


