import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
from tf_transformations import quaternion_from_euler
from action_msgs.srv import CancelGoal
import json
import requests
import math
import transforms3d as tf3d
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import time
from geometry_msgs.msg import Quaternion, Twist, TransformStamped
from tf2_ros import TransformBroadcaster
import serial
import threading
import os

class OdomNode(Node):
    def __init__(self):
        super().__init__("odom_node")
        self.publisher_odom = self.create_publisher(Odometry, "odometry", 5)
        self.garbage_detcted_sub = self.create_subscription(Bool, "garbage_detected", self.garbage_detected_callback, 5)
        self.merged_odom_sub = self.create_subscription(Odometry, "merged_odom", self.merged_odom_callback, 5)
        self.new_odom = Odometry()   
        
        self.cancel_goal_client = self.create_client(CancelGoal, "/navigate_to_pose/_action/cancel_goal")    

        self.garbage_detected = False

    def merged_odom_callback(self, msg):
        msg.child_frame_id = "base_link"
        self.new_odom = msg    
        self.publisher_odom.publish(self.new_odom)    
     
    def garbage_detected_callback(self, msg):
        self.garbage_detected = msg.data

        if self.garbage_detected:
            if not self.cancel_goal_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warn("Nav2 cancel service not available")
                return

            request = CancelGoal.Request()
        #    request.goal_info.goal_id = [0] * 16
            request.goal_info.stamp.sec = 0
            request.goal_info.stamp.nanosec = 0

            self.cancel_goal_client.call_async(request)

def main(args=None):
    rclpy.init(args=args)
    odom_node = OdomNode()
    try:
        rclpy.spin(odom_node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        odom_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
