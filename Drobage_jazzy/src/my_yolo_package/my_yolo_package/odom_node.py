import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu, Image
from tf_transformations import quaternion_from_euler
import json
import requests
import math
import transforms3d as tf3d
from nav_msgs.msg import Odometry
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
        self.merged_odom_sub = self.create_subscription(Odometry, "merged_odom", self.merged_odom_callback, 20)
        self.new_odom = Odometry()        

    def merged_odom_callback(self, msg):
        msg.child_frame_id = "base_link"
        self.new_odom = msg    
        self.publisher_odom.publish(self.new_odom)    


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
