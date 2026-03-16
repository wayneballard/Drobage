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
from geometry_msgs.msg import Quaternion

class OdomNode(Node):
    def __init__(self):
        super().__init__("odom_node")
        self.publisher_odom = self.create_publisher(Odometry, "odom", 5)
        self.create_timer(float(1/10), self.imu_callback)
        self.session = requests.Session()
        self.odom = Odometry()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = time.time()

        # Wheel parameters
        self.wheel_base = 0.23  # meters
        self.left_wheel_speed = 0.0  # m/s
        self.right_wheel_speed = 0.0   
        self.subscription = self.create_subscription(
        Image, 
        'image_depth', 
        self.depth_callback,
        5)   

    def depth_callback(self, msg: Image):
        self.timestamp = msg.header.stamp

        self.odom.header.stamp = self.timestamp
        print(self.timestamp)


    def imu_callback(self):
        now = time.time()
        dt = now - self.last_time
        self.last_time = now 
        odom = self.session.get("http://192.168.4.1/js?json={\"T\":126}")        
        data = odom.json()
        if data is None:
            return
        print(data)

        vl = data["M1"] + data["M4"]
        vr = data["M2"] + data["M3"]

        # Differential drive kinematics
        v = (vr + vl) / 2.0
        omega = (vr - vl) / self.wheel_base

        # Update pose
        self.theta += omega * dt
        self.x += v * math.cos(self.theta) * dt
        self.y += v * math.sin(self.theta) * dt

        # Construct message
        self.odom = Odometry()
        self.odom.header.frame_id = "odom_link"
        self.odom.child_frame_id = "base_link"

        self.odom.pose.pose.position.x = self.x
        self.odom.pose.pose.position.y = self.y
        self.odom.pose.pose.position.z = 0.0

        q = Quaternion()
        q.w = math.cos(self.theta / 2)
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(self.theta / 2)
        self.odom.pose.pose.orientation = q

        self.odom.twist.twist.linear.x = v
        self.odom.twist.twist.angular.z = omega

        self.publisher_odom.publish(self.odom)


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


