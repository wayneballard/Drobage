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


class OdomNode(Node):
    def __init__(self):
        super().__init__("odom_node")
        self.publisher_odom = self.create_publisher(Odometry, "odometry", 5)
        self.cmd_vel_sub = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 20)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.t = TransformStamped()
        self.q = Quaternion()
        self.linear_velocity = None
        self.angular_velocity = None

        self.create_timer(float(1/20), self.twist_callback)
#        self.create_timer(float(1/20), self.odom_callback)
#        self.create_timer(float(1/20), self.publish_dynamic_transform)
#        self.create_timer(float(1/20), self.publish_odometry)

        self.session = requests.Session()
        self.ser = serial.Serial('/dev/ttyACM1', baudrate=115200)

#        self.imu = Imu()
        self.odom = Odometry()
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        self.last_time = time.time()
        self.timestamp = None
        # Wheel parameters
        self.wheel_base = 0.15  # meters
        self.left_wheel_speed = 0.0  # m/s
        self.right_wheel_speed = 0.0   
        self.subscription = self.create_subscription(
        Image, 
        'image_depth', 
        self.depth_callback,
        5)   
        self.current_time = self.get_clock().now().to_msg()
        self.zero_vel_count = 0

    def depth_callback(self, msg: Image):
        self.timestamp = msg.header.stamp
        print(self.timestamp)

#        self.odom.header.stamp = self.timestamp
#        print(self.timestamp)

    def deadzone(self, value, threshold=5.5):
        if abs(value) < threshold:
            return 0.0
        return value
#####
    def cmd_vel_callback(self, msg):
        self.linear_velocity = msg.linear.x
        self.angular_velocity = msg.angular.z

#        if linear_velocity == 0.0 and angular_velocity == 0.0:
#            self.zero_vel_count += 1
#            if self.zero_vel_count > self.zero_vel_limit:
#                return  
#        else:
#            self.zero_vel_count = 0  
    def twist_callback(self):
        if self.linear_velocity is None or self.angular_velocity is None:
            return
        # Apply minimum threshold to angular velocity if linear velocity is zero
        if self.linear_velocity == 0.0:
            if 0 < self.angular_velocity < 0.2:
                self.angular_velocity = 0.2
            elif -0.2 < self.angular_velocity < 0:
                self.angular_velocity = -0.2

        # Send the velocity data to the UGV as a JSON string
        data = f'{{"T":{13},"X":{self.linear_velocity},"Z":{self.angular_velocity}}}'

#        data = json.dumps({'T': 13, 'X': self.linear_velocity, 'Z': self.angular_velocity}) + "\n"
        print(data)
        self.send_command(data)

    def send_command(self, data) -> None:
        json_command = data
        try:
            self.ser.write((json_command + "\n").encode("utf-8"))
            print("SUCCESS")
            self.ser.flush()     
        except Exception as e:
            print("HTTP error: ", e)

#####
#    def odom_callback(self):
#        now = time.time()
#        dt = now - self.last_time
#        self.last_time = now 
#        odom = self.session.get("http://192.168.4.1/js?json={\"T\":126}")        
#        data = odom.json()
#        json_command = f'"T":{130}'
#        self.current_time = self.get_clock().now().to_msg()
#        try:
#            self.ser.write((json_command + "\n").encode("utf-8"))
#            data_encoded = self.ser.readline().decode('utf-8').strip()
#            data = json.loads(data_encoded)
#            print(data)
#            vl_clean = self.deadzone(data["M1"] + data["M4"])
#            vr_clean = self.deadzone(data["M2"] + data["M3"])
            
#            rpm_to_ms = (0.1 / 60) * (2.0 * math.pi * 0.0325)
#            vl = (data["M1"] + data["M4"]) / 2.0 * rpm_to_ms
#            vr = (data["M2"] + data["M3"]) / 2.0 * rpm_to_ms
#            vl = vl_clean / 2.0 * rpm_to_ms
#            vr = vr_clean / 2.0 * rpm_to_ms   
        # Differential drive kinematics
#            v = (vr + vl) / 2.0
#            omega = (vr - vl) / self.wheel_base

        # Update pose
#            self.theta += omega * dt
#            self.x += v * math.cos(self.theta) * dt
#            self.y += v * math.sin(self.theta) * dt

        # Construct message
#            self.odom = Odometry()
#            self.odom.header.frame_id = "odom"
#            self.odom.child_frame_id = "base_link"

#            self.odom.pose.pose.position.x = self.x
#            self.odom.pose.pose.position.y = self.y
#            self.odom.pose.pose.position.z = 0.0
#            self.odom.header.stamp = self.current_time #self.get_clock().now().to_msg()

#            self.q.w = math.cos(self.theta / 2)
#            self.q.x = 0.0
#            self.q.y = 0.0
#            self.q.z = math.sin(self.theta / 2)
#            self.odom.pose.pose.orientation = self.q

#            self.odom.twist.twist.linear.x = v
#            self.odom.twist.twist.angular.z = omega

#        except Exception as e:
#            self.get_logger().error(f"An exception occured: {e}") 

#    def publish_dynamic_transform(self):
#        self.t.header.stamp = self.current_time #self.get_clock().now().to_msg()
#        self.t.header.frame_id = "odom"
#        self.t.child_frame_id = "base_link"
#        self.t.transform.translation.x = self.x
#        self.t.transform.translation.y = self.y
#        self.t.transform.rotation = self.q

#        self.tf_broadcaster.sendTransform(self.t)

#    def publish_odometry(self):
#        self.publisher_odom.publish(self.odom)

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
