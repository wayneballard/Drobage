import depthai as dai
import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo, Imu
import cv2
import numpy as np
from cv_bridge import CvBridge
import atexit
import os
import time
import requests
import json
import math
import transforms3d as tf3d

from ament_index_python.packages import get_package_share_directory

class OakCameraNode(Node):
    def __init__(self):
        super().__init__("oak_camera_node")

        cnfg_abs_path = "/home/wb/Desktop/Drobage/src/my_yolo_package/share"
        print(cnfg_abs_path)
        self.get_logger().info("Trying to load calibration file...")
        jsonfile = os.path.join(cnfg_abs_path, "config", "184430101153051300_09_28_25_13_00.json")
        print(jsonfile)
        self.get_logger().info("Calibration file loaded succesfully")

        self.cam_info = CameraInfo()
        self.cam_info.width = 640
        self.cam_info.height = 480
        self.cam_info.distortion_model = "plumb_bob"
        self.cam_info.k =[457.798,0.0,313.368,0.0,456.957,224.113,0.0,0.0,1.0]
        self.cam_info.r = [1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0]
        self.cam_info.p = [457.798,0.0,313.368,-34.334,0.0,456.957,224.113,0.0,0.0,0.0,1.0,0.0]
        self.cam_info.header.frame_id = "camera_link"

        self.publisher_rgb = self.create_publisher(Image, 'image_raw', 1)
        self.publisher_depth = self.create_publisher(Image, 'depth_frame_to_inference', 1)
        self.publisher_depth_original = self.create_publisher(Image, 'image_depth', 1)
        self.publisher_caminfo = self.create_publisher(CameraInfo, 'camera_info', 1)
####
        self.publisher_imu = self.create_publisher(Imu, "imu", 1)
        self.create_timer(float(1/30), self.imu_callback)
        self.imu = Imu()   
####

        self.bridge = CvBridge()

        self.get_logger().info("Configuring DepthAI pipeline...")

        self.pipeline = dai.Pipeline()
        self.cam_rgb = self.pipeline.create(dai.node.Camera).build()
        self.videoQueue = self.cam_rgb.requestOutput((640, 480), fps=10) #.createOutputQueue()

        calibData = dai.CalibrationHandler(jsonfile)
        self.pipeline.setCalibrationData(calibData)
        self.intrinsics  = calibData.getCameraIntrinsics(dai.CameraBoardSocket.CAM_B)

        self.monoLeft = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_B)
        self.monoRight = self.pipeline.create(dai.node.Camera).build(dai.CameraBoardSocket.CAM_C)
        self.stereo = self.pipeline.create(dai.node.StereoDepth)

        self.monoLeftOut = self.monoLeft.requestFullResolutionOutput(fps=10)
        self.monoRightOut = self.monoRight.requestFullResolutionOutput(fps=10)

        self.monoLeftOut.link(self.stereo.left)
        self.monoRightOut.link(self.stereo.right)
    
#        self.stereo.setOutputSize(640,480)
        self.stereo.setLeftRightCheck(True)
        self.stereo.setRectification(True)
        self.stereo.setSubpixel(False)
        self.stereo.setExtendedDisparity(True)
#        self.stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)

        self.stereo.initialConfig.postProcessing.temporalFilter.enable = False
        self.stereo.initialConfig.postProcessing.spatialFilter.enable = True

#        self.syncedLeftQueue = self.stereo.syncedLeft.createOutputQueue()
#        self.syncedRightQueue = self.stereo.syncedRight.createOutputQueue()

        self.disparityQueue = self.stereo.disparity.createOutputQueue()
        self.maxDisparity = 1
#        self.pipeline.start() ##########
#        self.img_align = self.pipeline.create(dai.node.ImageAlign)
#        self.stereo.disparity.link(self.img_align.input)
#        self.videoQueue.link(self.img_align.inputAlignTo)

        self.sync = self.pipeline.create(dai.node.Sync)
        self.monoLeftOut.link(self.sync.inputs['self.monoLeft'])
        self.monoRightOut.link(self.sync.inputs["self.monoRight"])
        self.videoQueue.link(self.sync.inputs["self.cam_rgb"])
        self.queue = self.sync.out.createOutputQueue()
#        self.pipeline.start()
##################       


        try:
            self.get_logger().info("Connecting to OAK-D Lite...")
            self.pipeline.start()
            self.get_logger().info("OAK-D Lite camera connected")
          #  self.videoIn = self.videoQueue.get()
            self.disparity = self.disparityQueue.get()

            messageGroup = self.queue.get()
            rgb_in = messageGroup['self.cam_rgb']
          #  depth_in = messageGroup['aligned_depth']  

          #  assert isinstance(self.videoIn, dai.ImgFrame)
          #  assert isinstance(self.disparity, dai.ImgFrame)
   
        except Exception as e:
            self.get_logger().error(f"Failed to connect to OAK-D Lite: {e}")
            rclpy.shutdown()
            return


        timer_period = float(1.0 / 30)
        self.timer = self.create_timer(timer_period, self.time_callback)

        atexit.register(self.cleanup)

####
    def imu_callback(self):
        imu = requests.get("http://192.168.4.1/js?json={\"T\":126}")        
        data = imu.json()
#        print(data)

        self.roll = math.radians(float(data["r"]))
        self.pitch = math.radians(float(data["p"]))
        self.yaw = float(data["y"])

        q = tf3d.euler.euler2quat(self.roll, self.pitch, self.yaw)

#        print(q)

        self.imu.orientation.x = q[1]
        self.imu.orientation.y = q[2]
        self.imu.orientation.z = q[3]
        self.imu.orientation.w = q[0]

        self.imu.orientation_covariance = [
        0.01,0.0,0.0,
        0.0,0.01,0.0,
        0.0,0.0,0.01
        ]

        self.imu.angular_velocity_covariance = [
        0.02,0.0,0.0,
        0.0,0.02,0.0,
        0.0,0.0,0.02
        ]

        self.imu.linear_acceleration_covariance = [
        0.04,0.0,0.0,
        0.0,0.04,0.0,
        0.0,0.0,0.04
        ]
        self.imu.linear_acceleration.x = float(data["ax"]) * 9.80665 / 1000
        self.imu.linear_acceleration.y = float(data["ay"]) * 9.80665 / 1000
        self.imu.linear_acceleration.z = float(data["az"]) * 9.80665 / 1000

        self.imu.angular_velocity.x = math.radians(float(data["gx"]))
        self.imu.angular_velocity.y = math.radians(float(data["gy"]))
        self.imu.angular_velocity.z = math.radians(float(data["gz"]))
###


    def time_callback(self):
#        rgb_in = self.videoQueue.get()
        depth_in = self.disparityQueue.get()
        messageGroup = self.queue.get()

        rgb_in = messageGroup['self.cam_rgb']
#        depth_in = messageGroup['aligned_depth']     

        rgb_in_stamp = rgb_in.getTimestamp()
        depth_in_stamp = depth_in.getTimestamp()
        print(f"RGB timestamp:{rgb_in_stamp}")
        print(f"depth timestamp:{depth_in_stamp}")
        if rgb_in is None and depth_in is None:
            self.get_logger().info("No frame received from OAK-D")
            return 

        rgb_frame = rgb_in.getCvFrame()

        self.npDisparity  = depth_in.getFrame()
        self.maxDisparity = max(self.maxDisparity, np.max(self.npDisparity))
        normalizedDisparity = ((self.npDisparity / self.maxDisparity) * 255).astype(np.uint8)
        stamp = self.get_clock().now().to_msg()

        ros_image_rgb = self.bridge.cv2_to_imgmsg(rgb_frame, "bgr8")
        ros_image_rgb.header.stamp = rclpy.time.Time(seconds=depth_in.getTimestamp().total_seconds()).to_msg()
        ros_image_rgb.header.frame_id = "camera_link"

        ros_image_depth_original = self.bridge.cv2_to_imgmsg(self.npDisparity)
        ros_image_depth_original.header.stamp = rclpy.time.Time(seconds=depth_in.getTimestamp().total_seconds()).to_msg()
        ros_image_depth_original.header.frame_id = "camera_link"

        ros_image_depth = self.bridge.cv2_to_imgmsg(normalizedDisparity, "mono8")
        ros_image_depth.header.stamp = rclpy.time.Time(seconds=depth_in.getTimestamp().total_seconds()).to_msg()
        ros_image_depth.header.frame_id = "depth_frame_to_inference"

        self.cam_info.header.stamp = rclpy.time.Time(seconds=depth_in.getTimestamp().total_seconds()).to_msg()
        self.cam_info.header.frame_id = "camera_link"
###
        self.imu.header.stamp = rclpy.time.Time(seconds=depth_in.getTimestamp().total_seconds()).to_msg()
        self.imu.header.frame_id = "imu_link"
#        self.publisher_imu.publish(self.imu)

###
        self.publisher_rgb.publish(ros_image_rgb)
        self.publisher_depth.publish(ros_image_depth)
        self.publisher_depth_original.publish(ros_image_depth_original)
        self.publisher_caminfo.publish(self.cam_info)        
        self.publisher_imu.publish(self.imu)



    def cleanup(self):
        if hasattr(self, 'device'):
            self.device.close()
            self.get_logger().info("OAK-D device closed.")
    
def main(args=None):
    rclpy.init(args=args)
    camera_node = OakCameraNode()
    try:
        rclpy.spin(camera_node)
    except (SystemExit, KeyboardInterrupt):
        pass
    finally:
        camera_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()




