from ultralytics import YOLO
import rclpy
import threading
import time
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Point
from vision_msgs.msg import Detection2DArray, Detection2D,ObjectHypothesisWithPose, Pose2D
from std_msgs.msg import Int32MultiArray, Bool
from cv_bridge import CvBridge
import cv2
import sys
import os
from ament_index_python.packages import get_package_share_directory

class YoloNode(Node):
    def __init__(self):
        print("Init")
        super().__init__('inference_node')
        self.cv_image = None
        self.latest_frame = None
        self.tracker = None 

        self.frame_lock = threading.Lock()
        self.frame_sequence = 0
        self.latest_frame = None

        self.frame_counter = 0
        self.YOLO_INTERVAL = 200                  

        params = cv2.TrackerNano_Params();
        params.neckhead = os.path.join("/Drobage_jazzy_310/src/my_yolo_package/share/models", "nanotrack_head_sim.onnx");
        params.backbone = os.path.join("/Drobage_jazzy_310/src/my_yolo_package/share/models", "nanotrack_backbone_sim.onnx");

        self.publisher_annotated = self.create_publisher(Image, "annotated_image", 5)
        self.publisher_depth = self.create_publisher(Image, "depth_frame", 5)
        self.publisher_detection = self.create_publisher(Detection2DArray, "detections", 5)
        self.publisher_garbage_detected = self.create_publisher(Bool, "garbage_detected", 5)  

        pkg_share_directory = "/Drobage_jazzy_310/src/my_yolo_package/share"
        defaults = {"model_path" : os.path.join(pkg_share_directory, "models", "best.pt"),
                    "conf":0.3,
                    "max_detections":1,
                    "class_detection":[16, 17], #47
                    "device" : 'cuda'
                    }
        self.params = {}

        for name, value in defaults.items():
            self.declare_parameter(name, value)
            self.params[name] = self.get_parameter(name).value
        
        self.model_path = self.params["model_path"]
        self.conf = self.params["conf"]
        self.max_detections = self.params["max_detections"]
        self.class_detection = self.params["class_detection"]
        self.device = self.params["device"]

        try:
            self.model = YOLO(self.model_path)
            self.get_logger().info(f"{self.model} was loaded succsefully")
        except Exception as e:
            self.get_logger().error(f"Could not load the model: {e}")
        try:
            self.tracker = cv2.TrackerNano_create(params);
            self.get_logger().info(f"{self.tracker} was loaded succesfully");
        except Exception as e:
            self.get_logger().error(f"Could not load the tracker: {e}");

               
        self.subscription = self.create_subscription(
        Image, 
        'image_raw', 
        self.image_callback,
        1)

        self.subscription_depth = self.create_subscription(
        Image, 
        "depth_frame_to_inference",
        self.depth_callback,
        1)
        
        self.bridge = CvBridge()
        self.message_received = False
        
        self.tracking = False
        self.garbage_detected = False

    def depth_callback(self, msg: Image):
        self.depth_frame_inference = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.depth_frame = self.bridge.cv2_to_imgmsg(self.depth_frame_inference, encoding = "passthrough")

        self.publisher_depth.publish(self.depth_frame)

    def image_callback(self, msg: Image):
        with self.frame_lock:
            self.latest_frame = msg
            self.frame_sequence += 1

    def init_tracker(self, frame, box):
        self.tracker.init(frame, box);
        self.tracking = True

    def update_tracker(self, frame):
        return self.tracker.update(frame);

            
    def processing_thread(self):
        last_processed_sequence = - 1
        while rclpy.ok():
            with self.frame_lock:
                if self.latest_frame is None:
                    frame = None
                    sequence = last_processed_sequence
                else:
                    frame = self.latest_frame
                    sequence = self.frame_sequence
            if frame is None or sequence == last_processed_sequence:
                time.sleep(0.005)
                continue
            
            last_processed_sequence = sequence

            try:
                print(self.class_detection)
                detection_2d_msg = Detection2DArray()
                detection_2d_msg.header = self.latest_frame.header
                detection_2d = Detection2D()
                hypothesis = ObjectHypothesisWithPose()

                self.get_logger().info("Message was received succesfully")

                self.cv_image = self.bridge.imgmsg_to_cv2(self.latest_frame, 'bgr8')
                if not self.tracking:
                    self.get_logger().info("Running YOLO for initial detection...")
                    infr_rslts = self.model(
                    source=self.cv_image,
                    device = self.device,
                    conf = self.conf, 
                    classes = self.class_detection, 
                    max_det = self.max_detections)
 
                    infr = infr_rslts[0]
                    x1 = y1 = x2 = y2 = None


                    if infr.boxes is not None and len(infr.boxes) > 0:
                        box = infr.boxes[0]
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        self.init_tracker(self.cv_image, (int(x1), int(y1), int(x2 - x1), int(y2 - y1)))

                        self.tracking = True
                        self.garbage_detected = True

                        self.get_logger().info("NanoTrack starts working...");
                        detection_2d.bbox.center.position.x = ((x1 + x2) / 2)
                        detection_2d.bbox.center.position.y = ((y1 + y2) / 2)
                        detection_2d.bbox.size_x = float((x2 - x1))
                        detection_2d.bbox.size_y = float((y2 - y1))

                        detection_2d_msg.detections.append(detection_2d)

                        x_center = int((x1 + x2) / 2)
                        y_center = int((y1 + y2) / 2)
                        cv2.circle(self.cv_image, (x_center, y_center), 5, (0, 255, 0), -1)
                        cv2.rectangle(self.cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                else:
                    ok, box = self.update_tracker(self.cv_image);
                    if not ok:
                        self.garbage_detected = False
                        self.tracking = False
                        self.frame_counter = 0

                    else:
                        self.get_logger().info("Publishing with NanoTrack...");
                        self.frame_counter += 1
                           
                        if self.frame_counter >= self.YOLO_INTERVAL:
                            self.get_logger().info("Running periodic YOLO re-detection")
                            infr_rslts = self.model(
                            source=self.cv_image,
                            device = self.device,
                            conf = self.conf, 
                            classes = self.class_detection, 
                            max_det = self.max_detections)
 
                            infr = infr_rslts[0]
                            x1 = y1 = x2 = y2 = None


                            if infr.boxes is not None and len(infr.boxes) > 0:
                                box = infr.boxes[0]
                                x1, y1,  x2, y2 = map(int, box.xyxy[0])
                                self.init_tracker(self.cv_image, (int(x1), int(y1), int(x2 - x1), int(y2 - y1)));
                                self.frame_counter = 0
                                self.garbage_detected = True
                            else:
                                self.get_logger().info("Garbage is gone")
                                self.garbage_detected = False
                                self.frame_counter = 0 
                                self.tracking = False
                                continue
                        else:
                            self.garbage_detected = True


                        x, y, w, h = map(int, box);
                        detection_2d.bbox.center.position.x = x + w / 2
                        detection_2d.bbox.center.position.y = y + h / 2
                        detection_2d.bbox.size_x = float(w)
                        detection_2d.bbox.size_y = float(h)

                        detection_2d_msg.detections.append(detection_2d)

                        cv2.circle(self.cv_image, (int(x + w / 2), int(y + h / 2)), 5, (0, 255, 0), -1)
                        cv2.rectangle(self.cv_image, (x, y), (x + w, y + h), (0, 255, 0), 2)
                
                self.annotated_image = self.bridge.cv2_to_imgmsg(self.cv_image, encoding = "bgr8")
                self.annotated_image.header = self.latest_frame.header

                self.publisher_detection.publish(detection_2d_msg)
                self.publisher_annotated.publish(self.annotated_image)
    
                msg_garbage_detected = Bool()
                msg_garbage_detected.data = self.garbage_detected

                self.publisher_garbage_detected.publish(msg_garbage_detected)         
            except Exception as e:
                self.get_logger().error(f"An exception occured: {e}")


def main(args=None):
    print("Start")
    rclpy.init(args=args)
    inference_node = YoloNode()

    process_thread = threading.Thread(target=inference_node.processing_thread, daemon=True)
    process_thread.start()

    try:
        rclpy.spin(inference_node)
    except(SystemExit, KeyboardInterrupt):
        pass
    finally:
        inference_node.destroy_node()
        rclpy.shutdown()

if __name__ =='__main__':
    main()
