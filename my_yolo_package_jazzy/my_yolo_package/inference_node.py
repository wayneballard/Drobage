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
        self.YOLO_INTERVAL = 50                  
        self.verifications_required = 7
        self.verification_counter = 0

        params = cv2.TrackerNano_Params();
        params.neckhead = os.path.join("/Drobage_jazzy_310/src/my_yolo_package/share/models", "nanotrack_head_sim.onnx");
        params.backbone = os.path.join("/Drobage_jazzy_310/src/my_yolo_package/share/models", "nanotrack_backbone_sim.onnx");

        self.publisher_annotated = self.create_publisher(Image, "annotated_image", 5)
        self.publisher_depth = self.create_publisher(Image, "depth_frame", 5)
        self.publisher_detection = self.create_publisher(Detection2DArray, "detections", 5)
        self.publisher_garbage_verified = self.create_publisher(Bool, "garbage_verified", 5)  

        pkg_share_directory = "/Drobage_jazzy_310/src/my_yolo_package/share"
        defaults = {"model_path" : os.path.join(pkg_share_directory, "models", "best.pt"),
                    "conf":0.4,
                    "max_detections":1,
                    "class_detection":[1, 16, 17], #47
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
            print(self.model.names)
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

        self.subscription_centered = self.create_subscription(
        Bool,
        "centered",
        self.centered_callback,
        5)
        

        self.bridge = CvBridge()
        self.message_received = False
        
        self.tracking = False
        self.garbage_verified = False

        self.centered = False

    def depth_callback(self, msg: Image):
        self.depth_frame_inference = self.bridge.imgmsg_to_cv2(msg, "passthrough")
        self.depth_frame = self.bridge.cv2_to_imgmsg(self.depth_frame_inference, encoding = "passthrough")

        self.publisher_depth.publish(self.depth_frame)

    def image_callback(self, msg: Image):
        with self.frame_lock:
            self.latest_frame = msg
            self.frame_sequence += 1

    def centered_callback(self, msg: Bool):
        self.centered = msg.data

    def init_tracker(self, frame, box):
        self.tracker.init(frame, box);
        self.tracking = True

    def update_tracker(self, frame):
        return self.tracker.update(frame);

    def compare_boxes(self, yolo_box, tracker_box):
        x1_yolo, y1_yolo, x2_yolo, y2_yolo = map(int, yolo_box.xyxy[0])
        x1_tracker, y1_tracker, w_tracker, h_tracker = map(int, tracker_box)
        
        x2_tracker = x1_tracker + w_tracker
        y2_tracker = y1_tracker + h_tracker        

        intersection_x1 = max(x1_yolo, x1_tracker)
        intersection_x2 = min(x2_yolo, x2_tracker)
        intersection_y1 = max(y1_yolo, y1_tracker)
        intersection_y2 = min(y2_yolo, y2_tracker)

        intersection_w = max(0.0, intersection_x2 - intersection_x1)
        intersection_h = max(0.0, intersection_y2 - intersection_y1)

        intersection_area = intersection_w * intersection_h
        yolo_box_area = max(0.0, x2_yolo - x1_yolo) * max(0.0, y2_yolo - y1_yolo)
        tracker_box_area = max(0.0, x2_tracker - x1_tracker) * max(0.0, y2_tracker - y1_tracker)
        
        union_area = yolo_box_area + tracker_box_area - intersection_area
        if union_area <= 0.0:
            return 0.0

        iou = intersection_area / union_area
        return iou

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
                        self.garbage_verified = False

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
                    ok, tracker_box = self.update_tracker(self.cv_image);
                    if not ok:
                        self.garbage_verified = False
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
                                x1, y1, x2, y2 = map(int, box.xyxy[0])
                               
                                iou = self.compare_boxes(box, tracker_box)
                                
                                if iou < 0.3:
                                    self.garbage_verified = False
                                    self.verification_counter = 0
                                    #return
                                    continue 
                                if self.centered:
                                    self.verification_counter += 1
                                    if self.verification_counter >= self.verifications_required:
                                        self.garbage_verified = True
                                        self.get_logger().info("Verified succesfully")
                                self.init_tracker(self.cv_image, (int(x1), int(y1), int(x2 - x1), int(y2 - y1)));
                                self.frame_counter = 0
                                #self.garbage_detected = True
                            else:
                                self.get_logger().info("Garbage is gone")
                                self.garbage_verified = False
                                self.frame_counter = 0 
                                self.tracking = False
                                continue
                        else:
                            self.garbage_verified = True


                        x, y, w, h = map(int, tracker_box);
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
    
                msg_garbage_verified = Bool()
                msg_garbage_verified.data = self.garbage_verified
                print(self.garbage_verified)
                self.publisher_garbage_verified.publish(msg_garbage_verified)         
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
