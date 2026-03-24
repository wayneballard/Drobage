import rclpy
import time
import requests
import math 
import numpy as np
from rclpy.node import Node
from cv_bridge import CvBridge
from std_msgs.msg import Float32, Int16, Bool, Float32MultiArray
import sys
import serial
import threading
import json

def lerp(a, b, t):
    return a + (b - a) * t
    
def interpolate_stages(stage_a, stage_b, t):
    keys = ["b", "s", "e", "t_joint", "r", "g"]
    return {k: lerp(stage_a[k], stage_b[k], t) for k in keys}


class Arm(Node):
    def __init__(self):
        super().__init__("arm_node")
        self.subscriber_stop = self.create_subscription(Bool, "stop", self.stop_callback, 5)
        self.subscriber_det = self.create_subscription(Bool, "detection_to_arm", self.detection_callback, >

        self.publisher_exec = self.create_publisher(Bool, "executing", 5)

        self.full_stop = False
        self.stop_execution = False
        self.executing = False
        self.detected = False
        self.ready_to_pick = False

        self.ser = serial.Serial("/dev/ttyUSB0", baudrate=115200)

        self.steps = 20
        self.spd =  0
        self.acc = 10
        self.move_time = 1.0
        self.stages_file = "/home/wb/Desktop/stages.json"
        self.current_stage = 0 
        self.current_step = 0

        self.timer = self.create_timer(float(self.move_time / self.steps), self.execute_step)
        #self.move_time / self.steps
        with open(self.stages_file) as f:
            self.stages = json.load(f)["stages"]


    def stop_callback(self, msg):
        self.full_stop = msg.data
        print(msg)

    def detection_callback(self, msg):
        self.detected = msg.data

    def send_command(self, ser, joints, spd, acc):
        cmd = {
            "T": 102,
            "base": joints["b"],
            "shoulder": joints["s"],
            "elbow": joints["e"],
            "wrist": joints["t_joint"],
            "roll": joints["r"],
            "hand": joints["g"],
            "spd": spd,
            "acc": acc,
        }
        self.ser.write(json.dumps(cmd).encode() + b"\n")
    

#    def read_serial(self):
#        while rclpy.ok():
#            data = self.ser.readline().decode('utf-8')
#            if data:
#                print(f"Received: {data}, {len(self.stages)}", end='')

    def execute_step(self):
        if self.full_stop and self.detected:
            self.ready_to_pick = True ##
        if self.ready_to_pick: ##
            self.executing = True  #
            if self.current_stage >= len(self.stages) or self.stop_execution:
                self.get_logger().info("Trajectory complete")
                self.executing = False #
                self.ready_to_pick = False
                self.current_stage = 0 ##
                self.current_step = 0 ##
                return

            if self.current_stage == 0:
                stage = self.stages[self.current_stage]
                self.send_command(self.ser, stage, self.spd, self.acc)
                self.current_stage += 1
                time.sleep(0.5 + self.move_time)
                return

            t = self.current_step / self.steps
            prev = self.stages[self.current_stage - 1]
            curr = self.stages[self.current_stage]
            interpolated = interpolate_stages(prev, curr, t)            
            self.send_command(self.ser, interpolated, self.spd, self.acc)
            self.current_step += 1        

            if self.current_step > self.steps:
                self.current_step = 0
                self.current_stage += 1

        msg_exec = Bool() #
        msg_exec.data = self.executing  #
        self.publisher_exec.publish(msg_exec)   #
        print(f"FULL STOP:{self.full_stop}")
        print(f"DETECTED:{self.detected}")

def main(args=None):
    rclpy.init(args=args)
    arm_node = Arm()

#   ser_thread = threading.Thread(target=arm_node.read_serial, daemon=True)
#   ser_thread.start()

    try:
        rclpy.spin(arm_node)
    except (KeyboardInterrupt, SystemExit):
        pass
    finally:
        arm_node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
