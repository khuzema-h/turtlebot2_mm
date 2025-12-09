#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32MultiArray
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import CompressedImage
import time
import json
from datetime import datetime
from language import LanguageModels
from cv_bridge import CvBridge
import cv2
import numpy as np
import ast

class MemoryNode(Node):
    def __init__(self):
        super().__init__('memory_node')
        
        # Subscribers - updated for your topics
        self.create_subscription(String, '/subtask', self.task_name_callback, 10)
        self.create_subscription(String, '/parameter', self.parameter_callback, 10)
        self.create_subscription(String, '/task_status', self.task_status_callback, 10)
        self.create_subscription(String, '/task_info', self.task_info_callback, 10)
        self.create_subscription(String, '/user_input', self.user_query_callback, 10)
        self.create_subscription(String, '/response_plan', self.response_plan_callback, 10)
        self.create_subscription(String, '/response_reason', self.response_reason_callback, 10)
        self.create_subscription(String, '/highlevel_response', self.sequence_callback, 10)
        self.create_subscription(Int32MultiArray, '/armpos', self.armpos_callback, 10)
        self.create_subscription(Odometry, '/tb2/odom', self.odom_callback, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/tb2/amcl_pose', self.amcl_callback, 10)
        self.create_subscription(CompressedImage, "/zed/zed_node/rgb/image_rect_color/compressed", self.rs_callback, 10)

        self.bridge = CvBridge()
        
        # Initialize variables
        self.task_name = "--"
        self.parameter = '--'
        self.arm_pos = str(['--', '--', '--', '--', '--', '--', '--'])
        self.user_query = "--"
        self.response_plan = "--"
        self.response_reason = "--"
        self.task_status = "--"
        self.odom_entry = " "
        self.amcl_entry = " "
        self.loc_options = ["ruthwik", "zahir", "amisha", "kasra", "home"]
        self.arm_options = ["start_pickup","complete_pickup","start_dropoff","complete_dropoff"]
        self.image = None
        self.last_amcl_entry = "0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0"
        self.task_info_sub = " "
        self.generate_captions = True
        self.llm = LanguageModels()
        
        # Timer for periodic logging
        self.create_timer(2.0, self.logging_callback)

    # Callback functions
    def task_name_callback(self, msg):
        self.task_name = msg.data

    def parameter_callback(self, msg):
        self.parameter = msg.data

    def armpos_callback(self, msg):
        self.arm_pos = str(msg.data)

    def user_query_callback(self, msg):
        self.user_query = msg.data

    def response_plan_callback(self, msg):
        self.response_plan = msg.data
    
    def response_reason_callback(self, msg):
        self.response_reason = msg.data
    
    def sequence_callback(self, msg):
        self.sequence = msg.data
        log = self.get_log(type="llm")
        self.get_logger().info(f"LLM Log: {json.dumps(log, indent=4)}")
        self.save_logs(log)
    
    def task_status_callback(self, msg):
        self.task_status = msg.data

    def task_info_callback(self, msg):
        self.task_info_sub = msg.data
    
    def odom_callback(self, msg):
        # Extract position (x, y, z)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        # Extract orientation (quaternion: x, y, z, w)
        ox = msg.pose.pose.orientation.x
        oy = msg.pose.pose.orientation.y
        oz = msg.pose.pose.orientation.z
        ow = msg.pose.pose.orientation.w
        # Store all in an array
        self.odom_entry = str([x, y, z, ox, oy, oz, ow])
   
    def amcl_callback(self, msg):
        # Extract position (x, y, z)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z
        # Extract orientation (quaternion: x, y, z, w)
        ox = msg.pose.pose.orientation.x
        oy = msg.pose.pose.orientation.y
        oz = msg.pose.pose.orientation.z
        ow = msg.pose.pose.orientation.w
        # Store all in a stringified array
        self.amcl_entry = str([x, y, z, ox, oy, oz, ow])

    def rs_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().warn(f"Failed to process image: {e}")

    def parse_amcl_entry(self, entry_str):
        try:
            return [float(x) for x in ast.literal_eval(entry_str)]
        except Exception:
            return []

    def has_moved(self, prev_entry, current_entry, threshold=0.01):
        prev = self.parse_amcl_entry(prev_entry)
        current = self.parse_amcl_entry(current_entry)

        if len(prev) != len(current):
            return True  # Something changed structurally

        for p, c in zip(prev, current):
            if abs(p - c) > threshold:
                return True  # Movement detected
        return False  # No meaningful movement

    def get_log(self, type):
        log = {}
        if type == "status":
            log["timestamp"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            log["type"] = "status"
            log["robot"] = {
                "position": {
                    "base_position": self.amcl_entry,
                    "arm_position": self.arm_pos
                }}
            
            if self.image is not None and self.generate_captions:
                try:
                    self.prev_caption = self.llm.get_vlm_feedback(task="caption", rs_image=self.image)
                    log["camera_observation"] = self.prev_caption
                except Exception as e:
                    log["camera_observation"] = f"Error generating caption: {e}"
            else:
                log["camera_observation"] = " "
                
            log["task_progress"] = {
                "task_name": self.task_name,
                "parameter": self.parameter,
                "task_status": self.task_status,
                "task_info": self.task_info_sub
            }
            self.last_amcl_entry = self.amcl_entry
            return log
        
        elif type == "llm":
            log["timestamp"] = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
            log["type"] = "llm"
            log["llm"] = {
                "user_input": self.user_query,
                "response": self.response_plan,
                "reasoning": self.response_reason,
                "sequence": self.sequence
            }
            return log

    def save_logs(self, log):
        log_file = "memory_files/robot_logs.jsonl"
        try:
            json_line = json.dumps(log) 
            with open(log_file, "a") as file:
                file.write(json_line + "\n")
        except Exception as e:
            self.get_logger().error(f"Error saving logs: {e}")

    def logging_callback(self):
        log = self.get_log(type="status")
        self.get_logger().info(f"Status Log: {json.dumps(log, indent=4)}")
        self.save_logs(log)

def main(args=None):
    rclpy.init(args=args)
    mem_node = MemoryNode()
    
    try:
        rclpy.spin(mem_node)
    except KeyboardInterrupt:
        pass
    finally:
        mem_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()