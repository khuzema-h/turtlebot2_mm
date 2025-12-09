#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
import yaml 
import time 
import json
from geometry_msgs.msg import Pose, Point, Quaternion, PoseStamped
from nav_msgs.msg import Odometry
from mobilegello.gello_controller import GELLOcontroller 
from language import LanguageModels
from unique_identifier_msgs.msg import UUID
from std_msgs.msg import String, Int32MultiArray
from sensor_msgs.msg import CompressedImage
import numpy as np
import cv2
import ast

from nav2_msgs.action import NavigateToPose
from builtin_interfaces.msg import Duration

class RobotTasks(Node):
    def __init__(self):
        super().__init__("robot_tasks")

        # Accessing saved locations
        self.pose_dict = {}        
        location_map = json.load(open("/home/nvidia/catkin_ws/src/nav_assistant/jsons/location_pose_map.json"))        
        for key, fl in location_map.items():            
            self.pose_dict[key] = self.read_pose_from_file(f"/home/nvidia/catkin_ws/src/nav_assistant/poses/{fl}.txt")        
        self.loc_options = ', '.join(list(location_map.keys()))
        self.arm_options = ["start_pickup","complete_pickup","start_dropoff","complete_dropoff"]
        
        # Instantiating
        self.mygello = GELLOcontroller("doodle", torque_start=True)
        self.llm = LanguageModels()

        # publishers
        self.goal_pub = self.create_publisher(PoseStamped, '/tb2/goal_pose', 10)
        self.cancel_pub = self.create_publisher(UUID, '/tb2/navigate_to_pose/_action/cancel_goal', 10)
        self.task_status_pub = self.create_publisher(String, '/task_status', 10)
        self.subtask_pub = self.create_publisher(String, '/subtask', 10)
        self.parameter_pub = self.create_publisher(String, '/parameter', 10)
        self.askuser_pub = self.create_publisher(String, '/askuser', 10)
        self.user_input_pub = self.create_publisher(String, '/user_input', 10)
        self.task_info_pub = self.create_publisher(String, '/task_info', 10)
        
        # Navigation action client
        self.nav_action_client = ActionClient(self, NavigateToPose, '/tb2/navigate_to_pose')
        
        # subscribers
        self.create_subscription(CompressedImage, "/zed/zed_node/rgb/image_rect_color/compressed", self.rs_callback, 10)
        self.create_subscription(String, '/highlevel_response', self.sequence_callback, 10)
        self.create_subscription(String, '/user_input', self.input_callback, 10)
        self.create_subscription(Odometry, '/tb2/odom', self.odom_callback, 10)
        self.create_subscription(String, '/task_status', self.task_status_callback, 10)

        # Initialize variables
        self.sequence = ""
        self.possible_tasks = ["navigate_to_person", "navigate_to_position", "navigate_to_object", "get_image_caption", "manipulate", "ask_user", "wait"]
        self.vlm_for_gripper = 0
        self.active_server = ""
        self.current_odom = None
        
        # Timer for main loop
        self.create_timer(1.0, self.main_loop)

    def sequence_callback(self, msg):
        self.sequence = msg.data

    def input_callback(self, msg):
        self.user_input = msg.data
        if self.user_input == "wait":
            self.wait()
        self.sequence = ""

    def odom_callback(self, msg):
        self.current_odom = msg

    def task_status_callback(self, msg):
        self.task_status = msg.data

    def rs_callback(self, msg):
        try:
            np_arr = np.frombuffer(msg.data, np.uint8)
            self.image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        except Exception as e:
            self.get_logger().warn(f"Failed to process image: {e}")

    def read_pose_from_file(self, filename):
        with open(filename, 'r') as file:
            parsed_data = yaml.safe_load(file)

        pose_data = parsed_data['pose']
        position = pose_data['position']
        orientation = pose_data['orientation']
        covariance = parsed_data['covariance']

        # Create a PoseStamped message for Nav2
        pose_msg = PoseStamped()
        pose_msg.pose = Pose(
            position=Point(x=position['x'], y=position['y'], z=position['z']),
            orientation=Quaternion(x=orientation['x'], y=orientation['y'], z=orientation['z'], w=orientation['w'])
        )
        pose_msg.header.frame_id = "map"
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        return pose_msg
    
    # PUBLIC METHODS
    def navigate_to_person(self, name: str):
        self.active_server = "navigation"
        if name.lower() in self.pose_dict:
            goal_pose = self.pose_dict[name.lower()]
            self.goal_pub.publish(goal_pose)
            self.get_logger().info(f"Navigating to {name}")
        else:
            self.get_logger().error(f"Unknown location: {name}")

    def navigate_to_position(self, coordinate):
        coordinate = tuple(ast.literal_eval(coordinate))
        self.active_server = "navigation"
        assert len(coordinate) == 7, "Coordinate should be a tuple of length 7"
        
        goal = PoseStamped()
        goal.header.frame_id = "map"
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose = Pose(
            position=Point(x=coordinate[0], y=coordinate[1], z=coordinate[2]),
            orientation=Quaternion(x=coordinate[3], y=coordinate[4], z=coordinate[5], w=coordinate[6])
        )
        self.goal_pub.publish(goal)
        self.get_logger().info(f"Navigating to position: {coordinate}")

    def navigate_to_object(self, object_name: str):
        self.get_logger().info(f"Object navigation requested for: {object_name}")
        # Implement object navigation logic here
        self.task_status_pub.publish(String(data="running"))

    def get_image_caption(self, data):
        if self.image is not None:
            try:
                response = self.llm.get_vlm_feedback(task="caption_2", rs_image=self.image, question=data)
                self.task_info_pub.publish(String(data=response))
                self.task_status_pub.publish(String(data="completed"))
            except Exception as e:
                self.task_info_pub.publish(String(data=f"Error: {e}"))
                self.task_status_pub.publish(String(data="error"))
        else:
            self.task_info_pub.publish(String(data="No image available"))
            self.task_status_pub.publish(String(data="error"))

    def manipulate(self, state: str):
        self.active_server = "arm"
        self.get_logger().info(f"Arm {state}")

        try:
            if state == "start_pickup":
                self.mygello.pickup()
            elif state == "complete_pickup":
                self.mygello.pickup_complete()
            elif state == "start_dropoff":
                self.mygello.dropoff()
            elif state == "complete_dropoff":
                self.mygello.dropoff_complete()
            
            self.task_status_pub.publish(String(data="completed"))
        except Exception as e:
            self.get_logger().error(f"Manipulation failed: {e}")
            self.task_status_pub.publish(String(data="error"))

    def ask_user(self, data: str):
        self.task_status_pub.publish(String(data="running"))
        self.askuser_pub.publish(String(data=data))
        self.wait()
        self.sequence = ""

    def wait(self, dummy=" "):
        # Cancel current navigation goal
        cancel_msg = UUID()
        self.cancel_pub.publish(cancel_msg)
        self.task_info_pub.publish(String(data="Waiting for user input"))
        self.sequence = ""

    def main_loop(self):
        if self.sequence != "":
            self.active_server = ""
            try:
                seq = json.loads(self.sequence)
                if "steps" in seq and len(seq["steps"]) > 0:
                    step = seq["steps"][0]
                    
                    self.get_logger().info(str(step))
                    if step["task"] in self.possible_tasks:
                        self.subtask_pub.publish(String(data=step["task"]))
                        self.parameter_pub.publish(String(data=step["parameter"]))
                        
                        getattr(self, step["task"])(step["parameter"])
                        
                        if self.active_server == "navigation":
                            self.task_info_pub.publish(String(data="Navigation in progress"))
                            self.task_status_pub.publish(String(data="running"))

                        elif self.active_server == "arm":
                            self.task_info_pub.publish(String(data="Manipulation in progress"))

                    else:
                        self.get_logger().warning(f"Unknown task: {step['task']}")
                        self.user_input_pub.publish(String(data=f"Unknown task: {step['task']}"))
            except json.JSONDecodeError as e:
                self.get_logger().error(f"Failed to parse sequence: {e}")

def main(args=None):
    rclpy.init(args=args)
    robot_tasks = RobotTasks()
    
    try:
        rclpy.spin(robot_tasks)
    except KeyboardInterrupt:
        pass
    finally:
        robot_tasks.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()