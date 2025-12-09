#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import String, Float64

class UserInputNode(Node):
    def __init__(self):
        super().__init__('user_input_node')

        # Initialize publishers
        self.input_pub = self.create_publisher(String, '/user_input', 10)
        self.task_status_pub = self.create_publisher(String, '/task_status', 10)
        self.time_newtask_pub = self.create_publisher(Float64, '/time_newtask', 10)
        self.task_info_pub = self.create_publisher(String, '/task_info', 10)
        
        # subscribers
        self.create_subscription(String, '/askuser', self.askuser_callback, 10)
        
        self.askuser_trigger = False
        self.askuser_sub = ""
        self.input_text = ""

    def askuser_callback(self, msg):
        self.askuser_sub = msg.data
        self.get_logger().info(f"askuser_sub: {self.askuser_sub}")
        self.askuser_trigger = True

    def publish_user_input(self):
        # Get user input
        self.input_text = input("Enter your command: ")
        # Publish the user input
        msg = String()
        msg.data = self.input_text
        self.input_pub.publish(msg)
        self.get_logger().info(f"Published user input: {self.input_text}")

def main(args=None):
    rclpy.init(args=args)
    UI = UserInputNode()
    
    # Publish initial time
    current_time = time.time()
    time_msg = Float64()
    time_msg.data = current_time
    UI.time_newtask_pub.publish(time_msg)
    
    try:
        while rclpy.ok():
            if UI.askuser_trigger:
                UI.askuser_trigger = False
                UI.task_info_pub.publish(String(data=UI.input_text))
                UI.task_status_pub.publish(String(data="completed"))
            else:
                UI.publish_user_input()
            
            rclpy.spin_once(UI, timeout_sec=0.1)
            
    except KeyboardInterrupt:
        pass
    finally:
        UI.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()