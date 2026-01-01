#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys

class UserInputNode(Node):
    def __init__(self):
        super().__init__('user_input_node')
        # Publish to a custom topic, NOT directly to the robot
        self.publisher_ = self.create_publisher(Twist, '/user_request', 10)
        self.get_logger().info('Input Node Started. Publishing to /user_request')

    def send_request(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.publisher_.publish(msg)

def main():
    rclpy.init()
    node = UserInputNode()

    print("--- Remote Control Interface ---")
    print("Enter 'linear angular' (e.g., '0.5 0.0')")
    print("Type 'x' to exit.")

    try:
        while rclpy.ok():

            userInput = input("\nCommand (lin ang): ")
            
            if userInput.lower() == 'x':
                break
            
            parts = userInput.split()
            if len(parts) != 2:
                print("Error: Enter exactly 2 numbers.")
                continue
            
            try:
                lin = float(parts[0])
                ang = float(parts[1])
                node.send_request(lin, ang)
            except ValueError:
                print("Error: Numbers only.")

    except KeyboardInterrupt:
        pass
    finally:
        # Send a stop command before killing the node
        node.send_request(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()