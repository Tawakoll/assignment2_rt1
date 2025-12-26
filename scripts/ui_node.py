#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys

# Create a simple class to handle the ROS 2 Publisher
class Commander(Node):
    def __init__(self):
        super().__init__('robot_controller')
        # Create a publisher on the /cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)

    def send_velocity(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing: Linear={linear}, Angular={angular}')

def user_interface():
    # 1. Initialize ROS 2 communication
    rclpy.init()

    # 2. Instantiate our node
    commander = Commander()

    print("--- ROS 2 Robot Interface ---")
    print("Enter Linear and Angular velocity to drive.")
    print("Type 'x' to exit.")

    try:
        while rclpy.ok():
            # 3. Get User Input
            input_str = input("\nEnter velocities (linear angular): ")

            # Check for exit command
            if input_str.lower() == 'x':
                print("Stopping interface...")
                break

            # Parse the input
            inputs = input_str.split()

            if len(inputs) != 2:
                print("Error: Please enter exactly two numbers separated by a space.")
                continue

            try:
                lin = float(inputs[0])
                ang = float(inputs[1])
                
                # 4. Use the node to publish the velocity
                commander.send_velocity(lin, ang)
                
            except ValueError:
                print("Error: Input must be numeric.")

    except KeyboardInterrupt:
        pass
    finally:
        # 5. Stop the robot before exiting
        commander.send_velocity(0.0, 0.0)
        
        # Cleanup
        commander.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    user_interface()