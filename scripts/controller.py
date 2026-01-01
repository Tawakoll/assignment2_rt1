#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from assignment2_rt1.msg import RobotStatus
import sys

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        
        # 1. Publisher to the real robot
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 2. Subscriber to Robot Status (updates distance & threshold)
        self.create_subscription(RobotStatus, '/robot_status', self.process_robot_status, 10)
        
        # 3. Subscriber to User Input (receives the requested velocity)
        self.create_subscription(Twist, '/user_request', self.process_user_request, 10)
        
        # Internal variables (initialized to defaults)
        self.distanceToObstacle = float('inf')
        self.safety_threshold = 1.0
        self.direction = "Unknown"
        
        #defined linear and angular velocities local to class since they will be used by multiple functions
        self.linear = 0.0
        self.angular = 0.0

        self.get_logger().info('Controller node started. Waiting for input...')

    def process_robot_status(self, msg):
        #uses distance reading from laser_status node published to /robot_status then controlls robot accordingly
        self.distanceToObstacle = msg.distance
        self.safety_threshold = msg.threshold
        self.direction = msg.direction
        twist_msg = Twist()
        # Log status (throttled to avoid spamming)
        self.get_logger().info(f'Received Robot Status - Distance: {msg.distance:.2f}, Direction: {msg.direction}, Threshold: {msg.threshold}')
            #  throttle_duration_sec=2.0

        #added 1.1 factor to safety threshold to give some buffer room 10% increase in threashold, because by testing the robot was stopping a bit late, due to latency, even when I directly plugged this logic in the laser_stauts script
        if self.distanceToObstacle <= 1.1* self.safety_threshold:
            self.get_logger().warn(f'Obstacle too close! Distance: {self.distanceToObstacle:.2f}m. GOING BACK!')

            # Stop movement (Logic from your code: set to 0.0)
            twist_msg.linear.x = 0.0  
            twist_msg.angular.z = 0.0

        elif self.distanceToObstacle > 1.1*self.safety_threshold:
            # Move forward
            twist_msg.linear.x = float(self.linear)
            twist_msg.angular.z = float(self.angular)            
        else:
            self.get_logger().info(f'error in process_robot_status logic')
        
        self.publisher_.publish(twist_msg)
        self.get_logger().info(f'Publishing: Linear={twist_msg.linear.x}, Angular={twist_msg.angular.z}')


    def process_user_request(self, msg_in):
        """
        Triggered when the Input Node sends a command.
        """
        self.linear = msg_in.linear.x
        self.angular = msg_in.angular.z

     



def main(args=None):
    rclpy.init(args=args)
    controller = Controller()
    
    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        # if program stops stop the robot
        stop_msg = Twist()
        controller.publisher_.publish(stop_msg)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()