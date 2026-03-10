#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from assignment2_rt1.msg import RobotStatus
from assignment2_rt1.srv import GetAvg
from collections import deque #  for storing the last 5 velocity commands
import sys

class Controller(Node):
    def __init__(self):
        super().__init__('controller')
        
        # 1. Publisher to the real robot
        self.velocityPublisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # 2. Subscriber to Robot Status custom message (contains distance & threshold)
        self.create_subscription(RobotStatus, '/robot_status', self.process_robot_status, 10)
       
        # 3. Subscriber to User Input commands
        self.create_subscription(Twist, '/user_request', self.process_user_request, 10)

        #4. server to GetAvg service
        self.avg_server = self.create_service(GetAvg, 'get_avg_service', self.get_avg_callback)
        
        # Create a deque to store the last 5 inputs
        self.input_history = deque(maxlen=5)
       
        # Internal variables (initialized to defaults)
        self.distanceToObstacle = float('inf')
        self.safety_threshold = 1.0
        self.direction = "Unknown"
        self.isReversing = False
        self.resetUserCommand = False
        
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
        # Log status (throttled to avoid spamming) throttle_duration_sec=2.0

        self.get_logger().info(f'Received Robot Status - Distance: {msg.distance:.2f}, Direction: {msg.direction}, Threshold: {msg.threshold}')


        #logic to contorl the robot, once distance to obstacle is less than or equal to the safety threshold:
        # robot goal is to move back to a previous safe position. 
        #first stop the robot, , save the current position as the initial position, take the last safe position as the target position.

        #added 1.1 factor to safety threshold to give some buffer room 10% increase in threashold, 
        # -- because by testing the robot was stopping a bit late, due to latency, even when I directly plugged this logic in the laser_stauts script
        if not self.isReversing and self.distanceToObstacle <= 1.1* self.safety_threshold:
            self.get_logger().warn(f'Obstacle too close! Distance: {self.distanceToObstacle:.2f}m. GOING BACK!')

            twist_msg.linear.x = 0.0  
            twist_msg.angular.z = 0.0
            #stop the robot first then reverse
            self.isReversing = True

            #ensure that after reversing the robot does not continue with previous user linear and angular velocoties
            self.resetUserCommand = True
        elif self.isReversing and self.distanceToObstacle <= 1.1* self.safety_threshold :
            #  reversing
            twist_msg.linear.x = - float(self.linear)
            twist_msg.angular.z = - float(self.angular)  

        elif self.distanceToObstacle > 1.1*self.safety_threshold:
            # Move forward
            #stop reversing once safe distance is reached
            self.isReversing = False

            if self.resetUserCommand:
                #reset user command to zero after reversing
                self.linear = 0.0
                self.angular = 0.0
                self.resetUserCommand = False

            twist_msg.linear.x = float(self.linear)
            twist_msg.angular.z = float(self.angular)            
        else:
            self.get_logger().info(f'error in process_robot_status logic')
        
        self.velocityPublisher.publish(twist_msg)
        self.get_logger().info(f'Publishing: Linear={twist_msg.linear.x}, Angular={twist_msg.angular.z}')


    def process_user_request(self, msg_in):
        """
        Triggered when the Input Node sends a command.
        """
        self.linear = msg_in.linear.x
        self.angular = msg_in.angular.z
        
        # Append the new command to history
        self.input_history.append(msg_in)

    # Service callback to compute average of last 5 user inputs
    def get_avg_callback(self, request, response):
        if not self.input_history:
            response.avg_linear = 0.0
            response.avg_angular = 0.0
            return response
            
        sum_linear = sum(cmd.linear.x for cmd in self.input_history)
        sum_angular = sum(cmd.angular.z for cmd in self.input_history)
        count = len(self.input_history)

        response.avg_linear = sum_linear / count
        response.avg_angular = sum_angular / count
        
        self.get_logger().info(f'Returning Avg - Linear: {response.avg_linear}, Angular: {response.avg_angular}')
        return response    
 

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
        controller.velocityPublisher.publish(stop_msg)
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()