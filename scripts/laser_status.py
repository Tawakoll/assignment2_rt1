#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from assignment2_rt1.msg import RobotStatus

class LaserStatus(Node):

    def __init__(self):
        super().__init__('laser_status')

        self.laser_subscription = self.create_subscription(LaserScan, '/scan', self.process_scan, 10)

        # Self-subscription (now safe from flooding)
        self.robot_subscription = self.create_subscription(RobotStatus, '/robot_status', self.process_robot_status, 10)

        self.robot_publisher = self.create_publisher(RobotStatus, '/robot_status', 10)

        self.safety_threshold = 5.0 

        self.get_logger().info('Laser Status node started.')

    def process_scan(self, msg):
        ranges = msg.ranges
        length = len(ranges)
        
        one_quarter = int(length / 4)
        
        min_distance = float('inf')
        index_of_closest = -1
        
        # Find the minimum distance and its index
        for i, distance in enumerate(ranges):
            if distance < min_distance and distance < msg.range_max:
                min_distance = distance
                index_of_closest = i

        if index_of_closest == -1:
            min_distance = msg.range_max
            direction = "Unknown"
        else:
            if index_of_closest < one_quarter:
                direction = "Back Right"
            elif index_of_closest < (2 * one_quarter):
                direction = "Front Right"
            elif index_of_closest < (3 * one_quarter):
                direction = "Front Left"
            elif index_of_closest < length:
                direction = "Back Left"
            else: 
                 direction = "Unknown"

        status_msg = RobotStatus()
        status_msg.distance = float(min_distance)
        status_msg.direction = str(direction)
        status_msg.threshold = float(self.safety_threshold)

        self.robot_publisher.publish(status_msg)

    def process_robot_status(self, msg):
        # LOGGING THROTTLED: This prints only once every 2.0 seconds
        self.get_logger().info(
            f'Received Robot Status - Distance: {msg.distance:.2f}, Direction: {msg.direction}, Threshold: {msg.threshold}',
            throttle_duration_sec=2.0
        )

def main(args=None):
    rclpy.init(args=args)
    status_node = LaserStatus()
    
    try:
        rclpy.spin(status_node)
    except KeyboardInterrupt:
        pass
    
    status_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()