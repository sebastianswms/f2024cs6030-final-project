#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from time import time
from ackermann_msgs.msg import AckermannDrive

# Author: Sebastian Smiley

class CustomControlNode(Node):

    def __init__(self) -> None:
        super().__init__('custom_control_node')
        self.publisher = self.create_publisher(AckermannDrive, '/carla/ego_vehicle/ackermann_cmd', 10)
        self.start_time = time()
        self.timer = self.create_timer(0.1, self.timer_callback)

    def degrees_to_radians(self, degrees) -> float:
        return degrees * math.pi / 180
    
    def kph_to_mps(self, kph) -> float:
        return kph / 3.6

    def timer_callback(self):
        current_time = time() - self.start_time
        msg = AckermannDrive()
        start_turn = 7.7
        turn_time = 2.8
        continue_time = 2
        end_turn = start_turn + turn_time
        stop_time = end_turn + continue_time
        if current_time <= start_turn:
            steering_angle_degrees = 0
            speed_kph = 30

            msg.steering_angle = self.degrees_to_radians(steering_angle_degrees)
            msg.speed = self.kph_to_mps(speed_kph)
        elif current_time <= end_turn:
            steering_angle_degrees = -30
            speed_kph = 20

            msg.steering_angle = self.degrees_to_radians(steering_angle_degrees)
            msg.speed = self.kph_to_mps(speed_kph)
        elif current_time <= stop_time:
            steering_angle_degrees = 0
            speed_kph = 30

            msg.steering_angle = self.degrees_to_radians(steering_angle_degrees)
            msg.speed = self.kph_to_mps(speed_kph)   
        else:
            steering_angle_degrees = 0
            speed_kph = 0

            msg.steering_angle = self.degrees_to_radians(steering_angle_degrees)
            msg.speed = self.kph_to_mps(speed_kph)
        self.publisher.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    custom_control_node = CustomControlNode()
    rclpy.spin(custom_control_node)
    custom_control_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
    