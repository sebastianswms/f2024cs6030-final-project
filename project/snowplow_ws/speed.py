#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, Bool
from ackermann_msgs.msg import AckermannDrive

class SpeedControlNode(Node):
    def __init__(self):
        super().__init__('combined_control_node')

        # Publishers
        self.control_publisher = self.create_publisher(AckermannDrive, '/carla/ego_vehicle/ackermann_cmd', 10)

        # Subscribers
        self.create_subscription(
            Float32, 
            '/lane_center_offset',
            self.control_steering,
            10,
        )
        self.create_subscription(
            Bool, '/stop_signal', self.control_speed, 10)

        # PID Gains for Speed Control
        self.kp_speed = 0.1
        self.ki_speed = 0.01
        self.kd_speed = 0.05
        self.integral_speed = 0.0
        self.previous_error_speed = 0.0

        # Variables for speed and steering control
        self.lane_offset = 0.0
        self.stop_flag = False
        self.current_speed = 10.0  # Default speed in m/s

        # Steering control parameters
        self.steering_gain = 0.5

        # Timer to publish the control command
        self.timer = self.create_timer(0.1, self.publish_control_command)

    def control_speed(self, msg):
        """Callback for updating the stop signal for longitudinal control."""
        self.stop_flag = msg.data
        self.get_logger().info(f'Received stop signal: {self.stop_flag}')

    def pid_speed_control(self, target_speed, current_speed):
        """PID control for speed adjustment."""
        error = target_speed - current_speed
        self.integral_speed += error
        derivative = error - self.previous_error_speed
        self.previous_error_speed = error

        # Compute control output
        output = (self.kp_speed * error) + (self.ki_speed * self.integral_speed) + (self.kd_speed * derivative)
        return max(0.0, min(1.0, output))  # Clamp throttle between 0 and 1

    def publish_control_command(self):
        """Publish Ackermann drive message with both speed and steering angle."""
        drive_msg = AckermannDrive()

        # Handle steering control
        steering_angle = -self.lane_offset * self.steering_gain
        drive_msg.steering_angle = steering_angle

        # Adjust speed based on steering angle
        if abs(steering_angle) > 0.3:  # Reduce speed on sharp turns
            target_speed = 5.0
        else:
            target_speed = 10.0

        # Handle stop signal
        if self.stop_flag:
            drive_msg.speed = 0.0
        else:
            throttle = self.pid_speed_control(target_speed, self.current_speed)
            drive_msg.speed = throttle * target_speed

        # Log the control command
        self.get_logger().info(f'Publishing control: speed={drive_msg.speed}, steering_angle={drive_msg.steering_angle}')

        # Publish the control message
        self.control_publisher.publish(drive_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SpeedControlNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()