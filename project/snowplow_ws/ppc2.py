#!/usr/bin/env python3
import math
import pandas as pd
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from carla_msgs.msg import CarlaActorList, CarlaEgoVehicleControl
from std_msgs.msg import Bool
from ackermann_msgs.msg import AckermannDrive
import carla
import numpy as np
import time

class SimpleController(Node):
    """
    A ROS2 node that controls a CARLA vehicle using waypoints and stops when an obstacle is
    detected.
    """

    def __init__(self):
        super().__init__('simple_controller')

        # CARLA Client Setup
        self.client = carla.Client('localhost', 2000)
        self.client.set_timeout(2.0)
        self.world = self.client.get_world()
        self.map = self.world.get_map()

        self.vehicle = None
        self.control = carla.VehicleControl()
        self.control.steer = 0.0 # Default steering value
        self.control.throttle = 0.5 # Default throttle value

        self.waypoints_lookahead = 30
        self.waypoint_manager = WaypointManager()
        self.last_waypoint_index = 0

        # PID controller gains for steering
        self.steering_kp = 0.8
        self.steering_ki = 0.0
        self.steering_kd = 0.1
        self.steering_integral = 0.0
        self.steering_previous_error = 0.0
        self.steering_previous_time = time.time()

        # PID controller gains for speed
        self.speed_kp = 0.8
        self.speed_ki = 0.0
        self.speed_kd = 0.1
        self.speed_integral = 0.0
        self.speed_previous_error = 0.0
        self.speed_previous_time = time.time()

        # Publishers
        self.control_publisher = self.create_publisher(AckermannDrive, '/carla/ego_vehicle/ackermann_cmd', 10)

        # Subscribers
        self.create_subscription(
            CarlaActorList,
            '/carla/actor_list',
            self.actor_list_callback,
            10,
        )
        self.create_subscription(
            Odometry,
            '/carla/ego_vehicle/odometry',
            self.odometry_callback,
            10,
        )
        self.create_subscription(
            CarlaEgoVehicleControl,
            '/carla/ego_vehicle/vehicle_status',
            self.vehicle_status_callback,
            10,
        )
        self.create_subscription(
            Bool,
            '/carla/ego_vehicle/obstacle_detected',
            self.obstacle_callback,
            10
        )

        # Timer to send control commands at regular intervals
        self.timer = self.create_timer(0.05, self.publish_control_command)
        self.logger.info('Simple Controller Node Initialized')

    @property
    def logger(self):
        return self.get_logger()

    def actor_list_callback(self, msg):
        """
        Callback to get the ego vehicle from the actor list.
        """
        for actor in msg.actors:
            if actor.rolename == 'ego_vehicle':
                if self.vehicle is None:
                    self.logger.info(f'Ego vehicle found: {actor.id}')
                else:
                    self.logger.debug(f'ego vehicle found: {actor.id}')

                self.vehicle = self.world.get_actor(actor.id)

    def odometry_callback(self, msg):
        """
        Callback to receive the vehicle's odometry and compute control commands.

        Goal is to update self.control.steer and self.control.throttle
        """
        if self.vehicle is None:
            self.logger.warn("Vehicle reference is not set.")
            return

        # Current vehicle position
        self.vehicle_x = msg.pose.pose.position.x
        self.vehicle_y = msg.pose.pose.position.y

        # Current vehicle yaw (assuming yaw is in radians)
        # Convert quaternion to Euler angles if necessary
        orientation = msg.pose.pose.orientation

        current_yaw = self.quaternion_to_yaw(orientation)

        # Get next waypoints
        waypoints_x, waypoints_y = self.waypoint_manager.next_waypoints(
            self.last_waypoint_index, length=30
        )

        if len(waypoints_x) == 0:
            self.logger.warn("No waypoints available.")
            return

        # Find the closest waypoint index
        closest_index = self.find_closest_waypoint(waypoints_x, waypoints_y, self.vehicle_x, self.vehicle_y)

        # Update last_waypoint_index
        self.last_waypoint_index += closest_index
        if self.last_waypoint_index >= self.waypoint_manager.num_waypoints:
            self.last_waypoint_index = 0  # Loop back to start or handle as needed

        # Determine desired yaw based on the waypoint before the target waypoint
        target_waypoint_index = self.last_waypoint_index + closest_index
        if target_waypoint_index == 0:
            previous_waypoint_index = self.waypoint_manager.num_waypoints - 1
        else:
            previous_waypoint_index = target_waypoint_index - 1

        desired_yaw = math.atan2(
            self.waypoint_manager.waypoints_y[target_waypoint_index] - self.waypoint_manager.waypoints_y[previous_waypoint_index],
            self.waypoint_manager.waypoints_x[target_waypoint_index] - self.waypoint_manager.waypoints_x[previous_waypoint_index]
        )

        # Compute yaw error
        yaw_error = self.normalize_angle(desired_yaw - current_yaw)

        # Get current time and compute delta time
        current_time = time.time()
        delta_time = current_time - self.steering_previous_time
        if delta_time <= 0.0:
            delta_time = 1e-3  # Prevent division by zero

        # Update integral and derivative
        self.steering_integral += yaw_error * delta_time
        derivative = (yaw_error - self.steering_previous_error) / delta_time

        # Compute steering using PID
        steering = (
            self.steering_kp * yaw_error +
            self.steering_ki * self.steering_integral +
            self.steering_kd * derivative
        )
        steering = np.clip(steering, -1.0, 1.0)
        self.control.steer = steering

        # For simplicity, set throttle to a constant value
        self.control.throttle = 0.2  # Adjust as needed

        # Update previous error and time for next iteration
        self.steering_previous_error = yaw_error
        self.steering_previous_time = current_time

        self.logger.debug(
            f"Yaw Error: {yaw_error:.4f}, Steering: {steering:.4f}, Throttle: {self.control.throttle}"
        )

    def find_closest_waypoint(self, waypoints_x, waypoints_y, x, y):
        """
        Finds the index of the waypoint closest to the current position.
        """
        distances = np.sqrt((waypoints_x - x) ** 2 + (waypoints_y - y) ** 2)
        closest_index = np.argmin(distances)
        return closest_index

    def quaternion_to_yaw(self, orientation):
        """
        Converts a quaternion into a yaw angle (in radians).
        """
        # Convert quaternion to Euler angles
        # Reference: https://en.wikipedia.org/wiki/Conversion_between_quaternions_and_Euler_angles
        qx = orientation.x
        qy = orientation.y
        qz = orientation.z
        qw = orientation.w

        # Yaw (z-axis rotation)
        siny_cosp = 2.0 * (qw * qz + qx * qy)
        cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        return yaw

    def normalize_angle(self, angle):
        """
        Normalizes the angle to be within [-pi, pi].
        """
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def vehicle_status_callback(self, msg):
        """
        Callback to receive the vehicle's status.
        """

    def obstacle_callback(self, msg):
        """
        Callback to handle obstacle detection.
        """
        obstacle_detected = msg.data
        if obstacle_detected:
            if self.obstacle is False:
                self.logger.info('Obstacle detected! Stopping the vehicle.')
            self.logger.debug("obstacle detected")
            self.obstacle = True
        else:
            if self.obstacle is False:
                self.logger.info('No more obstacles. Resuming vehicle movement!')
            self.logger.debug("no obstacles detected")
            self.obstacle = False

    def publish_control_command(self):
        """
        Publish the control command to the vehicle.
        """
        if self.vehicle is None:
            return
        self.vehicle.apply_control(self.control)
        self.logger.info(f'Applied Control: Steer={self.control.steer},Throttle={self.control.throttle}')


class WaypointManager:
    def __init__(self, filename="waypoints.csv"):
        waypoints = pd.read_csv(filename)
        self.waypoints_x = waypoints['x'].to_numpy()
        self.waypoints_y = waypoints['y'].to_numpy()
        self.num_waypoints = len(self.waypoints_x)

    def next_waypoints(self, last_waypoint_index, length=30):
        """
        Returns the next 'length' waypoints starting from 'last_waypoint_index'.
        Wraps around if the end of the list is reached.
        """
        start = last_waypoint_index
        end = start + length
        if end > self.num_waypoints:
            end = self.num_waypoints
        return (
            self.waypoints_x[start:end],
            self.waypoints_y[start:end]
        )


def main(args=None):
    """
    Main function to run the node.
    """
    rclpy.init(args=args)
    node = SimpleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()