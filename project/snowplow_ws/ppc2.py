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

        # PID
        self.previous_time = time.time()
        self.steering_integral = 0.0
        self.steering_previous_error = 0.0
        self.speed_integral = 0.0
        self.speed_previous_error = 0.0




        # =================
        # TUNING PARAMETERS
        # =================

        # PID controller gains for steering
        self.steering_kp = 3
        self.steering_ki = 0.0
        self.steering_kd = 0.0

        # PID controller gains for speed
        self.speed_kp = 0.8
        self.speed_ki = 0.0
        self.speed_kd = 0.1

        # Other tuning parameters
        self.target_speed_kph = 50



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

        with open("odometry_callback.log", "w") as f:
            pass

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

        log_entry = "*** Yaw Calculation ***\n"

        if self.vehicle is None:
            self.logger.warn("Vehicle reference is not set.")
            return
        
        # Get current time and compute delta time
        current_time = time.time()
        delta_time = current_time - self.previous_time
        if delta_time <= 0.0:
            delta_time = 1e-3  # Prevent division by zero

        # Current vehicle position
        self.vehicle_x = msg.pose.pose.position.x
        self.vehicle_y = msg.pose.pose.position.y
        log_entry += f"Vehicle X: {self.vehicle_x}\nVehicle Y: {self.vehicle_y}\n"

        # Current vehicle yaw (assuming yaw is in radians)
        # Convert quaternion to Euler angles if necessary
        orientation = msg.pose.pose.orientation
        log_entry += f"Raw Orientation: {orientation}\n"

        current_yaw = self.quaternion_to_yaw(orientation)
        log_entry += f"Calculated Yaw: {current_yaw}\n"

        # Get next waypoints
        waypoints_x, waypoints_y = self.waypoint_manager.next_waypoints(
            self.last_waypoint_index, length=30
        )

        if len(waypoints_x) == 0:
            self.logger.warn("No waypoints available.")
            return

        # Find the closest waypoint index
        closest_index = self.find_closest_waypoint(waypoints_x, waypoints_y, self.vehicle_x, self.vehicle_y)

        log_entry += f"Last Waypoint Index: {self.last_waypoint_index}\n"

        # Update last_waypoint_index
        self.last_waypoint_index += closest_index
        if self.last_waypoint_index >= self.waypoint_manager.num_waypoints:
            raise RuntimeError("Waypoints went overboard")

        # Determine desired yaw based on the waypoint before the target waypoint
        target_waypoint_index = self.last_waypoint_index + closest_index
        if target_waypoint_index == 0:
            target_waypoint_index += 1
            self.last_waypoint_index += 1
        previous_waypoint_index = target_waypoint_index - 1

        log_entry += f"Target Waypoint Index: {target_waypoint_index}\n"
        log_entry += f"Previous Waypoint Index: {previous_waypoint_index}\n"
        log_entry += f"Target Waypoint: X={self.waypoint_manager.waypoints_x[target_waypoint_index]}, Y={self.waypoint_manager.waypoints_y[target_waypoint_index]}\n"
        log_entry += f"Previous Waypoint: X={self.waypoint_manager.waypoints_x[previous_waypoint_index]}, Y={self.waypoint_manager.waypoints_y[previous_waypoint_index]}\n"

        desired_yaw = math.atan2(
            self.waypoint_manager.waypoints_y[target_waypoint_index] - self.waypoint_manager.waypoints_y[previous_waypoint_index],
            self.waypoint_manager.waypoints_x[target_waypoint_index] - self.waypoint_manager.waypoints_x[previous_waypoint_index]
        )

        log_entry += f"Computed Desired Yaw: {desired_yaw}\n"

        # Compute yaw error
        yaw_error = -1 * self.normalize_angle(desired_yaw - current_yaw)

        log_entry += f"Computed Yaw Error: {yaw_error}\n"
        log_entry += f"\n*** Steering PID Calculation ***\n"
        log_entry += f"Delta Time: {delta_time}\n"

        # Update integral and derivative
        self.steering_integral += yaw_error * delta_time
        steering_derivative = (yaw_error - self.steering_previous_error) / delta_time

        log_entry += f"Steering P (aka Yaw Error): {yaw_error}\n"
        log_entry += f"Steering I: {self.steering_integral}\n"
        log_entry += f"Steering D: {steering_derivative}\n"

        # Compute steering using PID
        original_steering = (
            self.steering_kp * yaw_error +
            self.steering_ki * self.steering_integral +
            self.steering_kd * steering_derivative
        )
        clipped_steering = np.clip(original_steering, -1.0, 1.0)
        self.control.steer = clipped_steering

        log_entry += f"Steering P Contribution: {self.steering_kp * yaw_error}\n"
        log_entry += f"Steering I Contribution: {self.steering_ki * self.steering_integral}\n"
        log_entry += f"Steering D Contribution: {self.steering_kd * steering_derivative}\n"
        log_entry += f"Calculated Steering: {original_steering}\n"
        log_entry += f"Clipped Steering: {clipped_steering}\n"

        target_speed_mps = self.target_speed_kph / 3.6

        log_entry += f"\n*** Speed PID Calculation ***\n"
        log_entry += f"Delta Time (again): {delta_time}\n"
        log_entry += f"Target Speed: {target_speed_mps} m/s\n"

        current_velocity = self.vehicle.get_velocity()
        current_speed = np.sqrt(current_velocity.x ** 2 + current_velocity.y ** 2 + current_velocity.z ** 2)

        speed_error = target_speed_mps - current_speed

        # Update integral and derivative
        self.speed_integral += speed_error * delta_time
        speed_derivative = (speed_error - self.speed_previous_error) / delta_time

        log_entry += f"Speed P: {speed_error}\n"
        log_entry += f"Speed I: {self.speed_integral}\n"
        log_entry += f"Speed D: {speed_derivative}\n"

        # Compute steering using PID
        original_throttle = (
            self.speed_kp * speed_error +
            self.speed_ki * self.speed_integral +
            self.speed_kd * speed_derivative
        )
        clipped_throttle = np.clip(original_throttle, 0, 1.0)
        self.control.throttle = clipped_throttle

        log_entry += f"Speed P Contribution: {self.speed_kp * speed_error}\n"
        log_entry += f"Speed I Contribution: {self.speed_ki * self.speed_integral}\n"
        log_entry += f"Speed D Contribution: {self.speed_kd * speed_derivative}\n"
        log_entry += f"Calculated Throttle: {original_steering}\n"
        log_entry += f"Clipped Throttle: {clipped_steering}\n"

        # Update previous error and time for next iteration
        self.steering_previous_error = yaw_error
        self.speed_previous_error = speed_error
        self.previous_time = current_time

        with open("odometry_callback.log", "a") as f:
            log_entry += "\n---\n\n"
            f.write(log_entry)

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