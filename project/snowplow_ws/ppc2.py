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
import pandas as pd
import numpy as np
from io import StringIO

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
        self.control.throttle = 0.0 # Default throttle value
        self.control.reverse = False  # Start in forward gear

        self.waypoint_manager = WaypointManager()
        self.last_waypoint_index = 0

        self.camera_obstacle = False
        self.lidar_radar_obstacle = False

        # PID
        self.previous_time = time.time()
        self.steering_integral = 0.0
        self.steering_previous_error = 0.0
        self.speed_integral = 0.0
        self.speed_previous_error = 0.0

        # ===================
        # <tuning_parameters>
        # ===================

        # PID controller gains for steering forward
        self.steering_forward_kp = 3
        self.steering_forward_ki = 0.0
        self.steering_forward_kd = 0.0


        # PID controller gains for steering backward
        self.steering_backward_kp = 0.1
        self.steering_backward_ki = 0.0
        self.steering_backward_kd = 0.04


        # PID controller gains for speed
        self.speed_kp = 0.8
        self.speed_ki = 0.0
        self.speed_kd = 0.1


        # Other tuning parameters
        self.target_speed_forward_kph = 30 # Kilometers Per Hour
        self.target_speed_backward_kph = 10 # Kilometers Per Hour

        self.max_waypoints_lookahead = 30 # Number of waypoints to look ahead, NOT distance
        self.min_waypoints_lookahead = 8 # Switch to the next batch of waypoints if less than this number remain

        # ====================
        # </tuning_parameters>
        # ====================

        # Gear management
        self.current_gear = 'forward'  # Can be 'forward' or 'reverse'
        self.switching_gear = False
        self.stop_threshold = 0.1  # Speed below which we consider the vehicle stopped
        self.stop_vehicle = False
        self.batch_index = 0

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
            '/carla/ego_vehicle/camera_obstacle_detected',
            self.camera_obstacle_callback,
            10
        )
        self.create_subscription(
            Bool,
            '/carla/ego_vehicle/lidar_radar_obstacle_detected',
            self.lidar_radar_obstacle_callback,
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

        if self.stop_vehicle:
            return

        if self.vehicle is None:
            self.logger.warn("Vehicle reference is not set.")
            return

        if self.switching_gear:
            self.control.throttle = 0.0
            self.control.brake = 1.0
            self.control.steer = 0.0

            current_velocity = self.vehicle.get_velocity()
            current_speed = np.sqrt(current_velocity.x ** 2 + current_velocity.y ** 2 + current_velocity.z ** 2)
            if current_speed < self.stop_threshold:
                if self.current_gear == 'forward':
                    self.current_gear = 'reverse'
                    self.control.reverse = True
                else:
                    self.current_gear = 'forward'
                    self.control.reverse = False
                self.logger.info(f'Switched to {self.current_gear} gear.')

                self.last_waypoint_index = 0

                if self.batch_index >= self.waypoint_manager.get_total_batches():
                    self.logger.info('No more batches. Stopping vehicle.')
                    self.control.throttle = 0.0
                    self.control.brake = 1.0
                    self.control.steer = 0.0
                    self.stop_vehicle = True
                else:
                    self.logger.info(f'Switched to batch {self.batch_index}')
                self.switching_gear = False
            else:
                return

        log_entry = "*** Yaw Calculation ***\n"
        
        # Get current time and compute delta time
        current_time = time.time()
        delta_time = current_time - self.previous_time
        if delta_time <= 0.0:
            delta_time = 1e-3  # Prevent division by zero

        self.vehicle_x = msg.pose.pose.position.x
        self.vehicle_y = msg.pose.pose.position.y
        log_entry += f"Vehicle X: {self.vehicle_x}\nVehicle Y: {self.vehicle_y}\n"

        orientation = msg.pose.pose.orientation
        log_entry += f"Raw Orientation: {orientation}\n"

        current_yaw = self.quaternion_to_yaw(orientation)
        log_entry += f"Calculated Yaw: {current_yaw}\n"

        waypoints_x, waypoints_y, next_batch = self.waypoint_manager.next_waypoints(
            self.batch_index,
            self.last_waypoint_index,
            self.max_waypoints_lookahead,
            self.min_waypoints_lookahead,
        )

        if len(waypoints_x) == 0:
            self.logger.warn("No waypoints available.")
            self.batch_index += 1
            self.switching_gear = True
            return

        closest_index = self.find_closest_waypoint(waypoints_x, waypoints_y, self.vehicle_x, self.vehicle_y)

        log_entry += f"Last Waypoint Index: {self.last_waypoint_index}\n"

        self.last_waypoint_index += closest_index
        if self.last_waypoint_index >= self.waypoint_manager.get_num_waypoints(self.batch_index):
            raise RuntimeError("Waypoints went overboard")

        # Determine desired yaw based on the waypoint before the target waypoint
        target_waypoint_index = self.last_waypoint_index + closest_index
        if target_waypoint_index == 0:
            target_waypoint_index += 1
            self.last_waypoint_index += 1
        previous_waypoint_index = target_waypoint_index - 1

        if target_waypoint_index >= self.waypoint_manager.get_num_waypoints(self.batch_index):
            self.logger.warn("Target waypoint out of range, switching to next batch.")
            self.batch_index += 1
            self.switching_gear = True
            return

        log_entry += f"Target Waypoint Index: {target_waypoint_index}\n"
        log_entry += f"Previous Waypoint Index: {previous_waypoint_index}\n"
        log_entry += f"Target Waypoint: X={self.waypoint_manager.get_waypoints_x(self.batch_index)[target_waypoint_index]}, Y={self.waypoint_manager.get_waypoints_y(self.batch_index)[target_waypoint_index]}\n"
        log_entry += f"Previous Waypoint: X={self.waypoint_manager.get_waypoints_x(self.batch_index)[previous_waypoint_index]}, Y={self.waypoint_manager.get_waypoints_y(self.batch_index)[previous_waypoint_index]}\n"

        desired_yaw = math.atan2(
            self.waypoint_manager.get_waypoints_y(self.batch_index)[target_waypoint_index] - self.waypoint_manager.get_waypoints_y(self.batch_index)[previous_waypoint_index],
            self.waypoint_manager.get_waypoints_x(self.batch_index)[target_waypoint_index] - self.waypoint_manager.get_waypoints_x(self.batch_index)[previous_waypoint_index]
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

        
        if self.current_gear == "forward":
            original_steering = (
                self.steering_forward_kp * yaw_error +
                self.steering_forward_ki * self.steering_integral +
                self.steering_forward_kd * steering_derivative
            )
            clipped_steering = np.clip(original_steering, -1.0, 1.0)
            self.control.steer = clipped_steering

            log_entry += f"Steering P Contribution: {self.steering_forward_kp * yaw_error}\n"
            log_entry += f"Steering I Contribution: {self.steering_forward_ki * self.steering_integral}\n"
            log_entry += f"Steering D Contribution: {self.steering_forward_kd * steering_derivative}\n"
            log_entry += f"Calculated Steering: {original_steering}\n"
            log_entry += f"Clipped Steering: {clipped_steering}\n"

            target_speed_mps = self.target_speed_forward_kph / 3.6
        else:
            original_steering = (
                self.steering_backward_kp * yaw_error +
                self.steering_backward_ki * self.steering_integral +
                self.steering_backward_kd * steering_derivative
            )
            clipped_steering = np.clip(original_steering, -1.0, 1.0)
            self.control.steer = clipped_steering

            log_entry += f"Steering P Contribution: {self.steering_backward_kp * yaw_error}\n"
            log_entry += f"Steering I Contribution: {self.steering_backward_ki * self.steering_integral}\n"
            log_entry += f"Steering D Contribution: {self.steering_backward_kd * steering_derivative}\n"
            log_entry += f"Calculated Steering: {original_steering}\n"
            log_entry += f"Clipped Steering: {clipped_steering}\n"

            target_speed_mps = self.target_speed_backward_kph / 3.6

        

        log_entry += f"\n*** Speed PID Calculation ***\n"
        log_entry += f"Delta Time (again): {delta_time}\n"
        log_entry += f"Target Speed: {target_speed_mps} m/s\n"

        current_velocity = self.vehicle.get_velocity()
        current_speed = np.sqrt(current_velocity.x ** 2 + current_velocity.y ** 2 + current_velocity.z ** 2)

        obstacle_detected = self.camera_obstacle or self.lidar_radar_obstacle
        if obstacle_detected:
            speed_error = -1 * current_speed
        else:
            speed_error = target_speed_mps - current_speed

        log_entry += f"Obstacle Detected? {obstacle_detected}\n"

        # Update integral and derivative
        self.speed_integral += speed_error * delta_time
        speed_derivative = (speed_error - self.speed_previous_error) / delta_time

        log_entry += f"Speed P: {speed_error}\n"
        log_entry += f"Speed I: {self.speed_integral}\n"
        log_entry += f"Speed D: {speed_derivative}\n"

        # Compute throttle and brake using PID
        original_throttle_brake = (
            self.speed_kp * speed_error +
            self.speed_ki * self.speed_integral +
            self.speed_kd * speed_derivative
        )
        clipped_throttle = np.clip(original_throttle_brake, 0, 1.0)
        clipped_brake = -1 * np.clip(original_throttle_brake, -1.0, 0)
        self.control.throttle = clipped_throttle
        self.control.brake = clipped_brake

        log_entry += f"Speed P Contribution: {self.speed_kp * speed_error}\n"
        log_entry += f"Speed I Contribution: {self.speed_ki * self.speed_integral}\n"
        log_entry += f"Speed D Contribution: {self.speed_kd * speed_derivative}\n"
        log_entry += f"Calculated Throttle/Brake: {original_throttle_brake}\n"
        log_entry += f"Clipped Throttle: {clipped_throttle}\n"
        log_entry += f"Clipped Brake: {clipped_brake}\n"

        # Update previous error and time for next iteration
        self.steering_previous_error = yaw_error
        self.speed_previous_error = speed_error
        self.previous_time = current_time

        if next_batch:
            self.batch_index += 1
            self.switching_gear = True

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

    def camera_obstacle_callback(self, msg):
        obstacle_detected = msg.data
        if obstacle_detected:
            if self.camera_obstacle is False:
                self.logger.info('Camera obstacle detected! Stopping the vehicle.')
            self.logger.debug("camera obstacle detected")
            self.camera_obstacle = True
        else:
            if self.camera_obstacle is True:
                self.logger.info('No more camera obstacles.')
            self.logger.debug("no camera obstacles detected")
            self.camera_obstacle = False

    def lidar_radar_obstacle_callback(self, msg):
        obstacle_detected = msg.data
        if obstacle_detected:
            if self.lidar_radar_obstacle is False:
                self.logger.info('Lidar/radar obstacle detected! Stopping the vehicle.')
            self.logger.debug("lidar/radar obstacle detected")
            self.lidar_radar_obstacle = True
        else:
            if self.lidar_radar_obstacle is True:
                self.logger.info('No more lidar/radar obstacles.')
            self.logger.debug("no lidar/radar obstacles detected")
            self.lidar_radar_obstacle = False

    def publish_control_command(self):
        """
        Publish the control command to the vehicle.
        """
        if self.vehicle is None:
            return
        self.vehicle.apply_control(self.control)
        self.logger.info(f'Applied Control: Steer={self.control.steer},Throttle={self.control.throttle},Brake={self.control.brake}')

class WaypointManager:
    def __init__(self, filename="waypoints.csv", delimiter="==="):
        self.batches = []  # List to store each batch's waypoints
        self._load_batches(filename, delimiter)
    
    def _load_batches(self, filename, delimiter):
        with open(filename, 'r') as file:
            content = file.read()
        
        for raw_batch in content.split(delimiter):
            raw_batch = raw_batch.strip()
            if not raw_batch:
                continue
            batch_df = pd.read_csv(StringIO(raw_batch))                
            waypoints_x = batch_df['x'].to_numpy()
            waypoints_y = batch_df['y'].to_numpy()
            num_waypoints = min(len(waypoints_x), len(waypoints_y))
            batch_data = {
                'waypoints_x': waypoints_x,
                'waypoints_y': waypoints_y,
                'num_waypoints': num_waypoints
            }
            self.batches.append(batch_data)
    
    def get_waypoints_x(self, batch_index):
        return self._get_batch_data(batch_index, 'waypoints_x')
    
    def get_waypoints_y(self, batch_index):
        return self._get_batch_data(batch_index, 'waypoints_y')
    
    def get_num_waypoints(self, batch_index):  # batch_index must be int
        return self._get_batch_data(batch_index, 'num_waypoints')
    
    def _get_batch_data(self, batch_index, key):
        if batch_index >= self.get_total_batches():
            raise IndexError(f"Batch index {batch_index} is out of range. Total batches: {len(self.batches)}.")
        return self.batches[batch_index][key]
    
    def get_total_batches(self):
        return len(self.batches)

    def next_waypoints(self, batch_index, last_waypoint_index, length, min_waypoints):
        """
        Returns the next 'length' waypoints starting from 'last_waypoint_index'.
        Truncates if the end of the list is reached.
        Switches gears and changes to the next batch if remaining waypoints < min_waypoints
        """
        start = last_waypoint_index
        end = start + length
        if end > self.get_num_waypoints(batch_index):
            end = self.get_num_waypoints(batch_index)

        next_batch = end < min_waypoints

        return (
            self.get_waypoints_x(batch_index)[start:end],
            self.get_waypoints_y(batch_index)[start:end],
            next_batch,
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
