#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import math
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2
from std_msgs.msg import Header, Bool
import message_filters

class SensorFusionNode(Node):
    def __init__(self):
        super().__init__('sensor_fusion_node')

        # Subscriber
        self.lidar_sub = message_filters.Subscriber(self, PointCloud2, '/carla/ego_vehicle/lidar')
        self.radar_sub = message_filters.Subscriber(self, PointCloud2, '/carla/ego_vehicle/radar_front')

        # Time synchronizer to sync lidar and radar messages
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [self.lidar_sub, self.radar_sub],
            queue_size=10,
            slop=0.1
        )
        self.ts.registerCallback(self.fuse_callback)

        # Publishers for the filtered lidar, radar, and fused point cloud data
        self.lidar_filtered_pub = self.create_publisher(PointCloud2, '/carla/ego_vehicle/lidar_filtered', 10)
        self.radar_filtered_pub = self.create_publisher(PointCloud2, '/carla/ego_vehicle/radar_filtered', 10)
        self.fused_pub = self.create_publisher(PointCloud2, '/carla/ego_vehicle/fused_objects', 10)

        # Stop flag publisher
        self.stop_publisher = self.create_publisher(Bool, '/carla/ego_vehicle/stop_flag', 10)

        # Distance threshold for stop flag (in meters)
        self.stop_distance_threshold = 5.0  # Stop if an object is closer than 5 meters

    def fuse_callback(self, lidar_msg, radar_msg):
        # Extract the lidar and radar points
        lidar_points = list(point_cloud2.read_points(lidar_msg, field_names=('x', 'y', 'z', 'intensity'), skip_nans=True))
        radar_points = list(point_cloud2.read_points(radar_msg, field_names=('x', 'y', 'z'), skip_nans=True))

        # Define sensor height offsets
        lidar_z_offset = 2.4
        radar_x_offset = 2.0
        radar_z_offset = 2.0

        # Filter lidar points to only include those within a 60-degree frontal cone (-30 to +30 degrees)
        lidar_filtered = [
            (x, y, z + lidar_z_offset, intensity) for x, y, z, intensity in lidar_points
            if -math.radians(30) <= math.atan2(y, x) <= math.radians(30)
        ]

        # Adjust the z-coordinate of the radar points by adding the radar_z_offset
        radar_filtered = [
            (x + radar_x_offset, y, z + radar_z_offset) for x, y, z in radar_points
            if math.sqrt(x**2 + y**2 + z**2) <= 20.0  # Only include radar points within 20 meters
        ]

        # Fuse Lidar and Radar Data
        fused_points = lidar_filtered + [(x, y, z, 0.0) for x, y, z in radar_filtered]

        # Check for stop flag: if any object is within the stop distance threshold
        stop_detected = False
        for point in lidar_filtered + radar_filtered:
            distance = math.sqrt(point[0]**2 + point[1]**2 + point[2]**2)
            if distance < self.stop_distance_threshold:
                stop_detected = True
                break  # Stop flag triggered, no need to check further

        if stop_detected:
            stop_msg = Bool()
            stop_msg.data = True
            self.stop_publisher.publish(stop_msg)
            self.get_logger().info('Object detected within stop distance! Stop flag published.')

        # Create a ROS 2 Header
        header = Header()
        header.stamp = self.get_clock().now().to_msg()
        header.frame_id = 'ego_vehicle'

        # Create and Publish PointCloud2 Messages
        lidar_filtered_cloud = point_cloud2.create_cloud(header, lidar_msg.fields, lidar_filtered)
        self.lidar_filtered_pub.publish(lidar_filtered_cloud)

        radar_filtered_cloud = point_cloud2.create_cloud_xyz32(header, radar_filtered)
        self.radar_filtered_pub.publish(radar_filtered_cloud)

        fused_cloud = point_cloud2.create_cloud(header, lidar_msg.fields, fused_points)
        self.fused_pub.publish(fused_cloud)

def main(args=None):
    rclpy.init(args=args)
    node = SensorFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
