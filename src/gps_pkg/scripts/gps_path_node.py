#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
import math

class GpsPathNode(Node):
    def __init__(self):
        super().__init__('gps_path_node')
        
        self.subscription = self.create_subscription(
            NavSatFix,
            'gps/fix',
            self.gps_callback,
            10)
            
        self.path_publisher = self.create_publisher(Path, 'gps/path', 10)
        
        self.path_msg = Path()
        self.path_msg.header.frame_id = "map"
        
        self.origin_lat = None
        self.origin_lon = None
        
        # Earth radius in meters
        self.R = 6378137.0 

    def gps_callback(self, msg):
        if msg.status.status < 0:
            return # Invalid fix

        if self.origin_lat is None:
            self.origin_lat = msg.latitude
            self.origin_lon = msg.longitude
            self.get_logger().info(f"Origin set to: {self.origin_lat}, {self.origin_lon}")

        # Convert to X, Y (meters) relative to origin using Equirectangular projection
        # Valid for small distances
        dlat = math.radians(msg.latitude - self.origin_lat)
        dlon = math.radians(msg.longitude - self.origin_lon)
        
        # Average latitude for scaling longitude
        lat_avg = math.radians((msg.latitude + self.origin_lat) / 2.0)
        
        x = self.R * dlon * math.cos(lat_avg)
        y = self.R * dlat
        
        # Create PoseStamped
        pose = PoseStamped()
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.header.frame_id = "map"
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = msg.altitude
        
        # Identity orientation (could use magnetometer for heading if fused)
        pose.pose.orientation.w = 1.0
        
        self.path_msg.header.stamp = self.get_clock().now().to_msg()
        self.path_msg.poses.append(pose)
        
        self.path_publisher.publish(self.path_msg)

def main(args=None):
    rclpy.init(args=args)
    node = GpsPathNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
