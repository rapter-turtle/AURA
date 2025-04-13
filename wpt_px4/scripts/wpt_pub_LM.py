#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from aura_msg.msg import Waypoint
from pyproj import Proj, transform
import geopy.distance
import math

class LawnMowerPattern:
    def __init__(self, distance, height, width, num_patterns=1, tilt_angle=0.0):
        self.distance = distance
        self.height = height
        self.width = width
        self.num_patterns = num_patterns
        self.tilt_angle = tilt_angle
        self.waypoints = []

    def rotate_point(self, x, y):
        """Rotates a point (x, y) by tilt_angle around the origin (0, 0)."""
        angle_rad = math.radians(self.tilt_angle)
        x_new = x * math.cos(angle_rad) - y * math.sin(angle_rad)
        y_new = x * math.sin(angle_rad) + y * math.cos(angle_rad)
        return x_new, y_new

    def generate_waypoints(self):
        """Generates waypoints only at the edges of a 'ㄹ' shaped pattern, repeated for 'num_patterns' times."""
        self.waypoints.clear()
        
        for pattern in range(self.num_patterns):
            # Left edge (going up)
            self.waypoints.append((2 * pattern * self.width, 0))
            self.waypoints.append((2 * pattern * self.width, self.height))
            # Top edge (moving right)
            self.waypoints.append((2 * pattern * self.width + self.width, self.height))
            # Right edge (going down)
            self.waypoints.append((2 * pattern * self.width + self.width, 0))
        
        # Final rightmost top edge point
        # self.waypoints.append((2 * self.num_patterns * self.width, self.height))
        
        # Apply rotation
        self.waypoints = [self.rotate_point(x, y) for x, y in self.waypoints]

    def get_waypoints(self):
        return self.waypoints


class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        self.gps_publisher = self.create_publisher(Waypoint, '/waypoints', 10)
        self.utm_publisher = self.create_publisher(Waypoint, '/waypoints_utm', 10)
        self.timer = self.create_timer(0.1, self.publish_waypoints)
        self.get_logger().info('Waypoint Publisher Node has been started.')

        # Define pattern parameters
        self.distance = 5.0
        self.width = 100.0*2.0
        self.height = 200.0*2.0
        self.num_patterns = 2
        self.tilt_angle = 180.0

        # Define the start point (latitude and longitude)        
        self.start_lat = 37.1853766
        self.start_lon = 126.6412587

        # Convert the start point to UTM
        self.wgs84_proj = Proj(proj='latlong', datum='WGS84')
        self.utm_proj = Proj(proj='utm', zone=52, datum='WGS84')
        self.start_utm_x, self.start_utm_y = transform(self.wgs84_proj, self.utm_proj, self.start_lon, self.start_lat)

        # Generate waypoints using the LawnMowerPattern
        self.pattern_generator = LawnMowerPattern(self.distance, self.height, self.width, self.num_patterns, self.tilt_angle)
        self.pattern_generator.generate_waypoints()
        generated_waypoints = self.pattern_generator.get_waypoints()

        # Convert waypoints to lat/lon and UTM
        self.wpt_x, self.wpt_y, self.wpt_utm_x, self.wpt_utm_y = [], [], [], []
        for point in generated_waypoints:
            utm_x = self.start_utm_x + point[0]
            utm_y = self.start_utm_y + point[1]
            self.wpt_utm_x.append(utm_x)
            self.wpt_utm_y.append(utm_y)
            lon, lat = transform(self.utm_proj, self.wgs84_proj, utm_x, utm_y)
            self.wpt_x.append(lat)
            self.wpt_y.append(lon)
        
        # Print UTM waypoints for debugging
        self.get_logger().info(f'UTM waypoints: {list(zip(self.wpt_utm_x, self.wpt_utm_y))}')

    def publish_waypoints(self):
        gps_msg = Waypoint()
        gps_msg.x_lat = self.wpt_x
        gps_msg.y_long = self.wpt_y
        self.gps_publisher.publish(gps_msg)
        
        utm_msg = Waypoint()
        utm_msg.x_lat = self.wpt_utm_x
        utm_msg.y_long = self.wpt_utm_y
        self.utm_publisher.publish(utm_msg)
        
        self.get_logger().info('Published GPS waypoints and UTM waypoints.')


def main(args=None):
    rclpy.init(args=args)
    node = WaypointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
