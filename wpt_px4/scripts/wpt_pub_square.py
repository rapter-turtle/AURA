#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from aura_msg.msg import Waypoint
from pyproj import Proj, transform
import geopy.distance
import math

class WaypointPublisher(Node):
    def __init__(self):
        super().__init__('waypoint_publisher')
        self.gps_publisher = self.create_publisher(Waypoint, '/waypoints', 10)
        self.utm_publisher = self.create_publisher(Waypoint, '/waypoints_utm', 10)
        self.timer = self.create_timer(0.1, self.publish_waypoints)
        self.get_logger().info('Waypoint Publisher Node has been started.')

        # Define square properties: center (latitude, longitude), width, height, and rotation angle
        self.center_lat = 37.1832010#37.1835000
        self.center_lon = 126.6395622#126.6395000
        self.width_m = 100.0*2  # Width in meters
        self.height_m = 200.0*2
          # Height in meters
        self.angle_deg = 90.0  # Rotation angle in degrees

        # Compute the four corners of the rotated square
        self.wpt_x, self.wpt_y = self.compute_rotated_square_waypoints()
        
        # Create Proj objects for WGS84 and UTM conversion
        self.wgs84_proj = Proj(proj='latlong', datum='WGS84')
        self.utm_proj = Proj(proj='utm', zone=52, datum='WGS84')  # Adjust zone as needed

        # Convert GPS waypoints to UTM
        self.wpt_utm_x = []
        self.wpt_utm_y = []
        for lat, lon in zip(self.wpt_x, self.wpt_y):
            try:
                utm_x, utm_y = transform(self.wgs84_proj, self.utm_proj, lon, lat)
                self.wpt_utm_x.append(utm_x)
                self.wpt_utm_y.append(utm_y)
            except Exception as e:
                self.get_logger().error(f"Error converting GPS to UTM: {e}")

    def compute_rotated_square_waypoints(self):
        """Compute the four waypoints of a rotated square given center, width, height, and angle."""
        half_width = self.width_m / 2.0
        half_height = self.height_m / 2.0
        angle_rad = math.radians(self.angle_deg)

        # Compute offsets for rotation
        cos_a = math.cos(angle_rad)
        sin_a = math.sin(angle_rad)

        offsets = [
            (-half_width * cos_a - half_height * sin_a, -half_width * sin_a + half_height * cos_a),
            (half_width * cos_a - half_height * sin_a, half_width * sin_a + half_height * cos_a),
            (half_width * cos_a + half_height * sin_a, half_width * sin_a - half_height * cos_a),
            (-half_width * cos_a + half_height * sin_a, -half_width * sin_a - half_height * cos_a),
        ]

        waypoints = []
        for dx, dy in offsets:
            lat_lon = geopy.distance.distance(meters=dy).destination(
                geopy.distance.distance(meters=dx).destination((self.center_lat, self.center_lon), 90), 0)
            waypoints.append((lat_lon.latitude, lat_lon.longitude))

        wpt_x, wpt_y = zip(*waypoints)
        return list(wpt_x), list(wpt_y)

    def publish_waypoints(self):
        # Publish GPS waypoints
        gps_msg = Waypoint()
        gps_msg.x_lat = list(self.wpt_x)
        gps_msg.y_long = list(self.wpt_y)
        self.gps_publisher.publish(gps_msg)
        self.get_logger().info(f'Published GPS waypoints: x_lat={gps_msg.x_lat}, y_long={gps_msg.y_long}')

        # Publish UTM waypoints
        utm_msg = Waypoint()
        utm_msg.x_lat = self.wpt_utm_x  # Use UTM x-coordinates
        utm_msg.y_long = self.wpt_utm_y  # Use UTM y-coordinates
        self.utm_publisher.publish(utm_msg)
        self.get_logger().info(f'Published UTM waypoints: x_lat={utm_msg.x_lat}, y_long={utm_msg.y_long}')


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
