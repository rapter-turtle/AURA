#!/usr/bin/env python3


import rclpy
from rclpy.node import Node
from aura_msg.msg import Waypoint
from pyproj import Proj, transform

class WaypointPublisher(Node):

    def __init__(self):
        super().__init__('waypoint_publisher')
        self.gps_publisher = self.create_publisher(Waypoint, '/waypoints', 10)
        self.utm_publisher = self.create_publisher(Waypoint, '/waypoints_utm', 10)
        self.timer = self.create_timer(0.1, self.publish_waypoints)
        self.get_logger().info('Waypoint Publisher Node has been started.')

        # Define GPS waypoints (latitude, longitude)
        #self.wpt_x = [37.1829005, 37.1876142, 37.1870086, 37.1875402]
        #self.wpt_y = [126.6387619, 126.6467351, 126.6471958, 126.6475287]
        # self.wpt_x = [37.1836374, 37.1834013, 37.1832026]
        # self.wpt_y = [126.6384293, 126.6390194, 126.6384913]
        # self.wpt_x = [37.1836374, 37.1834446, 37.1829854, 37.1832026]#37.1834446 #37.1833562
        # self.wpt_y = [126.6384293, 126.6402458, 126.6401691, 126.6384913]#126.6402458 #126.6401821
        self.wpt_x = [37.1836532, 37.1834136, 37.1819960, 37.1829713]#37.1834446 #37.1833562
        self.wpt_y = [126.6415152, 126.6371030, 126.6368401, 126.6420305]#126.6402458 #126.6401821


        self.x_offset = -2000.0
        self.y_offset = 200.0

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

    def publish_waypoints(self):
        # Publish GPS waypoints
        gps_msg = Waypoint()
        gps_msg.x_lat = self.wpt_x
        gps_msg.y_long = self.wpt_y
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
