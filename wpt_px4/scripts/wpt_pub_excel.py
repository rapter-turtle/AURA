#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from aura_msg.msg import Waypoint
import pandas as pd
from pyproj import Proj, transform

# 37.182106, 126.635817
# 37.10592, 126.38271


class ExcelWaypointPublisher(Node):

    def __init__(self):
        super().__init__('excel_waypoint_publisher')
        self.gps_publisher = self.create_publisher(Waypoint, '/waypoints', 10)
        self.utm_publisher = self.create_publisher(Waypoint, '/waypoints_utm', 10)
        self.timer = self.create_timer(0.5, self.publish_waypoints)
        self.get_logger().info('Excel Waypoint Publisher Node started.')

        # Load waypoints from Excel (.xlsx)
        self.excel_file = '/home/fims/aura_ws/src/wpt/scripts/wpt_square.xlsx'  # <-- Set to your .xlsx file path
        # self.excel_file = '/home/fims/aura_ws/src/wpt/scripts/wpt.xlsx'  
        # self.excel_file = '/home/user/aura_ws/src/wpt/scripts/wpt_lm.xlsx'  
        self.read_excel_waypoints()

        # Coordinate conversion setup
        self.wgs84_proj = Proj(proj='latlong', datum='WGS84')
        self.utm_proj = Proj(proj='utm', zone=52, datum='WGS84')  # Adjust the zone if needed

        # Convert to UTM
        self.wpt_utm_x = []
        self.wpt_utm_y = []
        for lat, lon in zip(self.wpt_x, self.wpt_y):
            try:
                utm_x, utm_y = transform(self.wgs84_proj, self.utm_proj, lon, lat)
                self.wpt_utm_x.append(utm_x)
                self.wpt_utm_y.append(utm_y)
            except Exception as e:
                self.get_logger().error(f"Error converting to UTM: {e}")

    def read_excel_waypoints(self):
        try:
            df = pd.read_excel(self.excel_file, engine='openpyxl')
            self.wpt_x = df['latitude'].tolist()
            self.wpt_y = df['longitude'].tolist()
            self.get_logger().info(f"Read {len(self.wpt_x)} waypoints from Excel (.xlsx).")
        except Exception as e:
            self.get_logger().error(f"Failed to read Excel file: {e}")
            self.wpt_x = []
            self.wpt_y = []

    def publish_waypoints(self):
        if not self.wpt_x or not self.wpt_y:
            self.get_logger().warn("No waypoints to publish.")
            return

        # Publish GPS waypoints
        gps_msg = Waypoint()
        gps_msg.x_lat = self.wpt_x
        gps_msg.y_long = self.wpt_y
        self.gps_publisher.publish(gps_msg)
        print("lat : ", self.wpt_x)
        print("lon : ", self.wpt_y)
        self.get_logger().info(f'Published GPS waypoints.')

        # Publish UTM waypoints
        utm_msg = Waypoint()
        utm_msg.x_lat = self.wpt_utm_x
        utm_msg.y_long = self.wpt_utm_y
        self.utm_publisher.publish(utm_msg)
        self.get_logger().info(f'Published UTM waypoints.')


def main(args=None):
    rclpy.init(args=args)
    node = ExcelWaypointPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
