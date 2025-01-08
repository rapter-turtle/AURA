import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from aura_msg.msg import Waypoint  # Assuming Waypoint.msg is in the aura_msg package
import numpy as np
import pyproj
import matplotlib.pyplot as plt
import matplotlib.patches as patches

class ShipPlotter(Node):
    def __init__(self):
        super().__init__('ship_plotter')
        
        # ROS2 settings
        self.create_subscription(Float64MultiArray, '/ekf/estimated_state', self.state_callback, 10)
        self.create_subscription(Waypoint, '/waypoints', self.waypoint_callback, 10)
        
        # UTM and WGS84 projections for coordinate transformation
        self.utm_proj = pyproj.Proj(proj="utm", zone=33, datum="WGS84")  # Set UTM zone (change as necessary)
        self.wgs84_proj = pyproj.Proj(proj="latlong", datum="WGS84")
        
        # Initialize plot
        self.fig, self.ax = plt.subplots()
        self.ax.set_xlabel('East (m)')
        self.ax.set_ylabel('North (m)')
        self.ax.set_title('Ship Position and Waypoints')
        self.ax.set_aspect('equal')

        # Ship data
        self.ship_position = None  # (utm_x, utm_y)
        self.ship_heading = None
        self.ship_arrow = None

        # Waypoints
        self.waypoints = []

        # Show plot in a separate thread
        plt.ion()
        plt.show()

    def state_callback(self, msg):
        """Callback to receive ship state (lat, lon, heading) and plot."""
        # Extract latitude, longitude, and heading (psi)
        lat = msg.data[0]
        lon = msg.data[1]
        heading = msg.data[2]
        
        # Convert lat/lon to UTM coordinates
        utm_x, utm_y = self.latlon_to_utm(lat, lon)
        
        # Update ship position and heading
        self.ship_position = (utm_x, utm_y)
        self.ship_heading = heading
        
        # Update the plot
        self.update_plot()

    def waypoint_callback(self, msg):
        """Callback to receive waypoints and plot."""
        # Convert waypoints from lat/lon to UTM
        self.waypoints = [
            self.latlon_to_utm(lat, lon)
            for lat, lon in zip(msg.x_lat, msg.y_long)
        ]
        
        # Update the plot
        self.update_plot()

    def latlon_to_utm(self, lat, lon):
        """Convert latitude and longitude to UTM coordinates."""
        return pyproj.transform(self.wgs84_proj, self.utm_proj, lon, lat)

    def update_plot(self):
        """Update the plot to include the ship and all waypoints."""
        self.ax.clear()
        self.ax.set_xlabel('East (m)')
        self.ax.set_ylabel('North (m)')
        self.ax.set_title('Ship Position and Waypoints')
        self.ax.set_aspect('equal')

        # Plot waypoints
        for utm_x, utm_y in self.waypoints:
            self.ax.plot(utm_x, utm_y, 'gx', markersize=7, label="Waypoint")

        # Plot ship position and heading
        if self.ship_position:
            utm_x, utm_y = self.ship_position
            self.ax.plot(utm_x, utm_y, 'ro', markersize=7, label="Ship Position")
            
            if self.ship_heading is not None:
                # Create an arrow representing the ship's heading
                arrow_length = 10  # Length of the ship's arrow (scale factor)
                dx = arrow_length * np.cos(self.ship_heading)
                dy = arrow_length * np.sin(self.ship_heading)
                self.ship_arrow = patches.FancyArrowPatch(
                    (utm_x, utm_y), (utm_x + dx, utm_y + dy), 
                    mutation_scale=15, color='b', label="Ship Heading"
                )
                self.ax.add_patch(self.ship_arrow)
        
        # Adjust plot limits to include all waypoints and the ship
        self.adjust_plot_limits()
        
        # Redraw the plot
        self.ax.legend()
        plt.draw()
        plt.pause(0.01)

    def adjust_plot_limits(self):
        """Adjust the plot limits to fit all waypoints and the ship."""
        x_coords = []
        y_coords = []

        if self.waypoints:
            x_coords.extend([x for x, _ in self.waypoints])
            y_coords.extend([y for _, y in self.waypoints])

        if self.ship_position:
            x_coords.append(self.ship_position[0])
            y_coords.append(self.ship_position[1])

        if x_coords and y_coords:
            margin = 50  # Add some margin around the data
            self.ax.set_xlim([min(x_coords) - margin, max(x_coords) + margin])
            self.ax.set_ylim([min(y_coords) - margin, max(y_coords) + margin])

def main(args=None):
    rclpy.init(args=args)
    plotter = ShipPlotter()
    
    # Spin the ROS2 node to keep the subscription active
    rclpy.spin(plotter)
    
    # Clean up when done
    plotter.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
