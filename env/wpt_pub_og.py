import rclpy
from rclpy.node import Node
from aura_msg.msg import Waypoint  # Assuming the package containing Waypoint.msg is aura_msg


class WaypointPublisher(Node):

    def __init__(self):
        super().__init__('waypoint_publisher')
        self.publisher_ = self.create_publisher(Waypoint, '/waypoints', 10)
        self.timer = self.create_timer(0.1, self.publish_waypoints)  # Publish every 0.1 seconds
        self.get_logger().info('Waypoint Publisher Node has been started.')

        # # Define the waypoint data
        # self.wpt_x = [37.187186, 37.187376, 37.186068, 37.184770]
        # self.wpt_y = [126.64764, 126.644345, 126.643061, 126.644678]
        self.wpt_x = [37.1870503, 37.1876142, 37.1870086, 37.1875402]
        self.wpt_y = [126.6463455, 126.6467351, 126.6471958, 126.6475287]

    def publish_waypoints(self):
        msg = Waypoint()
        msg.x_lat = self.wpt_x
        msg.y_long = self.wpt_y

        self.publisher_.publish(msg)
        self.get_logger().info(f'Published waypoints: x_lat={msg.x_lat}, y_long={msg.y_long}')


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
