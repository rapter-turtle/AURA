import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
from aura_msg.msg import Waypoint, Parameter
from sensor_msgs.msg import Imu, NavSatFix
from math import atan2, sqrt, pi, cos, sin, log
from geographiclib.geodesic import Geodesic
from pyproj import Transformer
import cvxpy as cp
import numpy as np
import casadi as ca


def clamp(value, low, high):
    return max(low, min(value, high))

class ActuatorPublisher(Node):
    def __init__(self):
        super().__init__('actuator_publisher')

        # Publishers
        self.publisher_ = self.create_publisher(Float64MultiArray, '/actuator_outputs', 10)

        # Subscribers
        self.create_subscription(Float64MultiArray, '/ship/utm', self.state_callback, 10)
        # self.create_subscription(Float64MultiArray, '/actuator_outputs', self.thrust_callback, 10)

        # Timer
        self.timer_ = self.create_timer(1.0, self.timer_callback)

        # Control parameters and state
        self.x = self.y = self.psi = self.u = self.v = self.r = 0.0
        self.desired_velocity = 0.0
        self.max_steer = 200
        self.max_thrust = 70
        self.max_thrust_diff = 100
        self.max_steer_diff = 200
        self.before_velocity_e = 0.0
        self.last_steering = 0.0
        self.last_thrust = 0.0
        self.gps_received = False
        self.imu_received = False

        self.count = 0.0
        self.thrust = 0.0

    def state_callback(self, msg):
        self.count = self.count + 1



    def convert_steering_to_pwm(self, steer):
        if steer >= 300:
            return 2000.0
        elif steer >= 0:
            return 1500.0 + steer * 1.6667
        elif steer >= -300:
            return 1500.0 + steer * 1.6667
        else:
            return 1000.0

    def convert_thrust_to_pwm(self, thrust):
        if thrust <= 0:
            return 1500.0
        else:
            pwm = 3.9 * thrust + 1550.0
            return clamp(pwm, 1500.0, 2000.0)


    def timer_callback(self):
        if self.count <= 5:
            pwm_steer = self.convert_steering_to_pwm(0.0)
            pwm_thrust = self.convert_thrust_to_pwm(0.0)

            msg = Float64MultiArray()
            msg.data = [pwm_steer, pwm_thrust, 0.0, 0.0]
            self.publisher_.publish(msg)
            
        print(self.count)
        self.count = 0.0
        


def main(args=None):
    rclpy.init(args=args)
    node = ActuatorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
