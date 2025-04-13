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
        self.utm_publisher_ = self.create_publisher(Float64MultiArray, '/cbf', 10)

        # Subscribers
        self.create_subscription(Float64MultiArray, '/ekf/estimated_state', self.state_callback, 10)
        self.create_subscription(Float64, '/desired_velocity', self.velocity_callback, 10)

        # Timer
        self.timer_ = self.create_timer(0.1, self.timer_callback)

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

        # System parameter
        self.Xu = 0.10531
        self.Xuu = 0.018405
        self.Yv = 8.7185e-09
        self.Yvv = 0.39199
        self.Yn = 1.1508e-08
        self.Nr = 9.1124e-08
        self.Nrr = 5.2726
        self.Nv = 6.1558e-09
        self.b1 = 0.00058466
        self.b2 = 0.0040635
        self.b3 = 0.31094
        self.hp = 3.5

        self.dt = 0.1
        self.eta_x_d = 0.0
        self.eta_y_d = 0.0
        self.a = 3.0

        self.alpha_clf = 2.0
        self.alpha_cbf = 0.1

        self.eta_ux = 0.0
        self.eta_uy = 0.0

        self.tau_x = 0.0
        self.tau_y = 0.0
        self.tau_u = 0.0
        self.tau_r = 0.0

        self.K_eta = 10

        x_actual_min = 289577.66
        x_actual_max = 291591.05
        y_actual_min = 4117065.30
        y_actual_max = 4118523.52
        
        station_keeping_point = np.array([(x_actual_min+x_actual_max)*0.5, (y_actual_min+y_actual_max)*0.5])        

        self.x0 = station_keeping_point[0] - 10
        self.y0 = station_keeping_point[1] + 40 

        self.steer_before = 0.0

    def state_callback(self, msg):
        self.x, self.y, self.psi, self.u, self.v, self.r = msg.data[:6]
        self.gps_received = True
        self.imu_received = True
        self.publish_utm_coordinates()

    def velocity_callback(self, msg):
        self.desired_velocity = msg.data / 1.94384  # knots to m/s

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

        psi = self.psi
        psi = psi - 2*pi if psi > pi else psi
        psi = psi + 2*pi if psi < -pi else psi

        # CLF-CBF optimization
        etaVx = self.u*cos(psi) - self.v*sin(psi) - self.r*self.hp*sin(psi)
        etaVy = self.u*sin(psi) + self.v*cos(psi) + self.r*self.hp*cos(psi) 
        etax = self.x + self.hp*cos(psi)
        etay = self.y + self.hp*sin(psi)

        u_dot = - self.Xu*self.u - self.Xuu*sqrt(self.u*self.u)*self.u
        v_dot = - self.Yv*self.v - self.Yn*self.r - self.Yvv*sqrt(self.v*self.v)*self.v
        r_dot = - self.Nr*self.r - self.Nv*self.v - self.Nrr*sqrt(self.r*self.r)*self.r
        
        clf_x = (etax - self.x0)*(etax - self.x0) 
        clf_y = (etay - self.y0)*(etay - self.y0)
  
        guide_cbf = -(etay - self.y0 - self.a*log((etax - self.x0)*(etax - self.x0) + 1))
        guide_cbf_dot = -(-2*self.a*(etax - self.x0)/((etax - self.x0)*(etax - self.x0) + 1))


        u_x = cp.Variable()
        u_y = cp.Variable()
        s1 = cp.Variable()
        s2 = cp.Variable()
        s3 = cp.Variable()
        objective = cp.Minimize(u_x**2 + u_y**2 + 1000*s1**2 + 1000*s2**2 + 1000*s3**2)

        next_etaVx = etaVx + u_x*self.dt
        next_etaVy = etaVy + u_y*self.dt

        constraints = [
            next_etaVx <= 1.0,
            next_etaVx >= -1.0,
            next_etaVy <= 1.0,
            next_etaVy >= -1.0,
            # u_x <= 10.0,   # Example constraint on u_x
            # u_x >= -10.0,  # Example constraint on u_x
            # u_y <= 10.0,   # Example constraint on u_y
            # u_y >= -10.0,             
            # s1 <= 0.0,
            # s2 >= 0.0,
            self.alpha_clf*clf_x + 2*(etax - self.x0)*next_etaVx + s1 <= 0.0,
            self.alpha_clf*clf_y + 2*(etay - self.y0)*next_etaVy + s2 <= 0.0
            # self.alpha_cbf*guide_cbf + (next_etaVy + guide_cbf_dot*next_etaVx) + s3 >= 0.0
        ]

        problem = cp.Problem(objective, constraints)
        problem.solve()

        if u_x.value == None:
            ux = 0.0
            uy = 0.0
            print("none")
        else:
            ux = u_x.value
            uy = u_y.value
            # print(ux, uy, clf_x, clf_y)

        self.eta_x_d = etaVx + self.dt*ux
        self.eta_y_d = etaVy + self.dt*uy

    
        self.tau_x = -self.K_eta*(etaVx - self.eta_x_d) - u_dot*cos(psi) + v_dot*sin(psi) + r_dot*self.hp*sin(psi) + self.u*self.r*sin(psi) + self.v*self.r*cos(psi) + self.hp*self.r*self.r*cos(psi)
        self.tau_y = -self.K_eta*(etaVy - self.eta_y_d) - u_dot*sin(psi) - v_dot*cos(psi) - r_dot*self.hp*cos(psi) - self.u*self.r*cos(psi) + self.v*self.r*sin(psi) + self.hp*self.r*self.r*sin(psi)


        opti = ca.Opti()

        u_1 = opti.variable()  # First control input (thrust)
        u_2 = opti.variable()  # Second control input (steering)
        s1 = opti.variable()  # Slack variable
        s2 = opti.variable()

        opti.minimize((u_1)**2 + (u_2)**2 + 1e10*s1**2 + 1e10*s2**2)

        tau_u = self.b1*u_1*u_1*ca.cos(self.b2*u_2)
        tau_v = self.b1*u_1*u_1*ca.sin(self.b2*u_2)
        tau_r = -self.b3*self.b1*u_1*u_1*ca.sin(self.b2*u_2)

        opti.subject_to(u_1 >= 0.0)
        opti.subject_to(u_1 <= 400.0)
        opti.subject_to(u_2 >= self.steer_before -self.max_steer_diff*0.1)
        opti.subject_to(u_2 <= self.steer_before + self.max_steer_diff*0.1)
        opti.subject_to(u_2 >= -self.max_steer)
        opti.subject_to(u_2 <= self.max_steer)
        opti.subject_to(tau_u*cos(psi) - tau_v*sin(psi) - tau_r*self.hp*sin(psi) == self.tau_x + s1)
        opti.subject_to(tau_u*sin(psi) + tau_v*cos(psi) + tau_r*self.hp*cos(psi) == self.tau_y + s2)

        opti.solver('ipopt')
        sol = opti.solve()


        # print("CLF : ",clf_x, clf_y)
        print("tau : ",self.tau_x, self.tau_y)

        # # Thrust allocation
        # thrust_angle = atan2(self.tau_y*cos(psi) - self.tau_x*sin(psi), (1 - self.hp*self.b3)*(self.tau_y*sin(psi) + self.tau_x*cos(psi)))

        # proposed_thrust = -self.tau_y/((cos(thrust_angle)*sin(psi) + (1 - self.b3*self.hp)*sin(thrust_angle)*cos(psi))*self.b1)        
        # proposed_thrust = 0.0 if proposed_thrust < 0.0 else proposed_thrust 
        # proposed_thrust = sqrt(clamp(sqrt(proposed_thrust), 0.0, self.max_thrust ** 2))

        # print("proposed_thrust : ", proposed_thrust, "Steer angle : ", thrust_angle*180/3.141592)

        # steer_input = -thrust_angle
        if sol.value(u_1) < 0:
            proposed_thrust = 0.0
        else:    
            proposed_thrust = sqrt(sol.value(u_1))
        steer_input = sol.value(u_2)
        print("proposed_thrust : ", proposed_thrust, "steer input : ", steer_input)
        # steer_input = clamp(steer_input/self.b2, -self.max_steer, self.max_steer)
        
        self.steer_before = steer_input
        steer_change = clamp(steer_input - self.last_steering, -self.max_steer_diff, self.max_steer_diff)
        thrust_change = clamp(proposed_thrust - self.last_thrust, -self.max_thrust_diff, self.max_thrust_diff)

        steer = self.last_steering + steer_change
        thrust = self.last_thrust + thrust_change
        self.last_steering = steer
        self.last_thrust = thrust

        pwm_steer = self.convert_steering_to_pwm(steer)
        pwm_thrust = self.convert_thrust_to_pwm(thrust)

        msg = Float64MultiArray()
        msg.data = [pwm_steer, pwm_thrust, 0.0, 0.0]
        self.publisher_.publish(msg)


    def publish_utm_coordinates(self):
        msg = Float64MultiArray()
        msg.data = [self.x0, self.y0, self.a, 1.0]
        self.utm_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ActuatorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
