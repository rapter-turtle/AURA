import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
from aura_msg.msg import Waypoint, Parameter
from sensor_msgs.msg import Imu, NavSatFix
from math import atan2, sqrt, pi, cos, sin, log
from geographiclib.geodesic import Geodesic
from pyproj import Transformer
import casadi as ca
import numpy as np


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
        self.max_steer = 500
        self.max_thrust = 70
        self.max_thrust_diff = 200
        self.max_steer_diff = 20
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
        self.bp = -3.5

        self.dt = 0.1
        self.eta_x_d = 0.0
        self.eta_y_d = 0.0
        self.a = 1.7
        self.b = 1.7

        self.alpha1 = 1.0
        self.alpha2 = 2.0
        self.alpha1d = 0.1
        self.alpha2d = 0.1

        self.eta_ux = 0.0
        self.eta_uy = 0.0

        self.tau_x = 0.0
        self.tau_y = 0.0
        self.tau_u = 0.0
        self.tau_r = 0.0

        self.K_steer = 800.0
        self.default_thrust = 35.0
        self.last_los = 0.0
        # self.last_thrust = 0.0
        self.time = 0.0

        self.V0 = 2.5

        self.before_thrust = 0.0
        self.last_dd = 0.0
        x_actual_min = 289577.66
        x_actual_max = 291591.05
        y_actual_min = 4117065.30
        y_actual_max = 4118523.52
        
        station_keeping_point = np.array([(x_actual_min+x_actual_max)*0.5, (y_actual_min+y_actual_max)*0.5])        
         
        self.x0 = station_keeping_point[0] - 20
        self.y0 = station_keeping_point[1] + 40 
        self.deck = self.x0 - 1.5

    def state_callback(self, msg):
        self.x, self.y, self.psi, self.u, self.v, self.r = msg.data[:6]
        self.gps_received = True
        self.imu_received = True
        self.publish_utm_coordinates()

    def velocity_callback(self, msg):
        self.desired_velocity = msg.data / 1.94384  # knots to m/s

    def convert_steering_to_pwm(self, steer):
        # if steer >= 300:
        #     return 2000.0
        # elif steer >= 0:
        #     return 1500.0 + steer * 1.6667
        # elif steer >= -300:
        #     return 1500.0 + steer * 1.6667
        # else:
        #     return 1000.0
        return 1500.0 + steer * 1.6667

    def convert_thrust_to_pwm(self, thrust):
        if thrust <= 0:
            return 1500.0
        else:
            pwm = 3.9 * thrust + 1550.0
            return clamp(pwm, 1500.0, 2000.0)


    def timer_callback(self):
        Vx = 4.0*cos(2.0*0.1*self.time)#*3*sin(1*0.1*self.time)
        Vy = 2.0*sin(1.0*0.1*self.time)+ self.V0#*sin(4*0.1*self.time)

        self.y0 = self.y0 + Vy*0.1
        self.x0 = self.x0 + Vx*0.1

        psi = self.psi
        psi = psi - 2*pi if psi > pi else psi
        psi = psi + 2*pi if psi < -pi else psi

        # LOS 
        dx = self.x0 - self.x
        dy = self.y0 - self.y

        LOS = atan2(dy, dx) - psi
        LOS = LOS - 2*pi if LOS > pi else LOS
        LOS = LOS + 2*pi if LOS < -pi else LOS
        
        Kr = 1000.0
        Kd = 600.0
        steer_input = -Kr*LOS - Kd*(LOS-self.last_los)
        self.last_los = LOS
        # proposed_thrust = 40.0

        # # CLF-CBF optimization
        etaVx = self.u*cos(psi) - self.v*sin(psi) - self.r*self.hp*sin(psi)
        etaVy = self.u*sin(psi) + self.v*cos(psi) + self.r*self.hp*cos(psi) 
        etax = self.x + self.hp*cos(psi)
        etay = self.y + self.hp*sin(psi)

        etaVx_back = self.u*cos(psi) - self.v*sin(psi) - self.r*self.bp*sin(psi)
        etaVy_back = self.u*sin(psi) + self.v*cos(psi) + self.r*self.bp*cos(psi) 
        etax_back = self.x + self.bp*cos(psi)
        etay_back = self.y + self.bp*sin(psi)


        u_dot = - self.Xu*self.u - self.Xuu*sqrt(self.u*self.u)*self.u
        v_dot = - self.Yv*self.v - self.Yn*self.r - self.Yvv*sqrt(self.v*self.v)*self.v
        r_dot = - self.Nr*self.r - self.Nv*self.v - self.Nrr*sqrt(self.r*self.r)*self.r
        

        opti = ca.Opti()

        u_1 = opti.variable()  # First control input (thrust)
        u_2 = opti.variable()  # Second control input (steering)
        s1 = opti.variable()  # Slack variable
        s2 = opti.variable()
        # opti.minimize(1.0*(u_1 - self.default_thrust*self.default_thrust)**2 + 1.0*(u_2 - steer_input)**2 + 1e10*s1**2 + 1e10*s2**2 )

        opti.minimize(1.0*(u_1 - self.default_thrust*self.default_thrust)**2 + 1.0*(u_2 - steer_input)**2 + 1e10*s1**2)

        tau_u = self.b1*u_1*ca.cos(self.b2*u_2)
        tau_v = self.b1*u_1*ca.sin(self.b2*u_2)
        tau_r = -self.b3*self.b1*u_1*ca.sin(self.b2*u_2)

        x_dotdot = (u_dot + tau_u)*cos(psi) - (v_dot + tau_v)*sin(psi) - (r_dot + tau_r)*self.hp*sin(psi) - self.r*self.u*sin(psi) - self.r*self.v*cos(psi) - self.r*self.r*self.hp*cos(psi)
        y_dotdot = (u_dot + tau_u)*sin(psi) + (v_dot + tau_v)*cos(psi) + (r_dot + tau_r)*self.hp*cos(psi) + self.r*self.u*cos(psi) - self.r*self.v*sin(psi) - self.r*self.r*self.hp*sin(psi)
        
        etaVx_next = etaVx + x_dotdot*0.1
        etaVy_next = etaVy + y_dotdot*0.1

        guide_cbf = -(etay - self.y0) - self.a*log((etax - self.x0)*(etax - self.x0) + 1)
        guide_cbf_dot = -(etaVy_next - Vy) - 2*self.a*(etax - self.x0)*(etaVx_next - Vx)/((etax - self.x0)*(etax - self.x0) + 1)
        
        guide_cbf_dotdot_c = (etaVx_next - Vx)*(2*(etaVx_next - Vx)/((etax - self.x0)*(etax - self.x0) + 1) - 4*(etax - self.x0)*(etax - self.x0)*(etaVx_next - Vx)/(((etax - self.x0)*(etax - self.x0) + 1)*((etax - self.x0)*(etax - self.x0) + 1)))
        guide_cbf_dotdot = -y_dotdot - guide_cbf_dotdot_c - 2*(etax - self.x0)*x_dotdot/((etax - self.x0)*(etax - self.x0) + 1)


        x_dotdot_back = (u_dot + tau_u)*cos(psi) - (v_dot + tau_v)*sin(psi) - (r_dot + tau_r)*self.bp*sin(psi) - self.r*self.u*sin(psi) - self.r*self.v*cos(psi) - self.r*self.r*self.bp*cos(psi)
        y_dotdot_back = (u_dot + tau_u)*sin(psi) + (v_dot + tau_v)*cos(psi) + (r_dot + tau_r)*self.bp*cos(psi) + self.r*self.u*cos(psi) - self.r*self.v*sin(psi) - self.r*self.r*self.bp*sin(psi)
        
        etaVx_next_back = etaVx_back + x_dotdot_back*0.1
        etaVy_next_back = etaVy_back + y_dotdot_back*0.1

        guide_cbf_back = -(etay_back - self.y0) - self.b*log((etax_back - self.x0)*(etax_back - self.x0) + 1)
        guide_cbf_dot_back = -(etaVy_next_back - Vy) - 2*self.b*(etax_back - self.x0)*(etaVx_next_back - Vx)/((etax_back - self.x0)*(etax_back - self.x0) + 1)
        
        guide_cbf_dotdot_c_back = (etaVx_next_back - Vx)*(2*(etaVx_next_back - Vx)/((etax_back - self.x0)*(etax_back - self.x0) + 1) - 4*(etax_back - self.x0)*(etax_back - self.x0)*(etaVx_next_back - Vx)/(((etax_back - self.x0)*(etax_back - self.x0) + 1)*((etax_back - self.x0)*(etax_back - self.x0) + 1)))
        guide_cbf_dotdot_back = -y_dotdot_back - guide_cbf_dotdot_c_back - 2*(etax_back - self.x0)*x_dotdot_back/((etax_back - self.x0)*(etax_back - self.x0) + 1)

        ## deck CBF
        deck_cbf = self.x - self.deck
        deck_cbf_dot = self.u*cos(self.psi) - self.v*sin(self.psi)
        deck_cbf_dotdot = (u_dot + tau_u)*cos(self.psi) - (v_dot + tau_v)*sin(self.psi) - self.r*self.u*sin(psi) - self.r*self.v*cos(psi)


        opti.subject_to(u_1 >= 0.0)
        opti.subject_to(u_2 >= -self.max_steer)
        opti.subject_to(u_2 <= self.max_steer)        
        # opti.subject_to(u_1 >= self.last_dd - 900.0*0.1)
        # opti.subject_to(u_1 <= self.last_dd + 900.0*0.1)
        # opti.subject_to(u_2 >= self.last_steering - 150*0.1)
        # opti.subject_to(u_2 <= self.last_steering + 150*0.1)        
        opti.subject_to(self.alpha1*self.alpha2*guide_cbf + (self.alpha1 + self.alpha2)*guide_cbf_dot + guide_cbf_dotdot + s1>= 0.0)
        opti.subject_to(self.alpha1d*self.alpha2d*deck_cbf + (self.alpha1d + self.alpha2d)*deck_cbf_dot + deck_cbf_dotdot + s2>= 0.0)
        # opti.subject_to(self.alpha1*self.alpha2*guide_cbf_back + (self.alpha1 + self.alpha2)*guide_cbf_dot_back + guide_cbf_dotdot_back + s2>= 0.0)

        opti.solver('ipopt')
        sol = opti.solve()
        optimal_u1 = sol.value(u_1)
        optimal_u2 = sol.value(u_2)
        print("proposed_thrust : ", optimal_u1, "steer input : ", optimal_u2)
        
        proposed_thrust = 0.0

        if optimal_u1 == None or optimal_u2 == None:
            proposed_thrust = 0.0
            steer_input = 0.0
            print("none")
        else:
            if optimal_u1 > 0:
                proposed_thrust = sqrt(optimal_u1)
            elif optimal_u1 == 0:
                proposed_thrust = 0
            steer_input = optimal_u2
        self.last_dd = optimal_u1

        # proposed_thrust = 35.0

        steer_input = clamp(steer_input, -self.max_steer, self.max_steer)
        
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

        self.time += 1.0



    def publish_utm_coordinates(self):
        msg = Float64MultiArray()
        msg.data = [self.x0, self.y0, self.a, self.b]
        self.utm_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = ActuatorPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
