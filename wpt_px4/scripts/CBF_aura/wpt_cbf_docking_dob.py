import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray, Float64
from aura_msg.msg import Waypoint, Parameter
from sensor_msgs.msg import Imu, NavSatFix
from math import atan2, sqrt, pi, cos, sin, log, exp
from geographiclib.geodesic import Geodesic
from pyproj import Transformer
import casadi as ca
import numpy as np

def L1_control(state, control, state_estim, param_filtered, param_estim, w_cutoff, dt):

    Xu = 0.10531
    Xuu = 0.018405
    Yv = 8.7185e-09
    Yvv = 0.39199
    Yn = 1.1508e-08
    Nr = 9.1124e-08
    Nrr = 5.2726
    Nv = 6.1558e-09
    b1 = 0.00058466
    b2 = 0.0040635
    b3 = 0.31094

    # set up states & controls

    psi  = state[2]
    u    = state[3]
    v    = state[4]
    r    = state[5]
    u_1  = control[0]
    u_2  = control[1]

    tau_u = b1*u_1*cos(b2*u_2)
    tau_v = b1*u_1*sin(b2*u_2)
    tau_r = -b3*b1*u_1*sin(b2*u_2)

    u_dot = - Xu*u - Xuu*sqrt(u*u)*u
    v_dot = - Yv*v - Yn*r - Yvv*sqrt(v*v)*v
    r_dot = - Nr*r - Nv*v - Nrr*sqrt(r*r)*r

    f_usv = np.array([u_dot + tau_u, v_dot + tau_v, r_dot + tau_r])

    virtual_state = np.array([u,v,r])

    virtual_control = np.array([f_usv[0],
                                f_usv[1],
                                f_usv[2]])

    x_error = state_estim - virtual_state 

    xdot = param_estim +  virtual_control - x_error

    x_t_plus = xdot*dt + state_estim

    gain = -1.0
    pi = (1/gain)*(np.exp(gain*dt)-1.0)
    param_estim = -np.exp(gain*dt)*x_error/pi
    

    # print(param_estim)
    # print(param_filtered)

    before_param_filtered = param_filtered
    param_filtered = param_filtered*np.exp(-w_cutoff*dt) - param_estim*(1-np.exp(-w_cutoff*dt))

    
    return x_t_plus, param_estim, param_filtered


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

        self.alpha1 = 0.3
        self.alpha2 = 0.5
        self.alpha1d = 0.1
        self.alpha2d = 0.1

        self.eta_ux = 0.0
        self.eta_uy = 0.0

        self.tau_x = 0.0
        self.tau_y = 0.0
        self.tau_u = 0.0
        self.tau_r = 0.0

        self.K_steer = 800.0
        self.default_thrust = 40.0
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

        self.w_cutoff = 1.0
        self.state = np.array([0.0,0.0,0.0,0.0,0.0,0.0])
        self.state_estim = np.array([0.0,0.0,0.0])
        self.param_estim = np.array([0.0,0.0,0.0])
        self.param_filtered = np.array([0.0,0.0,0.0])
        self.control = np.array([0.0,0.0])

        self.optimal_u1 = 0.0
        self.optimal_u2 = 0.0

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
        Vx = 0.0#4.0*cos(2.0*0.1*self.time)#*3*sin(1*0.1*self.time)
        Vy = self.V0 #+ 2.0*sin(1.0*0.1*self.time)#*sin(4*0.1*self.time)

        self.y0 = self.y0 + Vy*0.1
        self.x0 = self.x0 + Vx*0.1

        psi = self.psi
        psi = psi - 2*pi if psi > pi else psi
        psi = psi + 2*pi if psi < -pi else psi

        ##DOB
        self.state = [self.x,self.y,self.psi,self.u,self.v,self.r]
        self.control = [self.optimal_u1, self.optimal_u2]
        
        self.state_estim, self.param_estim, self.param_filtered = L1_control(self.state, self.control, self.state_estim, self.param_filtered, self.param_estim, self.w_cutoff, self.dt)
        du = 0.0
        dv = 0.0
        dr = 0.0
        # du = -self.param_filtered[0]
        # dv = -self.param_filtered[1]
        # dr = -self.param_filtered[2]


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

        # # CLF-CBF optimization
        etaVx = self.u*cos(psi) - self.v*sin(psi) - self.r*self.hp*sin(psi)
        etaVy = self.u*sin(psi) + self.v*cos(psi) + self.r*self.hp*cos(psi) 
        etax = self.x + self.hp*cos(psi)
        etay = self.y + self.hp*sin(psi)


        u_dot = - self.Xu*self.u - self.Xuu*sqrt(self.u*self.u)*self.u + du
        v_dot = - self.Yv*self.v - self.Yn*self.r - self.Yvv*sqrt(self.v*self.v)*self.v + dv
        r_dot = - self.Nr*self.r - self.Nv*self.v - self.Nrr*sqrt(self.r*self.r)*self.r + dr
        

        opti = ca.Opti()

        u_1 = opti.variable()  # First control input (thrust)
        u_2 = opti.variable()  # Second control input (steering)
        s1 = opti.variable()  # Slack variable
        s2 = opti.variable()
        opti.minimize(1.0*(u_1 - self.default_thrust*self.default_thrust)**2 + 1000.0*(u_2 - steer_input)**2 + 1e10*s1**2 + 1e10*s2**2 )

        # opti.minimize(100.0*(u_1 - self.default_thrust*self.default_thrust)**2 + 1.0*(u_2 - steer_input)**2 + 1e10*s1**2)

        tau_u = self.b1*u_1*ca.cos(self.b2*u_2)
        tau_v = self.b1*u_1*ca.sin(self.b2*u_2)
        tau_r = -self.b3*self.b1*u_1*ca.sin(self.b2*u_2)

        ### front guide CBF
        x_dotdot = (u_dot + tau_u)*cos(psi) - (v_dot + tau_v)*sin(psi) - (r_dot + tau_r)*self.hp*sin(psi) - self.r*self.u*sin(psi) - self.r*self.v*cos(psi) - self.r*self.r*self.hp*cos(psi)
        y_dotdot = (u_dot + tau_u)*sin(psi) + (v_dot + tau_v)*cos(psi) + (r_dot + tau_r)*self.hp*cos(psi) + self.r*self.u*cos(psi) - self.r*self.v*sin(psi) - self.r*self.r*self.hp*sin(psi)
        
        etaVx_next = etaVx + x_dotdot*0.1
        etaVy_next = etaVy + y_dotdot*0.1

        guide_cbf = -(etay - self.y0) - self.a*log((etax - self.x0)*(etax - self.x0) + 1)
        guide_cbf_dot = -(etaVy_next - Vy) - 2*self.a*(etax - self.x0)*(etaVx_next - Vx)/((etax - self.x0)*(etax - self.x0) + 1)
        
        guide_cbf_dotdot_c = (etaVx_next - Vx)*(2*(etaVx_next - Vx)/((etax - self.x0)*(etax - self.x0) + 1) - 4*(etax - self.x0)*(etax - self.x0)*(etaVx_next - Vx)/(((etax - self.x0)*(etax - self.x0) + 1)*((etax - self.x0)*(etax - self.x0) + 1)))
        guide_cbf_dotdot = -y_dotdot - guide_cbf_dotdot_c - 2*(etax - self.x0)*x_dotdot/((etax - self.x0)*(etax - self.x0) + 1)


        ## deck CBF
        deck_cbf = self.x - self.deck
        deck_cbf_dot = self.u*cos(self.psi) - self.v*sin(self.psi)
        deck_cbf_dotdot = (u_dot + tau_u)*cos(self.psi) - (v_dot + tau_v)*sin(self.psi) - self.r*self.u*sin(psi) - self.r*self.v*cos(psi)


        opti.subject_to(u_1 >= 0.0)
        opti.subject_to(u_2 >= -self.max_steer)
        opti.subject_to(u_2 <= self.max_steer)        
        opti.subject_to(u_1 >= self.last_dd - 5000.0*0.1)
        opti.subject_to(u_1 <= self.last_dd + 5000.0*0.1)
        # opti.subject_to(u_2 >= self.last_steering - 150*0.1)
        # opti.subject_to(u_2 <= self.last_steering + 150*0.1)        
        opti.subject_to(self.alpha1*self.alpha2*guide_cbf + (self.alpha1 + self.alpha2)*guide_cbf_dot + guide_cbf_dotdot + s1>= 0.0)
        # opti.subject_to(self.alpha1d*self.alpha2d*deck_cbf + (self.alpha1d + self.alpha2d)*deck_cbf_dot + deck_cbf_dotdot + s2>= 0.0)
        # opti.subject_to(self.alpha1*self.alpha2*guide_cbf_back + (self.alpha1 + self.alpha2)*guide_cbf_dot_back + guide_cbf_dotdot_back + s2>= 0.0)

        opti.solver('ipopt')
        sol = opti.solve()
        self.optimal_u1 = sol.value(u_1)
        self.optimal_u2 = sol.value(u_2)
        print("proposed_thrust : ", self.optimal_u1, "steer input : ", self.optimal_u2)
        print("du = ", du, ", dv = ", dv, ", dr = ", dr)
        
        proposed_thrust = 0.0

        if self.optimal_u1 == None or self.optimal_u2 == None:
            proposed_thrust = 0.0
            steer_input = 0.0
            print("none")
        else:
            if self.optimal_u1 > 0:
                proposed_thrust = sqrt(self.optimal_u1)
            elif self.optimal_u1 == 0:
                proposed_thrust = 0
            steer_input = self.optimal_u2
        self.last_dd = self.optimal_u1

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
