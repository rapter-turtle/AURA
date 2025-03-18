import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from aura_msg.msg import ActuatorOutputs
from sensor_msgs.msg import Imu
import numpy as np
# from tf_transformations import quaternion_from_euler
import pyproj  # Import pyproj for UTM to lat/lon conversion

class ShipSimulator(Node):
    def __init__(self):
        super().__init__('ship_simulator')
        
        # ROS2 settings
        self.state_pub = self.create_publisher(Float64MultiArray, '/ekf/estimated_state', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.control_sub = self.create_subscription(Float64MultiArray, '/actuator_outputs', self.control_callback, 10)

        # Station keeping point in UTM coordinates (Easting, Northing)
        #Jeongok
        x_actual_min = 289577.66
        x_actual_max = 291591.05
        y_actual_min = 4117065.30
        y_actual_max = 4118523.52
        
        station_keeping_point = np.array([(x_actual_min+x_actual_max)*0.5, (y_actual_min+y_actual_max)*0.5])
        # station_keeping_point = np.array([291126, 4118387])


        # Initialize state [x, y, psi, u, v, r]
        self.ship_state = np.zeros(6)
        self.ship_state[0] = station_keeping_point[0]  # UTM X (easting)
        self.ship_state[1] = station_keeping_point[1]  # UTM Y (northing)
        self.ship_state[2] = 3.141592 * 0.8  # Heading (orientation)
        self.control_input = np.zeros(2)  # [n1, n2]
        self.dt = 0.1  # Time step for the simulation
        self.disturbance_state = np.zeros((6))
        
        # Timer for the simulation loop
        self.timer = self.create_timer(self.dt, self.run)

        # UTM and WGS84 projections for coordinate transformation
        self.utm_proj = pyproj.Proj(proj="utm", zone=52, datum="WGS84")  # Set UTM zone as 33 (change as necessary)
        self.wgs84_proj = pyproj.Proj(proj="latlong", datum="WGS84")

    def control_callback(self, msg):
        """Callback to update control input from the control node."""
        
        # Convert PWM to steering
        def convert_pwm_to_steering(pwm):
            # Assuming steering PWM range is 1000 to 2000 and maps to [-pi/6, pi/6]
            pwm_center = 1500
            pwm_per_radian = 1.6667
            return (pwm - pwm_center) / pwm_per_radian

        # Convert PWM to thrust
        def convert_pwm_to_thrust(pwm):
            # Assuming thrust PWM range is 1500 to 2000 maps to [0, 100]
            if pwm <= 1550:
                return 0.0
            return (pwm - 1550) * 0.26

        # Convert actuator PWM inputs to control inputs
        self.control_input[0] = convert_pwm_to_steering(msg.data[0])  # Steer
        self.control_input[1] = convert_pwm_to_thrust(msg.data[1])  # Thrust
        # print(self.control_input[1])


    def wave_disturbance(self, disturbance_state, wave_direction, wind_speed, omega, lamda, Kw, sigmaF1, sigmaF2, dt):
        omega_e = np.abs(omega - (omega * omega / 9.81) * wind_speed * np.cos(wave_direction))
        x1 = disturbance_state[0]
        x2 = disturbance_state[1]

        omegaF1 = np.random.normal(0.0, sigmaF1)
        omegaF2 = np.random.normal(0.0, sigmaF2)

        xdot = np.array([x2, -omega_e * omega_e * x1 - 2 * lamda * omega_e * x2 + Kw * omegaF1, omegaF2])
        disturbance_state = xdot * dt + disturbance_state

        disturbance_force = disturbance_state[1] + disturbance_state[2]
        return disturbance_state, disturbance_force

    def recover_simulator(self, ship, control_input, dt):
        """Simulate the ship dynamics given the current state and control input."""
        M = 1.0  # Mass [kg]
        I = 1.0   # Inertial tensor [kg m^2]
        # Xu = 0.1#0.081
        # Xuu = 0.0
        # Nr = 0.081*2
        # Nrrr = 0.0
        # Yv = 0.06*2
        # Yvv = 0
        # Yr = 0.081*0.5
        # Nv = 0.081*0.5
        # dist = 0.3  # 30cm
        Xu = 0.10531
        Xuu = 0.018405
        Yv = 8.7185e-09
        Yvv = 0.39199
        Yr = 1.1508e-08
        Nr = 9.1124e-08
        Nrr = 5.2726
        Nv = 6.1558e-09
        b1=  0.00058466
        b2 =  0.0040635
        b3 =  0.31094

        # Extract states and controls
        psi, u, v, r = ship[2], ship[3], ship[4], ship[5]
        thrust, steer = control_input[1], control_input[0]
        eps = 1e-5

        # Wave and wind disturbance calculation
        wind_direction = 0.3
        wind_speed = 3.0

        self.disturbance_state[:3], XY_wave_force = self.wave_disturbance(
            self.disturbance_state[:3], wind_direction, wind_speed, 0.8, 0.1, 0.64, 6, 2, dt)
        self.disturbance_state[3:6], N_wave_force = self.wave_disturbance(
            self.disturbance_state[3:6], wind_direction, wind_speed, 0.8, 0.1, 1.0, 1, 0.1, dt)

        X_wave_force = XY_wave_force * np.cos(wind_direction - psi)
        Y_wave_force = XY_wave_force * np.sin(wind_direction - psi)

        U_wave_force = X_wave_force * np.cos(psi) + Y_wave_force * np.sin(psi)
        V_wave_force = -X_wave_force * np.sin(psi) + Y_wave_force * np.cos(psi)

        # Wind disturbance
        Afw = 0.3
        Alw = 0.44
        LOA = 1.3
        lau = 1.2
        CD_lAF = 0.55
        delta = 0.6
        CDl = CD_lAF * Afw / Alw
        CDt = 0.8

        u_rel_wind = u - wind_speed * np.cos(wind_direction - psi)
        v_rel_wind = v - wind_speed * np.sin(wind_direction - psi)
        gamma = -np.arctan2(v_rel_wind, u_rel_wind)

        Cx = CD_lAF * np.cos(gamma) / (1 - delta * 0.5 * (1 - CDl / CDt) * (np.sin(2 * gamma))**2)
        Cy = CDt * np.sin(gamma) / (1 - delta * 0.5 * (1 - CDl / CDt) * (np.sin(2 * gamma))**2)
        Cn = -0.18 * (gamma - np.pi * 0.5) * Cy

        X_wind_force = 0.0#0.5 * lau * (u_rel_wind**2 + v_rel_wind**2) * Cx * Afw
        Y_wind_force = 0.0#0.5 * lau * (u_rel_wind**2 + v_rel_wind**2) * Cy * Alw
        N_wind_force = 0.0#0.5 * lau * (u_rel_wind**2 + v_rel_wind**2) * Cn * Alw * LOA

        U_wind_force = X_wind_force * np.cos(psi) + Y_wind_force * np.sin(psi)
        V_wind_force = -X_wind_force * np.sin(psi) + Y_wind_force * np.cos(psi)

        u_dis = 0.0#U_wind_force + U_wave_force
        v_dis = 0.0#V_wind_force + V_wave_force
        N_wave_force = 0.0

        # Dynamics calculation
        xdot = np.array([
            u * np.cos(psi) - v * np.sin(psi),
            u * np.sin(psi) + v * np.cos(psi),
            r,
            (b1*thrust*thrust*np.cos(b2*steer) - (Xu + Xuu * np.sqrt(u * u + eps)) * u) / M,
            (b1*thrust*thrust*np.sin(b2*steer) - Yv * v - Yvv * np.sqrt(v * v + eps) * v - Yr * r) / M,
            ( -b3*b1*thrust*thrust*np.sin(b2*steer) - (Nr + Nrr  * np.sqrt(r * r + eps)) * r - Nv * v) / I
        ])

    
        ship = xdot * dt + ship

        while ship[2] > np.pi:
            ship[2] -= 2 * np.pi
        while ship[2] < -np.pi:
            ship[2] += 2 * np.pi


        return ship

    def utm_to_latlon(self, utm_x, utm_y):
        """Convert UTM coordinates to latitude and longitude."""
        lon, lat = pyproj.transform(self.utm_proj, self.wgs84_proj, utm_x, utm_y)
        return lat, lon

    def publish_imu(self):
        """Publish simulated IMU data."""
        imu_msg = Imu()
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        imu_msg.header.frame_id = "base_link"

        # quaternion = quaternion_from_euler(0, 0, self.ship_state[2])
        imu_msg.orientation.x = 0.0#quaternion[0]
        imu_msg.orientation.y = 0.0#quaternion[1]
        imu_msg.orientation.z = 0.0#quaternion[2]
        imu_msg.orientation.w = 0.0#quaternion[3]

        imu_msg.angular_velocity.z = self.ship_state[5]
        imu_msg.linear_acceleration.x = self.ship_state[3]
        imu_msg.linear_acceleration.y = self.ship_state[4]
 
        self.imu_pub.publish(imu_msg)

    def run(self):
        """Main simulation loop."""
        self.ship_state = self.recover_simulator(self.ship_state, self.control_input, self.dt)

        # Convert UTM to Lat/Lon
        lat, lon = self.utm_to_latlon(self.ship_state[0], self.ship_state[1])

        # Prepare the state message to publish
        state_msg = Float64MultiArray()
        # Now send lat, lon along with the ship's state (psi, u, v, r)
        state_msg.data = np.concatenate((self.ship_state,self.ship_state)).tolist()
        # state_msg.data = np.concatenate(([lat, lon], self.ship_state[2:],[0,0,0,0,0,0,0,0,0,0,0])).tolist()

        self.state_pub.publish(state_msg)

        self.publish_imu()

def main(args=None):
    rclpy.init(args=args)
    simulator = ShipSimulator()
    rclpy.spin(simulator)
    simulator.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
