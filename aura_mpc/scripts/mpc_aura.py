from gen_ref import *
from acados_setting import *
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from aura_msg.msg import MPCTraj, MPCState, ObsState
import math
import time

ship_state_x = (289577.66 + 291591.05)*0.5  # UTM X (easting)
ship_state_y = (4117065.30 + 4118523.52)*0.5  

traj_xy = (ship_state_x, ship_state_y)
offset = np.array([ship_state_x, ship_state_y])


class AuraMPC(Node):
    def __init__(self):
        super().__init__('aura_mpc')      
        # ROS setting
        self.publisher_ = self.create_publisher(Float64MultiArray, '/actuator_outputs', 10)
        self.ekf_sub = self.create_subscription(Float64MultiArray, '/ekf/estimated_state', self.ekf_callback, 10)
        self.mpcvis_pub = self.create_publisher(MPCTraj, '/mpc_vis', 10)

        # Initial states and inputs
        self.x = self.y = self.p = self.u = self.v = self.r = 0.0
        self.delta = self.F = 0.0
        self.delta_pwm = self.F_pwm = 1500.0
        self.states = np.zeros(8)
        
        # MPC parameter settings
        self.Tf = 20 # prediction time 4 sec
        self.N = 40 # prediction horizon
        self.con_dt = 0.5 # control sampling time
        self.ocp_solver = setup_trajectory_tracking(self.states, self.N, self.Tf)

        
        # reference trajectory generation
        self.A = 80.0
        self.B = 80.0
        self.C = 30.0
        self.theta = np.pi/2
        self.plot_traj_xy = (ship_state_x+10, ship_state_y+20)
        
        self.ref_dt = 0.01
        self.ref_iter = int(self.con_dt/self.ref_dt)
        self.reference = generate_figure_eight_trajectory_con(1000, self.ref_dt, self.plot_traj_xy, self.theta, self.A, self.B, self.C) # reference dt = 0.01 sec, 1000 sec trajectory generation
        # self.reference = generate_figure_eight_trajectory_con(1000, self.ref_dt) # reference dt = 0.01 sec, 1000 sec trajectory generation
        self.reference = self.reference[::self.ref_iter,:]
        
        
        self.k = 0
        self.create_timer(self.con_dt, self.run)
    
    

    def clamp(self, value, min_value, max_value):
        """Clamp the value to the range [min_value, max_value]"""
        return max(min_value, min(value, max_value))

    def convert_steering_to_pwm(self,steer):
        """Map steering value to PWM based on the given formula"""
        pwm_center = 1500.0

        if steer >= 300.0:
            # Steer above 300 maps directly to PWM 2000
            return 2000.0
        elif 0 <= steer < 300.0:
            # Steer in the range [0, 300] maps linearly between PWM = 1500 and PWM = 2000
            return 1500.0 + (steer * 1.6667)
        elif -300.0 <= steer < 0:
            # Steer in the range [-300, 0] maps linearly between PWM = 1000 and PWM = 1500
            return 1500.0 + (steer * 1.6667)
        elif steer < -300.0:
            # Steer below -300 maps directly to PWM 1000
            return 1000.0

    def convert_thrust_to_pwm(self, thrust):
        """Convert thrust level to PWM signal"""
        if thrust <= 0.0:
            return 1500.0  # Any value <= 0 thrust maps to PWM 1000
        else:
            # Calculate PWM based on the thrust
            pwm = 3.9 * thrust + 1550.0
            # You can switch the formula if needed, using the commented one
            # pwm = 5.0 * thrust + 1500
            return self.clamp(pwm, 1500.0, 2000.0)  # Ensure PWM is within the bounds
    
    def ekf_callback(self, msg):# - frequency = gps callback freq. 
        """Callback to update states from EKF estimated state."""
        self.x, self.y, self.p, self.u, self.v, self.r = msg.data[:6]
        self.states = np.array([self.x-offset[0], self.y-offset[1], self.p, self.u, self.v, self.r, self.delta, self.F])


    def yaw_discontinuity(self, ref):
        """Handle yaw angle discontinuities."""
        flag = [0.0] * 3
        flag[0] = abs(self.states[2] - ref)
        flag[1] = abs(self.states[2] - (ref - 2 * math.pi))
        flag[2] = abs(self.states[2] - (ref + 2 * math.pi))
        min_element_index = flag.index(min(flag))

        if min_element_index == 0:
            ref = ref
        elif min_element_index == 1:
            ref = ref - 2 * math.pi
        elif min_element_index == 2:
            ref = ref + 2 * math.pi
        return ref


    def run(self):
        k = self.k # -> 현재 시간을 index로 표시 -> 그래야 ref trajectory설정가능(******** todo ********)
                
    
        t = time.time()
        ##### Reference States ######
        for j in range(self.N+1):
            refs = self.reference[k+j,:]
            refs[2] = self.yaw_discontinuity(refs[2])
            yref = np.hstack((refs[0]-offset[0],refs[1]-offset[1],refs[2],0,0,0,0,0,0,0))
            if j == self.N:
                yref = np.hstack((refs[0]-offset[0],refs[1]-offset[1],refs[2],0,0,0,0,0))
            self.ocp_solver.cost_set(j, "yref", yref)
        

        ##### Obstacle Position ######
        # obs_pos = np.array([self.x+20-offset[0], self.y+40-offset[1], 3,  # Obstacle-1: x, y, radius
        #                     self.x+-10-offset[0], self.y+20-offset[1], 4]) # Obstacle-2: x, y, radius
        obs_pos = np.array([35.0, +40.0, 6.0,  # Obstacle-1: x, y, radius
                            -0.0, +15.0, 8.0]) # Obstacle-2: x, y, radius
        
        for j in range(self.N+1):
            self.ocp_solver.set(j, "p", obs_pos)
    
        # do stuff
        elapsed = time.time() - t
        print(elapsed)
        
        
        # preparation phase
        self.ocp_solver.options_set('rti_phase', 1)
        status = self.ocp_solver.solve()
        t_preparation = self.ocp_solver.get_stats('time_tot')

        # set initial state
        self.ocp_solver.set(0, "lbx", self.states)
        self.ocp_solver.set(0, "ubx", self.states)

        # feedback phase
        self.ocp_solver.options_set('rti_phase', 2)
        status = self.ocp_solver.solve()
        t_feedback = self.ocp_solver.get_stats('time_tot')

        # obtain mpc input
        del_con = self.ocp_solver.get(0, "u")
        self.delta += del_con[0]*self.con_dt
        self.F += del_con[1]*self.con_dt

        self.get_logger().info(f"MPC Computation Time: {t_preparation + t_feedback:.4f}s")


        
        # Publish the control inputs (e.g., thrust commands)
        self.delta_pwm = self.convert_steering_to_pwm(self.delta)
        self.F_pwm = self.convert_thrust_to_pwm(self.F)                        
        actuator_msg = Float64MultiArray()
        actuator_msg.data = [self.delta_pwm, self.F_pwm, 0.0, 0.0]
        print(del_con[0], del_con[1])
        self.publisher_.publish(actuator_msg)                                
        
        
        # Publish predicted states and reference states
        mpc_data_stack = MPCTraj()
        # mpc_data_stack.header.stamp = self.get_clock()
        mpc_data_stack.pred_num = float(self.N)
        mpc_data_stack.sampling_time = self.con_dt
        mpc_data_stack.cpu_time = t_preparation + t_feedback	
        mpc_data_stack.ref_num = 1000.0	
        mpc_data_stack.ref_dt = self.ref_dt	
        mpc_data_stack.traj_x = self.plot_traj_xy[0]	
        mpc_data_stack.traj_y = self.plot_traj_xy[1]
        mpc_data_stack.theta = self.theta	
        mpc_data_stack.a = self.A	
        mpc_data_stack.b = self.B	
        mpc_data_stack.c = self.C	
        
        for j in range(self.N+1):
            mpc_pred = MPCState()
            mpc_ref = MPCState()
            mpc_pred.x = self.ocp_solver.get(j, "x")[0]+offset[0]
            mpc_pred.y = self.ocp_solver.get(j, "x")[1]+offset[1]
            mpc_pred.p = self.ocp_solver.get(j, "x")[2]
            mpc_pred.u = self.ocp_solver.get(j, "x")[3]
            mpc_pred.v = self.ocp_solver.get(j, "x")[4]
            mpc_pred.r = self.ocp_solver.get(j, "x")[5]
            mpc_pred.delta = self.ocp_solver.get(j, "x")[6]
            mpc_pred.f = self.ocp_solver.get(j, "x")[7]
            mpc_data_stack.state.append(mpc_pred)            
            # print(mpc_pred.u)
            mpc_ref.x = self.reference[k+j,0]
            mpc_ref.y = self.reference[k+j,1]
            mpc_ref.p = self.reference[k+j,2]
            mpc_ref.u = self.reference[k+j,3]
            mpc_ref.v = 0.0
            mpc_ref.r = self.reference[k+j,4]
            mpc_ref.delta = 0.0
            mpc_ref.f = 0.0
            mpc_data_stack.ref.append(mpc_ref)            


        obs_state = ObsState()
        obs_state.x   = obs_pos[0]+offset[0]
        obs_state.y   = obs_pos[1]+offset[1]
        obs_state.rad = obs_pos[2]
        mpc_data_stack.obs.append(obs_state)
        obs_state = ObsState()
        obs_state.x   = obs_pos[3]+offset[0]
        obs_state.y   = obs_pos[4]+offset[1]
        obs_state.rad = obs_pos[5]
        mpc_data_stack.obs.append(obs_state)        

        self.mpcvis_pub.publish(mpc_data_stack)
        
        # Increment the index for the reference trajectory
        self.k += 1
        if self.k + self.N >= len(self.reference):
            self.k = 0  # Reset the index if it goes beyond the reference length



def main(args=None):
    rclpy.init(args=args)
    node = AuraMPC()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()