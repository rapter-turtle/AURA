import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from aura_msg.msg import MPCTraj, MPCState, ObsState
from pynput import keyboard

def clamp(value, min_value, max_value):
    """Clamp the value to the range [min_value, max_value]"""
    return max(min_value, min(value, max_value))

def convert_steering_to_pwm(steer):
    """Map steering value to PWM based on the given formula"""
    pwm_center = 1500.0

    if steer >= 300.0:
        # Steer above 300 maps directly to PWM 2000
        return 2000.0
    elif 0 <= steer < 300.0:
        # Steer in the range [0, 300] maps linearly between PWM = 1500 and PWM = 2000
        return 1550.0 + (steer * 1.6667)
    elif -300.0 <= steer < 0:
        # Steer in the range [-300, 0] maps linearly between PWM = 1000 and PWM = 1500
        return 1450.0 + (steer * 1.6667)
    elif steer < -300.0:
        # Steer below -300 maps directly to PWM 1000
        return 1000.0

def convert_thrust_to_pwm(thrust):
    """Convert thrust level to PWM signal"""
    if thrust < 0.0:
        pwm = 3.9 * thrust + 1450.0
        return clamp(pwm, 1000.0, 1500.0)  # Any value <= 0 thrust maps to PWM 1000
    else:
        # Calculate PWM based on the thrust
        pwm = 3.9 * thrust + 1550.0
        # You can switch the formula if needed, using the commented one
        # pwm = 5.0 * thrust + 1500
        return clamp(pwm, 1500.0, 2000.0)  # Ensure PWM is within the bounds



class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control_node')

        # Publisher setup
        self.publisher_ = self.create_publisher(Float64MultiArray, '/actuator_outputs', 10)
        self.mpcvis_pub = self.create_publisher(Float64MultiArray, '/ship/utm', 10)
# Float64MultiArray, '/ship/utm'
        # Initial values for pwm_steer, pwm_thrust, and Kd
        self.pwm_steer = 0.0
        self.pwm_thrust = 0.0
        self.Kd = 0.0
        self.acceptance_radius = 0.0

        # Set the rate at which we publish messages
        self.timer = self.create_timer(0.1, self.timer_callback)

        # Start the listener for keyboard input
        self.listener = keyboard.Listener(on_press=self.on_press)
        self.listener.start()

    def timer_callback(self):
        # Clamp pwm_steer and pwm_thrust within the max PWM values before publishing
        self.pwm_steer = clamp(self.pwm_steer, -300.0, 300.0)  # Clamp steering within the range [-300, 300]
        self.pwm_thrust = clamp(self.pwm_thrust, -100.0, 100.0)  # Example: Clamping thrust to [-100, 100] range

        # Create and publish actuator message
        actuator_msg = Float64MultiArray()
        actuator_msg.data = [convert_steering_to_pwm(self.pwm_steer), convert_thrust_to_pwm(self.pwm_thrust), 0.0, 0.0]
        self.publisher_.publish(actuator_msg)
        self.get_logger().info(f"Published actuator values: {actuator_msg.data}")

        mpcvis_msg = Float64MultiArray()
        mpcvis_msg.data = [0.0, 0.0, 0.0, 0.0]
        self.mpcvis_pub.publish(mpcvis_msg)



    def on_press(self, key):
        try:
            if key == keyboard.Key.up:
                self.pwm_thrust += 1.0  # Increase thrust when up arrow is pressed
                self.get_logger().info(f"Thrust increased to: {self.pwm_thrust}")
            elif key == keyboard.Key.down:
                self.pwm_thrust -= 1.0  # Decrease thrust when down arrow is pressed
                self.get_logger().info(f"Thrust decreased to: {self.pwm_thrust}")
            elif key == keyboard.Key.left:
                self.pwm_steer += 0.5  # Decrease steering when left arrow is pressed
                self.get_logger().info(f"Steering decreased to: {self.pwm_steer}")
            elif key == keyboard.Key.right:
                self.pwm_steer -= 0.5  # Increase steering when right arrow is pressed
                self.get_logger().info(f"Steering increased to: {self.pwm_steer}")
            elif key == keyboard.Key.esc:
                self.listener.stop()  # Stop listener if Escape key is pressed
                self.get_logger().info("Escape key pressed. Stopping the listener.")
            elif key == keyboard.KeyCode.from_char('r'):
                # Reset pwm_steer and pwm_thrust to 0.0 when 'r' key is pressed
                self.reset_values()
        except AttributeError:
            pass  # Handle special keys like shift, ctrl, etc.
 
    def reset_values(self):
        # Reset pwm_steer and pwm_thrust to 0.0
        self.pwm_steer = 0.0
        self.pwm_thrust = 0.0
        self.get_logger().info("Values reset to zero (pwm_steer and pwm_thrust).")

def main(args=None):
    rclpy.init(args=args)

    # Create the ROS2 node
    node = KeyboardControlNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


# u_data = np.array([1.55, 2.54, 2.9, 3.2, 3.77, 3.98, 4.04, 4.23, 4.63])
# n_data = np.array([25.35, 27.38, 30.41, 32.0, 35.49, 36.5, 38.53, 40.56, 42.59])