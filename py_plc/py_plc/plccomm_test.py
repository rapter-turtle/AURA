import rclpy
from rclpy.node import Node

from mavros_msgs.msg import ActuatorOutputs



class ActuatorPublisher(Node):

    def __init__(self):
        super().__init__('actuator_publisher')
        self.publisher_ = self.create_publisher(ActuatorOutputs, 'actuatoroutput', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def timer_callback(self):
        msg = ActuatorOutputs()
        msg.actuator[1] = 1000.0 
        msg.actuator[3] = 1000.0
        # msg.actuator = [1000.0] * 32
        self.publisher_.publish(msg)
        print("Publishing: ", msg.actuator[1], msg.actuator[3])


def main(args=None):
    rclpy.init(args=args)

    actuator_publisher = ActuatorPublisher()

    rclpy.spin(actuator_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    actuator_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
