#!/usr/bin/env python3

import threading
from time import sleep
import rclpy
from rclpy.node import Node
import socket
from plc import PLCPacket  

from aura_msg.msg import ActuatorOutputs
from aura_msg.msg import PlcStatus
from std_msgs.msg import Float64MultiArray

class ActuatorSubscriber(Node):

    def __init__(self):
        super().__init__('actuator_subscriber')
        self.px4_listen_port = 2006
        self.plc_ip = '192.168.2.88' # self.plc_ip = '127.0.0.1'
        self.plc_port = 2005
        self.px4_ip = '192.168.2.4' #'127.0.0.1' # self.px4_ip = '192.168.2.200'
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # self.sock.bind((self.px4_ip, self.px4_listen_port))
        self.plcPacket = PLCPacket()
         
        qos2 = rclpy.qos.QoSProfile(
            depth=10,
            reliability=rclpy.qos.QoSReliabilityPolicy.BEST_EFFORT
        )

        self.subscription = self.create_subscription(
            Float64MultiArray,
            '/actuator_outputs',
            self.listener_callback,
            # 10,
            qos_profile=qos2
            )
        # thread로 0.5초마다 send_read_request 메소드를 실행
        # self.create_timer(0.5, self.send_read_request)
        # thread로 handle_read_response 메소드를 실행
        # thread = threading.Thread(target=self.handle_read_response, args=(self.sock,))
    def listener_callback(self, msg):
        # 수신하면 메시지를 받아서 plc로 전송 
        # socket 생성해서
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        throttle = self.cal_throttle(int(msg.data[1]))# throttle = int(msg.actuator[3])
        steering = self.cal_steering(int(msg.data[0]))# steering = int(msg.actuator[1])
        clutch = self.cal_clutch(throttle)
        # print("a")

        sock.sendto(self.plcPacket.makeWritePacket2(int(throttle), int(steering), int(clutch)), (self.plc_ip, self.plc_port))
        print("sending: ", throttle, steering, clutch)

    # 0.5초마다 plc에게 'hello' 메시지를 보내는 send_read_request 메소드를 생성
    def send_read_request(self):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind((self.px4_ip, self.px4_listen_port))
        sock.sendto(self.plcPacket.makeReadPacket(), (self.plc_ip, self.plc_port))
        print("sending read request")
        
    def handle_read_response(self, sock):
        # plc로부터 응답을 받아서 처리하는 메소드
        # plc로부터 받은 데이터를 출력
        plc_publish = self.create_publisher(PlcStatus, 'plc_status', 10)
        
        px4_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        px4_socket.bind(('', self.px4_listen_port))
        while True:
            data, addr = px4_socket.recvfrom(1024)
            if data[20] == self.plc_packet.RESPONSE_READ:
                if data[19] == self.plc_packet.getCheckSum(data, 0, 19):
                    print('-----PLC RESPONSE_READ Received!!! -------------')
                    plc_packet = PlcToPx4Packet()
                    plc_packet.parseDataBytes(data[32:])
                    plc_packet.printData()
                    # 32번째 index부터 30bytes 데이터를 short int 15개에 담는다.
                    # publish 
                    plc_status = PlcStatus()
                    plc_status.auto_control_status = plc_packet.auto_control_status
                    plc_status.emergency_stop_status = plc_packet.emergency_stop_status
                    plc_status.engine_rpm_status = plc_packet.engine_rpm_status
                    plc_status.clutch_status = plc_packet.clutch_status
                    plc_status.steering_angle_status = plc_packet.steering_angle_status
                    plc_status.trim_angle_status = plc_packet.trim_angle_status
                    plc_status.engine_running_status = plc_packet.engine_running_status
                    plc_status.bow_thruster_power_status = plc_packet.bow_thruster_power_status
                    plc_status.bow_thruster_rev_status = plc_packet.bow_thruster_rev_status
                    plc_publish.publish(plc_status)
            # print(data)
            print(f"Received data: {data} from {addr} length : {len(data)}")

        # data에서 read-request에 대한 응답인지 확인
        # data를 parsing해서 publish
        
        print(data)

    # 1500 일때 0
    # 2000 일때 300
    # 1000 일때 -300
    # 1750 일때 150
    # 1250 일때 -150

    def cal_steering(self, pwm):
        if pwm <= 1550 and pwm >= 1450:
            return 300
        elif pwm > 1550:
            return (pwm - 1550) * 0.66 + 300
        elif pwm < 1450:
            return (pwm - 1000) * 0.66
        else:
            return 300

    # 2000일때 100
    # 1500일때 0
    # 1000 일때 0
    # 1750일때 50

    def cal_throttle(self, pwm):
        if pwm <= 1550 : 
            return 0 
        elif pwm > 1550:
            return (pwm-1550) * 0.26 
        else:
            return 0

    def cal_clutch(self, throttle):
        if throttle < 3:
            return 0
        else:
            return 1

def main(args=None):
    rclpy.init(args=args)

    actuator_subscriber = ActuatorSubscriber()

    rclpy.spin(actuator_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    actuator_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()