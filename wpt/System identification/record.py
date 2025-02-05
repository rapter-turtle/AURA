#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import pandas as pd
import threading
import os
import csv

class DataRecorder(Node):
    def __init__(self):
        super().__init__('data_recorder')

        # Store latest data from topics
        self.latest_ekf = None
        self.latest_actuator = None
        self.lock = threading.Lock()

        # File path for saving CSV data (efficient for large datasets)
        self.file_path = os.path.join(os.getcwd(), "recorded_data.csv")

        # Open file in append mode
        with open(self.file_path, 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow([
                "timestamp", "x", "y", "p", "u", "v", "r",
                "x_sensor", "y_sensor", "p_sensor", "u_sensor", "v_sensor", "r_sensor",
                "steering", "throttle"
            ])

        # Subscribers
        self.ekf_sub = self.create_subscription(
            Float64MultiArray, '/ekf/estimated_state', self.ekf_callback, 10)
        self.actuator_sub = self.create_subscription(
            Float64MultiArray, '/actuator_outputs', self.actuator_callback, 10)

        self.get_logger().info(f"Recording data to {self.file_path}")

    def ekf_callback(self, msg):
        """Callback for /ekf/estimated_state topic"""
        with self.lock:
            self.latest_ekf = [
                msg.data[0], msg.data[1], msg.data[2],  # x, y, p
                msg.data[3], msg.data[4], msg.data[5],  # u, v, r
                msg.data[6], msg.data[7], msg.data[8],  # x_sensor, y_sensor, p_sensor
                msg.data[9], msg.data[10], msg.data[11] # u_sensor, v_sensor, r_sensor
            ]
            self.record_data()

    def actuator_callback(self, msg):
        """Callback for /actuator_outputs topic"""
        with self.lock:
            self.latest_actuator = [msg.data[0], msg.data[1]]  # Steering, Throttle
            self.record_data()

    def record_data(self):
        """Records data when both topics have values, writing directly to CSV."""
        if self.latest_ekf is not None and self.latest_actuator is not None:
            timestamp = self.get_clock().now().nanoseconds / 1e9

            # Combine latest EKF and Actuator data
            data_row = [timestamp] + self.latest_ekf + self.latest_actuator
            
            # Write row to CSV file immediately to avoid memory buildup
            with open(self.file_path, 'a', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(data_row)

def main(args=None):
    rclpy.init(args=args)
    recorder = DataRecorder()

    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        recorder.get_logger().info("Recording stopped.")
    finally:
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
