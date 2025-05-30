#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from sensor_msgs.msg import Imu, NavSatFix
from std_msgs.msg import Float64MultiArray
import matplotlib.pyplot as plt
from std_msgs.msg import Float64
from matplotlib.widgets import TextBox, Button
from matplotlib.animation import FuncAnimation
from matplotlib import font_manager
import matplotlib.patches as patches
import numpy as np
from matplotlib.widgets import Button
from aura_msg.msg import Waypoint
import os
import threading
import utm
import matplotlib
import time
matplotlib.use('TkAgg')  # Or 'Agg' if you want to save plots as images without a GUI


#Duck Pond
# x_actual_min = 352571.00
# x_actual_max = 353531.94
# y_actual_min = 4026778.56
# y_actual_max = 4025815.16

# x_actual_min = 353071.00
# x_actual_max = 353200.94
# y_actual_min = 4026047.56
# y_actual_max = 4025953.16

#Jeongok
x_actual_min = 289577.66
x_actual_max = 291591.05
y_actual_min = 4117065.30
y_actual_max = 4118523.52

#ulsan
# x_actual_min = 538481.49 
# x_actual_max = 544529.99 
# y_actual_min = 3925954.33
# y_actual_max = 3928941.42

x_end = int(x_actual_max - x_actual_min)
y_end = int(y_actual_max - y_actual_min)
# 291591.05427209206, 4118523.5289364266
# 289577.6632260475, 4117065.3023964665

x_width = x_actual_max - x_actual_min

class SensorFusionEKF(Node):
    def __init__(self):
        super().__init__('sensor_fusion_ekf_plotter')
        self.time_data = []
        self.x_data = []
        self.y_data = []
        self.p_data = []
        self.u_data = []
        self.v_data = []
        self.r_data = []
        self.steering_data = []
        self.throttle_data = []
        self.los_data = []

        self.steering = 0
        self.throttle = 0

        self.x_sensor_data = []
        self.y_sensor_data = []
        self.p_sensor_data = []
        self.u_sensor_data = []
        self.v_sensor_data = []
        self.r_sensor_data = []
        self.du_data = []

        self.x_map = 0.0
        self.y_map = 0.0

        self.x_map_sensor = 0.0
        self.y_map_sensor = 0.0

        self.x = 0.0
        self.y = 0.0
        self.p = 0.0
        self.u = 0.0
        self.v = 0.0
        self.r = 0.0

        self.x_sensor = 0.0
        self.y_sensor = 0.0
        self.p_sensor = 0.0
        self.u_sensor = 0.0
        self.v_sensor = 0.0
        self.r_sensor = 0.0

        self.start_time = self.get_clock().now()
        self.current_time = self.get_clock().now()

        self.waypoints_x = []
        self.waypoints_y = []
        self.waypoint_plot = None  # For updating waypoints dynamically
        self.waypoint_circles = []
        self.acceptance_radius = 8

        self.LOS = 0.0


        # Define the layout for subplots using a grid
        self.fig = plt.figure(figsize=(15, 7))
        self.gs = self.fig.add_gridspec(3, 4)  # Create a 3x4 grid for axes

        # Place axes within the grid, ensuring they don’t overlap
        self.ax1 = self.fig.add_subplot(self.gs[0:3, 0:2])  # Main plot for trajectory
        self.ax2 = self.fig.add_subplot(self.gs[0, 2:3])   # Speed plot
        self.ax3 = self.fig.add_subplot(self.gs[1, 2:3])   # LOS plot
        self.ax4 = self.fig.add_subplot(self.gs[0, 3:4])   # Steer plot
        self.ax5 = self.fig.add_subplot(self.gs[1, 3:4])   # Throttle plot
        self.ax6 = self.fig.add_subplot(self.gs[2, 3:4])   # Heading plot

        # self.fig.tight_layout(pad=2.0)  # Adjust padding between subplots


        # self.fig, self.ax = plt.subplots(3, 4, figsize=(15, 7))
        # self.ax1 = plt.subplot2grid((3, 4), (0, 0), colspan=2, rowspan=3, fig=self.fig)
        self.line1_m, = self.ax1.plot([], [], 'r-', label='measurement')
        self.line1, = self.ax1.plot([], [], 'k-', label='ekf')
        self.line1test, = self.ax1.plot([], [], 'm.', label='plot every 1-sec')
        # kaist_img = plt.imread("/home/user/aura_ws/src/wpt/kaist.png")
        kaist_img = plt.imread("/home/fims/aura_ws/src/wpt/jeongok.png")
        # kaist_img = plt.imread("/home/fims/aura_ws/src/wpt/ulsan.png")
        map_height, map_width = kaist_img.shape[:2]
        self.map_height = map_height
        self.map_width = map_width
        self.ax1.imshow(kaist_img[::-1], origin='lower')
        self.ax1.grid()
        self.ax1.set_xlabel('x position')
        self.ax1.set_ylabel('y position')
        self.ax1.set_title('Real-time Trajectory Plot')
        self.ax1.legend()

        # self.ax2 = plt.subplot2grid((3, 4), (0, 2), fig=self.fig)
        self.line2_m, = self.ax2.plot([], [], 'r-', label='Speed', alpha=0.75)
        self.line2, = self.ax2.plot([], [], 'k-', label='desired speed')
        self.ax2.set_xlabel('Time')
        self.ax2.set_ylabel('Speed (knots)')

        # self.ax3 = plt.subplot2grid((3, 4), (1, 2), fig=self.fig)
        self.line3_m, = self.ax3.plot([], [], 'r-', label='LOS (degree)', alpha=0.75)
        # self.line3, = self.ax3.plot([], [], 'k-', label='ekf')
        self.ax3.set_xlabel('Time')
        self.ax3.set_ylabel('LOS (degree)')

        # self.ax4 = plt.subplot2grid((3, 4), (0, 3), fig=self.fig)
        self.line4_m, = self.ax4.plot([], [], 'r-', label='Steer', alpha=0.75)
        # self.line4, = self.ax4.plot([], [], 'k-', label='ekf')
        self.ax4.set_xlabel('Time')
        self.ax4.set_ylabel('Steer')

        # self.ax5 = plt.subplot2grid((3, 4), (1, 3), fig=self.fig)
        self.line5_m, = self.ax5.plot([], [], 'g-', label='Throttle', alpha=0.75)
        # self.line5, = self.ax5.plot([], [], 'k-', label='ekf')
        self.ax5.set_xlabel('Time')
        self.ax5.set_ylabel('Throttle')

        # self.ax6 = plt.subplot2grid((3, 4), (2, 3), fig=self.fig)
        self.line6_left, = self.ax6.plot([], [], 'r-', label='Heading')
        # self.line6_right, = self.ax6.plot([], [], 'g-', label='throttle')
        self.ax6.set_xlabel('Time')
        self.ax6.set_ylabel('Heading (degree)')


        # self.ax7.legend()
        # self.ax8.legend()
        # self.ax9.legend()   
        self.ax6.legend()
        self.ax5.legend()
        self.ax4.legend()
        self.ax3.legend()
        self.ax2.legend()
                     

        self.velocity_pub = self.create_publisher(Float64, '/desired_velocity', 10)
             
        # Adjust layout to add space for controls below
        self.fig.subplots_adjust(bottom=0.3)  # Very large space at the bottom (0.95)

        font_properties = font_manager.FontProperties(size=12)
        # Set up the TextBox for entering desired speed (smaller size)
        self.ax_input = plt.axes([0.15, 0.02, 0.05, 0.05])  # Smaller TextBox size, lower position
        self.desired_speed_textbox = TextBox(self.ax_input, 'Desired Speed (knots):', initial='0.0')
        self.desired_speed_textbox.label.set_fontproperties(font_properties)
        # self.desired_speed_textbox.textbox.set_fontsize(12) # Set font for the text inside TextBox

        # Set up the button to publish desired speed (smaller size)
        self.ax_button = plt.axes([0.22, 0.02, 0.12, 0.05])  # Smaller Button size, lower position
        # self.publish_button = Button(self.ax_button, 'Publish Desired Speed')
        # font_properties = font_manager.FontProperties(size=20)
        # self.publish_button.on_clicked(self.publish_desired_speed, fontproperties=font_properties)
  # Set font size
        self.publish_button = Button(self.ax_button, 'Publish Desired Speed')
        self.publish_button.label.set_fontproperties(font_properties)
        self.publish_button.on_clicked(self.publish_desired_speed)
        # self.publish_button.on_clicked(self.publish_desired_speed)

        self.current_desired_speed = 0.0  # Default desired speed

        self.fig.tight_layout(pad=2.0)  



        self.arrow_length = 5.5
        size = 1.5
        hullLength = 0.7 * size  # Length of the hull
        hullWidth = 0.2 * size  # Width of each hull
        separation = 0.45 * size  # Distance between the two hulls
        bodyWidth = 0.25 * size  # Width of the body connecting the hulls

        # Define the vertices of the two hulls
        self.hull1 = np.array([[-hullLength / 2, hullLength / 2, hullLength / 2, -hullLength / 2, -hullLength / 2, -hullLength / 2],
                               [hullWidth / 2, hullWidth / 2, -hullWidth / 2, -hullWidth / 2, 0, hullWidth / 2]])

        self.hull2 = np.array([[-hullLength / 2, hullLength / 2, hullLength / 2, -hullLength / 2, -hullLength / 2, -hullLength / 2],
                               [hullWidth / 2, hullWidth / 2, -hullWidth / 2, -hullWidth / 2, 0, hullWidth / 2]])

        # Define the vertices of the body connecting the hulls
        self.body = np.array([[-bodyWidth / 2, bodyWidth / 2, bodyWidth / 2, -bodyWidth / 2, -bodyWidth / 2],
                              [(separation - hullWidth) / 2, (separation - hullWidth) / 2, -(separation - hullWidth) / 2, -(separation - hullWidth) / 2, (separation - hullWidth) / 2]])

        # Combine hulls into a single structure
        self.hull1[1, :] = self.hull1[1, :] + separation / 2
        self.hull2[1, :] = self.hull2[1, :] - separation / 2

        # Rotation matrix for the heading
        R = np.array([[np.cos(self.p), -np.sin(self.p)],
                      [np.sin(self.p), np.cos(self.p)]])
        # Rotate the hulls and body
        hull1_R = np.dot(R, self.hull1)
        hull2_R = np.dot(R, self.hull2)
        body_R = np.dot(R, self.body)
        # Translate the hulls and body to the specified position
        hull1_R += np.array([0, 0]).reshape(2, 1)
        hull2_R += np.array([0, 0]).reshape(2, 1)
        body_R += np.array([0, 0]).reshape(2, 1)
        direction = np.array([np.cos(self.p), np.sin(self.p)]) * self.arrow_length
        # Plot the ASV
        self.heron_p1 = self.ax1.fill(hull1_R[0, :], hull1_R[1, :], 'b', alpha=0.35)
        self.heron_p2 = self.ax1.fill(hull2_R[0, :], hull2_R[1, :], 'b', alpha=0.35)
        self.heron_p3 = self.ax1.fill(body_R[0, :], body_R[1, :], 'b', alpha=0.35)
        self.heron_p4 = self.ax1.arrow(0, 0, direction[0], direction[1], head_width=0.1, head_length=0.1, fc='b', ec='b')

        # Create the inset axes using plt.axes
        self.axins = plt.subplot2grid((3, 4), (2, 2), fig=self.fig)


        self.fig.tight_layout()  # axes 사이 간격을 적당히 벌려줍니다.
        self.ekf_sub = self.create_subscription(Float64MultiArray, '/ekf/estimated_state', self.ekf_callback, 10)
        self.thrust_sub = self.create_subscription(Float64MultiArray, '/actuator_outputs', self.thrust_callback, 10)
        self.waypoint_sub = self.create_subscription(Waypoint, "/waypoints", self.waypoints_callback, 10)

        reset_ax = plt.axes([0.41, 0.05, 0.05, 0.03])
        self.reset_button = Button(reset_ax, 'Reset')
        self.reset_button.on_clicked(self.reset_plots)

    # def publish_desired_speed(self, event):
    #     """Publish the desired velocity to the ROS topic"""
    #     try:
    #         self.current_desired_speed = float(self.desired_speed_textbox.text)  # Get the text input
    #         self.get_logger().info(f"Publishing desired speed: {self.current_desired_speed} knots")
            
    #         # Create and publish the desired velocity message
    #         msg = Float64()
    #         msg.data = self.current_desired_speed
    #         self.velocity_pub.publish(msg)

    #     except ValueError:
    #         self.get_logger().warn("Invalid input. Please enter a valid numeric value for the desired speed.")


    def publish_desired_speed(self, event):
        """Publish the desired velocity 10 times in a separate thread"""
        try:
            # Get the text input from the TextBox
            self.current_desired_speed = float(self.desired_speed_textbox.text)
            self.get_logger().info(f"Publishing desired speed: {self.current_desired_speed} knots")
            
            # Create and publish the desired velocity message 10 times with a small delay
            msg = Float64()
            msg.data = self.current_desired_speed
            
            # Create a separate thread for publishing
            publish_thread = threading.Thread(target=self.publish_multiple_times, args=(msg,))
            publish_thread.start()

        except ValueError:
            self.get_logger().warn("Invalid input. Please enter a valid numeric value for the desired speed.")

    def publish_multiple_times(self, msg):
        """Publish the message 10 times with a small delay"""
        for _ in range(10):
            self.velocity_pub.publish(msg)
            self.get_logger().info(f"Published {msg.data} knots")
            time.sleep(0.1)  # Small delay between publications

    def thrust_callback(self, msg):  # - 주기가 gps callback 주기랑 같음 - gps data callback받으면 ekf에서 publish 하기때문
        self.steering = msg.data[0]
        self.throttle = msg.data[1]
        self.LOS = msg.data[2]

    def ekf_callback(self, msg):  # - 주기가 gps callback 주기랑 같음 - gps data callback받으면 ekf에서 publish 하기때문
        
        self.x = msg.data[0]
        self.y = msg.data[1]
        self.p = -msg.data[2]  ##################
        self.u = msg.data[3]
        self.v = msg.data[4]
        self.r = msg.data[5]

        self.x_sensor = msg.data[6]
        self.y_sensor = msg.data[7]
        self.p_sensor = -msg.data[8]   ######################
        self.u_sensor = msg.data[9]
        self.v_sensor = msg.data[10]
        self.r_sensor = msg.data[11]


        # self.current_time = (self.get_clock().now() - self.start_time).to_sec()
        self.current_time = (self.get_clock().now() - self.start_time).nanoseconds / 1e9

        self.x_map = (self.x - x_actual_min) / (x_actual_max - x_actual_min) * self.map_width
        self.y_map = self.map_height + (self.y - y_actual_max) / (y_actual_max - y_actual_min) * self.map_height

        self.x_map_sensor = (self.x_sensor - x_actual_min) / (x_actual_max - x_actual_min) * self.map_width
        self.y_map_sensor = self.map_height + (self.y_sensor - y_actual_max) / (y_actual_max - y_actual_min) * self.map_height
        # print(x_map, y_map)


        if len(self.x_data) != len(self.y_data):
            # Reset the x_data and y_data lists if they are not of the same length
            self.time_data = []
            self.x_data = []
            self.y_data = []
            self.p_data = []
            self.u_data = []
            self.v_data = []
            self.r_data = []

            self.x_sensor_data = []
            self.y_sensor_data = []
            self.p_sensor_data = []
            self.u_sensor_data = []
            self.v_sensor_data = []
            self.r_sensor_data = []

            self.steering_data = []
            self.throttle_data = []
            self.los_data = []
            self.du_data = []
            print("Resetting x_data and y_data due to length mismatch.")

        self.x_data.append(self.x_map)
        self.y_data.append(self.y_map)

        self.p_data.append(self.p * 180 / np.pi)
        self.u_data.append(self.u*1.94384)
        self.v_data.append(self.v)
        self.r_data.append(self.r)

        self.x_sensor_data.append(self.x_map_sensor)
        self.y_sensor_data.append(self.y_map_sensor)
        self.p_sensor_data.append(self.p_sensor * 180 / np.pi)
        self.u_sensor_data.append(self.u_sensor*1.94384)
        self.v_sensor_data.append(self.v_sensor)
        self.r_sensor_data.append(self.r_sensor)

        self.steering_data.append(self.steering)
        self.throttle_data.append(self.throttle)
        self.los_data.append(self.LOS* 180 / np.pi)
        self.du_data.append(self.current_desired_speed)

        self.time_data.append(self.current_time)

    def update_plot(self, frame):
        if len(self.time_data) > 1:

            if len(self.x_data) != len(self.y_data):
                # Reset the x_data and y_data lists if they are not of the same length
                self.time_data = []
                self.x_data = []
                self.y_data = []
                self.p_data = []
                self.u_data = []
                self.v_data = []
                self.r_data = []

                self.x_sensor_data = []
                self.y_sensor_data = []
                self.p_sensor_data = []
                self.u_sensor_data = []
                self.v_sensor_data = []
                self.r_sensor_data = []

                self.steering_data = []
                self.throttle_data = []
                self.los_data = []
                self.du_data = []
                print("Resetting x_data and y_data due to length mismatch.")


            self.line1_m.set_data(self.x_sensor_data, self.y_sensor_data)
            self.line1.set_data(self.x_data, self.y_data)
            # self.line1test.set_data(self.x_data[::20], self.y_data[::20])
            # self.ax1.set_xlim(530, 620)
            # self.ax1.set_ylim(230, 300)
            # self.ax1.set_xlim(0, x_end)
            # self.ax1.set_ylim(0, y_end)

            # Rotation matrix for the heading
            # R = np.array([[np.cos(self.p), -np.sin(self.p)],
            #               [np.sin(self.p), np.cos(self.p)]])
            R = np.array([[np.cos(self.p), np.sin(self.p)],
                          [-np.sin(self.p), np.cos(self.p)]])
            # Rotate the hulls and body
            hull1_R = np.dot(R, self.hull1)
            hull2_R = np.dot(R, self.hull2)
            body_R = np.dot(R, self.body)
            # Translate the hulls and body to the specified position
            hull1_R += np.array([self.x_map, self.y_map]).reshape(2, 1)
            hull2_R += np.array([self.x_map, self.y_map]).reshape(2, 1)
            body_R += np.array([self.x_map, self.y_map]).reshape(2, 1)
            direction = np.array([np.cos(self.p), -np.sin(self.p)]) * self.arrow_length
            # Plot the ASV
            self.heron_p1[0].set_xy(np.column_stack((hull1_R[0, :], hull1_R[1, :])))
            self.heron_p2[0].set_xy(np.column_stack((hull2_R[0, :], hull2_R[1, :])))
            self.heron_p3[0].set_xy(np.column_stack((body_R[0, :], body_R[1, :])))

            self.heron_p4.remove()
            self.heron_p4 = self.ax1.arrow(self.x_map, self.y_map, direction[0], direction[1], head_width=0.1, head_length=0.1, fc='g', ec='g')


            # Remove old waypoint circles before drawing new ones
            for circle in self.waypoint_circles:
                circle.remove()
            self.waypoint_circles.clear()

            # Draw updated waypoints as circles
            for x, y in zip(self.waypoints_x, self.waypoints_y):
                circle = patches.Circle((x, y), radius=self.acceptance_radius/(x_actual_max - x_actual_min) * self.map_width, color='red', fill=False)
                self.ax1.add_patch(circle)
                self.waypoint_circles.append(circle)

            # Refresh the figure
            self.fig.canvas.draw_idle()


            if len(self.x_data) != len(self.y_data):
                # Reset the x_data and y_data lists if they are not of the same length
                self.time_data = []
                self.x_data = []
                self.y_data = []
                self.p_data = []
                self.u_data = []
                self.v_data = []
                self.r_data = []

                self.x_sensor_data = []
                self.y_sensor_data = []
                self.p_sensor_data = []
                self.u_sensor_data = []
                self.v_sensor_data = []
                self.r_sensor_data = []

                self.steering_data = []
                self.throttle_data = []
                self.los_data = []
                self.du_data = []
                print("Resetting x_data and y_data due to length mismatch.")

            self.line2_m.set_data(self.time_data, self.u_sensor_data)
            self.line2.set_data(self.time_data, self.du_data)
            self.ax2.set_xlim(self.time_data[-1] - 20, self.time_data[-1])
            self.ax2.set_ylim(-1, 25.0)

            self.line3_m.set_data(self.time_data, self.los_data)
            # self.line3.set_data(self.time_data, self.v_data)
            self.ax3.set_xlim(self.time_data[-1] - 20, self.time_data[-1])
            self.ax3.set_ylim(-90, 90)

            self.line4_m.set_data(self.time_data, self.steering_data)
            # self.line4.set_data(self.time_data, self.r_data)
            self.ax4.set_xlim(self.time_data[-1] - 20, self.time_data[-1])
            self.ax4.set_ylim(900.0, 2000.0)

            self.line5_m.set_data(self.time_data, self.throttle_data)
            # self.line5.set_data(self.time_data, self.p_data)
            self.ax5.set_xlim(self.time_data[-1] - 20, self.time_data[-1])
            self.ax5.set_ylim(900.0, 2000.0)

            self.line6_left.set_data(self.time_data, self.p_sensor_data)
            # self.line6_right.set_data(self.time_data, self.throttle_data)
            self.ax6.set_xlim(self.time_data[-1] - 20, self.time_data[-1])
            self.ax6.set_ylim(-180, 180)

                    

            self.ax1.relim()
            self.ax2.relim()
            self.ax3.relim()
            self.ax4.relim()
            self.ax5.relim()
            self.ax6.relim()
            # self.ax7.relim()
            # self.ax8.relim()
            # self.ax9.relim()

            self.ax1.autoscale_view()
            self.ax2.autoscale_view()
            self.ax3.autoscale_view()
            self.ax4.autoscale_view()
            self.ax5.autoscale_view()
            self.ax6.autoscale_view()
            # self.ax7.autoscale_view()
            # self.ax8.autoscale_view()
            # self.ax9.autoscale_view()

            # Update the inset plot
            self.axins.clear()
            if (len(self.x_data) != len(self.y_data)) or (len(self.x_sensor_data) != len(self.y_sensor_data)):
                # Reset the x_data and y_data lists if they are not of the same length
                self.time_data = []
                self.x_data = []
                self.y_data = []
                self.p_data = []
                self.u_data = []
                self.v_data = []
                self.r_data = []

                self.x_sensor_data = []
                self.y_sensor_data = []
                self.p_sensor_data = []
                self.u_sensor_data = []
                self.v_sensor_data = []
                self.r_sensor_data = []

                self.steering_data = []
                self.throttle_data = []
                self.los_data = []
                self.du_data = []
                print("Resetting x_data and y_data due to length mismatch.")
            # self.axins.plot(self.x_data, self.y_data, 'k-')
            min_len = min(len(self.x_data), len(self.y_data))
            self.axins.plot(self.x_data[:min_len], self.y_data[:min_len], 'k-')

            # self.axins.plot(self.x_sensor_data, self.y_sensor_data, 'r-')
            self.axins.fill(hull1_R[0, :], hull1_R[1, :], 'b', alpha=0.35)
            self.axins.fill(hull2_R[0, :], hull2_R[1, :], 'b', alpha=0.35)
            self.axins.fill(body_R[0, :], body_R[1, :], 'b', alpha=0.35)
            self.axins.arrow(self.x_map, self.y_map, direction[0], direction[1], head_width=0.1, head_length=0.1, fc='g', ec='g')
            self.axins.axis('equal')
            self.axins.set_xlim(self.x_map - 3, self.x_map + 3)
            self.axins.set_ylim(self.y_map - 3, self.y_map + 3)
            # self.axins.get_xaxis().set_visible(False)
            # self.axins.get_yaxis().set_visible(False)

            # self.fig.tight_layout()  # axes 사이 간격을 적당히 벌려줍니다.

            return self.line1, self.line2, self.line3_m, self.line4_m, self.line5_m

    def waypoints_callback(self, msg):
        """Convert waypoints from latitude/longitude to GUI map coordinates and update the plot."""
        self.waypoints_x.clear()
        self.waypoints_y.clear()

        for i in range(len(msg.x_lat)):
            lat, lon = msg.x_lat[i], msg.y_long[i]
            
            # Convert lat/lon to UTM coordinates using the `utm` library
            utm_easting, utm_northing, _, _ = utm.from_latlon(lat, lon)
            
            # Convert UTM coordinates to GUI coordinates
            x_map = (utm_easting - x_actual_min) / (x_actual_max - x_actual_min) * self.map_width
            y_map = self.map_height + (utm_northing - y_actual_max) / (y_actual_max - y_actual_min) * self.map_height

        # self.x_map = (self.x - x_actual_min) / (x_actual_max - x_actual_min) * self.map_width
        # self.y_map = self.map_height + (self.y - y_actual_max) / (y_actual_max - y_actual_min) * self.map_height


            self.waypoints_x.append(x_map)
            self.waypoints_y.append(y_map)

    def reset_plots(self, event):
        self.start_time = self.get_clock().now()
        self.current_time = self.get_clock().now()


        self.time_data = []
        self.x_data = []
        self.y_data = []
        self.p_data = []
        self.u_data = []
        self.v_data = []
        self.r_data = []

        self.x_sensor_data = []
        self.y_sensor_data = []
        self.p_sensor_data = []
        self.u_sensor_data = []
        self.v_sensor_data = []
        self.r_sensor_data = []

        self.steering_data = []
        self.throttle_data = []
        self.los_data = []
        self.du_data = []

        self.line1_m.set_data([], [])
        self.line1.set_data([], [])
        self.line1test.set_data([], [])
        self.line2_m.set_data([], [])
        self.line2.set_data([], [])
        self.line3_m.set_data([], [])
        # self.line3.set_data([], [])
        self.line4_m.set_data([], [])
        # self.line4.set_data([], [])
        self.line5_m.set_data([], [])
        # self.line5.set_data([], [])
        self.line6_left.set_data([], [])
        # self.line6_right.set_data([], [])
        # self.line7_left.set_data([], [])



        self.ax1.relim()
        self.ax1.autoscale_view()
        self.ax2.relim()
        self.ax2.autoscale_view()
        self.ax3.relim()
        self.ax3.autoscale_view()
        self.ax4.relim()
        self.ax4.autoscale_view()
        self.ax5.relim()
        self.ax5.autoscale_view()
        self.ax6.relim()
        self.ax6.autoscale_view()


    def run(self):
        """Run Matplotlib animation with ROS2 spin in a separate thread."""
        thread = threading.Thread(target=rclpy.spin, args=(self,), daemon=True)
        thread.start()

        ani = FuncAnimation(self.fig, self.update_plot, blit=False, interval=100)
        plt.show()

        # Shutdown ROS2 after plot is closed
        rclpy.shutdown()




def main(args=None):
    rclpy.init(args=args)
    ekf_plotter = SensorFusionEKF()

    try:
        ekf_plotter.run()
    except KeyboardInterrupt:
        pass
    finally:
        ekf_plotter.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

