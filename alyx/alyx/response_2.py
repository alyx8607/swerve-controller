#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
import math

import matplotlib.pyplot as plt


class CmdVsOdomPlotter(Node):

    def __init__(self):
        super().__init__('cmd_vs_odom_plotter')

        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 50)
        self.create_subscription(Odometry, '/dlio/odom_node/odom', self.odom_callback, 50)

        # keep full history
        self.time_data = []
        self.cmd_linear = []
        self.cmd_angular = []
        self.odom_linear_x = []
        self.odom_linear_y = []
        self.imu_gyro = []

        self.latest_cmd_linear = 0.0
        self.latest_cmd_angular = 0.0
        self.latest_imu_gyro = 0.0

        self.start_time = None

        plt.ion()

        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1)

        self.line_cmd_lin, = self.ax1.plot([], [], label="cmd_vel.linear.x")
        self.line_odom_lin_x, = self.ax1.plot([], [], label="odom.linear.x")
        self.line_odom_lin_y, = self.ax1.plot([], [], label="odom.linear.x_prev")

        self.line_cmd_ang, = self.ax2.plot([], [], label="cmd_vel.angular.z")
        self.line_imu_gyro, = self.ax2.plot([], [], label="imu.angular_velocity.z")

        self.ax1.legend()
        self.ax2.legend()

        self.timer = self.create_timer(0.05, self.update_plot)

    def cmd_callback(self, msg):
        self.latest_cmd_linear = msg.linear.x
        self.latest_cmd_angular = msg.angular.z

    def imu_callback(self, msg):
        self.latest_imu_gyro = msg.angular_velocity.z

    def odom_callback(self, msg):

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.start_time is None:
            self.start_time = t

        rel_time = t - self.start_time

        self.time_data.append(rel_time)
        self.cmd_linear.append(self.latest_cmd_linear)
        self.cmd_angular.append(self.latest_cmd_angular)
        self.odom_linear_x.append((((self.latest_cmd_linear) / (abs(self.latest_cmd_linear))) if self.latest_cmd_linear != 0.0 else 1) * math.sqrt(msg.twist.twist.linear.x * msg.twist.twist.linear.x + msg.twist.twist.linear.y * msg.twist.twist.linear.y))
        self.odom_linear_y.append(msg.twist.twist.linear.x)
        self.imu_gyro.append(self.latest_imu_gyro)

    def update_plot(self):

        if len(self.time_data) < 2:
            return

        self.line_cmd_lin.set_data(self.time_data, self.cmd_linear)
        self.line_odom_lin_x.set_data(self.time_data, self.odom_linear_x)
        self.line_odom_lin_y.set_data(self.time_data, self.odom_linear_y)

        self.line_cmd_ang.set_data(self.time_data, self.cmd_angular)
        self.line_imu_gyro.set_data(self.time_data, self.imu_gyro)

        self.ax1.relim()
        self.ax1.autoscale_view()

        self.ax2.relim()
        self.ax2.autoscale_view()

        plt.draw()
        plt.pause(0.001)


def main(args=None):

    rclpy.init(args=args)

    node = CmdVsOdomPlotter()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
