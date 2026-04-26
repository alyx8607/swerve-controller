#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

import matplotlib.pyplot as plt
from collections import deque
import numpy as np


class CmdVsImuPlotter(Node):

    def __init__(self):
        super().__init__('cmd_vs_imu_plotter')

        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.create_subscription(Imu, '/imu/data', self.imu_callback, 50)

        self.max_points = 400

        self.time_data = deque(maxlen=self.max_points)
        self.cmd_linear = deque(maxlen=self.max_points)
        self.cmd_angular = deque(maxlen=self.max_points)

        self.imu_velocity = deque(maxlen=self.max_points)
        self.imu_gyro = deque(maxlen=self.max_points)

        self.latest_cmd_linear = 0.0
        self.latest_cmd_angular = 0.0

        self.start_time = None
        self.last_time = None

        # velocity estimate
        self.velocity = 0.0

        # bias calibration
        self.bias_samples = []
        self.acc_bias = 0.0

        # low pass filter
        self.filtered_acc = 0.0
        self.alpha = 0.9

        plt.ion()

        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1)

        self.line_cmd_lin, = self.ax1.plot([], [], label="cmd_vel.linear.x")
        self.line_imu_vel, = self.ax1.plot([], [], label="imu_estimated_velocity")

        self.line_cmd_ang, = self.ax2.plot([], [], label="cmd_vel.angular.z")
        self.line_imu_gyro, = self.ax2.plot([], [], label="imu.angular_velocity.z")

        self.ax1.set_title("Linear Velocity Response")
        self.ax2.set_title("Angular Response")

        self.ax1.set_ylabel("Linear Velocity (m/s)")
        self.ax2.set_ylabel("Angular Velocity (rad/s)")
        self.ax2.set_xlabel("Time (s)")

        self.ax1.legend()
        self.ax2.legend()

        self.timer = self.create_timer(0.05, self.update_plot)

    def cmd_callback(self, msg):
        self.latest_cmd_linear = msg.linear.x
        self.latest_cmd_angular = msg.angular.z

    def quat_to_rot(self, qx, qy, qz, qw):

        R = np.array([
            [1 - 2*(qy*qy + qz*qz),     2*(qx*qy - qz*qw),     2*(qx*qz + qy*qw)],
            [2*(qx*qy + qz*qw),         1 - 2*(qx*qx + qz*qz), 2*(qy*qz - qx*qw)],
            [2*(qx*qz - qy*qw),         2*(qy*qz + qx*qw),     1 - 2*(qx*qx + qy*qy)]
        ])

        return R

    def imu_callback(self, msg):

        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9

        if self.start_time is None:
            self.start_time = t

        if self.last_time is None:
            self.last_time = t
            return

        dt = t - self.last_time
        self.last_time = t

        rel_time = t - self.start_time

        acc_sensor = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])

        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        R = self.quat_to_rot(qx, qy, qz, qw)

        acc_world = R @ acc_sensor

        gravity = np.array([0.0, 0.0, 9.81])

        acc_motion = acc_world - gravity

        ax = acc_motion[0]

        # collect bias during first 2 seconds
        if rel_time < 1.0:
            self.bias_samples.append(ax)
            return

        if self.acc_bias == 0.0 and len(self.bias_samples) > 0:
            self.acc_bias = np.mean(self.bias_samples)
            self.get_logger().info(f"Estimated accel bias: {self.acc_bias}")

        ax = ax - self.acc_bias

        # low pass filter
        self.filtered_acc = self.alpha * self.filtered_acc + (1 - self.alpha) * ax

        # integrate velocity
        self.velocity += self.filtered_acc * dt

        # zero velocity constraint
        if abs(self.latest_cmd_linear) < 0.05:
            self.velocity = 0.0

        self.time_data.append(rel_time)

        self.cmd_linear.append(self.latest_cmd_linear)
        self.cmd_angular.append(self.latest_cmd_angular)

        self.imu_velocity.append(self.velocity)
        self.imu_gyro.append(msg.angular_velocity.z)

    def update_plot(self):

        if len(self.time_data) < 2:
            return

        self.line_cmd_lin.set_data(self.time_data, self.cmd_linear)
        self.line_imu_vel.set_data(self.time_data, self.imu_velocity)

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

    node = CmdVsImuPlotter()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()