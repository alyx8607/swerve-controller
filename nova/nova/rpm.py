import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray

import serial
import math
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import threading
from collections import deque
import signal
import sys
import time


class CmdVelToRPM(Node):
    def __init__(self, port="/dev/ttyACM2", baud=115200):
        super().__init__('cmdvel_to_rpm')

        # Open USB serial
        try:
            self.ser = serial.Serial(port, baud, timeout=1)
            self.get_logger().info(f"Opened serial on {port} @ {baud}")
        except Exception as e:
            self.get_logger().error(f"Failed to open serial: {e}")
            raise

        # Robot params
        self.wheel_separation = 0.92  # meters
        self.wheel_diameter = 0.37    # meters

        # ROS2 subscriptions & publishers
        self.subscription = self.create_subscription(
            Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.pub_debug = self.create_publisher(Float32MultiArray, '/pid_debug', 10)

        # Start a background thread to read STM32 feedback
        threading.Thread(target=self.read_serial, daemon=True).start()

    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z

        # Differential-drive kinematics
        v_left = v - (w * self.wheel_separation / 2.0)
        v_right = v + (w * self.wheel_separation / 2.0)

        # Convert to RPM
        rpm_left = (v_left / (math.pi * self.wheel_diameter)) * 60.0
        rpm_right = (v_right / (math.pi * self.wheel_diameter)) * 60.0

        # Example PID constants
        p, i, d = 62.5, 8.7, 0.01

        data = f"{int(rpm_left)} {int(rpm_right)} {p} {i} {d}\n"
        try:
            self.ser.write(data.encode('utf-8'))
            self.get_logger().info(f"Sent: {data.strip()}")
        except Exception as e:
            self.get_logger().error(f"Serial write failed: {e}")

    def read_serial(self):
        """Read STM32 feedback and publish on /pid_debug"""
        while rclpy.ok():
            try:
                line = self.ser.readline().decode().strip()
                if not line:
                    continue
                parts = line.split(",")
                if len(parts) == 4:
                    rightAct, rightExp, leftAct, leftExp = map(float, parts)
                    msg = Float32MultiArray()
                    msg.data = [rightAct, rightExp, leftAct, leftExp]
                    self.pub_debug.publish(msg)
                else:
                    self.get_logger().warn(f"Skipping invalid line: {line}")
            except Exception as e:
                self.get_logger().error(f"Parse error: {e}")

    def send_zero_rpm(self, duration=0.5):
        """Send 0 0 RPM repeatedly for given duration (default 0.5s)"""
        p, i, d = 62.5, 8.7, 0.01
        data = f"0 0 {p} {i} {d}\n".encode('utf-8')
        end_time = time.time() + duration
        try:
            while time.time() < end_time:
                self.ser.write(data)
                time.sleep(0.05)
            self.get_logger().info("Sent zero RPM before shutdown.")
        except Exception as e:
            self.get_logger().error(f"Failed to send zero RPM: {e}")

    def close_serial(self):
        """Safely close the serial connection"""
        try:
            if self.ser.is_open:
                self.ser.close()
                self.get_logger().info("Serial connection closed.")
        except Exception as e:
            self.get_logger().error(f"Error closing serial: {e}")


class LivePlotter(Node):
    def __init__(self, max_points=200):
        super().__init__('live_plotter')

        self.data_buffer = deque(maxlen=max_points)
        self.subscription = self.create_subscription(
            Float32MultiArray, '/pid_debug', self.debug_callback, 10
        )

        self.max_points = max_points

        # ---- Create 2 subplots ----
        self.fig, (self.ax_r, self.ax_l) = plt.subplots(2, 1, figsize=(10, 8), sharex=True)

        x = np.arange(max_points)

        # --- Right Motor ---
        self.line_r_act, = self.ax_r.plot(x, np.zeros(max_points), label="Right Actual", linestyle='-', color='b')
        self.line_r_exp, = self.ax_r.plot(x, np.zeros(max_points), label="Right Expected", linestyle='--', color='y')
        self.ax_r.set_ylim(-60, 60)
        self.ax_r.set_ylabel("RPM")
        self.ax_r.set_title("Right Motor PID Data")
        self.ax_r.legend()
        self.ax_r.grid(True)

        # --- Left Motor ---
        self.line_l_act, = self.ax_l.plot(x, np.zeros(max_points), label="Left Actual", linestyle='-', color='b')
        self.line_l_exp, = self.ax_l.plot(x, np.zeros(max_points), label="Left Expected", linestyle='--', color='y')
        self.ax_l.set_ylim(-60, 60)
        self.ax_l.set_xlabel("Samples")
        self.ax_l.set_ylabel("RPM")
        self.ax_l.set_title("Left Motor PID Data")
        self.ax_l.legend()
        self.ax_l.grid(True)

        # Quit on 'q'
        def on_key(event):
            if event.key == 'q':
                plt.close(event.canvas.figure)
        self.fig.canvas.mpl_connect('key_press_event', on_key)

        # Animation loop
        self.ani = animation.FuncAnimation(
            self.fig, self.update_plot, interval=50, blit=True
        )

    def debug_callback(self, msg):
        # msg.data = [rightAct, rightExp, leftAct, leftExp]
        self.data_buffer.append(tuple(msg.data))

    def update_plot(self, frame):
        if not self.data_buffer:
            return (self.line_r_act, self.line_r_exp,
                    self.line_l_act, self.line_l_exp)

        # Unpack data buffer
        rightAct  = [d[0] for d in self.data_buffer]
        rightExp  = [d[1] for d in self.data_buffer]
        leftAct   = [d[2] for d in self.data_buffer]
        leftExp   = [d[3] for d in self.data_buffer]

        # Padding
        pad = self.max_points - len(rightAct)

        self.line_r_act.set_ydata(np.pad(rightAct, (pad, 0)))
        self.line_r_exp.set_ydata(np.pad(rightExp, (pad, 0)))
        self.line_l_act.set_ydata(np.pad(leftAct, (pad, 0)))
        self.line_l_exp.set_ydata(np.pad(leftExp, (pad, 0)))

        return (self.line_r_act, self.line_r_exp,
                self.line_l_act, self.line_l_exp)

def main(args=None):
    rclpy.init(args=args)

    node_cmd = CmdVelToRPM(port="/dev/ttyACM0", baud=115200)
    node_plot = LivePlotter()

    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node_cmd)
    executor.add_node(node_plot)

    # Define signal handler for Ctrl + C
    def handle_sigint(sig, frame):
        node_cmd.get_logger().info("Ctrl+C detected, sending zero RPM and shutting down...")
        node_cmd.send_zero_rpm(duration=0.5)
        node_cmd.close_serial()
        executor.shutdown()
        node_cmd.destroy_node()
        node_plot.destroy_node()
        rclpy.shutdown()
        plt.close('all')
        sys.exit(0)

    signal.signal(signal.SIGINT, handle_sigint)

    # Run ROS2 spinning in a background thread
    spin_thread = threading.Thread(target=executor.spin, daemon=True)
    spin_thread.start()

    # Run matplotlib in main thread (safe)
    try:
        plt.show()
    except KeyboardInterrupt:
        handle_sigint(None, None)

    # Cleanup after plot window is closed
    handle_sigint(None, None)


if __name__ == '__main__':
    main()
