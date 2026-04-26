import rclpy
from rclpy.node import Node
import time
from sensor_msgs.msg import Imu, NavSatFix, NavSatStatus

from pymavlink import mavutil
import threading
import math

CONNECTION_STRING = "/dev/ttyACM0"
BAUD = 115200


def euler_to_quaternion(roll, pitch, yaw):
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)

    return (
        float(sr * cp * cy - cr * sp * sy),
        float(cr * sp * cy + sr * cp * sy),
        float(cr * cp * sy - sr * sp * cy),
        float(cr * cp * cy + sr * sp * sy)
    )

def wrap_angle(angle):
    while angle > math.pi:
        angle -= 2 * math.pi
    while angle < -math.pi:
        angle += 2 * math.pi
    return angle


class CubeSensorNode(Node):

    def __init__(self):
        super().__init__('cube_sensor_node')

        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.gps_pub = self.create_publisher(NavSatFix, '/fix', 10)

        self.get_logger().info("Connecting to Cube...")

        self.master = mavutil.mavlink_connection(CONNECTION_STRING, baud=BAUD)
        self.master.wait_heartbeat()

        self.get_logger().info("Connected to FCU")

        self.master.mav.request_data_stream_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_DATA_STREAM_ALL,
            50,
            1
        )

        self.qx = 0.0
        self.qy = 0.0
        self.qz = 0.0
        self.qw = 1.0

        threading.Thread(target=self.read_loop, daemon=True).start()

    def read_loop(self):
        while rclpy.ok():
            msg = self.master.recv_match(blocking=True)
            if msg is None:
                continue

            t = msg.get_type()

            if t == 'ATTITUDE':
                self.handle_euler(msg)

            elif t == 'RAW_IMU':
                self.handle_imu(msg)

            elif t == 'GLOBAL_POSITION_INT':
                self.handle_gps(msg)

    # ---------- ORIENTATION (ONLY NED → ENU) ----------

    def handle_euler(self, msg):
        qx, qy, qz, qw = euler_to_quaternion(
            msg.roll,
            msg.pitch,
            wrap_angle(msg.yaw - math.pi/2)
        )


        self.qx = qx
        self.qy = -qy
        self.qz = -qz
        self.qw = qw

    # ---------- IMU (ONLY NED → ENU) ----------

    def handle_imu(self, msg):
        imu = Imu()

        imu.header.stamp = self.get_clock().now().to_msg()
        imu.header.frame_id = "imu_link"

        # Convert units
        ax = float(msg.xacc * 9.81 / 1000.0)
        ay = float(msg.yacc * 9.81 / 1000.0)
        az = float(msg.zacc * 9.81 / 1000.0)

        gx = float(msg.xgyro / 1000.0)
        gy = float(msg.ygyro / 1000.0)
        gz = float(msg.zgyro / 1000.0)

        # NED → ENU (ONLY)
        imu.linear_acceleration.x = ax
        imu.linear_acceleration.y = -ay
        imu.linear_acceleration.z = -az

        imu.angular_velocity.x = gx
        imu.angular_velocity.y = -gy
        imu.angular_velocity.z = -gz

        imu.orientation.x = self.qx
        imu.orientation.y = self.qy
        imu.orientation.z = self.qz
        imu.orientation.w = self.qw

        imu.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.02
        ]

        imu.angular_velocity_covariance = [
            0.02, 0.0, 0.0,
            0.0, 0.02, 0.0,
            0.0, 0.0, 0.04
        ]

        imu.linear_acceleration_covariance = [
            0.1, 0.0, 0.0,
            0.0, 0.1, 0.0,
            0.0, 0.0, 0.2
        ]

        self.imu_pub.publish(imu)

    # ---------- GPS (UNCHANGED) ----------

    def handle_gps(self, msg):
        gps = NavSatFix()

        gps.header.stamp = self.get_clock().now().to_msg()
        gps.header.frame_id = "gps_link"

        gps.latitude = msg.lat / 1e7
        gps.longitude = msg.lon / 1e7
        gps.altitude = msg.alt / 1000.0

        gps.status.status = NavSatStatus.STATUS_FIX
        gps.status.service = NavSatStatus.SERVICE_GPS

        self.gps_pub.publish(gps)


def main():
    rclpy.init()

    node = CubeSensorNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
