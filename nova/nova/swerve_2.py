import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
import math
import serial
import time


class SwerveController(Node):

    def __init__(self):
        super().__init__('swerve_controller')

        self.L = 0.694    # wheelbase length
        self.W = 0.4132    # wheelbase width

        self.wheel_radius = 0.128  # meters
        self.max_rpm = 300.0

        self.last_serial_time = 0.0
        self.serial_period = 0.025  # seconds

        self.current_angles = {
            'fl': 0.0,
            'fr': 0.0,
            'bl': 0.0,
            'br': 0.0
        }

        self.flipped = {
            'fl': False,
            'fr': False,
            'bl': False,
            'br': False
        }

        self.wheel_odom = {
            'fl': (0.0, 0.0),
            'fr': (0.0, 0.0),
            'bl': (0.0, 0.0),
            'br': (0.0, 0.0)
        }

        self.mode = 3
        self.theta = 0
        self.last_odom_time = None
        self.x = 0.0
        self.y = 0.0
        self.odom_recv = False
        self.serial_buffer_bytes = b""

        try:
            # ---- Serial ----
            self.ser = serial.Serial(
                port='/dev/ttyUSB0',
                baudrate=115200,
                timeout=0
            )
            time.sleep(0.5)
            self.mode = self.read_feedback()
        except:
            print("Failed to open serial port")

        self.sub_1 = self.create_subscription(
            Twist,
            'cmd_vel_1',
            self.cmd_vel_callback_1,
            10
        )

        self.sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, '/alyx/wheel/odom', 10)

        self.create_timer(0.02, self.update_serial)

    def update_serial(self):
        self.mode = self.read_feedback()

        if self.odom_recv:
            self.publish_odom()

    def cmd_vel_callback_1(self, msg):
        if self.mode == 2:
            now = time.time()

            if now - self.last_serial_time < self.serial_period:
                return
            
            self.last_serial_time = now
            
            vx = msg.linear.x     # m/s
            vy = msg.linear.y     # m/s
            omega = msg.angular.z # rad/s

            wheel_states = self.compute_swerve(vx, vy, omega)
            
            self.send_serial(wheel_states)

    def cmd_vel_callback(self, msg):
        if self.mode == 3 or self.mode == 4:
            now = time.time()

            if now - self.last_serial_time < self.serial_period:
                return
            
            self.last_serial_time = now
            
            vx = msg.linear.x     # m/s
            vy = msg.linear.y     # m/s
            omega = msg.angular.z # rad/s

            wheel_states = self.compute_swerve(vx, vy, omega)
            
            self.send_serial(wheel_states)


    def read_feedback(self):
        self.odom_recv = False

        # ---- Read all available bytes (non-blocking) ----
        try:
            if self.ser.in_waiting > 0:
                data = self.ser.read(self.ser.in_waiting)
                self.serial_buffer_bytes += data
        except Exception as e:
            print("Serial read error:", e)
            return self.mode

        i = 0
        buffer = self.serial_buffer_bytes

        while i < len(buffer):

            # ==============================
            # 🔵 MODE PACKET (0xAA mode checksum)
            # ==============================
            if buffer[i] == 0xAA:

                # Ensure full packet exists
                if i + 2 >= len(buffer):
                    break  # wait for next cycle

                mode = buffer[i + 1]
                checksum = buffer[i + 2]

                if checksum == (0xAA ^ mode):
                    self.mode = int(mode)
                    # print(f"Mode: {mode}")
                    i += 3
                else:
                    print("Checksum failed")
                    i += 1

            # ==============================
            # 🟢 ASCII ODOM PACKET (@ ... \n)
            # ==============================
            elif buffer[i] == ord('@'):

                newline_index = buffer.find(b'\n', i)

                if newline_index == -1:
                    break  # incomplete line

                line_bytes = buffer[i:newline_index]
                i = newline_index + 1

                try:
                    line = line_bytes.decode(errors='ignore').strip('\r\n ')
                    line = line.lstrip('@')

                    parts = line.split(';')
                    data = {}

                    for p in parts:
                        p = p.strip()
                        if not p:
                            continue
                        key = p[0:2]
                        value = float(p[2:])
                        data[key] = value

                    self.wheel_odom = {
                        'fl': (-data['B1'], 360 * data['S1'] / 4000),
                        'fr': (data['B2'], 360 * data['S2'] / 4000),
                        'bl': (-data['B3'], 360 * data['S3'] / 4000),
                        'br': (-data['B4'], 360 * data['S4'] / 4000)
                        # 'fl': (data['B1'], data['S1']),
                        # 'fr': (data['B2'], data['S2']),
                        # 'bl': (data['B3'], data['S3']),
                        # 'br': (data['B4'], data['S4'])
                    }
                    print(self.wheel_odom)

                    self.odom_recv = True

                except Exception as e:
                    print("Odom parse error:", e)

            else:
                # Skip unknown byte
                i += 1

        # Keep only unprocessed data
        self.serial_buffer_bytes = buffer[i:]

        return self.mode


    def compute_swerve(self, vx, vy, omega):
        L = self.L
        W = self.W

        # Intermediate values
        a = vx - omega * (W / 2.0)
        b = vx + omega * (W / 2.0)
        c = vy - omega * (L / 2.0)
        d = vy + omega * (L / 2.0)

        wheels = {
            'fl': (a, d),
            'fr': (b, d),
            'bl': (a, c),
            'br': (b, c)
        }

        states = {}

        # Compute raw speeds + angles
        max_speed = 0.0
        for wheel, (vx_w, vy_w) in wheels.items():
            speed = math.hypot(vx_w, vy_w)
            angle = math.degrees(math.atan2(-vy_w, vx_w))
            states[wheel] = {'speed': speed, 'angle': angle}
            max_speed = max(max_speed, speed)

        # Convert to RPM + optimize steering
        for name, s in states.items():
            rpm = self.speed_to_rpm(s['speed'])
            angle, rpm = self.optimize_angle(
                s['angle'],
                self.current_angles[name],
                rpm,
                self.flipped[name],
                name
            )
            self.current_angles[name] = angle
            s['rpm'] = rpm
            s['angle'] = angle

        return states

    
    def optimize_angle(self, target, current, rpm, flipped, name):
        delta = self.wrap_angle(target - current)

        effective_target = self.wrap_angle(target - 180.0) if flipped else target

        # Forbidden zone
        crosses_forbidden = (
            (effective_target <= -90 and -90 <= current < 0) or
            (current <= -100 and -100 <= effective_target < 0)
        )

        print(int(effective_target), " ", int(current), " ", self.flipped[name], " ", crosses_forbidden)

        self.flipped[name] = False

        # Flip if large rotation OR forbidden crossing
        # if (abs(delta) > 90.0 and not crosses_forbidden) or (not (abs(delta) > 90) and crosses_forbidden):
        #     target = self.wrap_angle(target - 180.0)
        #     rpm *= -1.0
        #     self.flipped[name] = True

        return target, rpm


    def wrap_angle(self, angle):
        while angle > 180.0:
            angle -= 360.0
        while angle < -180.0:
            angle += 360.0
        return angle


    def speed_to_rpm(self, speed):
        return (speed / (2 * math.pi * self.wheel_radius)) * 60.0
    
    def rpm_to_speed(self, rpm):
        return (rpm * 2.0 * math.pi / 60.0) * self.wheel_radius
    
    def compute_odom(self, dt):
        v1 = self.rpm_to_speed(self.wheel_odom['fl'][0])
        v2 = self.rpm_to_speed(self.wheel_odom['fr'][0])
        v3 = self.rpm_to_speed(self.wheel_odom['bl'][0])
        v4 = self.rpm_to_speed(self.wheel_odom['br'][0])
        s1 = self.wheel_odom['fl'][1]
        s2 = self.wheel_odom['fr'][1]
        s3 = self.wheel_odom['bl'][1]
        s4 = self.wheel_odom['br'][1]
        
        r1 = np.array([self.L / 2, self.W / 2])
        r2 = np.array([self.L / 2, -1 * self.W / 2])
        r3 = np.array([-1 * self.L / 2, self.W / 2])
        r4 = np.array([-1 * self.L / 2, -1 * self.W / 2])

        vx1 = v1 * math.cos(s1 * math.pi / 180)
        vx2 = v2 * math.cos(s2 * math.pi / 180)
        vx3 = v3 * math.cos(s3 * math.pi / 180)
        vx4 = v4 * math.cos(s4 * math.pi / 180)

        vy1 = -1 * v1 * math.sin(s1 * math.pi / 180)
        vy2 = -1 * v2 * math.sin(s2 * math.pi / 180)
        vy3 = -1 * v3 * math.sin(s3 * math.pi / 180)
        vy4 = -1 * v4 * math.sin(s4 * math.pi / 180)

        v1_vec = np.array([vx1, vy1])
        v2_vec = np.array([vx2, vy2])
        v3_vec = np.array([vx3, vy3])
        v4_vec = np.array([vx4, vy4])

        omega = (np.cross(r1, v1_vec) + np.cross(r2, v2_vec) + np.cross(r3, v3_vec) + np.cross(r4, v4_vec)) / (4 * ((self.L / 2)**2 + (self.W / 2)**2))

        vx = (vx1 + vx2 + vx3 + vx4) / 4
        vy = (vy1 + vy2 + vy3 + vy4) / 4

        theta_mid = self.theta + omega * dt / 2

        vx_w = vx * np.cos(theta_mid) - vy * np.sin(theta_mid)
        vy_w = vx * np.sin(theta_mid) + vy * np.cos(theta_mid)

        self.x += vx_w * dt
        self.y += vy_w * dt

        self.theta += omega * dt
        self.theta = self.wrap_angle(self.theta * 180 / math.pi) * math.pi / 180

        # Quaternion
        qx = 0.0
        qy = 0.0
        qz = np.sin(self.theta / 2)
        qw = np.cos(self.theta / 2)

        return (self.x, self.y), (qx, qy, qz, qw), (vx, vy, omega)
    

    def publish_odom(self):
        now_time = self.get_clock().now()

        if self.last_odom_time is None:
            self.last_odom_time = now_time
            return  # skip first iteration

        dt = (now_time - self.last_odom_time).nanoseconds * 1e-9
        dt = min(dt, 0.1)
        self.last_odom_time = now_time

        (x, y), (qx, qy, qz, qw), (vx, vy, omega) = self.compute_odom(dt)

        now = now_time.to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"

        # Position
        odom.pose.pose.position.x = x
        odom.pose.pose.position.y = y
        odom.pose.pose.position.z = 0.0

        # Orientation
        odom.pose.pose.orientation.x = qx
        odom.pose.pose.orientation.y = qy
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Velocity
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.z = omega

        # Covariances
        for i in range(36):
            odom.pose.covariance[i] = 1e3
            odom.twist.covariance[i] = 1e3

        # Pose covariance
        odom.pose.covariance[0]  = 0.01   # x
        odom.pose.covariance[7]  = 0.01   # y
        odom.pose.covariance[35] = 0.02   # yaw

        # Twist covariance
        odom.twist.covariance[0]  = 0.03  # vx
        odom.twist.covariance[7]  = 0.03  # vy
        odom.twist.covariance[35] = 0.08  # wz

        self.odom_pub.publish(odom)


    def send_serial(self, states):
        msg = (
            f"B1 {int(states['fl']['rpm'])}; S1 {int(states['fl']['angle'])}; "
            f"B2 {int(states['fr']['rpm'])}; S2 {int(states['fr']['angle'])}; "
            f"B3 {int(states['bl']['rpm'])}; S3 {int(states['bl']['angle'])}; "
            f"B4 {int(states['br']['rpm'])}; S4 {int(states['br']['angle'])};\n"
        )
        print(msg)
        try:
            self.ser.write(msg.encode())
        except:
            print("Error writing to serial")


def main():
    rclpy.init()
    node = SwerveController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
