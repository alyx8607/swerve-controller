import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
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

        try:
            # ---- Serial ----
            self.ser = serial.Serial(
                port='/dev/ttyUSB0',
                baudrate=115200,
                timeout=0.01
            )
            time.sleep(2)
        except:
            print("Failed to open serial port")

        self.sub = self.create_subscription(
            Twist,
            'cmd_vel',
            self.cmd_vel_callback,
            10
        )


    def cmd_vel_callback(self, msg):
        now = time.time()

        if now - self.last_serial_time < self.serial_period:
            return
        
        self.last_serial_time = now
        
        vx = msg.linear.x     # m/s
        vy = msg.linear.y     # m/s
        omega = msg.angular.z # rad/s

        wheel_states = self.compute_swerve(vx, vy, omega)
        
        self.send_serial(wheel_states)


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