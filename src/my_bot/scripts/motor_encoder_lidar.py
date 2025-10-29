#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import serial
import re
import math
from tf_transformations import quaternion_from_euler
import time

class RobotBridge(Node):
    def __init__(self):
        super().__init__('robot_bridge')

        # ---- Serial ----
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        time.sleep(2)  # wait for Arduino reset
        self.ser.write(b'SCAN:START\n')  # trigger servo sweep

        # ---- ROS pubs/subs ----
        self.create_subscription(Twist, '/cmd_vel', self.cmd_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.timer = self.create_timer(0.1, self.read_serial)  # 10Hz

        # ---- Robot geometry ----
        self.ticks_per_rev = 500
        self.wheel_radius = 0.033
        self.wheel_base = 0.16
        self.last_left = 0
        self.last_right = 0
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # ---- Lidar ----
        self.num_points = 90
        self.angle_min = math.radians(10)
        self.angle_max = math.radians(170)
        self.angle_increment = (self.angle_max - self.angle_min) / (self.num_points - 1)
        self.scan_ranges = [0.0] * self.num_points

    def cmd_callback(self, msg):
        # Convert Twist to left/right PWM
        v = msg.linear.x
        w = msg.angular.z
        v_l = v - w * self.wheel_base / 2
        v_r = v + w * self.wheel_base / 2
        pwm_l = int(max(min(v_l * 300, 255), -255))
        pwm_r = int(max(min(v_r * 300, 255), -255))
        cmd = f"L:{pwm_l} R:{pwm_r}\n"
        self.ser.write(cmd.encode())

    def read_serial(self):
        try:
            line = self.ser.readline().decode(errors='ignore').strip()
            if not line:
                return

            # Parse Arduino serial: "EL:123 ER:456 LID:angle,distance"
            el_match = re.search(r"EL:(-?\d+)", line)
            er_match = re.search(r"ER:(-?\d+)", line)
            lid_match = re.search(r"LID:(\d+),([\d.-]+)", line)

            # ---- 1. Odometry from encoders ----
            if el_match and er_match:
                left_ticks = int(el_match.group(1))
                right_ticks = int(er_match.group(1))

                dL = (left_ticks - self.last_left) * 2 * math.pi * self.wheel_radius / self.ticks_per_rev
                dR = (right_ticks - self.last_right) * 2 * math.pi * self.wheel_radius / self.ticks_per_rev
                self.last_left = left_ticks
                self.last_right = right_ticks

                dC = (dL + dR) / 2
                dTheta = (dR - dL) / self.wheel_base

                self.x += dC * math.cos(self.theta + dTheta / 2)
                self.y += dC * math.sin(self.theta + dTheta / 2)
                self.theta += dTheta
                q = quaternion_from_euler(0, 0, self.theta)

                odom = Odometry()
                odom.header.stamp = self.get_clock().now().to_msg()
                odom.header.frame_id = 'odom'
                odom.child_frame_id = 'base_link'
                odom.pose.pose.position.x = self.x
                odom.pose.pose.position.y = self.y
                odom.pose.pose.orientation.x = q[0]
                odom.pose.pose.orientation.y = q[1]
                odom.pose.pose.orientation.z = q[2]
                odom.pose.pose.orientation.w = q[3]
                self.odom_pub.publish(odom)

            # ---- 2. Lidar scan ----
            if lid_match:
                angle = float(lid_match.group(1))
                distance = float(lid_match.group(2)) / 100.0  # convert cm to meters
                idx = int((angle - 10) / (170 - 10) * (self.num_points - 1))
                if 0 <= idx < self.num_points:
                    self.scan_ranges[idx] = distance

                scan = LaserScan()
                scan.header.stamp = self.get_clock().now().to_msg()
                scan.header.frame_id = 'base_link'
                scan.angle_min = self.angle_min
                scan.angle_max = self.angle_max
                scan.angle_increment = self.angle_increment
                scan.range_min = 0.02
                scan.range_max = 4.0
                scan.ranges = self.scan_ranges
                self.scan_pub.publish(scan)

        except Exception as e:
            self.get_logger().error(str(e))

        time.sleep(0.05)


def main():
    rclpy.init()
    node = RobotBridge()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
