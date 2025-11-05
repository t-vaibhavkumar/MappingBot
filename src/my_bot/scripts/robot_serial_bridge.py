#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import quaternion_from_euler
from tf2_ros import TransformBroadcaster
import serial
import time
import math


class RobotSerialBridge(Node):
    def __init__(self):
        super().__init__('robot_serial_bridge')

        # ------------------ Parameters ------------------
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_radius', 0.03)
        self.declare_parameter('wheel_separation', 0.30)
        self.declare_parameter('ticks_per_rev', 36.0)
        self.declare_parameter('range_min_m', 0.02)
        self.declare_parameter('range_max_m', 4.0)
        self.declare_parameter('angle_min_deg', 10.0)
        self.declare_parameter('angle_max_deg', 170.0)
        self.declare_parameter('step_deg', 2.0)

        # ------------------ ROS interfaces ------------------
        self.cmd_sub = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.get_logger().info('Subscribed to /cmd_vel and publishing odom, scan, and TFs')

        # ------------------ Serial setup ------------------
        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)
        self.get_logger().info(f'Connected to {port} at {baud} baud')

        # ------------------ Parameters ------------------
        self.wheel_radius = self.get_parameter('wheel_radius').get_parameter_value().double_value
        self.wheel_sep = self.get_parameter('wheel_separation').get_parameter_value().double_value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').get_parameter_value().double_value
        self.range_min = self.get_parameter('range_min_m').get_parameter_value().double_value
        self.range_max = self.get_parameter('range_max_m').get_parameter_value().double_value
        self.angle_min_deg = self.get_parameter('angle_min_deg').get_parameter_value().double_value
        self.angle_max_deg = self.get_parameter('angle_max_deg').get_parameter_value().double_value
        self.step_deg = self.get_parameter('step_deg').get_parameter_value().double_value

        # ------------------ Odometry state ------------------
        self.prev_left_ticks = 0
        self.prev_right_ticks = 0
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0
        self.last_time = self.get_clock().now()

        # ------------------ LIDAR buffer ------------------
        self.expected_steps = int((self.angle_max_deg - self.angle_min_deg) / self.step_deg) + 1
        self.scan_buffer = [float('inf')] * self.expected_steps

        # Create scan timer for continuous publishing
        self.create_timer(0.5, self.publish_scan)  # 2 Hz
        self.create_timer(0.01, self.read_serial)

        self.get_logger().info('RobotSerialBridge started ✅')

    # ------------------ /cmd_vel handling ------------------
    def cmd_vel_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        v_left = linear - (angular * self.wheel_sep / 2.0)
        v_right = linear + (angular * self.wheel_sep / 2.0)
        cmd = f"VEL {v_left:.3f} {v_right:.3f}\n"

        try:
            self.ser.write(cmd.encode('utf-8'))
        except Exception as e:
            self.get_logger().error(f"Failed to send cmd_vel: {e}")

    # ------------------ Serial Reading ------------------
    def read_serial(self):
        while self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            parts = line.split()
            if len(parts) < 3:
                continue

            if parts[0] == "ENC":
                self.handle_encoder(parts)
            elif parts[0] == "LID":
                self.handle_lidar(parts)

    # ------------------ Encoder Handling ------------------
    def handle_encoder(self, parts):
        try:
            left_ticks = int(parts[1])
            right_ticks = int(parts[2])
        except ValueError:
            return

        dl = left_ticks - self.prev_left_ticks
        dr = right_ticks - self.prev_right_ticks
        self.prev_left_ticks = left_ticks
        self.prev_right_ticks = right_ticks

        dist_l = 2 * math.pi * self.wheel_radius * (dl / self.ticks_per_rev)
        dist_r = 2 * math.pi * self.wheel_radius * (dr / self.ticks_per_rev)
        dist = (dist_r + dist_l) / 2.0
        dth = (dist_r - dist_l) / self.wheel_sep

        self.x += dist * math.cos(self.th + dth / 2.0)
        self.y += dist * math.sin(self.th + dth / 2.0)
        self.th += dth

        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        vx_raw = dist / dt if dt > 0 else 0.0
        vth_raw = dth / dt if dt > 0 else 0.0
        self.last_time = now

        # --- Smooth velocities (exponential moving average) ---
        ALPHA = 0.3  # higher = more responsive, lower = smoother
        if not hasattr(self, 'vx_filtered'):
            self.vx_filtered = vx_raw
            self.vth_filtered = vth_raw
        else:
            self.vx_filtered = ALPHA * vx_raw + (1 - ALPHA) * self.vx_filtered
            self.vth_filtered = ALPHA * vth_raw + (1 - ALPHA) * self.vth_filtered

        vx = self.vx_filtered
        vth = self.vth_filtered

        q = quaternion_from_euler(0, 0, self.th)

        # TF
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = 'odom'
        t.child_frame_id = 'base_link'
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        t.transform.translation.z = 0.0
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)

        # Odometry message
        msg = Odometry()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'odom'
        msg.child_frame_id = 'base_link'
        msg.pose.pose.position.x = self.x
        msg.pose.pose.position.y = self.y
        msg.pose.pose.orientation.x = q[0]
        msg.pose.pose.orientation.y = q[1]
        msg.pose.pose.orientation.z = q[2]
        msg.pose.pose.orientation.w = q[3]
        msg.twist.twist.linear.x = vx
        msg.twist.twist.angular.z = vth
        self.odom_pub.publish(msg)

    # ------------------ LIDAR Handling ------------------

    def handle_lidar(self, parts):
        try:
            ang = float(parts[1])
            rng_mm = float(parts[2])
        except ValueError:
            return

        MAX_VALID_RANGE = 0.5  # meters, depends on your sensor
        if rng_mm >= 60000:
            rng = float('inf')
        else:
            rng = rng_mm / 1000.0
            if rng < self.range_min:
                rng = float('nan')
            elif rng > MAX_VALID_RANGE:
                rng = float('inf')

        idx = int((ang - self.angle_min_deg) / self.step_deg)
        if 0 <= idx < len(self.scan_buffer):
            self.scan_buffer[idx] = rng  # continuous update

    # ------------------ Continuous LIDAR Publishing ------------------
    def publish_scan(self):
        msg = LaserScan()
        now = self.get_clock().now()
        msg.header.stamp = now.to_msg()
        msg.header.frame_id = 'lidar_link'
        msg.angle_min = math.radians(self.angle_min_deg)
        msg.angle_max = math.radians(self.angle_max_deg)
        msg.angle_increment = math.radians(self.step_deg)
        msg.range_min = self.range_min
        msg.range_max = self.range_max
        msg.ranges = self.scan_buffer
        msg.intensities = []
        msg.scan_time = 0.1
        msg.time_increment = msg.scan_time / len(self.scan_buffer)
        self.scan_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotSerialBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
