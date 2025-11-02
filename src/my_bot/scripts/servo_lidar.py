#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
import serial
import time
from sensor_msgs.msg import LaserScan
import math


class ServoLidarBridge(Node):
    def __init__(self):
        super().__init__('servo_lidar_bridge')

        # ----- Parameters -----
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('angle_min_deg', 0.0)
        self.declare_parameter('angle_max_deg', 180.0)
        self.declare_parameter('step_deg', 2.0)
        self.declare_parameter('range_max_m', 4.0)
        self.declare_parameter('range_min_m', 0.02)

        port = self.get_parameter('port').get_parameter_value().string_value
        baud = self.get_parameter('baud').get_parameter_value().integer_value
        self.angle_min = math.radians(self.get_parameter('angle_min_deg').get_parameter_value().double_value)
        self.angle_max = math.radians(self.get_parameter('angle_max_deg').get_parameter_value().double_value)
        self.step_deg = self.get_parameter('step_deg').get_parameter_value().double_value
        self.step = math.radians(self.step_deg)
        self.range_max = self.get_parameter('range_max_m').get_parameter_value().double_value
        self.range_min = self.get_parameter('range_min_m').get_parameter_value().double_value

        # ----- Serial setup -----
        self.ser = serial.Serial(port, baud, timeout=1)
        time.sleep(2)
        self.get_logger().info(f'✅ Connected to {port} at {baud} baud')

        # ----- ROS publisher -----
        self.pub = self.create_publisher(LaserScan, 'scan', 10)

        # ----- Internal buffers -----
        self.buffer = {}
        self.last_angle = None
        self.last_direction = None  # 1 = increasing, -1 = decreasing

        # Expected number of steps
        steps = int(round((math.degrees(self.angle_max) - math.degrees(self.angle_min)) / self.step_deg)) + 1
        self.expected_steps = steps

        # Timers
        self.create_timer(0.05, self.read_serial)         # Read serial data
        self.create_timer(1.0, self.publish_partial_scan) # Publish partial scans every second

    # ------------------------------------------------------------------
    def read_serial(self):
        """Continuously read data from Arduino over serial."""
        while self.ser.in_waiting:
            line = self.ser.readline().decode('utf-8', errors='ignore').strip()
            if not line:
                continue

            parts = line.split()
            if parts[0] != 'LID' or len(parts) < 3:
                continue

            try:
                ang = float(parts[1])
                rng_mm = float(parts[2])
            except ValueError:
                continue

            # Convert mm to meters and clamp
            if rng_mm >= 60000:  # sentinel for no echo
                rng = float('inf')
            else:
                rng = rng_mm / 1000.0
                if rng < self.range_min:
                    rng = float('nan')
                elif rng > self.range_max:
                    rng = float('inf')

            # Store reading
            self.buffer[int(round(ang))] = rng

            # Detect direction change (when servo reverses)
            if self.last_angle is not None:
                direction = 1 if ang > self.last_angle else -1
                if self.last_direction is not None and direction != self.last_direction:
                    # Servo reversed direction — publish full scan
                    self.publish_full_scan()
                    self.buffer.clear()

                self.last_direction = direction
            self.last_angle = ang

    # ------------------------------------------------------------------
    def publish_partial_scan(self):
        """Publish a partial scan periodically (every second)."""
        if not self.buffer:
            return

        msg = LaserScan()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = 'lidar_link'
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.step
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        # Use current buffer (sorted)
        angles = sorted(self.buffer.keys())
        ranges = [self.buffer[a] for a in angles]

        msg.ranges = ranges
        msg.scan_time = 1.0
        msg.time_increment = msg.scan_time / max(1, len(ranges))
        self.pub.publish(msg)
        self.get_logger().info(f'🌀 Partial scan published ({len(ranges)} pts)')

    # ------------------------------------------------------------------
    def publish_full_scan(self):
        """Publish a full scan when servo completes a sweep."""
        if not self.buffer:
            return

        msg = LaserScan()
        now = self.get_clock().now().to_msg()
        msg.header.stamp = now
        msg.header.frame_id = 'lidar_link'
        msg.angle_min = self.angle_min
        msg.angle_max = self.angle_max
        msg.angle_increment = self.step
        msg.range_min = self.range_min
        msg.range_max = self.range_max

        # Sort and fill ranges
        angles = sorted(self.buffer.keys())
        ranges = [self.buffer[a] for a in angles]

        msg.ranges = ranges
        msg.scan_time = 0.5
        msg.time_increment = msg.scan_time / max(1, len(ranges))
        self.pub.publish(msg)
        self.get_logger().info(f'✅ Full scan published ({len(ranges)} pts)')


# ----------------------------------------------------------------------
def main(args=None):
    rclpy.init(args=args)
    node = ServoLidarBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
