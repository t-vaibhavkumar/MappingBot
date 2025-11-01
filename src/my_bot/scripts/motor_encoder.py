#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import serial
import math
import time
import tf_transformations
from tf2_ros import TransformBroadcaster


class MotorEncoderBridge(Node):
    def __init__(self):
        super().__init__('motor_encoder_bridge')

        # Parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baud', 115200)
        self.declare_parameter('wheel_radius', 0.03)
        self.declare_parameter('wheel_separation', 0.30)
        self.declare_parameter('ticks_per_rev', 60.0)

        port = self.get_parameter('port').value
        baud = self.get_parameter('baud').value

        self.ser = serial.Serial(port, baud, timeout=0.1)
        time.sleep(2)
        self.get_logger().info(f"Connected to Arduino on {port}")

        # Robot parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_sep = self.get_parameter('wheel_separation').value
        self.ticks_per_rev = self.get_parameter('ticks_per_rev').value

        # Robot state
        self.left_ticks_prev = 0
        self.right_ticks_prev = 0
        self.x = 0.0
        self.y = 0.0
        self.th = 0.0

        # ROS publishers/subscribers
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        self.last_time = self.get_clock().now()
        self.create_timer(0.1, self.read_serial)  # 10 Hz

    # ---------------------------
    # Send velocity commands
    # ---------------------------
    def cmd_vel_callback(self, msg):
        v = msg.linear.x
        w = msg.angular.z
        v_l = v - w * self.wheel_sep / 2.0
        v_r = v + w * self.wheel_sep / 2.0

        cmd = f"VEL {v_l:.3f} {v_r:.3f}\n"
        self.ser.write(cmd.encode())

    # ---------------------------
    # Read encoders and publish odom
    # ---------------------------
    def read_serial(self):
        line = self.ser.readline().decode('utf-8').strip()
        if not line.startswith('ENC'):
            return

        try:
            _, l_ticks, r_ticks = line.split()
            l_ticks = int(l_ticks)
            r_ticks = int(r_ticks)
        except ValueError:
            return

        dl = (l_ticks - self.left_ticks_prev) * (2 * math.pi * self.wheel_radius / self.ticks_per_rev)
        dr = (r_ticks - self.right_ticks_prev) * (2 * math.pi * self.wheel_radius / self.ticks_per_rev)
        self.left_ticks_prev = l_ticks
        self.right_ticks_prev = r_ticks

        dt = (self.get_clock().now() - self.last_time).nanoseconds / 1e9
        self.last_time = self.get_clock().now()
        if dt <= 0:
            return

        v = (dr + dl) / 2.0
        w = (dr - dl) / self.wheel_sep
        self.th += w * dt
        self.x += v * math.cos(self.th) * dt
        self.y += v * math.sin(self.th) * dt

        # Publish Odometry
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = math.sin(self.th / 2)
        odom.pose.pose.orientation.w = math.cos(self.th / 2)
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        # Publish TF
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = "odom"
        t.child_frame_id = "base_link"
        t.transform.translation.x = self.x
        t.transform.translation.y = self.y
        q = tf_transformations.quaternion_from_euler(0, 0, self.th)
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = MotorEncoderBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
