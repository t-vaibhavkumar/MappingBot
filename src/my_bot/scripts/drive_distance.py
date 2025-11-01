#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import math

class DriveDistance(Node):
    def __init__(self, distance=5.0, speed=1):
        super().__init__('drive_distance')
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.sub = self.create_subscription(Odometry, 'odom', self.odom_cb, 10)
        self.start_x = None
        self.start_y = None
        self.distance = distance
        self.speed = speed
        self.reached = False
        self.timer = self.create_timer(0.1, self.send_cmd)

    def odom_cb(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.start_x is None:
            self.start_x = x; self.start_y = y
        dx = x - self.start_x; dy = y - self.start_y
        traveled = math.hypot(dx, dy)
        if traveled >= self.distance and not self.reached:
            self.reached = True
            self.get_logger().info(f"Reached target distance: {traveled:.3f} m")
            # Stop robot
            t = Twist()
            self.pub.publish(t)

    def send_cmd(self):
        if not self.reached:
            t = Twist()
            t.linear.x = self.speed
            self.pub.publish(t)
        else:
            # cancel timer and exit
            self.timer.cancel()
            self.get_logger().info("Stopping node.")
            rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = DriveDistance(distance=2.0, speed=1.0)  # change distance/speed here
    rclpy.spin(node)

if __name__ == '__main__':
    main()
