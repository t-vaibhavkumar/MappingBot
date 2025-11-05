#!/usr/bin/env python3
"""
ultrasonic_slam_node.py

Lightweight SLAM / mapping node for an ultrasonic-on-servo sensor.
Publishes /map (nav_msgs/OccupancyGrid) and provides /save_map service.

Author: ChatGPT (adapt and tune to your robot)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid, MapMetaData, Odometry
from std_srvs.srv import Empty
from geometry_msgs.msg import Pose, PoseStamped
import numpy as np
import math
import time
from PIL import Image
import yaml
import os
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

# ---------------------------
# Utility functions
# ---------------------------

def rotation_matrix(theta):
    c = math.cos(theta)
    s = math.sin(theta)
    return np.array([[c, -s], [s, c]])

def bresenham(x0, y0, x1, y1):
    # integer Bresenham from (x0,y0) to (x1,y1); yields points along line
    x0 = int(x0); y0 = int(y0); x1 = int(x1); y1 = int(y1)
    pts = []
    dx = abs(x1 - x0)
    dy = -abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx + dy
    while True:
        pts.append((x0, y0))
        if x0 == x1 and y0 == y1:
            break
        e2 = 2 * err
        if e2 >= dy:
            err += dy
            x0 += sx
        if e2 <= dx:
            err += dx
            y0 += sy
    return pts

# ---------------------------
# Main node
# ---------------------------

class UltrasonicSLAM(Node):
    def __init__(self):
        super().__init__('ultrasonic_slam')

        # Parameters (tweak these for your robot)
        self.declare_parameter('map_size_x', 500)          # grid cells
        self.declare_parameter('map_size_y', 500)
        self.declare_parameter('resolution', 0.05)         # meters / cell (5 cm)
        self.declare_parameter('origin_x', -12.5)          # meters (map origin in world)
        self.declare_parameter('origin_y', -12.5)
        self.declare_parameter('max_laser_range', 0.5)
        self.declare_parameter('min_laser_range', 0.02)
        self.declare_parameter('match_search_radius', 0.2)  # meters (coarse)
        self.declare_parameter('match_search_angle', 0.2)   # radians (coarse)
        self.declare_parameter('match_coarse_step', 0.05)
        self.declare_parameter('match_fine_step', 0.01)
        self.declare_parameter('map_publish_period', 1.0)

        # load params
        mx = self.get_parameter('map_size_x').value
        my = self.get_parameter('map_size_y').value
        self.resolution = self.get_parameter('resolution').value
        self.origin_x = self.get_parameter('origin_x').value
        self.origin_y = self.get_parameter('origin_y').value
        self.max_laser_range = self.get_parameter('max_laser_range').value
        self.min_laser_range = self.get_parameter('min_laser_range').value
        self.match_search_radius = self.get_parameter('match_search_radius').value
        self.match_search_angle = self.get_parameter('match_search_angle').value
        self.coarse_step = self.get_parameter('match_coarse_step').value
        self.fine_step = self.get_parameter('match_fine_step').value
        self.map_pub_period = self.get_parameter('map_publish_period').value

        # map as int8: -1 unknown, 0 free, 100 occ
        self.map_width = int(mx)
        self.map_height = int(my)
        self.occupancy = np.zeros((self.map_height, self.map_width), dtype=np.int8) - 1  # unknown
        # store log-odds occupancy for incremental update (float)
        self.log_odds = np.zeros((self.map_height, self.map_width), dtype=np.float32)

        # occupancy mapping parameters
        self.log_odds_occ = 0.85
        self.log_odds_free = -0.4
        self.log_odds_min = -4.0
        self.log_odds_max = 4.0

        # pose (map frame)
        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_theta = 0.0

        # odom last (for initial guess)
        self.last_odom = None

        qos_profile = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE
        )
        # subscribers & publishers
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_cb, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_cb, 20)
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', qos_profile)
        self.pose_pub = self.create_publisher(PoseStamped, '/slam_pose', 10)

        # service to save map
        self.save_srv = self.create_service(Empty, 'save_map', self.save_map_cb)

        # timer to publish map periodically
        self.last_map_pub = time.time()
        self.create_timer(0.2, self.periodic_publish)

        self.get_logger().info('Ultrasonic SLAM node started')

    # ---------------------------
    # Callbacks
    # ---------------------------
    def odom_cb(self, msg: Odometry):
        # keep last odom pose for initial guess
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # compute yaw from quaternion
        q = msg.pose.pose.orientation
        yaw = math.atan2(2.0*(q.w*q.z + q.x*q.y), 1.0 - 2.0*(q.y*q.y + q.z*q.z))
        self.last_odom = (x, y, yaw)

    def scan_cb(self, scan: LaserScan):
        # convert ranges to numpy array and filter
        ranges = np.array(scan.ranges, dtype=np.float32)
        angles = np.arange(scan.angle_min, scan.angle_max + 1e-9, scan.angle_increment)
        if len(ranges) != len(angles):
            # ensure matching lengths (sometimes float round)
            angles = np.linspace(scan.angle_min, scan.angle_max, num=len(ranges), endpoint=True)

        # filter invalid ranges
        mask = np.logical_and(ranges >= self.min_laser_range, ranges <= self.max_laser_range)
        if not np.any(mask):
            return
        ranges = ranges[mask]
        angles = angles[mask]

        # points in robot frame (2 x N)
        xs = ranges * np.cos(angles)
        ys = ranges * np.sin(angles)
        scan_points = np.stack([xs, ys], axis=1)  # N x 2

        # initial guess from odom if available, else use last pose
        if self.last_odom is not None:
            guess_x, guess_y, guess_th = self.last_odom
        else:
            guess_x, guess_y, guess_th = self.pose_x, self.pose_y, self.pose_theta

        # transform guess into map frame if your odom is in same frame; we assume odom in map frame or relative small drift
        # do scan matching (coarse -> fine)
        best_pose = self.scan_match(scan_points, guess_x, guess_y, guess_th)
        # update internal pose
        self.pose_x, self.pose_y, self.pose_theta = best_pose

        # integrate scan into map
        self.integrate_scan(scan_points, self.pose_x, self.pose_y, self.pose_theta)

        # publish pose for visualization
        pmsg = PoseStamped()
        pmsg.header.stamp = self.get_clock().now().to_msg()
        pmsg.header.frame_id = 'map'
        pmsg.pose.position.x = float(self.pose_x)
        pmsg.pose.position.y = float(self.pose_y)
        pmsg.pose.position.z = 0.0
        # quaternion
        qz = math.sin(self.pose_theta/2.0)
        qw = math.cos(self.pose_theta/2.0)
        pmsg.pose.orientation.z = qz
        pmsg.pose.orientation.w = qw
        self.pose_pub.publish(pmsg)

    # ---------------------------
    # Scan matching (coarse to fine correlation)
    # ---------------------------
    def scan_match(self, scan_points, guess_x, guess_y, guess_th):
        # Transform utility: transform scan points by candidate pose and compute score
        def score_for_candidate(px, py, pth):
            R = rotation_matrix(pth)
            pts_map = (R @ scan_points.T).T + np.array([px, py])
            # convert to cell indices
            cell_x = ((pts_map[:,0] - self.origin_x) / self.resolution).astype(int)
            cell_y = ((pts_map[:,1] - self.origin_y) / self.resolution).astype(int)
            # check bounds
            valid = np.logical_and.reduce((
                cell_x >= 0, cell_x < self.map_width,
                cell_y >= 0, cell_y < self.map_height
            ))
            cell_x = cell_x[valid]; cell_y = cell_y[valid]
            if len(cell_x) == 0:
                return 0.0
            # score: count of hits on occupied cells (or strong log-odds)
            vals = self.log_odds[cell_y, cell_x]  # note indexing row=y, col=x
            # prefer cells with higher log-odds
            score = np.sum(np.clip(vals, 0.0, None))
            return float(score)

        # Coarse search
        best_score = -1e9
        best_pose = (guess_x, guess_y, guess_th)
        # search ranges
        max_r = self.match_search_radius
        max_a = self.match_search_angle
        # coarse grid
        rx = np.arange(-max_r, max_r + 1e-9, self.coarse_step)
        ry = np.arange(-max_r, max_r + 1e-9, self.coarse_step)
        rt = np.arange(-max_a, max_a + 1e-9, self.coarse_step)
        for dx in rx:
            for dy in ry:
                for dt in rt:
                    cand = (guess_x + dx, guess_y + dy, guess_th + dt)
                    sc = score_for_candidate(*cand)
                    if sc > best_score:
                        best_score = sc
                        best_pose = cand

        # Fine search around best_pose
        gx, gy, gth = best_pose
        rx = np.arange(-self.coarse_step, self.coarse_step + 1e-9, self.fine_step)
        ry = np.arange(-self.coarse_step, self.coarse_step + 1e-9, self.fine_step)
        rt = np.arange(-self.coarse_step, self.coarse_step + 1e-9, self.fine_step)
        for dx in rx:
            for dy in ry:
                for dt in rt:
                    cand = (gx + dx, gy + dy, gth + dt)
                    sc = score_for_candidate(*cand)
                    if sc > best_score:
                        best_score = sc
                        best_pose = cand

        # if score is too small (meaning map empty), accept guess (avoid wild correction)
        if best_score < 1e-6:
            return (guess_x, guess_y, guess_th)
        return best_pose

    # ---------------------------
    # Map integration
    # ---------------------------
    def integrate_scan(self, scan_points, pose_x, pose_y, pose_theta):
        # transform points to map frame
        R = rotation_matrix(pose_theta)
        pts_map = (R @ scan_points.T).T + np.array([pose_x, pose_y])
        # convert robot origin into cells
        robot_cx = int((pose_x - self.origin_x) / self.resolution)
        robot_cy = int((pose_y - self.origin_y) / self.resolution)

        for pt in pts_map:
            # endpoint
            ex = pt[0]; ey = pt[1]
            cell_x = int((ex - self.origin_x) / self.resolution)
            cell_y = int((ey - self.origin_y) / self.resolution)
            # check bounds
            if not (0 <= cell_x < self.map_width and 0 <= cell_y < self.map_height):
                continue
            # free cells along the ray
            for (cx, cy) in bresenham(robot_cx, robot_cy, cell_x, cell_y)[:-1]:
                # update log odds for free
                self.log_odds[cy, cx] += self.log_odds_free
                if self.log_odds[cy, cx] < self.log_odds_min:
                    self.log_odds[cy, cx] = self.log_odds_min
            # endpoint as occupied
            self.log_odds[cell_y, cell_x] += self.log_odds_occ
            if self.log_odds[cell_y, cell_x] > self.log_odds_max:
                self.log_odds[cell_y, cell_x] = self.log_odds_max

        # update occupancy from log-odds
        self.occupancy[self.log_odds > 0.5] = 100
        self.occupancy[self.log_odds < 0.0] = 0
        # leave unknown (-1) where between thresholds

    # ---------------------------
    # Map publish
    # ---------------------------
    def periodic_publish(self):
        now = time.time()
        if now - self.last_map_pub < self.map_pub_period:
            return
        self.last_map_pub = now
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        meta = MapMetaData()
        meta.map_load_time = self.get_clock().now().to_msg()
        meta.resolution = self.resolution
        meta.width = self.map_width
        meta.height = self.map_height
        meta.origin.position.x = self.origin_x
        meta.origin.position.y = self.origin_y
        msg.info = meta
        # flatten occupancy as int8 list (row-major starting at (0,0) which is origin_y)
        # nav_msgs expects data starting from [row0col0 ...] where row0 is Y=0 (lowest)
        flat = self.occupancy.flatten(order='C')
        msg.data = [int(x) for x in flat]
        self.map_pub.publish(msg)

    # ---------------------------
    # Save map service callback
    # ---------------------------
    def save_map_cb(self, request, response):
        # convert to image: unknown -> 127, free -> 254, occ -> 0 (visualization)
        img_arr = np.zeros((self.map_height, self.map_width), dtype=np.uint8)
        img_arr[self.occupancy == -1] = 127
        img_arr[self.occupancy == 0] = 254
        img_arr[self.occupancy == 100] = 0
        img = Image.fromarray(img_arr, mode='L')
        ts = int(time.time())
        png_name = f'map_{ts}.png'
        yaml_name = f'map_{ts}.yaml'
        img.save(png_name)
        # write yaml (like map_server)
        map_yaml = {
            'image': png_name,
            'resolution': float(self.resolution),
            'origin': [float(self.origin_x), float(self.origin_y), 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.196
        }
        with open(yaml_name, 'w') as f:
            yaml.dump(map_yaml, f)
        self.get_logger().info(f'Saved map -> {png_name}, {yaml_name}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = UltrasonicSLAM()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
