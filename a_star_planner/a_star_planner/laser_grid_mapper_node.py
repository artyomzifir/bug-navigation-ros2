#!/usr/bin/env python3
"""Laser-based occupancy-grid mapper.

Builds a 2D occupancy grid in the `odom` frame by raycasting each LIDAR
beam through a fixed-size grid, marking free cells along the ray and an
occupied cell at the endpoint. Publishes the grid on /map so the A*
planner can consume it. No RTAB-Map / map_server / saved .pgm needed —
the map is built live from sensor data.

This is "odometric SLAM-lite": there's no loop closure, so over long
traverses the map will smear. For room-scale navigation it's perfectly
adequate, and it's zero-configuration: launch it with the simulation and
go.

Grid representation (standard nav_msgs/OccupancyGrid):
    -1  unknown
     0  free
   100  occupied
Internally we use a log-odds float buffer for proper probabilistic
updates, then threshold to int8 on publish.

Subscribes:  /scan, /odom
Publishes:   /map (latched-style, TRANSIENT_LOCAL)
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import Pose
from nav_msgs.msg import Odometry, OccupancyGrid
from sensor_msgs.msg import LaserScan


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _bresenham(r0, c0, r1, c1):
    """Integer line from (r0,c0) to (r1,c1), inclusive of both endpoints.
    Yields (row, col) pairs."""
    dr = abs(r1 - r0)
    dc = abs(c1 - c0)
    sr = 1 if r0 < r1 else -1
    sc = 1 if c0 < c1 else -1
    err = dc - dr
    r, c = r0, c0
    while True:
        yield r, c
        if r == r1 and c == c1:
            return
        e2 = 2 * err
        if e2 > -dr:
            err -= dr
            c += sc
        if e2 < dc:
            err += dc
            r += sr


class LaserGridMapper(Node):
    def __init__(self):
        super().__init__('laser_grid_mapper_node')

        # Grid geometry. Centred at (0, 0) in the odom frame by default —
        # robot usually spawns there, so symmetric bounds cover everything
        # reachable.
        self.declare_parameter('resolution', 0.05)           # m / cell
        self.declare_parameter('size_m', 20.0)               # square side length
        self.declare_parameter('origin_x', -10.0)
        self.declare_parameter('origin_y', -10.0)
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('publish_rate_hz', 2.0)
        self.declare_parameter('max_update_range', 5.0)      # clamp long rays

        # Log-odds increments. Tuned conservatively so noise doesn't flip
        # cells too quickly, but the map builds up within a few seconds.
        self.declare_parameter('l_occ', 0.85)
        self.declare_parameter('l_free', -0.4)
        self.declare_parameter('l_min', -2.0)
        self.declare_parameter('l_max', 3.5)
        # Thresholds on log-odds for converting to int8 on publish.
        self.declare_parameter('occ_threshold', 0.7)
        self.declare_parameter('free_threshold', -0.4)

        gp = self.get_parameter
        self.res = gp('resolution').value
        self.size_m = gp('size_m').value
        self.origin_x = gp('origin_x').value
        self.origin_y = gp('origin_y').value
        self.frame_id = gp('frame_id').value
        self.max_range = gp('max_update_range').value
        self.l_occ = gp('l_occ').value
        self.l_free = gp('l_free').value
        self.l_min = gp('l_min').value
        self.l_max = gp('l_max').value
        self.occ_thresh = gp('occ_threshold').value
        self.free_thresh = gp('free_threshold').value

        cells = int(round(self.size_m / self.res))
        self.H = cells
        self.W = cells

        # log-odds buffer; 0.0 == unknown (50% probability).
        self.logodds = np.zeros((self.H, self.W), dtype=np.float32)

        # Latest robot pose in the odom frame.
        self.robot_x = None
        self.robot_y = None
        self.robot_yaw = None

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.map_pub = self.create_publisher(OccupancyGrid, '/map', latched)

        self.create_subscription(LaserScan, '/scan', self._scan_cb,
                                 rclpy.qos.qos_profile_sensor_data)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)

        rate = gp('publish_rate_hz').value
        self.create_timer(1.0 / rate, self._publish_map)

        self.get_logger().info(
            f'Laser mapper ready: {self.W}x{self.H} cells at {self.res:.2f} m/cell, '
            f'origin=({self.origin_x:.1f}, {self.origin_y:.1f}) in "{self.frame_id}"')

    # ---------- callbacks ----------
    def _odom_cb(self, msg: Odometry):
        self.robot_x = msg.pose.pose.position.x
        self.robot_y = msg.pose.pose.position.y
        self.robot_yaw = _yaw_from_quaternion(msg.pose.pose.orientation)

    def _scan_cb(self, msg: LaserScan):
        if self.robot_x is None:
            return
        self._integrate_scan(msg)

    # ---------- mapping ----------
    def _world_to_cell(self, x, y):
        col = int(math.floor((x - self.origin_x) / self.res))
        row = int(math.floor((y - self.origin_y) / self.res))
        return row, col

    def _in_bounds(self, r, c):
        return 0 <= r < self.H and 0 <= c < self.W

    def _integrate_scan(self, scan: LaserScan):
        """Raycast every beam, updating log-odds along the ray.

        Free cells along the ray get `l_free`. The endpoint (if within
        `max_update_range` and the beam returned a valid range) gets
        `l_occ`. Beams that are out of range don't mark an endpoint —
        we only know space is free up to `max_update_range`.
        """
        rx, ry, ryaw = self.robot_x, self.robot_y, self.robot_yaw
        r0, c0 = self._world_to_cell(rx, ry)
        if not self._in_bounds(r0, c0):
            return

        angle = scan.angle_min + ryaw
        ainc = scan.angle_increment
        rmin = scan.range_min
        max_r = min(self.max_range, scan.range_max)

        for rng in scan.ranges:
            ray_angle = angle
            angle += ainc

            if math.isnan(rng) or math.isinf(rng) or rng < rmin:
                # Still know the first max_r are free.
                hit = False
                r_used = max_r
            elif rng >= max_r:
                hit = False
                r_used = max_r
            else:
                hit = True
                r_used = rng

            ex = rx + r_used * math.cos(ray_angle)
            ey = ry + r_used * math.sin(ray_angle)
            r1, c1 = self._world_to_cell(ex, ey)

            # Walk the line and mark free; stop at grid bounds.
            last_r, last_c = r0, c0
            for rr, cc in _bresenham(r0, c0, r1, c1):
                if not self._in_bounds(rr, cc):
                    break
                last_r, last_c = rr, cc
                # Don't pre-mark the endpoint as free — we'll overwrite with
                # occupied below if `hit` is True.
                if (rr, cc) == (r1, c1):
                    break
                self.logodds[rr, cc] += self.l_free

            if hit and self._in_bounds(r1, c1):
                self.logodds[r1, c1] += self.l_occ

        np.clip(self.logodds, self.l_min, self.l_max, out=self.logodds)

    def _publish_map(self):
        if self.robot_x is None:
            return

        grid = np.full((self.H, self.W), -1, dtype=np.int8)
        grid[self.logodds > self.occ_thresh] = 100
        grid[self.logodds < self.free_thresh] = 0

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        msg.info.resolution = float(self.res)
        msg.info.width = self.W
        msg.info.height = self.H
        origin = Pose()
        origin.position.x = float(self.origin_x)
        origin.position.y = float(self.origin_y)
        origin.orientation.w = 1.0
        msg.info.origin = origin
        msg.data = grid.flatten().tolist()
        self.map_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = LaserGridMapper()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
