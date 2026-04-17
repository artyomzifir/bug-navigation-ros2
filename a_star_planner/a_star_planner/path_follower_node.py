#!/usr/bin/env python3
"""Carrot-follower for the A* planner output.

Reads robot pose from /odom (same frame the planner plans in, `odom`),
so no TF chain setup is required. When /planned_path is updated mid-
motion (because the A* node re-planned on a new map), the follower
seamlessly switches to the new path.

Subscribes:
    /planned_path  nav_msgs/Path
    /odom          nav_msgs/Odometry

Publishes:
    /cmd_vel       geometry_msgs/Twist
"""
import math

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry, Path


def _yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _normalize_angle(a):
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class PathFollower(Node):
    def __init__(self):
        super().__init__('path_follower')

        self.declare_parameter('linear_speed', 0.3)
        self.declare_parameter('angular_speed', 0.75)
        self.declare_parameter('waypoint_tolerance', 0.2)
        # Heading error above this => rotate in place.
        self.declare_parameter('turn_in_place_threshold_deg', 25.0)
        self.declare_parameter('rate_hz', 20.0)

        gp = self.get_parameter
        self.v = gp('linear_speed').value
        self.w_max = gp('angular_speed').value
        self.tol = gp('waypoint_tolerance').value
        self.turn_thresh = math.radians(gp('turn_in_place_threshold_deg').value)

        self.path = []          # list[(x, y)]
        self.current_idx = 0
        self.pose = None        # (x, y, yaw)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Path, '/planned_path', self._path_cb, 10)
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.create_timer(1.0 / gp('rate_hz').value, self._tick)

        self.get_logger().info('Path follower ready.')

    def _odom_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.pose = (p.x, p.y, _yaw_from_quaternion(msg.pose.pose.orientation))

    def _path_cb(self, msg: Path):
        new_path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        if not new_path:
            return
        # If we already had a path, keep progressing — snap to the nearest
        # waypoint on the new path instead of restarting from index 0.
        if self.path and self.pose is not None:
            x, y, _ = self.pose
            nearest = min(range(len(new_path)),
                          key=lambda i: math.hypot(new_path[i][0] - x,
                                                   new_path[i][1] - y))
            self.current_idx = nearest
        else:
            self.current_idx = 0
        self.path = new_path
        self.get_logger().info(
            f'New path with {len(self.path)} waypoints '
            f'(resuming at idx {self.current_idx})')

    def _tick(self):
        if not self.path or self.current_idx >= len(self.path):
            return
        if self.pose is None:
            return
        x, y, yaw = self.pose

        # Skip waypoints already within tolerance.
        while self.current_idx < len(self.path) - 1:
            wx, wy = self.path[self.current_idx]
            if math.hypot(wx - x, wy - y) < self.tol:
                self.current_idx += 1
            else:
                break

        wx, wy = self.path[self.current_idx]
        dx, dy = wx - x, wy - y
        dist = math.hypot(dx, dy)

        if self.current_idx == len(self.path) - 1 and dist < self.tol:
            self.cmd_pub.publish(Twist())
            self.get_logger().info('Final waypoint reached.')
            self.current_idx = len(self.path)
            return

        heading_err = _normalize_angle(math.atan2(dy, dx) - yaw)
        cmd = Twist()
        if abs(heading_err) > self.turn_thresh:
            cmd.angular.z = math.copysign(self.w_max, heading_err)
        else:
            cmd.linear.x = self.v * max(0.0, math.cos(heading_err))
            cmd.angular.z = max(-self.w_max, min(self.w_max, 1.5 * heading_err))
        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = PathFollower()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cmd_pub.publish(Twist())
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
