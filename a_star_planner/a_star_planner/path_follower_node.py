#!/usr/bin/env python3
"""Carrot-follower for the A* planner output.

Subscribes:
    /planned_path  nav_msgs/Path

Publishes:
    /cmd_vel       geometry_msgs/Twist

Robot pose is read from TF (`global_frame` -> `base_frame`).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration

from geometry_msgs.msg import Twist
from nav_msgs.msg import Path

import tf2_ros


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

        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.6)
        self.declare_parameter('waypoint_tolerance', 0.15)
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('rate_hz', 20.0)

        gp = self.get_parameter
        self.v = gp('linear_speed').value
        self.w_max = gp('angular_speed').value
        self.tol = gp('waypoint_tolerance').value
        self.base_frame = gp('base_frame').value
        self.global_frame = gp('global_frame').value

        self.path = []          # list[(x, y)]
        self.current_idx = 0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Path, '/planned_path', self._path_cb, 10)
        self.create_timer(1.0 / gp('rate_hz').value, self._tick)

        self.get_logger().info('Path follower ready.')

    def _path_cb(self, msg: Path):
        self.path = [(p.pose.position.x, p.pose.position.y) for p in msg.poses]
        self.current_idx = 0
        self.get_logger().info(f'New path with {len(self.path)} waypoints.')

    def _robot_pose(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.2))
        except Exception:
            return None
        t = tf.transform.translation
        return t.x, t.y, _yaw_from_quaternion(tf.transform.rotation)

    def _tick(self):
        if not self.path or self.current_idx >= len(self.path):
            return
        pose = self._robot_pose()
        if pose is None:
            return
        x, y, yaw = pose

        # Skip waypoints that are already within tolerance.
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
        if abs(heading_err) > math.radians(25):
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
        rclpy.shutdown()


if __name__ == '__main__':
    main()
