#!/usr/bin/env python3
"""Bug 1 reactive planner.

Adds two pieces of bookkeeping to the reference Tangent Bug node:

1. While following the wall after a hit, record the boundary point closest
   to the goal (`closest_point`) along with its distance (`min_dist_to_goal`).
2. Detect when the robot has completed a full lap by re-entering a small
   tolerance ball around the original `obstacle_hit_point` AFTER it has
   travelled at least `min_circumnav_distance` along the wall (otherwise
   the trigger fires immediately on the first scan tick).

A new state `GO_TO_CLOSEST_POINT` drives the robot from the hit point along
the boundary to the closest point. Once it gets there, normal `GO_TO_GOAL`
resumes — and the wall is now provably to one side, so progress is made.

Bug 1 is complete (reaches any reachable goal) but inefficient: it walks the
full perimeter of every obstacle it touches before committing to a leave
point.

Subscribes:  /goal_pose, /odom, /scan
Publishes:   /cmd_vel, /bug1_path
"""
import math
from enum import Enum

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped, Point
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path

import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs  # noqa: F401


def euler_from_quaternion(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(t0, t1)
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else (-1.0 if t2 < -1.0 else t2)
    pitch = math.asin(t2)
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(t3, t4)
    return roll, pitch, yaw


class BugState(Enum):
    GO_TO_GOAL = 1
    WALL_FOLLOW = 2            # circumnavigation
    GO_TO_CLOSEST_POINT = 3    # navigate along boundary back to closest point
    GOAL_REACHED = 4


class Bug1PlannerNode(Node):
    def __init__(self):
        super().__init__('bug1_planner_node')

        self.declare_parameter('wall_follow_dist', 0.4)
        self.declare_parameter('max_scan_range', 10.0)
        self.declare_parameter('obstacle_threshold', 0.3)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('linear_speed', 0.2)
        self.declare_parameter('angular_speed', 0.5)
        self.declare_parameter('control_loop_freq', 10.0)
        self.declare_parameter('target_frame', 'odom')
        # Bug 1 specific
        self.declare_parameter('hit_point_tolerance', 0.3)
        self.declare_parameter('min_circumnav_distance', 1.0)

        self.goal_pose_stamped = None
        self.goal_internal = None

        self.current_pose = None
        self.current_yaw = 0.0
        self.odom_frame = None
        self.scan_data = None

        self.state = BugState.GO_TO_GOAL

        # bug 1 bookkeeping
        self.obstacle_hit_point = None      # geometry_msgs/Point
        self.closest_point = None           # geometry_msgs/Point on boundary
        self.min_dist_to_goal = float('inf')
        self.distance_along_wall = 0.0
        self.last_pos_for_dist = None

        self.path_msg = Path()

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_publisher = self.create_publisher(Path, 'bug1_path', 10)

        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback,
                                 rclpy.qos.qos_profile_sensor_data)

        freq = self.get_parameter('control_loop_freq').get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / freq, self.control_loop)

        self.get_logger().info('Bug 1 planner ready. Send a goal on /goal_pose.')

    # ---------------- callbacks ----------------
    def goal_callback(self, msg):
        self.get_logger().info(f"New goal in frame '{msg.header.frame_id}'")
        self.goal_pose_stamped = msg
        self.goal_internal = None
        self.state = BugState.GO_TO_GOAL
        self._reset_circumnav_state()
        self.path_msg.poses.clear()

    def _reset_circumnav_state(self):
        self.obstacle_hit_point = None
        self.closest_point = None
        self.min_dist_to_goal = float('inf')
        self.distance_along_wall = 0.0
        self.last_pos_for_dist = None

    def odom_callback(self, msg):
        new_pos = msg.pose.pose.position
        # Track distance travelled while in WALL_FOLLOW (for the lap detector).
        if self.state == BugState.WALL_FOLLOW and self.last_pos_for_dist is not None:
            self.distance_along_wall += math.hypot(
                new_pos.x - self.last_pos_for_dist.x,
                new_pos.y - self.last_pos_for_dist.y,
            )
        self.last_pos_for_dist = new_pos

        self.current_pose = new_pos
        self.odom_frame = msg.header.frame_id
        _, _, self.current_yaw = euler_from_quaternion(msg.pose.pose.orientation)

        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.odom_frame
        ps.pose = msg.pose.pose
        self.path_msg.poses.append(ps)
        if len(self.path_msg.poses) > 1000:
            self.path_msg.poses = self.path_msg.poses[-1000:]
        self.path_msg.header = ps.header
        self.path_publisher.publish(self.path_msg)

    def scan_callback(self, msg):
        self.scan_data = msg

    # ---------------- helpers ----------------
    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    @staticmethod
    def distance(p1, p2):
        return math.hypot(p1.x - p2.x, p1.y - p2.y)

    def _scan_in_arc(self, center_angle, half_width):
        if self.scan_data is None:
            return float('inf')
        ranges = self.scan_data.ranges
        n = len(ranges)
        if n == 0:
            return float('inf')
        amin = self.scan_data.angle_min
        ainc = self.scan_data.angle_increment
        max_range = self.get_parameter('max_scan_range').get_parameter_value().double_value
        rmin = self.scan_data.range_min

        i_lo = int(round((center_angle - half_width - amin) / ainc))
        i_hi = int(round((center_angle + half_width - amin) / ainc))
        i_lo = max(0, min(n - 1, i_lo))
        i_hi = max(0, min(n - 1, i_hi))
        if i_lo > i_hi:
            i_lo, i_hi = i_hi, i_lo

        best = float('inf')
        for i in range(i_lo, i_hi + 1):
            r = ranges[i]
            if math.isinf(r) or math.isnan(r):
                continue
            if r < rmin or r > max_range:
                continue
            if r < best:
                best = r
        return best

    def _drive_toward(self, target):
        """Pure heading-then-drive controller used for both GO_TO_GOAL and
        GO_TO_CLOSEST_POINT."""
        v = self.get_parameter('linear_speed').get_parameter_value().double_value
        w = self.get_parameter('angular_speed').get_parameter_value().double_value
        twist = Twist()
        ang_to = math.atan2(target.y - self.current_pose.y,
                            target.x - self.current_pose.x)
        diff = self.normalize_angle(ang_to - self.current_yaw)
        twist.angular.z = max(-w, min(w, w * diff))
        twist.linear.x = v if abs(diff) < math.radians(10.0) else 0.0
        return twist

    def _wall_following_twist(self):
        v = self.get_parameter('linear_speed').get_parameter_value().double_value
        w = self.get_parameter('angular_speed').get_parameter_value().double_value
        wall_d = self.get_parameter('wall_follow_dist').get_parameter_value().double_value

        front = self._scan_in_arc(0.0, math.radians(15.0))
        right = self._scan_in_arc(math.radians(-67.5), math.radians(22.5))

        twist = Twist()
        error = wall_d - right
        turn = max(-w, min(w, -2.0 * error))
        twist.angular.z = turn

        if front < wall_d * 1.5:
            twist.linear.x = 0.0
            twist.angular.z = w
        elif abs(turn) > 0.5 * w:
            twist.linear.x = 0.5 * v
        else:
            twist.linear.x = v
        return twist, front, right

    # ---------------- main loop ----------------
    def control_loop(self):
        if self.scan_data is None or self.current_pose is None or self.odom_frame is None:
            self.get_logger().warn('Waiting for scan / odom...', throttle_duration_sec=2)
            return
        if self.goal_pose_stamped is None:
            if self.state != BugState.GOAL_REACHED:
                self.stop_robot()
            return

        target_frame = self.get_parameter('target_frame').get_parameter_value().string_value
        if self.goal_internal is None:
            try:
                if self.odom_frame != target_frame:
                    self.get_logger().error(
                        f"odom frame '{self.odom_frame}' != target_frame '{target_frame}'",
                        once=True)
                    self.stop_robot()
                    return
                tg = self.tf_buffer.transform(
                    self.goal_pose_stamped, target_frame,
                    timeout=rclpy.duration.Duration(seconds=0.5))
                self.goal_internal = tg.pose.position
                self.get_logger().info(f"Goal transformed into '{target_frame}'")
            except TransformException as e:
                self.get_logger().error(f'TF goal transform failed: {e}')
                self.stop_robot()
                return

        dist_to_goal = self.distance(self.current_pose, self.goal_internal)
        goal_tol = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        if dist_to_goal < goal_tol:
            if self.state != BugState.GOAL_REACHED:
                self.get_logger().info(f'Goal reached. dist={dist_to_goal:.2f}')
                self.state = BugState.GOAL_REACHED
                self.stop_robot()
            return

        obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value
        hit_tol = self.get_parameter('hit_point_tolerance').get_parameter_value().double_value
        min_circ = self.get_parameter('min_circumnav_distance').get_parameter_value().double_value

        # ------- GO_TO_GOAL -------
        if self.state == BugState.GO_TO_GOAL:
            front_min = self._scan_in_arc(0.0, math.radians(15.0))
            if front_min <= obstacle_threshold:
                # store hit point and start a fresh circumnavigation
                hit = Point(); hit.x = self.current_pose.x; hit.y = self.current_pose.y
                self.obstacle_hit_point = hit
                cp = Point(); cp.x = hit.x; cp.y = hit.y
                self.closest_point = cp
                self.min_dist_to_goal = dist_to_goal
                self.distance_along_wall = 0.0
                self.last_pos_for_dist = self.current_pose
                self.get_logger().info(
                    f'Hit obstacle at ({hit.x:.2f}, {hit.y:.2f}) -> WALL_FOLLOW '
                    f'(circumnavigate)')
                self.state = BugState.WALL_FOLLOW
                self.stop_robot()
                return

            twist = self._drive_toward(self.goal_internal)
            self.cmd_vel_publisher.publish(twist)
            return

        # ------- WALL_FOLLOW (circumnavigation) -------
        if self.state == BugState.WALL_FOLLOW:
            # Update the closest-point estimate every tick.
            if dist_to_goal < self.min_dist_to_goal:
                self.min_dist_to_goal = dist_to_goal
                cp = Point(); cp.x = self.current_pose.x; cp.y = self.current_pose.y
                self.closest_point = cp

            # Lap completed? Must have travelled far enough first.
            back_to_hit = (self.distance(self.current_pose, self.obstacle_hit_point)
                           < hit_tol)
            if self.distance_along_wall > min_circ and back_to_hit:
                self.get_logger().info(
                    f'Lap complete (travelled {self.distance_along_wall:.2f} m). '
                    f'Closest point: ({self.closest_point.x:.2f}, '
                    f'{self.closest_point.y:.2f}), '
                    f'dist={self.min_dist_to_goal:.2f} -> GO_TO_CLOSEST_POINT')
                self.state = BugState.GO_TO_CLOSEST_POINT
                self.stop_robot()
                return

            twist, _, _ = self._wall_following_twist()
            self.cmd_vel_publisher.publish(twist)
            return

        # ------- GO_TO_CLOSEST_POINT -------
        if self.state == BugState.GO_TO_CLOSEST_POINT:
            if self.distance(self.current_pose, self.closest_point) < goal_tol:
                self.get_logger().info('Reached closest boundary point -> GO_TO_GOAL')
                self.state = BugState.GO_TO_GOAL
                self._reset_circumnav_state()
                return

            # Keep following the wall until we get there. Switching to a straight
            # drive can cut the corner into the obstacle, so we deliberately stay
            # along the boundary.
            front = self._scan_in_arc(0.0, math.radians(15.0))
            if front <= obstacle_threshold * 1.2:
                twist, _, _ = self._wall_following_twist()
            else:
                twist = self._drive_toward(self.closest_point)
            self.cmd_vel_publisher.publish(twist)
            return

        # ------- GOAL_REACHED -------
        self.stop_robot()

    def stop_robot(self):
        self.cmd_vel_publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = Bug1PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
