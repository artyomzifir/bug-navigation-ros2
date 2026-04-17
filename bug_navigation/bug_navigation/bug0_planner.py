#!/usr/bin/env python3
"""Bug 0 reactive planner.

Behaviour summary:
* GO_TO_GOAL: drive at the goal until the front cone sees an obstacle
  closer than `obstacle_threshold`.
* WALL_FOLLOW: keep the wall on the right with four sectors (front,
  right-front, right, right-back). Stay in this state for at least
  `min_wall_follow_time_s` seconds, then look for a leave condition.
* Leave condition: goal must be roughly in front, AND both the goal
  cone and the front cone must be clear by `leave_clearance_ratio` *
  obstacle_threshold (hysteresis vs the entry threshold).

The minimum-time-in-WALL_FOLLOW guard is what kills the chatter where the
robot oscillates between the two states every 100 ms. Without it, any
random scan glitch can flip the state machine.

Subscribes:  /goal_pose, /odom, /scan
Publishes:   /cmd_vel, /bug0_path
"""
import math
from enum import Enum

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry, Path

import tf2_ros
from tf2_ros import TransformException
import tf2_geometry_msgs  # noqa: F401  (registers do_transform_pose_stamped)


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
    WALL_FOLLOW = 2
    GOAL_REACHED = 3


class Bug0PlannerNode(Node):
    def __init__(self):
        super().__init__('bug0_planner_node')

        # ── Speeds and tolerances ──────────────────────────────────────────
        self.declare_parameter('wall_follow_dist', 0.6)
        self.declare_parameter('max_scan_range', 10.0)
        self.declare_parameter('obstacle_threshold', 0.55)
        self.declare_parameter('goal_tolerance', 0.2)
        self.declare_parameter('linear_speed', 0.15)
        self.declare_parameter('angular_speed', 0.7)
        self.declare_parameter('control_loop_freq', 10.0)
        self.declare_parameter('target_frame', 'odom')

        # ── Cone widths ────────────────────────────────────────────────────
        # Goal cone: narrow. Wider values cause false "clear" reads.
        self.declare_parameter('goal_clear_arc_deg', 12.0)
        # Front cone: WIDE for wide robots. Default 35° handles a robot half
        # ~0.25 m at threshold ~0.55 m. Capped to half the LIDAR FoV at
        # runtime, so on a 120° LIDAR the effective max is 60°.
        self.declare_parameter('front_check_arc_deg', 35.0)
        # Side cone for wall-follow sectors.
        self.declare_parameter('side_check_arc_deg', 25.0)

        # ── State-machine guards ───────────────────────────────────────────
        # Leave-clearance ratio: front and goal cones must be clear by
        # `obstacle_threshold * this` before we leave the wall.
        self.declare_parameter('leave_clearance_ratio', 1.8)
        # Goal direction must be within +/- this many degrees of straight
        # ahead before we even consider leaving.
        self.declare_parameter('goal_in_front_arc_deg', 45.0)
        # Hard minimum time spent in WALL_FOLLOW before any leave check is
        # even allowed. Eliminates the per-tick GO_TO_GOAL <-> WALL_FOLLOW
        # ping-pong that was happening before.
        self.declare_parameter('min_wall_follow_time_s', 2.5)

        # --- Goal handling ---
        self.goal_pose_stamped = None
        self.goal_internal = None  # geometry_msgs/Point in target_frame

        # --- Robot state ---
        self.current_pose = None
        self.current_yaw = 0.0
        self.odom_frame = None
        self.scan_data = None

        # --- State machine ---
        self.state = BugState.GO_TO_GOAL
        self.wall_follow_start_time = None  # rclpy.time.Time

        # --- Path viz ---
        self.path_msg = Path()

        # --- TF ---
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # --- I/O ---
        self.cmd_vel_publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.path_publisher = self.create_publisher(Path, 'bug0_path', 10)

        self.create_subscription(PoseStamped, '/goal_pose', self.goal_callback, 10)
        self.create_subscription(Odometry, 'odom', self.odom_callback, 10)
        self.create_subscription(LaserScan, 'scan', self.scan_callback,
                                 rclpy.qos.qos_profile_sensor_data)

        freq = self.get_parameter('control_loop_freq').get_parameter_value().double_value
        self.timer = self.create_timer(1.0 / freq, self.control_loop)

        self.get_logger().info('Bug 0 planner ready. Send a goal on /goal_pose.')

    # ---------------- callbacks ----------------
    def goal_callback(self, msg):
        self.get_logger().info(f"New goal in frame '{msg.header.frame_id}'")
        self.goal_pose_stamped = msg
        self.goal_internal = None
        self.state = BugState.GO_TO_GOAL
        self.wall_follow_start_time = None
        self.path_msg.poses.clear()

    def odom_callback(self, msg):
        self.current_pose = msg.pose.pose.position
        self.odom_frame = msg.header.frame_id
        _, _, self.current_yaw = euler_from_quaternion(msg.pose.pose.orientation)

        ps = PoseStamped()
        ps.header.stamp = self.get_clock().now().to_msg()
        ps.header.frame_id = self.odom_frame
        ps.pose = msg.pose.pose
        self.path_msg.poses.append(ps)
        if len(self.path_msg.poses) > 500:
            self.path_msg.poses = self.path_msg.poses[-500:]
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

    def _laser_half_fov(self):
        if self.scan_data is None:
            return math.pi
        return 0.5 * (self.scan_data.angle_max - self.scan_data.angle_min)

    def _front_half(self):
        req = math.radians(
            self.get_parameter('front_check_arc_deg').get_parameter_value().double_value)
        return min(req, self._laser_half_fov())

    def _side_half(self):
        req = math.radians(
            self.get_parameter('side_check_arc_deg').get_parameter_value().double_value)
        return min(req, self._laser_half_fov())

    def _scan_in_arc(self, center_angle, half_width):
        """Min valid range within +/- half_width of center_angle (laser frame)."""
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

    def _seconds_in_wall_follow(self):
        if self.wall_follow_start_time is None:
            return 0.0
        elapsed = self.get_clock().now() - self.wall_follow_start_time
        return elapsed.nanoseconds * 1e-9

    def can_leave_wall(self):
        """Bug 0 leave condition. Returns (can_leave: bool, debug_info: str)."""
        if self.goal_internal is None or self.scan_data is None or self.current_pose is None:
            return False, 'no data'

        # Hard minimum dwell time — main anti-chatter guard.
        min_dwell = self.get_parameter(
            'min_wall_follow_time_s').get_parameter_value().double_value
        elapsed = self._seconds_in_wall_follow()
        if elapsed < min_dwell:
            return False, f'dwell {elapsed:.1f}<{min_dwell:.1f}s'

        obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value
        leave_margin = obstacle_threshold * self.get_parameter(
            'leave_clearance_ratio').get_parameter_value().double_value
        clear_arc = math.radians(
            self.get_parameter('goal_clear_arc_deg').get_parameter_value().double_value)
        in_front_max = math.radians(
            self.get_parameter('goal_in_front_arc_deg').get_parameter_value().double_value)

        angle_to_goal = math.atan2(self.goal_internal.y - self.current_pose.y,
                                   self.goal_internal.x - self.current_pose.x)
        rel = self.normalize_angle(angle_to_goal - self.current_yaw)

        # 1. Goal must be roughly in front.
        if abs(rel) > in_front_max:
            return False, f'goal at {math.degrees(rel):.0f}° not in front'

        # 2. Narrow goal cone clear (clamped to laser FoV).
        rel_clamped = max(-self._laser_half_fov(),
                          min(self._laser_half_fov(), rel))
        goal_dir_min = self._scan_in_arc(rel_clamped, clear_arc)
        if goal_dir_min < leave_margin:
            return False, f'goal_dir {goal_dir_min:.2f}<{leave_margin:.2f}'

        # 3. Wide front cone also clear.
        front_min = self._scan_in_arc(0.0, self._front_half())
        if front_min < leave_margin:
            return False, f'front {front_min:.2f}<{leave_margin:.2f}'

        return True, f'front={front_min:.2f} goal_dir={goal_dir_min:.2f} >= {leave_margin:.2f}'

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

        dist = self.distance(self.current_pose, self.goal_internal)
        goal_tol = self.get_parameter('goal_tolerance').get_parameter_value().double_value
        if dist < goal_tol:
            if self.state != BugState.GOAL_REACHED:
                self.get_logger().info(f'Goal reached. dist={dist:.2f}')
                self.state = BugState.GOAL_REACHED
                self.stop_robot()
            return

        twist = Twist()
        v = self.get_parameter('linear_speed').get_parameter_value().double_value
        w = self.get_parameter('angular_speed').get_parameter_value().double_value
        obstacle_threshold = self.get_parameter('obstacle_threshold').get_parameter_value().double_value

        # ------- GO_TO_GOAL -------
        if self.state == BugState.GO_TO_GOAL:
            front_min = self._scan_in_arc(0.0, self._front_half())
            if front_min <= obstacle_threshold:
                self.get_logger().info(
                    f'Obstacle ahead ({front_min:.2f} m, cone ±{math.degrees(self._front_half()):.0f}°) '
                    f'-> WALL_FOLLOW')
                self.state = BugState.WALL_FOLLOW
                self.wall_follow_start_time = self.get_clock().now()
                self.stop_robot()
                return

            angle_to_goal = math.atan2(self.goal_internal.y - self.current_pose.y,
                                       self.goal_internal.x - self.current_pose.x)
            angle_diff = self.normalize_angle(angle_to_goal - self.current_yaw)
            twist.angular.z = max(-w, min(w, w * angle_diff))
            twist.linear.x = v if abs(angle_diff) < math.radians(10.0) else 0.0
            self.cmd_vel_publisher.publish(twist)
            return

        # ------- WALL_FOLLOW -------
        if self.state == BugState.WALL_FOLLOW:
            can_leave, why = self.can_leave_wall()
            if can_leave:
                self.get_logger().info(f'Leaving wall: {why} -> GO_TO_GOAL')
                self.state = BugState.GO_TO_GOAL
                self.wall_follow_start_time = None
                # Don't try to drive this tick — let GO_TO_GOAL handle it next tick.
                self.stop_robot()
                return
            else:
                # Throttled debug log — useful when tuning.
                self.get_logger().debug(f'wall: {why}', throttle_duration_sec=1.0)

            self._follow_wall_right()
            return

        # ------- GOAL_REACHED -------
        self.stop_robot()

    def _follow_wall_right(self):
        """Right-hand wall following with four sectors. The right-back
        sector catches door jambs and outer corners that scrape the rear."""
        v = self.get_parameter('linear_speed').get_parameter_value().double_value
        w = self.get_parameter('angular_speed').get_parameter_value().double_value
        wall_d = self.get_parameter('wall_follow_dist').get_parameter_value().double_value

        front_half = self._front_half()
        side_half = self._side_half()
        max_side = self._laser_half_fov()

        front_c       = 0.0
        right_front_c = max(-max_side, math.radians(-45))
        right_c       = max(-max_side, math.radians(-90))
        right_back_c  = max(-max_side, math.radians(-130))

        front       = self._scan_in_arc(front_c,       front_half)
        right_front = self._scan_in_arc(right_front_c, side_half)
        right       = self._scan_in_arc(right_c,       side_half)
        right_back  = self._scan_in_arc(right_back_c,  side_half)

        safety_margin = wall_d * 0.6

        twist = Twist()

        if front < wall_d * 1.2:
            twist.linear.x = 0.0
            twist.angular.z = w
        elif right_front < safety_margin or right_back < safety_margin:
            twist.linear.x = 0.3 * v
            twist.angular.z = 0.6 * w
        else:
            error = wall_d - right
            turn = max(-w, min(w, -1.5 * error))
            twist.angular.z = turn
            twist.linear.x = v if abs(turn) < 0.4 * w else 0.5 * v

        self.cmd_vel_publisher.publish(twist)

        self.get_logger().debug(
            f'[WF] F={front:.2f} RF={right_front:.2f} R={right:.2f} RB={right_back:.2f} '
            f'lin={twist.linear.x:.2f} ang={twist.angular.z:.2f}',
            throttle_duration_sec=0.5)

    def stop_robot(self):
        self.cmd_vel_publisher.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = Bug0PlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.stop_robot()
    finally:
        node.destroy_node()
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()