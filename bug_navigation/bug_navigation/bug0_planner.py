#!/usr/bin/env python3
"""
Bug 0 Planner — ROS 2 Jazzy / Gazebo Harmonic
───────────────────────────────────────────────
States:
  GO_TO_GOAL   — крутимся к цели, едем прямо
  WALL_FOLLOW  — следуем вдоль стены (правило правой руки)
  GOAL_REACHED — стоп

Условие выхода из WALL_FOLLOW (Bug 0):
  Как только сектор лидара в направлении цели свободен — немедленно
  переходим в GO_TO_GOAL. M-line не проверяем.

Топики:
  /scan       sensor_msgs/LaserScan
  /odom       nav_msgs/Odometry
  /goal_pose  geometry_msgs/PoseStamped
  /cmd_vel    geometry_msgs/TwistStamped  (Jazzy bridge требует TwistStamped)
"""

import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from tf_transformations import euler_from_quaternion

# ── параметры ──────────────────────────────────────────────────────────────────
LINEAR_SPEED   = 0.15   # м/с
ANGULAR_SPEED  = 0.5    # рад/с
OBSTACLE_DIST  = 0.40   # м  — порог препятствия спереди
WALL_DIST      = 0.35   # м  — желаемое расстояние до стены справа
GOAL_TOL       = 0.1   # м  — "цель достигнута"
FRONT_HALF_DEG = 25     # °  — полуширина конуса "спереди"
GOAL_HALF_DEG  = 15     # °  — полуширина сектора проверки пути к цели
MIN_WALL_STEPS = 15     # минимум итераций в WALL_FOLLOW перед выходом (~1.5 сек)
CLEAR_DIST     = 0.60   # м  — путь к цели считается свободным только если дальше этого
# ──────────────────────────────────────────────────────────────────────────────


class Bug0Node(Node):
    GO_TO_GOAL   = 0
    WALL_FOLLOW  = 1
    GOAL_REACHED = 2

    def __init__(self):
        super().__init__('bug0_planner')

        self.cmd_pub = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        self.create_subscription(LaserScan,   '/scan',      self._cb_scan, 10)
        self.create_subscription(Odometry,    '/odom',      self._cb_odom, 10)
        self.create_subscription(PoseStamped, '/goal_pose', self._cb_goal, 10)
        self.create_timer(0.1, self._loop)

        self.scan  = None
        self.x = self.y = self.yaw = 0.0
        self.goal  = None
        self.state = self.GO_TO_GOAL
        self.wall_steps = 0   # счётчик итераций в WALL_FOLLOW

        self.get_logger().info(
            'Bug0 готов. Задайте цель через /goal_pose или RViz → 2D Goal Pose.')

    # ── callbacks ──────────────────────────────────────────────────────────────

    def _cb_scan(self, msg: LaserScan):
        self.scan = msg

    def _cb_odom(self, msg: Odometry):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x, self.y = p.x, p.y
        _, _, self.yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])

    def _cb_goal(self, msg: PoseStamped):
        self.goal  = (msg.pose.position.x, msg.pose.position.y)
        self.state = self.GO_TO_GOAL
        self.get_logger().info(f'Новая цель: {self.goal}')

    # ── геометрия ──────────────────────────────────────────────────────────────

    def _dist_to_goal(self) -> float:
        if self.goal is None:
            return float('inf')
        return math.hypot(self.goal[0] - self.x, self.goal[1] - self.y)

    def _angle_to_goal(self) -> float:
        """Угол от текущего курса до цели, [-π, π]."""
        dx, dy = self.goal[0] - self.x, self.goal[1] - self.y
        err = math.atan2(dy, dx) - self.yaw
        return math.atan2(math.sin(err), math.cos(err))

    def _sector_min(self, center_deg: float, half_deg: float) -> float:
        """Минимальное значение лидара в секторе center_deg ± half_deg."""
        if self.scan is None:
            return float('inf')
        s = self.scan
        lo = int((math.radians(center_deg - half_deg) - s.angle_min) / s.angle_increment)
        hi = int((math.radians(center_deg + half_deg) - s.angle_min) / s.angle_increment)
        lo = max(0, min(len(s.ranges) - 1, lo))
        hi = max(0, min(len(s.ranges) - 1, hi))
        if lo > hi:
            lo, hi = hi, lo
        vals = [r for r in s.ranges[lo:hi + 1] if math.isfinite(r)]
        return min(vals) if vals else float('inf')

    def _front_blocked(self) -> bool:
        return self._sector_min(0.0, FRONT_HALF_DEG) < OBSTACLE_DIST

    def _goal_path_clear(self) -> bool:
        """Путь к цели свободен — проверяем широким сектором с запасом."""
        deg = math.degrees(self._angle_to_goal())
        return self._sector_min(deg, GOAL_HALF_DEG) > CLEAR_DIST

    # ── публикация ────────────────────────────────────────────────────────────

    def _publish(self, linear_x: float = 0.0, angular_z: float = 0.0):
        # защита от nan/inf — Gazebo падает с Invalid joint velocity
        if not math.isfinite(linear_x):
            linear_x = 0.0
        if not math.isfinite(angular_z):
            angular_z = 0.0
        msg = TwistStamped()
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x  = linear_x
        msg.twist.angular.z = angular_z
        self.cmd_pub.publish(msg)

    # ── главный цикл ──────────────────────────────────────────────────────────

    def _loop(self):
        if self.goal is None or self.scan is None:
            return

        if self._dist_to_goal() < GOAL_TOL:
            if self.state != self.GOAL_REACHED:
                self.get_logger().info('Цель достигнута!')
                self.state = self.GOAL_REACHED
            self._publish()
            return

        if self.state == self.GO_TO_GOAL:
            if self._front_blocked():
                self.get_logger().info('Препятствие → WALL_FOLLOW')
                self.state = self.WALL_FOLLOW
                self.wall_steps = 0
            else:
                self._cmd_to_goal()

        elif self.state == self.WALL_FOLLOW:
            self.wall_steps += 1
            # выходим только после минимального времени у стены
            if self.wall_steps > MIN_WALL_STEPS and self._goal_path_clear():
                self.get_logger().info(f'Путь свободен (шаг {self.wall_steps}) → GO_TO_GOAL')
                self.state = self.GO_TO_GOAL
            else:
                self._cmd_wall_follow()

    def _cmd_to_goal(self):
        err = self._angle_to_goal()
        if abs(err) > math.radians(8):
            self._publish(angular_z=math.copysign(ANGULAR_SPEED, err))
        else:
            self._publish(linear_x=LINEAR_SPEED, angular_z=1.5 * err)

    def _cmd_wall_follow(self):
        """Правило правой руки: держим стену справа."""
        front = self._sector_min(0.0, FRONT_HALF_DEG)
        right = self._sector_min(-90.0, 15.0)

        if front < OBSTACLE_DIST:
            self._publish(angular_z=ANGULAR_SPEED)
        else:
            # P-регулятор, клампим чтобы не было inf/nan если стена не видна
            raw = -0.8 * (right - WALL_DIST)
            angular = max(-ANGULAR_SPEED, min(ANGULAR_SPEED, raw))
            self._publish(linear_x=LINEAR_SPEED, angular_z=angular)


def main(args=None):
    rclpy.init(args=args)
    node = Bug0Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()