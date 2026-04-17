#!/usr/bin/env python3
"""A* global planner ROS 2 node with frontier-exploration fallback.

Main behaviour:
1. On a new goal (or every map update), try to plan directly to the goal.
2. If that fails because the goal is unreachable given what's known so
   far, find the nearest "frontier" (boundary between known-free and
   unknown), plan to a point just on the known-free side of it, and go
   there. This drives the robot into unknown territory, which makes the
   mapper fill in more cells.
3. On the next map update we re-plan to the real goal — so as soon as
   the corridor to the goal is observed, the robot redirects there
   without the user having to do anything.

Footprint
---------
Two params describe the robot's collision size:
  * `robot_radius_m`  — physical circumscribed radius (0.25 m for this
    URDF: sqrt(0.20² + 0.15²) to the far corner of a 0.4×0.3 base).
  * `safety_margin_m` — extra clearance; inflation radius = sum of the
    two. Paths stay this far from any wall, covering odometry drift and
    controller overshoot.
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry, Path

from a_star_planner.a_star import (
    astar_grid, inflate_obstacles, smooth_path,
    world_to_grid, grid_to_world, is_free,
)


class AStarPlannerNode(Node):
    def __init__(self):
        super().__init__('a_star_planner_node')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('global_frame', 'odom')

        # ── Footprint ─────────────────────────────────────────────────────
        self.declare_parameter('robot_radius_m', 0.25)
        self.declare_parameter('safety_margin_m', 0.10)

        self.declare_parameter('occupied_threshold', 50)
        # With a live-built map, "unknown" = "not yet scanned". For the
        # DIRECT plan we keep this False so we can optimistically route
        # through unknown space; for the FRONTIER-EXPLORATION fallback
        # we force unknown as blocked (so the sub-goal is reachable
        # entirely through known-free space).
        self.declare_parameter('treat_unknown_as_obstacle', False)
        self.declare_parameter('smooth', True)
        self.declare_parameter('replan_every_n_maps', 1)

        # Clear disc around the robot; 0 = auto (inflation radius).
        self.declare_parameter('clear_disc_radius_m', 0.0)
        # Goal snapping if target cell is blocked.
        self.declare_parameter('goal_snap_radius_cells', 15)

        # ── Frontier exploration ──────────────────────────────────────────
        # Max cells sampled as exploration candidates (bigger = slower but
        # finds further frontiers). 400 is fine for 400×400 grids.
        self.declare_parameter('max_frontier_candidates', 400)
        # Weight on "distance to goal" vs "distance to robot" when picking
        # a frontier. 1.0 = prefer closest to robot; >1.0 = prefer ones
        # closer to the goal (so exploration is biased toward the target).
        self.declare_parameter('frontier_goal_bias', 1.5)
        # Don't bother with frontiers closer than this to the robot
        # (otherwise the robot "explores" its own footprint).
        self.declare_parameter('frontier_min_distance_m', 0.8)
        # Goal reached if planner's endpoint is within this of the user's
        # commanded goal (so we don't keep re-planning to a cell the
        # follower is already on).
        self.declare_parameter('goal_reached_tolerance_m', 0.25)

        gp = self.get_parameter
        self.global_frame = gp('global_frame').value
        self.robot_radius = gp('robot_radius_m').value
        self.safety_margin = gp('safety_margin_m').value
        self.inflation_radius = self.robot_radius + self.safety_margin

        self.occ_thresh = gp('occupied_threshold').value
        self.unknown_blocked = gp('treat_unknown_as_obstacle').value
        self.do_smooth = gp('smooth').value
        self.replan_every = max(1, int(gp('replan_every_n_maps').value))

        clear_param = gp('clear_disc_radius_m').value
        self.clear_disc_m = clear_param if clear_param > 0 else self.inflation_radius
        self.goal_snap_cells = int(gp('goal_snap_radius_cells').value)

        self.max_frontier_candidates = int(gp('max_frontier_candidates').value)
        self.frontier_goal_bias = gp('frontier_goal_bias').value
        self.frontier_min_dist = gp('frontier_min_distance_m').value
        self.goal_reached_tol = gp('goal_reached_tolerance_m').value

        self.map: OccupancyGrid | None = None
        self.inflated_grid: np.ndarray | None = None
        self.raw_grid: np.ndarray | None = None     # pre-inflation, for frontier detection
        self.map_update_counter = 0

        self.robot_xy = None
        self.active_goal: PoseStamped | None = None

        latched = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, gp('map_topic').value,
                                 self._map_cb, latched)
        self.create_subscription(Odometry, gp('odom_topic').value,
                                 self._odom_cb, 10)
        self.create_subscription(PoseStamped, gp('goal_topic').value,
                                 self._goal_cb, 10)
        self.path_pub = self.create_publisher(Path, gp('path_topic').value, latched)

        self.get_logger().info(
            f'A* planner ready. Footprint: robot_radius={self.robot_radius:.2f} m, '
            f'safety_margin={self.safety_margin:.2f} m, '
            f'inflation={self.inflation_radius:.2f} m. '
            f'Frontier exploration enabled.')

    # ---------- callbacks ----------
    def _odom_cb(self, msg: Odometry):
        self.robot_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def _map_cb(self, msg: OccupancyGrid):
        self.map = msg
        H, W = msg.info.height, msg.info.width
        grid = np.array(msg.data, dtype=np.int8).reshape(H, W)
        self.raw_grid = grid  # for frontier detection
        radius_cells = max(0, int(math.ceil(self.inflation_radius / msg.info.resolution)))
        self.inflated_grid = inflate_obstacles(grid, radius_cells, self.occ_thresh)
        self.map_update_counter += 1

        if (self.active_goal is not None
                and self.map_update_counter % self.replan_every == 0):
            self._plan_and_publish()

    def _goal_cb(self, goal: PoseStamped):
        self.active_goal = goal
        self.get_logger().info(
            f'New goal: ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}) '
            f'in "{goal.header.frame_id}"')
        self._plan_and_publish()

    # ---------- grid helpers ----------
    def _clear_disc_around(self, grid, row, col, radius_cells):
        H, W = grid.shape
        r = radius_cells
        for dr in range(-r, r + 1):
            for dc in range(-r, r + 1):
                if dr * dr + dc * dc > r * r:
                    continue
                rr, cc = row + dr, col + dc
                if 0 <= rr < H and 0 <= cc < W:
                    grid[rr, cc] = 0

    def _find_nearest_free(self, grid, row, col, max_radius,
                           unknown_blocked=None):
        """Ring-expansion search for nearest free cell."""
        if unknown_blocked is None:
            unknown_blocked = self.unknown_blocked
        H, W = grid.shape
        if is_free(grid, row, col, self.occ_thresh, unknown_blocked):
            return (row, col)
        for r in range(1, max_radius + 1):
            for dr in range(-r, r + 1):
                for dc in (-r, r):
                    rr, cc = row + dr, col + dc
                    if (0 <= rr < H and 0 <= cc < W
                            and is_free(grid, rr, cc, self.occ_thresh,
                                        unknown_blocked)):
                        return (rr, cc)
                for dc in range(-r + 1, r):
                    for dr_side in (-r, r):
                        rr, cc = row + dr_side, col + dc
                        if (0 <= rr < H and 0 <= cc < W
                                and is_free(grid, rr, cc, self.occ_thresh,
                                            unknown_blocked)):
                            return (rr, cc)
        return None

    # ---------- frontier detection ----------
    def _find_frontier_cells(self, inflated):
        """Return indices of frontier cells.

        A frontier cell is a known-free cell (inflated grid < occ_thresh
        and != -1) that has at least one unknown (-1) 8-neighbour.
        Using the inflated grid ensures frontier cells are actually
        reachable by the robot.
        """
        H, W = inflated.shape
        free_mask = (inflated >= 0) & (inflated < self.occ_thresh)
        unknown_mask = (inflated == -1)

        # Dilate unknown mask by 1 via 8-neighbour OR. Any free cell that
        # ends up under the dilated mask is a frontier cell.
        dil = np.zeros_like(unknown_mask)
        dil[1:, :]   |= unknown_mask[:-1, :]
        dil[:-1, :]  |= unknown_mask[1:, :]
        dil[:, 1:]   |= unknown_mask[:, :-1]
        dil[:, :-1]  |= unknown_mask[:, 1:]
        dil[1:, 1:]   |= unknown_mask[:-1, :-1]
        dil[1:, :-1]  |= unknown_mask[:-1, 1:]
        dil[:-1, 1:]  |= unknown_mask[1:, :-1]
        dil[:-1, :-1] |= unknown_mask[1:, 1:]

        frontier = free_mask & dil
        return np.argwhere(frontier)  # array of (row, col)

    def _pick_frontier(self, frontier_cells, start_rc, goal_rc, res):
        """Choose the best frontier by weighted distance.

        score = dist_to_robot + bias * dist_to_goal
        Lower score = better (closer to robot AND closer to goal).
        Skips frontiers very close to the robot's footprint.
        """
        if len(frontier_cells) == 0:
            return None

        if len(frontier_cells) > self.max_frontier_candidates:
            idx = np.random.choice(len(frontier_cells),
                                   self.max_frontier_candidates, replace=False)
            frontier_cells = frontier_cells[idx]

        dr_robot = frontier_cells[:, 0] - start_rc[0]
        dc_robot = frontier_cells[:, 1] - start_rc[1]
        d_robot = np.hypot(dr_robot, dc_robot) * res

        dr_goal = frontier_cells[:, 0] - goal_rc[0]
        dc_goal = frontier_cells[:, 1] - goal_rc[1]
        d_goal = np.hypot(dr_goal, dc_goal) * res

        # Exclude frontiers too close to the robot.
        mask = d_robot >= self.frontier_min_dist
        if not mask.any():
            return None

        score = d_robot[mask] + self.frontier_goal_bias * d_goal[mask]
        best_i = int(np.argmin(score))
        valid = frontier_cells[mask]
        chosen = tuple(valid[best_i])
        return (int(chosen[0]), int(chosen[1])), float(score.min())

    # ---------- planning ----------
    def _plan_and_publish(self):
        if self.map is None or self.inflated_grid is None:
            self.get_logger().warn('No map yet.', throttle_duration_sec=2)
            return
        if self.robot_xy is None:
            self.get_logger().warn('No odometry yet.', throttle_duration_sec=2)
            return
        if self.active_goal is None:
            return

        info = self.map.info
        ox, oy, res = info.origin.position.x, info.origin.position.y, info.resolution
        H, W = info.height, info.width

        working = self.inflated_grid.copy()

        start_rc = world_to_grid(*self.robot_xy, ox, oy, res)
        gx = self.active_goal.pose.position.x
        gy = self.active_goal.pose.position.y
        goal_rc = world_to_grid(gx, gy, ox, oy, res)

        for name, (r, c) in (('start', start_rc), ('goal', goal_rc)):
            if not (0 <= r < H and 0 <= c < W):
                self.get_logger().error(
                    f'{name} cell {(r, c)} outside grid (W={W}, H={H}). '
                    f'Increase mapper size_m.')
                return

        # Goal already reached? Don't flip-flop a stale plan.
        dist_to_goal_m = math.hypot(self.robot_xy[0] - gx, self.robot_xy[1] - gy)
        if dist_to_goal_m < self.goal_reached_tol:
            return

        # Clear disc around robot.
        disc_cells = max(1, int(math.ceil(self.clear_disc_m / res)))
        self._clear_disc_around(working, *start_rc, disc_cells)

        # Snap goal to nearest free if blocked.
        goal_free = is_free(working, *goal_rc, self.occ_thresh, self.unknown_blocked)
        if not goal_free:
            snapped = self._find_nearest_free(working, *goal_rc, self.goal_snap_cells)
            if snapped is not None:
                self.get_logger().info(
                    f'Goal {goal_rc} blocked, snapped to {snapped}')
                goal_rc = snapped

        if not is_free(working, *start_rc, self.occ_thresh, self.unknown_blocked):
            self.get_logger().error(
                f'Start cell {start_rc} not free after clear disc. '
                f'value={int(working[start_rc])}')
            return

        # ── 1. Try the direct plan ────────────────────────────────────────
        path_cells = None
        if goal_free or self._find_nearest_free(
                working, *goal_rc, 1) is not None:
            path_cells = astar_grid(working, start_rc, goal_rc,
                                    self.occ_thresh, self.unknown_blocked)

        # ── 2. Fallback: frontier exploration ─────────────────────────────
        used_frontier = False
        if path_cells is None:
            frontier_cells = self._find_frontier_cells(working)
            if len(frontier_cells) == 0:
                self._log_no_path_and_bail(working, start_rc, goal_rc,
                                           reason='no frontiers left')
                return

            picked = self._pick_frontier(frontier_cells, start_rc, goal_rc, res)
            if picked is None:
                self._log_no_path_and_bail(working, start_rc, goal_rc,
                                           reason='all frontiers too close')
                return
            frontier_rc, score = picked

            # For the exploration sub-plan we BLOCK unknown cells so the
            # robot can actually traverse to the frontier through known-
            # free space only.
            path_cells = astar_grid(working, start_rc, frontier_rc,
                                    self.occ_thresh,
                                    treat_unknown_as_obstacle=True)
            if path_cells is None:
                self._log_no_path_and_bail(working, start_rc, goal_rc,
                                           reason='frontier unreachable')
                return

            used_frontier = True
            self.get_logger().info(
                f'Direct plan failed → exploring frontier at {frontier_rc} '
                f'(score={score:.2f}). Will re-plan to goal on next map update.')

        if self.do_smooth:
            path_cells = smooth_path(path_cells)

        # ── Publish path ──────────────────────────────────────────────────
        out = Path()
        out.header.frame_id = self.global_frame
        out.header.stamp = self.get_clock().now().to_msg()
        for r, c in path_cells:
            wx, wy = grid_to_world(r, c, ox, oy, res)
            ps = PoseStamped()
            ps.header = out.header
            ps.pose.position.x = wx
            ps.pose.position.y = wy
            ps.pose.orientation.w = 1.0
            out.poses.append(ps)
        if out.poses and not used_frontier:
            out.poses[-1].pose.orientation = self.active_goal.pose.orientation

        self.path_pub.publish(out)
        tag = 'FRONTIER' if used_frontier else 'DIRECT'
        self.get_logger().info(
            f'[{tag}] Plan: {len(out.poses)} waypoints, '
            f'{start_rc} -> {path_cells[-1]}')

    def _log_no_path_and_bail(self, grid, start_rc, goal_rc, reason):
        H, W = grid.shape
        occ = int((grid >= self.occ_thresh).sum())
        unk = int((grid == -1).sum())
        free = H * W - occ - unk
        self.get_logger().warn(
            f'A*: no path from {start_rc} to {goal_rc} ({reason}). '
            f'Grid: {free} free / {unk} unknown / {occ} occupied.',
            throttle_duration_sec=3)


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerNode()
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
