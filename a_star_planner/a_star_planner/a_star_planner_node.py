#!/usr/bin/env python3
"""A* global planner ROS 2 node.

Subscribes:
    /map        nav_msgs/OccupancyGrid     (from map_server or RTAB-Map)
    /goal_pose  geometry_msgs/PoseStamped  (use the "2D Goal Pose" tool in RViz)

Publishes:
    /planned_path nav_msgs/Path             (in the global_frame, default 'map')

Robot start pose is read from TF (`global_frame` -> `base_frame`).
"""
import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path

import tf2_ros

from a_star_planner.a_star import (
    astar_grid, inflate_obstacles, smooth_path,
    world_to_grid, grid_to_world,
)


class AStarPlannerNode(Node):
    def __init__(self):
        super().__init__('a_star_planner_node')

        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('path_topic', '/planned_path')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('global_frame', 'map')
        self.declare_parameter('robot_radius_m', 0.18)
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('treat_unknown_as_obstacle', True)
        self.declare_parameter('smooth', True)

        gp = self.get_parameter
        self.base_frame = gp('base_frame').value
        self.global_frame = gp('global_frame').value
        self.robot_radius = gp('robot_radius_m').value
        self.occ_thresh = gp('occupied_threshold').value
        self.unknown_blocked = gp('treat_unknown_as_obstacle').value
        self.do_smooth = gp('smooth').value

        self.map: OccupancyGrid | None = None
        self.inflated_grid: np.ndarray | None = None

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # /map is published with TRANSIENT_LOCAL durability by map_server, so
        # match it or we'll never receive the message.
        map_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(OccupancyGrid, gp('map_topic').value,
                                 self._map_cb, map_qos)
        self.create_subscription(PoseStamped, gp('goal_topic').value,
                                 self._goal_cb, 10)
        # Same TRANSIENT_LOCAL trick on the output so RViz can pick up the
        # latest plan even if it subscribes after the fact.
        self.path_pub = self.create_publisher(Path, gp('path_topic').value, map_qos)

        self.get_logger().info('A* planner ready. Waiting for map and goal...')

    # ---------- callbacks ----------
    def _map_cb(self, msg: OccupancyGrid):
        self.map = msg
        H, W = msg.info.height, msg.info.width
        grid = np.array(msg.data, dtype=np.int8).reshape(H, W)
        radius_cells = max(0, int(math.ceil(self.robot_radius / msg.info.resolution)))
        self.inflated_grid = inflate_obstacles(grid, radius_cells, self.occ_thresh)
        self.get_logger().info(
            f'Map received: {W}x{H} @ {msg.info.resolution:.3f} m/cell, '
            f'inflated by {radius_cells} cells (robot_radius={self.robot_radius:.2f} m)')

    def _goal_cb(self, goal: PoseStamped):
        if self.map is None or self.inflated_grid is None:
            self.get_logger().warn('No map yet; ignoring goal.')
            return

        start_xy = self._lookup_robot_xy()
        if start_xy is None:
            self.get_logger().warn('TF lookup failed; cannot plan.')
            return

        info = self.map.info
        ox, oy, res = info.origin.position.x, info.origin.position.y, info.resolution
        H, W = info.height, info.width

        start_rc = world_to_grid(*start_xy, ox, oy, res)
        goal_rc = world_to_grid(goal.pose.position.x, goal.pose.position.y, ox, oy, res)

        for name, (r, c) in (('start', start_rc), ('goal', goal_rc)):
            if not (0 <= r < H and 0 <= c < W):
                self.get_logger().error(f'{name} {(r, c)} outside map bounds')
                return

        self.get_logger().info(
            f'Planning {start_xy} (cell {start_rc}) -> '
            f'({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}) (cell {goal_rc})')

        path_cells = astar_grid(self.inflated_grid, start_rc, goal_rc,
                                self.occ_thresh, self.unknown_blocked)
        if path_cells is None:
            self.get_logger().warn('A*: no path found.')
            return

        if self.do_smooth:
            path_cells = smooth_path(path_cells)

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
        if out.poses:
            out.poses[-1].pose.orientation = goal.pose.orientation

        self.path_pub.publish(out)
        self.get_logger().info(f'Published path with {len(out.poses)} waypoints.')

    # ---------- helpers ----------
    def _lookup_robot_xy(self):
        try:
            tf = self.tf_buffer.lookup_transform(
                self.global_frame, self.base_frame,
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5))
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            self.get_logger().warn(f'TF error: {e}')
            return None
        return tf.transform.translation.x, tf.transform.translation.y


def main(args=None):
    rclpy.init(args=args)
    node = AStarPlannerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
