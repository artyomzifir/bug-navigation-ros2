"""Launch A*: live LIDAR mapper + A* planner (with frontier exploration)
+ path follower.

No external map source needed — the mapper builds /map in real time from
/scan and /odom. Everything runs in the `odom` frame.

Topics:
    /scan, /odom        -> laser_grid_mapper_node  -> /map
    /map, /odom, /goal_pose -> a_star_planner_node -> /planned_path
    /planned_path, /odom    -> path_follower       -> /cmd_vel
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),

        # ── 1. Live occupancy-grid mapper from LIDAR ──────────────────────
        Node(
            package='a_star_planner',
            executable='laser_grid_mapper_node',
            name='laser_grid_mapper_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'resolution': 0.05,
                'size_m': 20.0,
                'origin_x': -10.0,
                'origin_y': -10.0,
                'frame_id': 'odom',
                'publish_rate_hz': 2.0,
                'max_update_range': 5.0,
            }],
        ),

        # ── 2. A* planner + frontier exploration ──────────────────────────
        Node(
            package='a_star_planner',
            executable='a_star_planner_node',
            name='a_star_planner_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'map_topic': '/map',
                'odom_topic': '/odom',
                'goal_topic': '/goal_pose',
                'path_topic': '/planned_path',
                'global_frame': 'odom',
                # Footprint (from URDF: base 0.4x0.3 m → corner at 0.25 m).
                'robot_radius_m': 0.25,
                # Extra clearance from walls — 10 cm covers odometry drift.
                'safety_margin_m': 0.10,
                # 0 = auto (= robot_radius + safety_margin).
                'clear_disc_radius_m': 0.0,
                'occupied_threshold': 50,
                'treat_unknown_as_obstacle': False,
                'smooth': True,
                'replan_every_n_maps': 1,
                'goal_snap_radius_cells': 15,
                # Frontier exploration tuning.
                'max_frontier_candidates': 400,
                'frontier_goal_bias': 1.5,
                'frontier_min_distance_m': 0.8,
                'goal_reached_tolerance_m': 0.25,
            }],
        ),

        # ── 3. Path follower ──────────────────────────────────────────────
        Node(
            package='a_star_planner',
            executable='path_follower_node',
            name='path_follower',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'linear_speed': 0.3,
                'angular_speed': 0.75,
                'waypoint_tolerance': 0.2,
                'turn_in_place_threshold_deg': 25.0,
                'rate_hz': 20.0,
            }],
        ),
    ])
