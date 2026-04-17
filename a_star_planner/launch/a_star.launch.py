"""Launch the A* planner + path follower.

Assumes the map is already being published on /map (e.g. by map_server).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='a_star_planner',
            executable='a_star_planner_node',
            name='a_star_planner_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'map_topic': '/map',
                'goal_topic': '/goal_pose',
                'path_topic': '/planned_path',
                'base_frame': 'base_link',
                'global_frame': 'map',
                'robot_radius_m': 0.18,
                'occupied_threshold': 50,
                'treat_unknown_as_obstacle': True,
                'smooth': True,
            }],
        ),
        Node(
            package='a_star_planner',
            executable='path_follower_node',
            name='path_follower',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'linear_speed': 0.2,
                'angular_speed': 0.6,
                'waypoint_tolerance': 0.15,
                'base_frame': 'base_link',
                'global_frame': 'map',
                'rate_hz': 20.0,
            }],
        ),
    ])
