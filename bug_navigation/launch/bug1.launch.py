from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true'),
        Node(
            package='bug_navigation',
            executable='bug1_planner_node',
            name='bug1_planner_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'wall_follow_dist': 0.4,
                'obstacle_threshold': 0.4,
                'goal_tolerance': 0.2,
                'linear_speed': 0.2,
                'angular_speed': 0.5,
                'control_loop_freq': 10.0,
                'target_frame': 'odom',
                'hit_point_tolerance': 0.3,
                'min_circumnav_distance': 1.0,
            }],
        ),
    ])
