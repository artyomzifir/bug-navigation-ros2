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
            executable='bug0_planner_node',
            name='bug0_planner_node',
            output='screen',
            parameters=[{
                'use_sim_time': use_sim_time,
                'wall_follow_dist': 0.42,
                'obstacle_threshold': 0.5,
                'goal_tolerance': 0.2,
                'linear_speed': 0.3,
                'angular_speed': 0.75,
                'control_loop_freq': 20.0,
                'target_frame': 'odom',
                'goal_clear_arc_deg': 15.0,
                'front_check_arc_deg': 30.0,
            }],
        ),
    ])
