"""
bug0.launch.py
──────────────
Запускает:
  1. TurtleBot3 Gazebo (turtlebot3_world)
  2. RViz2 (пустой, с фиксированным фреймом map)
  3. Bug0 planner node

Использование:
  ros2 launch bug_navigation bug0.launch.py
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    rviz_config = PathJoinSubstitution([
        FindPackageShare('bug_navigation'),
        'config',
        'config.rviz',
    ])

    # ── 1. TurtleBot3 Gazebo ──────────────────────────────────────────────────
    tb3_gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('turtlebot3_gazebo'),
                'launch',
                'turtlebot3_world.launch.py',
            ])
        ]),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )

    # ── 2. RViz2 (пустой) ─────────────────────────────────────────────────────
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],          # без конфига — пустой
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # ── 3. Bug0 planner ───────────────────────────────────────────────────────
    bug0 = Node(
        package='bug_navigation',
        executable='bug0_planner',
        name='bug0_planner',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='true',
                              description='Use simulation clock'),
        tb3_gazebo,
        rviz,
        bug0,
    ])