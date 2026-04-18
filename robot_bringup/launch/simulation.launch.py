"""
simulation.launch.py — запускает Gazebo + robot_state_publisher + RViz одной командой.

Использование:
  ros2 launch robot_bringup simulation.launch.py
  ros2 launch robot_bringup simulation.launch.py rviz:=false
  ros2 launch robot_bringup simulation.launch.py world:=/path/to/custom.world
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = FindPackageShare(package='robot_bringup')
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros')

    # ── Launch arguments ─────────────────────────────────────────────────────

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock',
    )
    world_arg = DeclareLaunchArgument(
        'world',
        default_value=PathJoinSubstitution([pkg_share, 'worlds', 'test.world']),
        description='Path to Gazebo world file',
    )
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch RViz visualization',
    )
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'robot.rviz']),
        description='Path to RViz config file',
    )
    robot_x_arg = DeclareLaunchArgument('robot_x', default_value='0.0', description='Spawn X')
    robot_y_arg = DeclareLaunchArgument('robot_y', default_value='0.0', description='Spawn Y')
    robot_yaw_arg = DeclareLaunchArgument('robot_yaw', default_value='0.0', description='Spawn Yaw')

    # ── LaunchConfigurations ─────────────────────────────────────────────────

    use_sim_time = LaunchConfiguration('use_sim_time')
    world = LaunchConfiguration('world')
    rviz = LaunchConfiguration('rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    robot_x = LaunchConfiguration('robot_x')
    robot_y = LaunchConfiguration('robot_y')
    robot_yaw = LaunchConfiguration('robot_yaw')

    # ── Robot description (xacro → URDF string) ──────────────────────────────

    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf.xacro'])
    robot_description = Command(['xacro ', xacro_file])

    # ── Nodes ────────────────────────────────────────────────────────────────

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }],
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_robot',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'diff_drive_robot',
            '-x', robot_x,
            '-y', robot_y,
            '-z', '0.1',
            '-Y', robot_yaw,
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        condition=IfCondition(rviz),
    )

    # ── Gazebo server + client ────────────────────────────────────────────────

    gzserver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzserver.launch.py'])
        ),
        launch_arguments={
            'world': world,
            'verbose': 'false',
            'pause': 'false',
        }.items(),
    )

    gzclient = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzclient.launch.py'])
        ),
        launch_arguments={'verbose': 'false'}.items(),
    )

    # ── Launch description ────────────────────────────────────────────────────

    ld = LaunchDescription()

    # Arguments
    ld.add_action(use_sim_time_arg)
    ld.add_action(world_arg)
    ld.add_action(rviz_arg)
    ld.add_action(rviz_config_arg)
    ld.add_action(robot_x_arg)
    ld.add_action(robot_y_arg)
    ld.add_action(robot_yaw_arg)

    # Gazebo
    ld.add_action(gzserver)
    ld.add_action(gzclient)

    # Robot
    ld.add_action(robot_state_publisher)
    ld.add_action(spawn_robot)

    # Visualization
    ld.add_action(rviz_node)

    return ld
