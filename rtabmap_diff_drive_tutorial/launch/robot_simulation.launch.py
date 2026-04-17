import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import xacro

def generate_launch_description():

    # Package paths
    pkg_share = FindPackageShare(package='rtabmap_diff_drive_tutorial')
    pkg_gazebo_ros = FindPackageShare(package='gazebo_ros')

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    # Default world path relative to the package
    default_world_path = PathJoinSubstitution([
        pkg_share, 
        'worlds', 
        'test.world'
    ])
    world_path = LaunchConfiguration('world', default=default_world_path) 

    # URDF/Xacro file path
    xacro_file = PathJoinSubstitution([pkg_share, 'urdf', 'robot.urdf.xacro'])
    # Use Command to process xacro file
    robot_description_raw = Command(['xacro ', xacro_file])

    # === Nodes ===

    # Robot State Publisher
    # Takes the URDF and publishes /tf based on joint states provided by Gazebo
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_raw,
            'use_sim_time': use_sim_time # Use Gazebo's clock
        }]
    )
    
    # Joint State Publisher
    # Optional but can help ensure joint states are published
    # node_joint_state_publisher = Node(
    #     package='joint_state_publisher',
    #     executable='joint_state_publisher',
    #     output='screen',
    #     parameters=[{
    #         'use_sim_time': use_sim_time,
    #         # Don't publish fixed joints to reduce noise
    #         'publish_default_positions': False,
    #         # Use existing joint states from diff drive plugin
    #         'source_list': ['/joint_states'],
    #     }]
    # )

    # Gazebo Simulation
    # Launches Gazebo server and client
    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzserver.launch.py'])
        ),
        # Pass the world file and set verbose output
        launch_arguments={'world': world_path, 'verbose': 'true', 'pause': 'false'}.items() # Start Gazebo unpaused
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg_gazebo_ros, 'launch', 'gzclient.launch.py'])
        ),
         launch_arguments={'verbose': 'true'}.items()
    )

    # Spawn Entity Node
    # Spawns the robot model into Gazebo from the /robot_description topic
    spawn_entity_node = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description', # Subscribe to the processed URDF
                   '-entity', 'diff_drive_robot', # Name of the model in Gazebo
                   '-x', '0.0', # Initial X position
                   '-y', '0.0', # Initial Y position
                   '-z', '0.1', # Initial Z position (slightly above ground)
                   '-Y', '0.0'], # Initial Yaw orientation
        output='screen'
    )

    # === Launch Description ===
    ld = LaunchDescription()

    # Declare launch arguments that can be passed via command line
    ld.add_action(DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock?'))
    ld.add_action(DeclareLaunchArgument(
        'world',
        default_value=default_world_path,
        description='Full path to world file to load'))

    # Add nodes and included launch files to the description
    ld.add_action(gzserver_cmd)            # Start Gazebo server
    ld.add_action(gzclient_cmd)            # Start Gazebo client GUI
    ld.add_action(node_robot_state_publisher) # Publish TF tree
    # ld.add_action(node_joint_state_publisher) # Publish joint states
    ld.add_action(spawn_entity_node)       # Spawn robot model

    return ld
