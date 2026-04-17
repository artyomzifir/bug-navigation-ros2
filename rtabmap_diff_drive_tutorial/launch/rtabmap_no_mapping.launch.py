import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():

    # Package paths
    pkg_share = FindPackageShare(package='rtabmap_diff_drive_tutorial')
    # rtabmap_ros_pkg_share = get_package_share_directory('rtabmap_ros') # Not used directly anymore

    # Default paths
    default_rviz_config_path = PathJoinSubstitution([pkg_share, 'config', 'rtabmap.rviz'])

    # Launch arguments
    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    qos = LaunchConfiguration('qos', default='2') # Use reliable QoS for simulation
    rviz_config_path = LaunchConfiguration('rviz_config', default=default_rviz_config_path)

    # Parameters for RTAB-Map node
    # NOTE: Changed string 'true'/'false' to boolean True/False
    rtabmap_params = {
        
        'Mem/IncrementalMemory': 'False',    # Use existing map, don't add new nodes
        'Mem/InitWMWithAllNodes': 'True',    # Load all map nodes on startup
        
        'subscribe_depth': True,
        'subscribe_rgb': True,

        'subscribe_odom_info': False, # Corrected: Use /odom topic directly, not /odom_info
        'frame_id': 'base_link',       # Robot base frame (must match URDF)
        'map_frame_id': 'map',         # Frame for the map
        'odom_frame_id': 'odom',       # Odometry frame (must match diff drive plugin)
        'approx_sync': True,       # Sync RGBD topics
        'wait_for_transform': 0.2,  # Corrected type: float
        'use_sim_time': use_sim_time, # LaunchConfiguration handles this correctly
        'qos_image': qos,
        'qos_imu': qos,
        'qos_odom': qos,
       # *** ENABLE LIDAR SUBSCRIPTION ***
       'subscribe_scan': True, # Now set to True
       'qos_scan': qos,        # Add QoS for scan topic

       # *** ADJUST GRID GENERATION (Optional but recommended) ***
       'Grid/FromDepth': 'False', # Use LIDAR for occupancy grid
       'Grid/FromScan': 'True',  # Use LIDAR for occupancy grid
       'Grid/RangeMax': '10.0',  # Adjust based on LIDAR range if needed


      # Disable features not needed for pure localization (optional but good practice)
       'RGBD/ProximityBySpace': 'False',
       'RGBD/LoopClosureReextractFeatures': 'False',
        # RTAB-Map Strategy (ICP vs Visual)
        'Reg/Strategy': '0',          # Use string
        'Reg/Force3DoF': 'True',      # Assume planar motion (differential drive)
        # Occupancy Grid Generation
        # 'Grid/FromDepth': 'True',     # Create occupancy grid from depth image
        # 'Grid/RangeMax': '5.0',        # Changed back to string due to node error
        # Visual features / Inliers
        'Vis/MinInliers': '12',       # Corrected type: integer
        'queue_size': 30,          # Use integer, not string
        'publish_tf_map': True ,     # Corrected type: boolean
        'database_path': LaunchConfiguration('database_path')
    }

    # Remappings
    # Adjusted to match actual topics published by gazebo_ros_camera plugin
    rtabmap_remaps = [
        ('odom', '/odom'),
        ('imu', '/imu/data'),
        ('rgb/image', '/camera/image_raw'),         # Default from Gazebo
        ('rgb/camera_info', '/camera/camera_info'), # Default from Gazebo
        ('depth/image', '/camera/depth/image_raw'), # Default from Gazebo
        # Add depth camera info remapping if needed by RTAB-Map 
        # (Usually needed if subscribe_depth=True and not approx_sync, but good practice)
        ('depth/camera_info', '/camera/depth/camera_info') ,
               # *** ADD SCAN REMAPPING ***
       ('scan', '/scan')
    ]

    # === Nodes ===

    # RTAB-Map Core Node
    # Performs SLAM
    rtabmap_node = Node(
        package='rtabmap_slam', # Adjusted package name based on find output
        executable='rtabmap',
        name='rtabmap',
        output='screen',
        parameters=[rtabmap_params],
        remappings=rtabmap_remaps,
        #arguments=['-d'] # Optional: Delete previous database on start
    )

    # RTAB-Map Viz Node
    # Provides visualization data for RViz
    rtabmap_viz_node = Node(
        package='rtabmap_viz', # Adjusted package name based on find output
        executable='rtabmap_viz',
        name='rtabmap_viz',
        output='screen',
        parameters=[{
             ### LIDAR 
           'subscribe_scan': True, # Enable scan subscription for viz
           'qos_scan': qos,        # Add QoS for scan topic
           'subscribe_odom_info': True,
           'subscribe_scan': False,
           'frame_id': 'base_link',
           'odom_frame_id': 'odom',
           'map_frame_id': 'map',
           'use_sim_time': use_sim_time,
           'qos_image': qos,
           'qos_imu': qos,
           'qos_odom': qos,
           'queue_size': 30, # Use integer, not string
        }],
        # Remap topics for visualization needs
        remappings=rtabmap_remaps # Use the same base remappings
    )

    # RViz Node
    # Visualizes the robot, map, and sensor data
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path], # Load custom RViz config
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # === Launch Description ===
    ld = LaunchDescription()

    # Declare launch arguments
    ld.add_action(DeclareLaunchArgument('use_sim_time', default_value='true', description='Use simulation (Gazebo) clock?'))
    ld.add_action(DeclareLaunchArgument('qos', default_value='2', description='QoS profile: 0=system default, 1=sensor data, 2=reliable'))
    ld.add_action(DeclareLaunchArgument('rviz_config', default_value=default_rviz_config_path, description='Full path to the RVIZ config file to use'))
    ld.add_action(DeclareLaunchArgument('database_path', default_value='~/.ros/rtabmap.db', description='Path to the RTAB-Map database file'))
    # Add nodes
    ld.add_action(rtabmap_node)
    ld.add_action(rtabmap_viz_node)
    ld.add_action(rviz_node)

    return ld
