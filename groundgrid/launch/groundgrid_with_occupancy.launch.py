#!/usr/bin/env python3
"""
Launch file for GroundGrid with OccupancyMapGenerator.

This launch file starts:
1. GroundGrid node for ground segmentation
2. OccupancyMapGenerator node for creating 2D occupancy grid maps
3. Static transforms (optional)
4. RViz (optional)

Bag file should be played separately.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Package share directory
    pkg_share = FindPackageShare('groundgrid')

    # ========== Topic configuration ==========
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic',
        default_value='/odom',
        description='Odometry topic'
    )

    pointcloud_topic_arg = DeclareLaunchArgument(
        'pointcloud_topic',
        default_value='/velodyne_points',
        description='Point cloud topic'
    )

    sensor_frame_arg = DeclareLaunchArgument(
        'sensor_frame',
        default_value='velodyne',
        description='LiDAR sensor frame name'
    )

    # ========== Occupancy map parameters ==========
    resolution_arg = DeclareLaunchArgument(
        'resolution',
        default_value='0.1',
        description='Map resolution in meters'
    )

    save_path_arg = DeclareLaunchArgument(
        'save_path',
        default_value='/home/soon/data2/',
        description='Path to save map files'
    )

    # Fixed map bounds
    x_min_arg = DeclareLaunchArgument('x_min', default_value='-200.0')
    x_max_arg = DeclareLaunchArgument('x_max', default_value='200.0')
    y_min_arg = DeclareLaunchArgument('y_min', default_value='-200.0')
    y_max_arg = DeclareLaunchArgument('y_max', default_value='200.0')

    # ========== Optional features ==========
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from bag'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz'
    )

    use_tf_arg = DeclareLaunchArgument(
        'use_tf',
        default_value='false',
        description='Publish static transforms (map->odom, sensor_frame->base_link)'
    )

    # ========== Static transforms ==========
    # map -> odom (identity)
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=IfCondition(LaunchConfiguration('use_tf')),
    )

    # sensor_frame -> base_link (identity)
    # Note: Using velodyne_front as default, adjust if needed
    sensor_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='sensor_to_base_link',
        arguments=['0', '0', '0', '0', '0', '0',
                   LaunchConfiguration('sensor_frame'), 'base_link'],
        condition=IfCondition(LaunchConfiguration('use_tf')),
    )

    # ========== GroundGrid node ==========
    groundgrid_config = PathJoinSubstitution([pkg_share, 'config', 'groundgrid.yaml'])

    groundgrid_node = Node(
        package='groundgrid',
        executable='groundgrid_node',
        name='groundgrid',
        output='screen',
        parameters=[
            groundgrid_config,  # Load YAML config file
            {
                # Override with launch arguments
                'sensor_frame': LaunchConfiguration('sensor_frame'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'points_topic': LaunchConfiguration('pointcloud_topic'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
    )

    # ========== OccupancyMapGenerator node ==========
    occupancy_map_generator_node = Node(
        package='groundgrid',
        executable='occupancy_map_generator',
        name='occupancy_map_generator',
        output='screen',
        parameters=[
            groundgrid_config,  # Load YAML config file
            {
                # Override with launch arguments
                'resolution': LaunchConfiguration('resolution'),
                'save_path': LaunchConfiguration('save_path'),
                'use_fixed_bounds': True,
                'x_min': LaunchConfiguration('x_min'),
                'x_max': LaunchConfiguration('x_max'),
                'y_min': LaunchConfiguration('y_min'),
                'y_max': LaunchConfiguration('y_max'),
                'odom_topic': LaunchConfiguration('odom_topic'),
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }
        ],
    )

    # ========== RViz ==========
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', PathJoinSubstitution([pkg_share, 'rviz', 'groundgrid.rviz'])],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    )

    return LaunchDescription([
        # Arguments
        odom_topic_arg,
        pointcloud_topic_arg,
        sensor_frame_arg,
        resolution_arg,
        save_path_arg,
        x_min_arg,
        x_max_arg,
        y_min_arg,
        y_max_arg,
        use_sim_time_arg,
        use_rviz_arg,
        use_tf_arg,

        # Static transforms
        map_to_odom_tf,
        sensor_to_base_tf,

        # Nodes
        groundgrid_node,
        occupancy_map_generator_node,
        rviz_node,
    ])
