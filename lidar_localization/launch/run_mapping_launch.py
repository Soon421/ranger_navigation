import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('lidar_localization')
    parameter_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    rviz_config_file = os.path.join(share_dir, 'config', 'rviz2_mapping.rviz')

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            share_dir, 'config', 'params_mapping.yaml'),
        description='Path to the ROS2 parameters file to use.')

    use_sim_time_declare = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time')

    return LaunchDescription([
        params_declare,
        use_sim_time_declare,

        # Mapping nodes
        Node(
            package='lidar_localization',
            executable='lidar_localization_imuPreintegration',
            name='lidar_localization_imuPreintegration',
            parameters=[parameter_file, {'use_sim_time': use_sim_time}],
            output='screen',
            respawn=True
        ),
        Node(
            package='lidar_localization',
            executable='lidar_localization_imageProjection',
            name='lidar_localization_imageProjection',
            parameters=[parameter_file, {'use_sim_time': use_sim_time}],
            output='screen',
            respawn=True
        ),
        Node(
            package='lidar_localization',
            executable='lidar_localization_featureExtraction',
            name='lidar_localization_featureExtraction',
            parameters=[parameter_file, {'use_sim_time': use_sim_time}],
            output='screen',
            respawn=True
        ),
        Node(
            package='lidar_localization',
            executable='lidar_localization_mapOptimization',
            name='lidar_localization_mapOptmization',
            parameters=[parameter_file, {'use_sim_time': use_sim_time}],
            output='screen',
            respawn=True
        ),

        # GPS Transformation node
        Node(
            package='lidar_localization',
            executable='lidar_localization_gpsTransformation',
            name='gps_transformation',
            parameters=[
                parameter_file,
                {'use_sim_time': use_sim_time},
                {'gps_origin_lat': 37.58091508003259},
                {'gps_origin_long': 127.02640766847915},
                {'gps_origin_alt': 25.077},
                {'gps_origin_yaw': 0.505401129400864},
                {'gps_origin_zone': '52S'},
                {'use_initial_altitude_as_origin': False},
                {'offset': [0.1, 0.0, 0.0]}
            ],
            remappings=[
                ('/gps/fix', '/gps/fix'),
                ('/odom/gps', '/odom/gps')
            ],
            output='screen'
        ),

        # Global map server
        # Node(
        #     package='lidar_localization',
        #     executable='lidar_localization_globalmapServer',
        #     name='globalmap_server',
        #     parameters=[
        #         parameter_file,
        #         {'use_sim_time': use_sim_time},
        #         {'globalmap_pcd': os.path.join(share_dir, 'map_3D', 'gongdae_all_map_raw.pcd')},
        #         {'globalmap_downsample_resolution': 0.2},
        #         {'voxelmap_min_points_per_voxel': 3}
        #     ],
        #     remappings=[
        #         ('/globalmap', '/globalmap')
        #     ],
        #     output='screen'
        # ),

        # Static transform publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_gps_origin',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'gps_origin'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='velodyne_transform',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'velodyne'],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Visualization - RViz2
        Node(
            package='rviz2',
            executable='rviz2',
            name='lidar_localization_rviz',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])
