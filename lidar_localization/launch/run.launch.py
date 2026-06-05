import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():

    share_dir = get_package_share_directory('lidar_localization')
    parameter_file = LaunchConfiguration('params_file')
    map_file = LaunchConfiguration('map_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    sensor = LaunchConfiguration('sensor')

    # 온라인(실시간): rviz2.rviz, 오프라인(rosbag): rviz2_localization.rviz
    rviz_config_file = LaunchConfiguration('rviz_config')

    # 가상 센서 선택: sensor:=xt32 또는 sensor:=vlp16
    # -> config/params_<sensor>.yaml 을 자동으로 사용.
    # params_file 을 직접 넘기면 그 값이 우선한다.
    sensor_declare = DeclareLaunchArgument(
        'sensor',
        default_value='ouster32',
        description="Simulated lidar preset to use: 'ouster32' or 'vlp16' "
                    "(selects config/params_<sensor>.yaml).")

    params_declare = DeclareLaunchArgument(
        'params_file',
        default_value=PythonExpression(
            ["'", os.path.join(share_dir, 'config', 'params_'),
             "' + '", sensor, "' + '.yaml'"]),
        description='Path to the ROS2 parameters file to use. '
                    'Overrides the sensor preset when set explicitly.')

    map_file_declare = DeclareLaunchArgument(
        'map_file',
        default_value=os.path.join(share_dir, 'map_3D', '03_18_Isaacmap.pcd'),
        description='Path to the global map PCD file for localization.')

    use_sim_time_declare = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time')

    rviz_config_declare = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(share_dir, 'config', 'rviz2.rviz'),
        description='Path to the RViz config file')

    return LaunchDescription([
        sensor_declare,
        params_declare,
        map_file_declare,
        use_sim_time_declare,
        rviz_config_declare,

        # Global map server for localization
        Node(
            package='lidar_localization',
            executable='lidar_localization_globalmapServer',
            name='globalmap_server',
            parameters=[
                parameter_file,
                {'globalmap_pcd': map_file},
                {'use_sim_time': use_sim_time}
            ],
            output='screen'
        ),

        # Static transform publishers
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments='0.0 0.0 0.0 0.0 0.0 0.0 map odom'.split(' '),
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        ),



        # LIO-SAM nodes
        Node(
            package='lidar_localization',
            executable='lidar_localization_imuPreintegration',
            name='lidar_localization_imuPreintegration',
            parameters=[parameter_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='lidar_localization',
            executable='lidar_localization_imageProjection',
            name='lidar_localization_imageProjection',
            parameters=[parameter_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='lidar_localization',
            executable='lidar_localization_featureExtraction',
            name='lidar_localization_featureExtraction',
            parameters=[parameter_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),
        Node(
            package='lidar_localization',
            executable='lidar_localization_localizationOptimization',
            name='lidar_localization_localizationOptimization',
            parameters=[parameter_file, {'use_sim_time': use_sim_time}],
            output='screen'
        ),

        # Visualization
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config_file],
            parameters=[{'use_sim_time': use_sim_time}],
            output='screen'
        )
    ])
