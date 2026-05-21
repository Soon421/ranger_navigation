import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directories
    isr_nav_dir = get_package_share_directory('nav2_navigation')
    lidar_loc_dir = get_package_share_directory('lidar_localization')

    # Default paths
    default_map_file = os.path.join(isr_nav_dir, 'map_2D', 'campus', 'campus.yaml')
    # default_map_file = os.path.join('/data2', 'map_test', 'map.yaml')  # 수정된 부분
    default_rviz_file = os.path.join(lidar_loc_dir, 'config', 'rviz2.rviz')

    # Launch configurations
    map_file = LaunchConfiguration('map_file')
    rviz_config = LaunchConfiguration('rviz_config')
    use_sim_time = LaunchConfiguration('use_sim_time')

    # Declare launch arguments
    declare_rosbag_path_cmd = DeclareLaunchArgument(
        'rosbag_path',
        default_value='',
        description='Path to the rosbag file to play'
    )

    declare_map_file_cmd = DeclareLaunchArgument(
        'map_file',
        default_value=default_map_file,
        description='Full path to the map yaml file'
    )

    declare_rviz_config_cmd = DeclareLaunchArgument(
        'rviz_config',
        default_value=default_rviz_file,
        description='Full path to the RViz config file'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from rosbag'
    )

    declare_rate_cmd = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Playback rate (e.g., 0.5 for half speed, 2.0 for double speed)'
    )

    # Static transform: map -> odom
    static_tf_map_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_map_odom',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Static transform: base_link -> velodyne
    static_tf_baselink_velodyne = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_baselink_velodyne',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'velodyne'],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Map server
    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            {'yaml_filename': map_file},
            {'use_sim_time': use_sim_time}
        ]
    )

    # Lifecycle manager for map server
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[
            {'use_sim_time': use_sim_time},
            {'autostart': True},
            {'node_names': ['map_server']}
        ]
    )

    # RViz
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # Function to create rosbag play commands with sequential execution
    def launch_rosbag_play(context):
        rosbag_path_str = context.launch_configurations['rosbag_path']
        rate = context.launch_configurations['rate']
        actions = []
        if rosbag_path_str:
            bags = [b.strip() for b in rosbag_path_str.split(',') if b.strip()]
            if bags:
                # Always use direct ExecuteProcess for first bag
                first_proc = ExecuteProcess(
                    cmd=['ros2', 'bag', 'play', bags[0], '--clock', '-r', rate],
                    output='screen',
                    name='rosbag_play_0'
                )
                actions.append(first_proc)

                # Chain additional bags with OnProcessExit
                prev_proc = first_proc
                for i, bag in enumerate(bags[1:], start=1):
                    next_proc = ExecuteProcess(
                        cmd=['ros2', 'bag', 'play', bag, '--clock', '-r', rate],
                        output='screen',
                        name=f'rosbag_play_{i}'
                    )
                    actions.append(RegisterEventHandler(
                        OnProcessExit(
                            target_action=prev_proc,
                            on_exit=[next_proc]
                        )
                    ))
                    prev_proc = next_proc
        return actions

    rosbag_play = OpaqueFunction(function=launch_rosbag_play)

    # Create launch description
    ld = LaunchDescription()

    # Declare arguments
    ld.add_action(declare_rosbag_path_cmd)
    ld.add_action(declare_map_file_cmd)
    ld.add_action(declare_rviz_config_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_rate_cmd)

    # Add nodes
    ld.add_action(static_tf_map_odom)
    ld.add_action(static_tf_baselink_velodyne)
    ld.add_action(map_server_node)
    ld.add_action(lifecycle_manager_node)
    ld.add_action(rviz_node)
    ld.add_action(rosbag_play)

    return ld
