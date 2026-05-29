import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch_ros.actions import Node, SetParameter
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    # Get paths
    isr_nav_dir = get_package_share_directory('nav2_navigation')

    # Launch configurations
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    use_composition = LaunchConfiguration('use_composition')

    # Paths
    nav2_launch_file = os.path.join(isr_nav_dir, 'launch', 'navigation_launch.py')
    default_params_file = os.path.join(isr_nav_dir, 'config', 'ranger_isaac_params.yaml')
    default_map_file = os.path.join(isr_nav_dir, 'map_2D', 'isaac_rivermark.yaml')
    goal_publisher_params_file = os.path.join(isr_nav_dir, 'config', 'goal_publisher_params.yaml')

    # Declare arguments
    declare_map_cmd = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Full path to map yaml file to load'
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Full path to the ROS2 parameters file'
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation time'
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        'use_composition',
        default_value='False',
        description='Load nav2 nodes into a single component container for zero-copy IPC'
    )

    # Component container for composition (active only when use_composition:=True)
    # Uses component_container_isolated (per-node executor) — matches upstream nav2_bringup choice.
    nav2_container = Node(
        package='rclcpp_components',
        executable='component_container_isolated',
        name='nav2_container',
        output='screen',
        condition=IfCondition(use_composition),
    )

    # Map Server Node
    map_server_node = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server_node",
        parameters=[
            {"yaml_filename": map_yaml_file},
        ],
        output="screen",
    )

    lifecycle_node = Node(
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map_server",
        emulate_tty=True,
        parameters=[
            {"autostart": True},
            {"node_names": ["map_server_node"]},
        ],
        output="screen",
    )

    # Goal Publisher Node
    goal_publisher_node = Node(
        package='nav2_navigation',
        executable='goal_publisher',
        name='goal_publisher',
        output='screen',
        parameters=[goal_publisher_params_file]
    )

    # Goal Publisher GUI Node
    goal_publisher_gui_node = Node(
        package='nav2_navigation',
        executable='goal_publisher_gui',
        name='goal_publisher_gui',
        output='screen'
    )

    nodes = GroupAction(
        actions=[
            SetParameter('use_sim_time', use_sim_time),
            nav2_container,
            map_server_node,
            lifecycle_node,
            goal_publisher_node,
            goal_publisher_gui_node,
        ]
    )

    return LaunchDescription([
        declare_map_cmd,
        declare_params_file_cmd,
        declare_use_sim_time_cmd,
        declare_use_composition_cmd,

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(nav2_launch_file),
            launch_arguments={
                'params_file': params_file,
                'use_sim_time': use_sim_time,
                'autostart': 'True',
                'use_composition': use_composition,
            }.items()
        ),

        nodes
    ])
