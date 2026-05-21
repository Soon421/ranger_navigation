from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import (
    AnyLaunchDescriptionSource,
    PythonLaunchDescriptionSource,
)
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    sensor_hostname_arg = DeclareLaunchArgument(
        'sensor_hostname',
        default_value='192.168.100.19',
        description='Ouster lidar hostname or IP address.',
    )
    um7_port_arg = DeclareLaunchArgument(
        'um7_port',
        default_value='/dev/ttyUSB0',
        description='UM7 IMU serial port. Host udev sets MODE=0666; symlink /dev/um7 exists on host but not in container.',
    )
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='false',
        description='Launch RealSense D435. Default false since autonomous driving uses lidar+IMU only.',
    )
    scan_ring_arg = DeclareLaunchArgument(
        'scan_ring',
        default_value='16',
        description='Ouster beam index used for /ouster/scan LaserScan. 16 ≈ horizontal for OS-32.',
    )

    ouster_launch = GroupAction([
        IncludeLaunchDescription(
            AnyLaunchDescriptionSource(
                PathJoinSubstitution([
                    FindPackageShare('ouster_ros'), 'launch', 'sensor.launch.xml',
                ])
            ),
            launch_arguments={
                'sensor_hostname': LaunchConfiguration('sensor_hostname'),
                'viz': 'false',
                'timestamp_mode': 'TIME_FROM_ROS_TIME',
                'scan_ring': LaunchConfiguration('scan_ring'),
            }.items(),
        )
    ])

    realsense_launch = GroupAction(
        [
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare('realsense2_camera'), 'launch', 'rs_launch.py',
                    ])
                ),
            )
        ],
        condition=IfCondition(LaunchConfiguration('use_camera')),
    )

    um7_node = Node(
        package='umx_driver',
        executable='um7_driver',
        name='um7_driver',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('um7_port'),
            'baud': 921600,
            'update_rate': 200,
            'mag_updates': False,
            'frame_id': 'imu_link',
        }],
    )

    return LaunchDescription([
        sensor_hostname_arg,
        um7_port_arg,
        use_camera_arg,
        scan_ring_arg,
        ouster_launch,
        realsense_launch,
        um7_node,
    ])
