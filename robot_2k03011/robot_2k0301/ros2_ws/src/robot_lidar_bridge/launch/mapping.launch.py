from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('robot_lidar_bridge'))
    board_ip = LaunchConfiguration('board_ip')
    lidar_port = LaunchConfiguration('lidar_port')
    odom_port = LaunchConfiguration('odom_port')
    enable_drive = LaunchConfiguration('enable_drive')
    enable_drive_obstacle_safety = LaunchConfiguration(
        'enable_drive_obstacle_safety')
    obstacle_stop_distance = LaunchConfiguration('obstacle_stop_distance_m')
    obstacle_slow_distance = LaunchConfiguration('obstacle_slow_distance_m')
    enable_ips200_map_display = LaunchConfiguration('enable_ips200_map_display')
    ips200_map_port = LaunchConfiguration('ips200_map_port')

    laser_bridge = Node(
        package='robot_lidar_bridge',
        executable='tcp_laser_scan',
        name='robot_lidar_tcp_bridge',
        parameters=[{
            'host': board_ip,
            'port': ParameterValue(lidar_port, value_type=int),
            'laser_x': 0.0,
            'laser_y': 0.0,
        }],
        output='screen',
    )
    odometry_bridge = Node(
        package='robot_lidar_bridge',
        executable='tcp_odometry',
        name='robot_odometry_tcp_bridge',
        parameters=[
            str(share / 'config' / 'odometry.yaml'),
            {
                'host': board_ip,
                'port': ParameterValue(odom_port, value_type=int),
                'enable_drive': ParameterValue(enable_drive, value_type=bool),
                'enable_drive_obstacle_safety': ParameterValue(
                    enable_drive_obstacle_safety, value_type=bool),
                'obstacle_stop_distance_m': ParameterValue(
                    obstacle_stop_distance, value_type=float),
                'obstacle_slow_distance_m': ParameterValue(
                    obstacle_slow_distance, value_type=float),
            },
        ],
        output='screen',
    )
    guard = Node(
        package='robot_lidar_bridge',
        executable='mapping_guard',
        name='mapping_guard',
        output='screen',
    )
    watchdog = Node(
        package='robot_lidar_bridge',
        executable='mapping_watchdog',
        name='mapping_watchdog',
        output='screen',
    )
    slam = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[str(share / 'config' / 'slam_mapping.yaml')],
        output='screen',
    )
    map_cells = Node(
        package='robot_lidar_bridge',
        executable='occupancy_grid_cells',
        name='occupancy_grid_cells',
        output='screen',
    )
    ips200_map_stream = Node(
        package='robot_lidar_bridge',
        executable='ips200_map_stream',
        name='ips200_map_stream',
        parameters=[{
            'host': board_ip,
            'port': ParameterValue(ips200_map_port, value_type=int),
            'frame_width': 240,
            'frame_height': 180,
            'max_fps': 2.0,
        }],
        condition=IfCondition(enable_ips200_map_display),
        output='screen',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(share / 'config' / 'mapping.rviz')],
        output='screen',
    )

    def on_guard_exit(event, _context):
        if event.returncode == 0:
            return [
                LogInfo(msg='Mapping inputs passed; starting SLAM and RViz.'),
                watchdog,
                slam,
                map_cells,
                ips200_map_stream,
                rviz,
            ]
        return [
            LogInfo(msg='ERROR: Mapping inputs failed; refusing to start SLAM.'),
            EmitEvent(event=Shutdown(reason='mapping input health check failed')),
        ]

    def on_watchdog_exit(event, _context):
        if event.returncode == 0 or event.returncode < 0:
            return []
        return [
            LogInfo(msg='ERROR: Mapping watchdog stopped the mapping run.'),
            EmitEvent(event=Shutdown(reason='mapping inputs became unhealthy')),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('board_ip', default_value='192.168.123.70'),
        DeclareLaunchArgument('lidar_port', default_value='2368'),
        DeclareLaunchArgument('odom_port', default_value='2369'),
        DeclareLaunchArgument('enable_drive', default_value='false'),
        DeclareLaunchArgument(
            'enable_drive_obstacle_safety', default_value='true'),
        DeclareLaunchArgument('obstacle_stop_distance_m', default_value='0.5'),
        DeclareLaunchArgument('obstacle_slow_distance_m', default_value='0.8'),
        DeclareLaunchArgument('enable_ips200_map_display', default_value='true'),
        DeclareLaunchArgument('ips200_map_port', default_value='2370'),
        laser_bridge,
        odometry_bridge,
        RegisterEventHandler(
            OnProcessExit(target_action=guard, on_exit=on_guard_exit)),
        RegisterEventHandler(
            OnProcessExit(target_action=watchdog, on_exit=on_watchdog_exit)),
        guard,
    ])
