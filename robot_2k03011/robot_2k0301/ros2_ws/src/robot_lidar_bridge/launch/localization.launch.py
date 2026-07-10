from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('robot_lidar_bridge'))
    map_file = LaunchConfiguration('map_file')
    board_ip = LaunchConfiguration('board_ip')
    lidar_port = LaunchConfiguration('lidar_port')
    odom_port = LaunchConfiguration('odom_port')

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
        name='localization_watchdog',
        output='screen',
    )
    slam = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            str(share / 'config' / 'slam_localization.yaml'),
            {'map_file_name': map_file},
        ],
        output='screen',
    )
    map_cells = Node(
        package='robot_lidar_bridge',
        executable='occupancy_grid_cells',
        name='occupancy_grid_cells',
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
                LogInfo(msg='Localization inputs passed; starting localization.'),
                watchdog,
                slam,
                map_cells,
                rviz,
            ]
        return [
            LogInfo(msg='ERROR: Localization inputs failed; refusing to start.'),
            EmitEvent(
                event=Shutdown(reason='localization input health check failed')),
        ]

    def on_watchdog_exit(event, _context):
        if event.returncode == 0 or event.returncode < 0:
            return []
        return [
            LogInfo(msg='ERROR: Localization watchdog stopped this run.'),
            EmitEvent(
                event=Shutdown(reason='localization inputs became unhealthy')),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('map_file', default_value='maps/library'),
        DeclareLaunchArgument('board_ip', default_value='192.168.123.70'),
        DeclareLaunchArgument('lidar_port', default_value='2368'),
        DeclareLaunchArgument('odom_port', default_value='2369'),
        laser_bridge,
        odometry_bridge,
        RegisterEventHandler(
            OnProcessExit(target_action=guard, on_exit=on_guard_exit)),
        RegisterEventHandler(
            OnProcessExit(target_action=watchdog, on_exit=on_watchdog_exit)),
        guard,
    ])
