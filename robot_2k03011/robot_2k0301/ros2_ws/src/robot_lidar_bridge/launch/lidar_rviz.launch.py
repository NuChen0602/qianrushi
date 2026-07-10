from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('robot_lidar_bridge'))
    board_ip = LaunchConfiguration('board_ip')
    lidar_port = LaunchConfiguration('lidar_port')
    return LaunchDescription([
        DeclareLaunchArgument('board_ip', default_value='192.168.123.70'),
        DeclareLaunchArgument('lidar_port', default_value='2368'),
        Node(
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
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', str(share / 'config' / 'lidar.rviz')],
            output='screen',
        ),
    ])
