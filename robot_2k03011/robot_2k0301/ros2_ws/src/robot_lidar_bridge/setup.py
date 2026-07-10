from glob import glob

from setuptools import find_packages, setup

package_name = 'robot_lidar_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', [
            'launch/lidar_rviz.launch.py',
            'launch/mapping.launch.py',
            'launch/localization.launch.py',
            'launch/localization_amcl.launch.py',
            'launch/navigation.launch.py',
        ]),
        ('share/' + package_name + '/config', [
            'config/ackermann_navigation.yaml',
            'config/lidar.rviz',
            'config/mapping.rviz',
            'config/navigation.rviz',
            'config/amcl.yaml',
            'config/odometry.yaml',
            'config/slam_mapping.yaml',
            'config/slam_localization.yaml',
        ]),
        ('share/' + package_name + '/web', glob('web/*')),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='chen',
    maintainer_email='chen@example.com',
    description='LS2K0301 lidar TCP to ROS 2 LaserScan bridge',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tcp_laser_scan = robot_lidar_bridge.tcp_laser_scan:main',
            'tcp_odometry = robot_lidar_bridge.tcp_odometry:main',
            'mapping_guard = robot_lidar_bridge.mapping_guard:main',
            'mapping_watchdog = robot_lidar_bridge.mapping_guard:watchdog_main',
            'keyboard_teleop = robot_lidar_bridge.keyboard_teleop:main',
            'ackermann_path_planner = robot_lidar_bridge.ackermann_path_planner:main',
            'goal_navigator = robot_lidar_bridge.goal_navigator:main',
            'navigation_safety = robot_lidar_bridge.navigation_safety:main',
            'navigation_dashboard = robot_lidar_bridge.navigation_dashboard:main',
            'occupancy_grid_cells = robot_lidar_bridge.occupancy_grid_cells:main',
            'ips200_map_stream = robot_lidar_bridge.ips200_map_stream:main',
        ],
    },
)
