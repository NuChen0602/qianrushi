from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    LogInfo,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown, matches_action
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode, Node
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from launch_ros.parameter_descriptions import ParameterValue
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    share = Path(get_package_share_directory('robot_lidar_bridge'))
    map_yaml = LaunchConfiguration('map_yaml')
    map_file = LaunchConfiguration('map_file')
    localizer = LaunchConfiguration('localizer')
    board_ip = LaunchConfiguration('board_ip')
    lidar_port = LaunchConfiguration('lidar_port')
    odom_port = LaunchConfiguration('odom_port')
    dashboard_port = LaunchConfiguration('dashboard_port')

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
                'enable_drive': True,
                'enable_drive_obstacle_safety': True,
                'front_sector_center_rad': 0.0,
                'obstacle_stop_distance_m': 0.08,
                'obstacle_slow_distance_m': 0.35,
            },
        ],
        output='screen',
    )
    guard = Node(
        package='robot_lidar_bridge',
        executable='mapping_guard',
        name='navigation_input_guard',
        output='screen',
    )
    watchdog = Node(
        package='robot_lidar_bridge',
        executable='mapping_watchdog',
        name='navigation_watchdog',
        output='screen',
    )
    map_server = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace='',
        parameters=[{
            'yaml_filename': ParameterValue(map_yaml, value_type=str),
            'topic_name': 'map',
            'frame_id': 'map',
        }],
        output='screen',
    )
    amcl = LifecycleNode(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        namespace='',
        parameters=[str(share / 'config' / 'amcl.yaml')],
        output='screen',
    )
    slam_localizer = Node(
        package='slam_toolbox',
        executable='localization_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[
            str(share / 'config' / 'slam_localization.yaml'),
            {'map_file_name': map_file},
        ],
        output='screen',
    )
    planner = Node(
        package='robot_lidar_bridge',
        executable='ackermann_path_planner',
        name='ackermann_path_planner',
        parameters=[str(share / 'config' / 'ackermann_navigation.yaml')],
        output='screen',
    )
    safety = Node(
        package='robot_lidar_bridge',
        executable='navigation_safety',
        name='navigation_safety',
        parameters=[str(share / 'config' / 'ackermann_navigation.yaml')],
        output='screen',
    )
    navigator = Node(
        package='robot_lidar_bridge',
        executable='goal_navigator',
        name='goal_navigator',
        parameters=[str(share / 'config' / 'ackermann_navigation.yaml')],
        output='screen',
    )
    map_cells = Node(
        package='robot_lidar_bridge',
        executable='occupancy_grid_cells',
        name='occupancy_grid_cells',
        output='screen',
    )
    dashboard = Node(
        package='robot_lidar_bridge',
        executable='navigation_dashboard',
        name='navigation_dashboard',
        parameters=[{
            'port': ParameterValue(dashboard_port, value_type=int),
        }],
        output='screen',
    )
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', str(share / 'config' / 'navigation.rviz')],
        output='screen',
    )

    configure_map = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(map_server),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))
    activate_map = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(map_server),
        transition_id=Transition.TRANSITION_ACTIVATE,
    ))
    configure_amcl = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(amcl),
        transition_id=Transition.TRANSITION_CONFIGURE,
    ))
    activate_amcl = EmitEvent(event=ChangeState(
        lifecycle_node_matcher=matches_action(amcl),
        transition_id=Transition.TRANSITION_ACTIVATE,
    ))

    def on_guard_exit(event, _context):
        use_slam = _context.perform_substitution(localizer).lower() == 'slam'
        if event.returncode == 0:
            if use_slam:
                return [
                    LogInfo(msg=(
                        'Navigation inputs passed; starting localization '
                        'and goal navigation.')),
                    watchdog,
                    LogInfo(msg=(
                        'Using slam_toolbox localization for map->odom '
                        'and /map.')),
                    slam_localizer,
                    safety,
                    planner,
                    navigator,
                    map_cells,
                    dashboard,
                    rviz,
                ]
            return [
                LogInfo(msg=(
                    'Navigation inputs passed; starting localization '
                    'and goal navigation.')),
                watchdog,
                map_server,
                LogInfo(msg='Using AMCL localization for map->odom.'),
                amcl,
                safety,
                planner,
                navigator,
                map_cells,
                dashboard,
                rviz,
                TimerAction(period=0.5, actions=[configure_map]),
            ]
        return [
            LogInfo(msg='ERROR: Navigation inputs failed; refusing to start.'),
            EmitEvent(event=Shutdown(
                reason='navigation input health check failed')),
        ]

    def on_watchdog_exit(event, _context):
        if event.returncode == 0 or event.returncode < 0:
            return []
        return [
            LogInfo(msg='ERROR: Navigation watchdog stopped this run.'),
            EmitEvent(event=Shutdown(
                reason='navigation inputs became unhealthy')),
        ]

    return LaunchDescription([
        DeclareLaunchArgument('map_yaml', default_value='maps/library.yaml'),
        DeclareLaunchArgument('map_file', default_value='maps/library'),
        DeclareLaunchArgument('localizer', default_value='slam'),
        DeclareLaunchArgument('board_ip', default_value='192.168.123.70'),
        DeclareLaunchArgument('lidar_port', default_value='2368'),
        DeclareLaunchArgument('odom_port', default_value='2369'),
        DeclareLaunchArgument('dashboard_port', default_value='8080'),
        laser_bridge,
        odometry_bridge,
        RegisterEventHandler(
            OnProcessExit(target_action=guard, on_exit=on_guard_exit)),
        RegisterEventHandler(
            OnProcessExit(target_action=watchdog, on_exit=on_watchdog_exit)),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=map_server,
            goal_state='inactive',
            entities=[activate_map],
        )),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=map_server,
            goal_state='active',
            entities=[
                LogInfo(msg='Fixed map active.'),
                TimerAction(
                    period=0.1,
                    actions=[configure_amcl],
                    condition=IfCondition(PythonExpression([
                        "'", localizer, "' == 'amcl'"]))),
            ],
        )),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=amcl,
            goal_state='inactive',
            entities=[activate_amcl],
        )),
        RegisterEventHandler(OnStateTransition(
            target_lifecycle_node=amcl,
            goal_state='active',
            entities=[LogInfo(msg=(
                'AMCL active; navigation is ready after initial pose.'))],
        )),
        guard,
    ])
