from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_library = get_package_share_directory('library_gazebo')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_slam_toolbox = get_package_share_directory('slam_toolbox')
    pkg_nav2_bringup = get_package_share_directory('nav2_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    start_vision = LaunchConfiguration('start_vision')
    start_dashboard = LaunchConfiguration('start_dashboard')
    start_web_dashboard = LaunchConfiguration('start_web_dashboard')
    web_dashboard_port = LaunchConfiguration('web_dashboard_port')
    world_file = LaunchConfiguration('world')

    urdf_file = os.path.join(pkg_library, 'urdf', 'patrol_robot.urdf')
    nav2_params = os.path.join(pkg_library, 'config', 'nav2_params.yaml')
    slam_params = os.path.join(pkg_library, 'config', 'slam_toolbox_online_async.yaml')

    with open(urdf_file, 'r', encoding='utf-8') as urdf:
        robot_description = urdf.read()

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use Gazebo clock for all ROS nodes.'
    )

    declare_start_vision = DeclareLaunchArgument(
        'start_vision',
        default_value='true',
        description='Start the C++ vision bridge node.'
    )

    declare_start_dashboard = DeclareLaunchArgument(
        'start_dashboard',
        default_value='false',
        description='Start the terminal patrol dashboard.'
    )

    declare_start_web_dashboard = DeclareLaunchArgument(
        'start_web_dashboard',
        default_value='true',
        description='Start the browser-based patrol dashboard.'
    )

    declare_web_dashboard_port = DeclareLaunchArgument(
        'web_dashboard_port',
        default_value='8080',
        description='HTTP port for the browser-based patrol dashboard.'
    )

    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_library, 'worlds', 'library.world'),
        description='Gazebo world file.'
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_description,
        }]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'true',
        }.items()
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-entity', 'patrol_robot',
            '-topic', 'robot_description',
            '-x', '1.0',
            '-y', '0.0',
            '-z', '0.10',
        ],
        output='screen'
    )

    slam = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam_toolbox, 'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'slam_params_file': slam_params,
        }.items()
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_nav2_bringup, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'params_file': nav2_params,
            'autostart': 'true',
        }.items()
    )

    vision_bridge = Node(
        package='sim_bridge_node',
        executable='vision_bridge',
        name='vision_bridge_node',
        output='screen',
        condition=IfCondition(start_vision)
    )

    patrol_dashboard = Node(
        package='library_gazebo',
        executable='patrol_dashboard.py',
        name='patrol_dashboard',
        output='screen',
        condition=IfCondition(start_dashboard)
    )

    patrol_web_dashboard = Node(
        package='library_gazebo',
        executable='patrol_web_dashboard.py',
        name='patrol_web_dashboard',
        output='screen',
        parameters=[{'port': web_dashboard_port}],
        condition=IfCondition(start_web_dashboard)
    )

    delayed_slam = TimerAction(
        period=4.0,
        actions=[slam]
    )

    delayed_nav2 = TimerAction(
        period=14.0,
        actions=[nav2]
    )

    delayed_vision = TimerAction(
        period=5.0,
        actions=[vision_bridge]
    )

    delayed_dashboard = TimerAction(
        period=2.0,
        actions=[patrol_dashboard, patrol_web_dashboard]
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_start_vision,
        declare_start_dashboard,
        declare_start_web_dashboard,
        declare_web_dashboard_port,
        declare_world,
        robot_state_publisher,
        gazebo,
        spawn_robot,
        delayed_slam,
        delayed_nav2,
        delayed_vision,
        delayed_dashboard,
    ])
