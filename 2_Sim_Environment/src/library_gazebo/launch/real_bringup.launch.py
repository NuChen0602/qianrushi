from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    board_host = LaunchConfiguration("board_host")
    board_port = LaunchConfiguration("board_port")
    local_port = LaunchConfiguration("local_port")
    camera_device = LaunchConfiguration("camera_device")
    camera_width = LaunchConfiguration("camera_width")
    camera_height = LaunchConfiguration("camera_height")
    camera_fps = LaunchConfiguration("camera_fps")
    start_camera = LaunchConfiguration("start_camera")
    start_vision = LaunchConfiguration("start_vision")
    start_web_dashboard = LaunchConfiguration("start_web_dashboard")
    start_voice = LaunchConfiguration("start_voice")
    web_dashboard_port = LaunchConfiguration("web_dashboard_port")

    return LaunchDescription([
        DeclareLaunchArgument("board_host", default_value="192.168.2.77"),
        DeclareLaunchArgument("board_port", default_value="15000"),
        DeclareLaunchArgument("local_port", default_value="15001"),
        DeclareLaunchArgument("camera_device", default_value="/dev/video0"),
        DeclareLaunchArgument("camera_width", default_value="640"),
        DeclareLaunchArgument("camera_height", default_value="480"),
        DeclareLaunchArgument("camera_fps", default_value="15.0"),
        DeclareLaunchArgument("start_camera", default_value="true"),
        DeclareLaunchArgument("start_vision", default_value="true"),
        DeclareLaunchArgument("start_web_dashboard", default_value="true"),
        DeclareLaunchArgument("start_voice", default_value="false"),
        DeclareLaunchArgument("web_dashboard_port", default_value="8080"),

        Node(
            package="library_gazebo",
            executable="real_motion_bridge.py",
            name="real_motion_bridge",
            output="screen",
            parameters=[{
                "board_host": board_host,
                "board_port": board_port,
                "local_port": local_port,
            }],
        ),
        Node(
            package="library_gazebo",
            executable="real_goal_driver.py",
            name="real_goal_driver",
            output="screen",
        ),
        Node(
            package="library_gazebo",
            executable="real_camera_publisher.py",
            name="real_camera_publisher",
            output="screen",
            condition=IfCondition(start_camera),
            parameters=[{
                "device": camera_device,
                "width": camera_width,
                "height": camera_height,
                "fps": camera_fps,
            }],
        ),
        Node(
            package="sim_bridge_node",
            executable="vision_bridge",
            name="vision_bridge_node",
            output="screen",
            condition=IfCondition(start_vision),
        ),
        Node(
            package="library_gazebo",
            executable="patrol_web_dashboard.py",
            name="patrol_web_dashboard",
            output="screen",
            parameters=[{"port": web_dashboard_port}],
            condition=IfCondition(start_web_dashboard),
        ),
        Node(
            package="library_gazebo",
            executable="llm_voice_navigator.py",
            name="llm_voice_navigator_node",
            output="screen",
            parameters=[{
                "use_nav2": False,
                "real_goal_wait_sec": 8.0,
            }],
            condition=IfCondition(start_voice),
        ),
    ])
