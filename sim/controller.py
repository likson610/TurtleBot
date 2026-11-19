from launch import LaunchDescription, LaunchService
from launch_ros.actions import Node

launch = LaunchService()
launch.include_launch_description(LaunchDescription([
    Node(package="joy", executable="game_controller_node", output="screen"),
    Node(
        package="teleop_twist_joy",
        executable="teleop_node",
        parameters=[{
            "publish_stamped_twist": True,
            "require_enable_button": False,
            "enable_button": 9,
            "axis_linear.x": 1,
            "scale_linear.x": 0.08,
            "axis_angular.yaw": 0,
            "scale_angular.yaw": 0.3,
        }],
        output="screen",
    ),
]))
raise SystemExit(launch.run())