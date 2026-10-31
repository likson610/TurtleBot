import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    pad_teleop = Node(
        package="joy_teleop",
        executable="joy_teleop",
        parameters=[
            os.path.join(get_package_share_directory("turtlebot_controller"), "config", "pad_teleop.yaml"),
            {"use_sim_time": False}
        ],
    )

    pad_node = Node(
        package="joy",
        executable="joy_node",
        name="joystick",
        parameters=[
            os.path.join(get_package_share_directory("turtlebot_controller"), "config", "pad_config.yaml"),
            {"use_sim_time": False}
        ]
    )

    return LaunchDescription(
        [
            pad_teleop,
            pad_node
        ]
    )