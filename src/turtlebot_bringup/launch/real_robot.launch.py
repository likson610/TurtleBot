import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    hardware = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("turtlebot_firmware"),
            "launch",
            "hardware.launch.py"
        ),
    )
    
    pad = IncludeLaunchDescription(
        os.path.join(
            get_package_share_directory("turtlebot_firmware"),
            "launch",
            "pad.launch.py"
        )
    )


    return LaunchDescription([
        hardware,
        pad,
    ])
