import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():

    urdf = ParameterValue(
        Command(
            [
                "xacro ",
                os.path.join(
                    get_package_share_directory("turtlebot_description"),
                    "urdf",
                    "turtlebot.urdf.xacro",
                )
            ]
        ),
        value_type=str,
    )

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": urdf}],
    )

    return LaunchDescription(
        [
            robot_state_publisher_node,
        ]
    )