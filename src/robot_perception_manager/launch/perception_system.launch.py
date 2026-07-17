from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory(
        "robot_perception_manager"
    )

    parameter_file = os.path.join(
        package_share,
        "config",
        "perception_params.yaml"
    )

    return LaunchDescription([
        Node(
            package="robot_perception_manager",
            executable="perception_manager",
            name="perception_manager",
            output="screen",
            parameters=[parameter_file]
        ),

        Node(
            package="robot_perception_manager",
            executable="camera_tf_broadcaster",
            name="camera_tf_broadcaster",
            output="screen",
            parameters=[parameter_file]
        )
    ])