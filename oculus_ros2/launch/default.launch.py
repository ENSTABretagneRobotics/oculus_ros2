# Copyright 2023 Forssea Robotics
# All rights reserved.
#
# Unauthorized copying of this code base via any medium is strictly prohibited.
# Proprietary and confidential.

import os

from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    ld = LaunchDescription()

    # ld.add_action(
    #     DeclareLaunchArgument(
    #         name="port",
    #         default_value="this_is_a_port",
    #         description="Filters Configuration",
    #     )
    # )

    config = os.path.join(
        get_package_share_directory("oculus_ros2"), "cfg", "default.yaml"
        # get_package_share_directory("oculus_ros2"), "cfg", "tmp.yaml"
    )

    oculus_sonar_node = Node(
        package="oculus_ros2",
        executable="oculus_sonar_node",
        name="oculus_sonar",
        parameters=[config],
        # arguments=["-port", LaunchConfiguration("port")],
        namespace="sonar",
        remappings=[
            ('status', 'status'),
            ('ping', 'ping'),
            ('temperature', 'temperature'),
            ('pressure', 'pressure'),
        ],
        output="screen",
    )

    # image_publisher_node = Node(
    #     package="oculus_ros2",
    #     executable="oculus_subscriber_to_image.py",
    #     name="oculus_subscriber_to_image",
    #     output="screen",
    # )

    rqt_reconfigure_node = Node(
        package="rqt_reconfigure",
        executable="rqt_reconfigure",
        name="rqt_reconfigure",
        output="screen",
    )

    ld.add_action(oculus_sonar_node)
    # ld.add_action(image_publisher_node)
    # ld.add_action(rqt_reconfigure_node)

    return ld
