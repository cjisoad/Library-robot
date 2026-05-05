from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    config = Path(get_package_share_directory("arm_aux_devices")) / "config" / "lifttable.yaml"

    return LaunchDescription(
        [
            Node(
                package="arm_aux_devices",
                executable="lifttable",
                name="lifttable",
                output="screen",
                parameters=[str(config)],
            )
        ]
    )
