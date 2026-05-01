from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    package_share = get_package_share_directory("car_ctrl")
    config_file = os.path.join(package_share, "config", "ddsm_hat_diff_drive.yaml")
    rviz_config_file = os.path.join(package_share, "rviz", "tf_only.rviz")

    return LaunchDescription([
        Node(
            package="car_ctrl",
            executable="ddsm_hat_diff_drive_node",
            name="ddsm_hat_diff_drive",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="car_ctrl",
            executable="imu_driver",
            name="imu_driver",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="car_ctrl",
            executable="car_odometry",
            name="car_odometry",
            output="screen",
            parameters=[config_file],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config_file],
        )
    ])
