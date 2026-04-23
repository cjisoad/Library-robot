from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")

    default_params = PathJoinSubstitution(
        [FindPackageShare("imu_car_ros2"), "config", "car_params.yaml"]
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "params_file",
                default_value=default_params,
                description="Path to the ROS2 parameters YAML file.",
            ),
            Node(
                package="imu_car_ros2",
                executable="car_controller",
                name="car_controller",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="imu_car_ros2",
                executable="imu_driver",
                name="imu_driver",
                output="screen",
                parameters=[params_file],
            ),
            Node(
                package="imu_car_ros2",
                executable="car_odometry",
                name="car_odometry",
                output="screen",
                parameters=[params_file],
            ),
        ]
    )
