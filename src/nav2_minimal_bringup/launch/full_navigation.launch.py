from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node

import os
from pathlib import Path


def _default_map_yaml(pkg_share: str) -> str:
    pkg_share_path = Path(pkg_share)
    workspace_root = pkg_share_path.parents[3]
    workspace_map = workspace_root / "maps" / "my_map.yaml"
    if workspace_map.is_file():
        return str(workspace_map)
    return str(pkg_share_path / "maps" / "my_map.yaml")


def _package_file(package_name: str, *relative_path: str) -> str:
    return os.path.join(get_package_share_directory(package_name), *relative_path)


def _car_node(executable_name: str, params_file: LaunchConfiguration) -> Node:
    return Node(
        package="car_ctrl",
        executable=executable_name,
        output="screen",
        emulate_tty=True,
        parameters=[params_file],
    )


def generate_launch_description():
    pkg_share = get_package_share_directory("mobile_robot_nav_bringup")
    car_params_default = _package_file("car_ctrl", "config", "ddsm_hat_diff_drive.yaml")
    lidar_params_default = _package_file("lslidar_driver", "params", "lsx10.yaml")
    lidar_rviz_default = _package_file("lslidar_driver", "rviz", "lslidar.rviz")

    map_yaml = LaunchConfiguration("map")
    nav2_params_file = LaunchConfiguration("nav2_params_file")
    car_params_file = LaunchConfiguration("car_params_file")
    lidar_params_file = LaunchConfiguration("lidar_params_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    autostart = LaunchConfiguration("autostart")
    navigation_autostart = LaunchConfiguration("navigation_autostart")
    startup_navigation_on_initial_pose = LaunchConfiguration("startup_navigation_on_initial_pose")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    use_nav_rviz = LaunchConfiguration("use_nav_rviz")
    use_lidar_rviz = LaunchConfiguration("use_lidar_rviz")
    nav_rviz_config = LaunchConfiguration("nav_rviz_config")
    lidar_rviz_config = LaunchConfiguration("lidar_rviz_config")
    log_level = LaunchConfiguration("log_level")

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, "launch", "navigation.launch.py")
        ),
        launch_arguments={
            "map": map_yaml,
            "params_file": nav2_params_file,
            "use_sim_time": use_sim_time,
            "autostart": autostart,
            "navigation_autostart": navigation_autostart,
            "startup_navigation_on_initial_pose": startup_navigation_on_initial_pose,
            "use_composition": use_composition,
            "use_respawn": use_respawn,
            "use_rviz": use_nav_rviz,
            "rviz_config": nav_rviz_config,
            "log_level": log_level,
        }.items(),
    )

    lidar_rviz = ExecuteProcess(
        cmd=[
            "rviz2",
            "-d",
            lidar_rviz_config,
        ],
        output="screen",
        emulate_tty=True,
        condition=IfCondition(use_lidar_rviz),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "map",
                default_value=_default_map_yaml(pkg_share),
                description="Full path to the map YAML file.",
            ),
            DeclareLaunchArgument(
                "nav2_params_file",
                default_value=os.path.join(pkg_share, "config", "nav2_params.yaml"),
                description="Full path to the Nav2 parameters file.",
            ),
            DeclareLaunchArgument(
                "car_params_file",
                default_value=car_params_default,
                description="Full path to the car controller and IMU parameters file.",
            ),
            DeclareLaunchArgument(
                "lidar_params_file",
                default_value=lidar_params_default,
                description="Full path to the lidar driver parameters file.",
            ),
            DeclareLaunchArgument(
                "use_sim_time",
                default_value="false",
                description="Use simulation clock if true.",
            ),
            DeclareLaunchArgument(
                "autostart",
                default_value="true",
                description="Automatically transition localization lifecycle nodes.",
            ),
            DeclareLaunchArgument(
                "navigation_autostart",
                default_value="false",
                description="Automatically transition navigation lifecycle nodes.",
            ),
            DeclareLaunchArgument(
                "startup_navigation_on_initial_pose",
                default_value="true",
                description="Start navigation lifecycle after an initial pose is received.",
            ),
            DeclareLaunchArgument(
                "use_composition",
                default_value="True",
                description="Compose Nav2 nodes into a single container to reduce SBC CPU and memory overhead.",
            ),
            DeclareLaunchArgument(
                "use_respawn",
                default_value="False",
                description="Respawn Nav2 nodes if they crash.",
            ),
            DeclareLaunchArgument(
                "use_nav_rviz",
                default_value="true",
                description="Start Nav2 RViz with the integrated bringup. Keep false on SBCs unless actively debugging.",
            ),
            DeclareLaunchArgument(
                "use_lidar_rviz",
                default_value="false",
                description="Start the lidar vendor RViz with the integrated bringup.",
            ),
            DeclareLaunchArgument(
                "nav_rviz_config",
                default_value=os.path.join(
                    get_package_share_directory("nav2_bringup"),
                    "rviz",
                    "nav2_default_view.rviz",
                ),
                description="Full path to the Nav2 RViz config file.",
            ),
            DeclareLaunchArgument(
                "lidar_rviz_config",
                default_value=lidar_rviz_default,
                description="Full path to the lidar RViz config file.",
            ),
            DeclareLaunchArgument(
                "log_level",
                default_value="info",
                description="Logging level for Nav2 nodes.",
            ),
            _car_node("ddsm_hat_diff_drive_node", car_params_file),
            _car_node("imu_driver", car_params_file),
            _car_node("car_odometry", car_params_file),
            Node(
                package="lslidar_driver",
                executable="lslidar_driver_node",
                output="screen",
                emulate_tty=True,
                parameters=[lidar_params_file],
            ),
            lidar_rviz,
            navigation_launch,
        ]
    )
