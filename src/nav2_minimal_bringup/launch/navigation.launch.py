from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
from nav2_common.launch import ReplaceString

import os
from pathlib import Path


NAV2_CONTAINER_NAME = 'nav2_container'


def _default_map_yaml(pkg_share: str) -> str:
    pkg_share_path = Path(pkg_share)
    workspace_root = pkg_share_path.parents[3]
    workspace_map = workspace_root / 'maps' / 'my_map.yaml'
    if workspace_map.is_file():
        return str(workspace_map)
    return str(pkg_share_path / 'maps' / 'my_map.yaml')


def generate_launch_description():
    pkg_share = get_package_share_directory('mobile_robot_nav_bringup')
    nav2_bringup_share = get_package_share_directory('nav2_bringup')
    behavior_tree_dir = os.path.join(pkg_share, 'behavior_trees')
    default_map_yaml = _default_map_yaml(pkg_share)

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    navigation_autostart = LaunchConfiguration('navigation_autostart')
    startup_navigation_on_initial_pose = LaunchConfiguration('startup_navigation_on_initial_pose')
    use_composition = LaunchConfiguration('use_composition')
    use_respawn = LaunchConfiguration('use_respawn')
    use_rviz = LaunchConfiguration('use_rviz')
    rviz_config = LaunchConfiguration('rviz_config')
    log_level = LaunchConfiguration('log_level')
    configured_params = ReplaceString(
        source_file=params_file,
        replacements={
            '/home/boreas/robot_nav/src/nav2_minimal_bringup/behavior_trees': behavior_tree_dir,
            '/home/boreas/robot_nav/src/maps/my_map.yaml': default_map_yaml,
        },
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'localization_launch.py')
        ),
        launch_arguments={
            'map': map_yaml,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'params_file': configured_params,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'container_name': NAV2_CONTAINER_NAME,
            'log_level': log_level,
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': navigation_autostart,
            'params_file': configured_params,
            'use_composition': use_composition,
            'use_respawn': use_respawn,
            'container_name': NAV2_CONTAINER_NAME,
            'log_level': log_level,
        }.items(),
    )

    nav2_container = Node(
        condition=IfCondition(use_composition),
        package='rclcpp_components',
        executable='component_container_isolated',
        name=NAV2_CONTAINER_NAME,
        output='screen',
        parameters=[{'autostart': autostart, 'use_sim_time': use_sim_time}],
        arguments=['--ros-args', '--log-level', log_level],
        remappings=[
            ('/tf', 'tf'),
            ('/tf_static', 'tf_static'),
        ],
    )

    rviz_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'rviz_launch.py')
        ),
        launch_arguments={
            'use_namespace': 'false',
            'rviz_config': rviz_config,
        }.items(),
        condition=IfCondition(use_rviz),
    )

    nav_startup_helper = Node(
        package='mobile_robot_nav_bringup',
        executable='initial_pose_nav_startup.py',
        name='initial_pose_nav_startup',
        output='screen',
        condition=IfCondition(startup_navigation_on_initial_pose),
    )

    # ENU: right turn around +Z is negative yaw.
    laser_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        output='screen',
        arguments=[
            '--x', '0.295',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '-0.78539816339',
            '--frame-id', 'base_link',
            '--child-frame-id', 'laser_link',
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'map',
            default_value=_default_map_yaml(pkg_share),
            description='Full path to the map YAML file.',
        ),
        DeclareLaunchArgument(
            'params_file',
            default_value=os.path.join(pkg_share, 'config', 'nav2_params.yaml'),
            description='Full path to the Nav2 parameters file.',
        ),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation clock if true.',
        ),
        DeclareLaunchArgument(
            'autostart',
            default_value='true',
            description='Automatically transition localization lifecycle nodes.',
        ),
        DeclareLaunchArgument(
            'navigation_autostart',
            default_value='false',
            description='Automatically transition navigation lifecycle nodes.',
        ),
        DeclareLaunchArgument(
            'startup_navigation_on_initial_pose',
            default_value='true',
            description='Start navigation lifecycle after an initial pose is received.',
        ),
        DeclareLaunchArgument(
            'use_composition',
            default_value='True',
            description='Compose Nav2 nodes into a single container to reduce SBC CPU and memory overhead.',
        ),
        DeclareLaunchArgument(
            'use_respawn',
            default_value='False',
            description='Respawn Nav2 nodes if they crash.',
        ),
        DeclareLaunchArgument(
            'use_rviz',
            default_value='false',
            description='Start RViz together with Nav2. Keep false on SBCs unless actively debugging.',
        ),
        DeclareLaunchArgument(
            'rviz_config',
            default_value=os.path.join(nav2_bringup_share, 'rviz', 'nav2_default_view.rviz'),
            description='Full path to the RViz config file.',
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Logging level for Nav2 nodes.',
        ),
        nav2_container,
        localization_launch,
        navigation_launch,
        nav_startup_helper,
        rviz_launch,
        laser_static_tf,
    ])
