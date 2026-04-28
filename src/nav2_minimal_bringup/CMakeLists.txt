cmake_minimum_required(VERSION 3.8)
project(mobile_robot_nav_bringup)

find_package(ament_cmake REQUIRED)

install(
  PROGRAMS
    scripts/keyboard_cmd_vel_test.py
    scripts/initial_pose_nav_startup.py
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch config maps behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES README.md
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
