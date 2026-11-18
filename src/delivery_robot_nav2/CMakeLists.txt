cmake_minimum_required(VERSION 3.8)
project(delivery_robot_nav2)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/test_mission_safe.py
  scripts/test_planner.py
  scripts/check_foundation.py
  scripts/check_nav2_stack.py
  scripts/test_controller_safe.py
  scripts/test_navigation_safe.py
  scripts/test_navigation_real_short.py
  scripts/stop_robot.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
