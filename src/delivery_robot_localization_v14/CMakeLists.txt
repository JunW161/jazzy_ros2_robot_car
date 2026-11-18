cmake_minimum_required(VERSION 3.8)
project(delivery_robot_localization_v14)
find_package(ament_cmake REQUIRED)
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/set_initial_pose.py
  scripts/check_official_odom.py
  scripts/check_amcl.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
