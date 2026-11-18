cmake_minimum_required(VERSION 3.8)
project(delivery_robot_sensor_fusion_v12)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/check_inputs.py
  scripts/compare_ekf.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
