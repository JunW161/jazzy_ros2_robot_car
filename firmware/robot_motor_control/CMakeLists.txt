cmake_minimum_required(VERSION 3.16)
# Project lives in micro_ros_espidf_component/examples/robot_motor_control
set(EXTRA_COMPONENT_DIRS "${CMAKE_CURRENT_LIST_DIR}/../..")
include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(Robot_Delivery_Main)
