cmake_minimum_required(VERSION 3.16)
project(arm_o7_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS scripts/thumb_coupler.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
