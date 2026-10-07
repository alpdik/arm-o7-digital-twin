cmake_minimum_required(VERSION 3.16)
project(arm_o7_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES SOURCE_ASSETS.md DESTINATION share/${PROJECT_NAME})

ament_package()
