cmake_minimum_required(VERSION 3.16)
project(loader_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY config launch urdf meshes
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
