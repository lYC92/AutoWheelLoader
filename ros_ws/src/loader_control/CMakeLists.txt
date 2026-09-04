cmake_minimum_required(VERSION 3.16)
project(loader_control)

find_package(ament_cmake REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(loader_sim_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(realtime_tools REQUIRED)

add_library(loader_command_controller SHARED src/loader_command_controller.cpp)
target_compile_features(loader_command_controller PUBLIC cxx_std_17)
target_compile_options(loader_command_controller PRIVATE -Wall -Wextra -Wpedantic)
ament_target_dependencies(
  loader_command_controller
  controller_interface
  hardware_interface
  loader_sim_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
  realtime_tools
)

pluginlib_export_plugin_description_file(controller_interface loader_control_plugins.xml)

install(TARGETS loader_command_controller
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(FILES loader_control_plugins.xml DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})

ament_package()
