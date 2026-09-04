cmake_minimum_required(VERSION 3.16)
project(loader_dynamics)

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(rclcpp REQUIRED)
find_package(loader_sim_msgs REQUIRED)

add_library(loader_dynamics_system SHARED src/loader_dynamics_system.cpp)
target_compile_features(loader_dynamics_system PUBLIC cxx_std_17)
target_compile_options(loader_dynamics_system PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(loader_dynamics_system gz-sim8::gz-sim8 gz-plugin2::register)
ament_target_dependencies(loader_dynamics_system rclcpp loader_sim_msgs)

install(TARGETS loader_dynamics_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
