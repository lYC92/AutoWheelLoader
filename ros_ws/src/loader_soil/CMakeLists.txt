cmake_minimum_required(VERSION 3.16)
project(loader_soil)

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(loader_sim_msgs REQUIRED)
find_package(rclcpp REQUIRED)

add_library(loader_soil_slice_system SHARED src/loader_soil_slice_system.cpp)
target_compile_features(loader_soil_slice_system PUBLIC cxx_std_17)
target_compile_options(loader_soil_slice_system PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(loader_soil_slice_system gz-sim8::gz-sim8 gz-plugin2::register)
ament_target_dependencies(loader_soil_slice_system loader_sim_msgs rclcpp)

install(TARGETS loader_soil_slice_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_package()
