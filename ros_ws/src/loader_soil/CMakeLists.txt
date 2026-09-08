cmake_minimum_required(VERSION 3.16)
project(loader_soil)

# Polygon clipping runs in the physics loop; unoptimized builds can stall the
# control feedback stream. Explicit Debug builds remain available for debugging.
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim8 REQUIRED)
find_package(gz_plugin_vendor REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(loader_sim_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)

add_library(loader_soil_slice_system SHARED src/loader_soil_slice_system.cpp)
target_compile_features(loader_soil_slice_system PUBLIC cxx_std_17)
target_compile_options(loader_soil_slice_system PRIVATE -Wall -Wextra -Wpedantic)
target_link_libraries(loader_soil_slice_system gz-sim8::gz-sim8 gz-plugin2::register)
ament_target_dependencies(loader_soil_slice_system loader_sim_msgs rclcpp std_srvs)

install(TARGETS loader_soil_slice_system
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

if(BUILD_TESTING)
  enable_testing()
  add_library(loader_payload_probe SHARED src/payload_probe.cpp)
  target_compile_features(loader_payload_probe PRIVATE cxx_std_17)
  target_link_libraries(loader_payload_probe gz-sim8::gz-sim8 gz-plugin2::register)
  install(TARGETS loader_payload_probe LIBRARY DESTINATION lib)
  add_executable(test_heightfield ../../../tools/soil_heightfield_3d/test_heightfield.cpp)
  target_compile_features(test_heightfield PRIVATE cxx_std_17)
  target_compile_options(test_heightfield PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME oriented_heightfield COMMAND test_heightfield)
  add_executable(test_sloped_blade ../../../tools/soil_heightfield_3d/test_sloped_blade.cpp)
  target_compile_features(test_sloped_blade PRIVATE cxx_std_17)
  add_test(NAME sloped_blade COMMAND test_sloped_blade)
  add_executable(test_deposition_support ../../../tools/soil_heightfield_3d/test_deposition_support.cpp)
  target_compile_features(test_deposition_support PRIVATE cxx_std_17)
  add_test(NAME deposition_support COMMAND test_deposition_support)
  add_executable(test_heightfield_robustness ../../../tools/soil_heightfield_3d/test_heightfield_robustness.cpp)
  target_compile_features(test_heightfield_robustness PRIVATE cxx_std_17)
  target_compile_options(test_heightfield_robustness PRIVATE -Wall -Wextra -Wpedantic)
  add_test(NAME heightfield_robustness COMMAND test_heightfield_robustness)
  set_tests_properties(heightfield_robustness PROPERTIES TIMEOUT 120)
endif()

ament_package()
