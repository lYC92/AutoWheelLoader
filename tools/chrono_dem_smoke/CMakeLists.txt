cmake_minimum_required(VERSION 3.22)
project(loader_chrono_dem_smoke LANGUAGES CXX)

find_package(Chrono REQUIRED COMPONENTS DEM CONFIG)

add_executable(loader_chrono_dem_smoke main.cpp)
target_compile_features(loader_chrono_dem_smoke PRIVATE cxx_std_17)
target_link_libraries(loader_chrono_dem_smoke PRIVATE ${CHRONO_TARGETS})

