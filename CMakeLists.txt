project(ExtendedKF)

cmake_minimum_required (VERSION 3.5)

add_definitions(-std=c++0x)

set(sources
    src/kalman_filter.cpp
    src/measurement_package.cpp
    src/laser_measurement_package.cpp
    src/radar_measurement_package.cpp
    src/main.cpp
    src/tools.cpp)

add_executable(ExtendedKF ${sources})
