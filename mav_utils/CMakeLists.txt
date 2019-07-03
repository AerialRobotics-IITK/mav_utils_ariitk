cmake_minimum_required(VERSION 2.8.3)
project(mav_utils)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

cs_add_executable(lander src/land.cpp)
cs_add_executable(state_estimation_node src/state_estimation_node.cpp)
cs_add_executable(controller_switch_node src/controller_switch_node.cpp)

cs_install()
cs_export()