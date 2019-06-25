cmake_minimum_required(VERSION 2.8.3)
project(mav_utils_msgs)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(CMAKE_BUILD_TYPE Release)

cs_install()
cs_export()
