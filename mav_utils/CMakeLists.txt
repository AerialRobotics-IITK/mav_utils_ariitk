cmake_minimum_required(VERSION 2.8.3)
project(mav_utils)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

add_definitions(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

find_package(GeographicLib REQUIRED)
include_directories(${GeographicLib_INCLUDE_DIRS})

#############
# LIBRARIES #
#############
cs_add_library(${PROJECT_NAME}
  src/state_estimation.cpp
)
target_link_libraries(${PROJECT_NAME} ${GeographicLib_LIBRARIES})

############
# BINARIES #
############
cs_add_executable(lander src/land.cpp)
cs_add_executable(state_estimation_node src/state_estimation_node.cpp)
target_link_libraries(state_estimation_node  ${PROJECT_NAME})
cs_add_executable(controller_switch_node src/controller_switch_node.cpp)
cs_add_executable(geoid_test src/geoid_test.cpp)
target_link_libraries(geoid_test ${GeographicLib_LIBRARIES})

##########
# EXPORT #
##########
cs_install()

cs_install_scripts(scripts/odmtag.py)
cs_install_scripts(scripts/piexif.py)

cs_export()
