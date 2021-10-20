cmake_minimum_required(VERSION 3.5)
project(traversability_estimation_filters)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(CMAKE_BUILD_TYPE RelWithDebInfo)

## Find catkin macros and libraries
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_ros REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(filters REQUIRED)
find_package(pluginlib REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########
set(dependencies
  rclcpp
  grid_map_core
  grid_map_ros
  grid_map_msgs
  filters
  pluginlib
)

## Specify additional locations of header files
include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME} SHARED
   src/SlopeFilter.cpp
   src/StepFilter.cpp
   src/RoughnessFilter.cpp
)
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)

install(FILES filter_plugins.xml
  DESTINATION share/${PROJECT_NAME}
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib 
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Mark cpp header files for installation
install(DIRECTORY include/filters/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_dependencies(ament_cmake)
pluginlib_export_plugin_description_file(filters filter_plugins.xml)

ament_package()
