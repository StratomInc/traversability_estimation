cmake_minimum_required(VERSION 3.5)
project(traversability_estimation)

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
find_package(grid_map_ros REQUIRED)
find_package(grid_map_core REQUIRED)
find_package(grid_map_msgs REQUIRED)
find_package(grid_map_filters REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(traversability_estimation_filters REQUIRED)
find_package(traversability_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rclcpp_components REQUIRED)

## System dependencies are found with CMake's conventions
find_package(Eigen3 REQUIRED)
find_package(kindr QUIET)
find_package(PkgConfig REQUIRED)
if(NOT kindr_FOUND)
  # Attempt to find package-based kindr
  pkg_check_modules(kindr kindr REQUIRED)
endif()

###########
## Build ##
###########
set(dependencies
  rclcpp
  tf2
  tf2_ros
  tf2_geometry_msgs
  traversability_estimation_filters
  traversability_msgs
  std_msgs
  std_srvs
  example_interfaces
  geometry_msgs
  sensor_msgs
  grid_map_ros
  grid_map_core
  grid_map_msgs
  grid_map_filters
  nav2_costmap_2d
  rclcpp_components
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
  include
  ${Eigen_INCLUDE_DIRS}
  ${kindr_INCLUDE_DIRS}
)

## Declare a cpp library
add_library(${PROJECT_NAME}_map SHARED
  src/TraversabilityMap.cpp
)
ament_target_dependencies(${PROJECT_NAME}_map
  ${dependencies}
)
add_library(${PROJECT_NAME}_library SHARED
  src/TraversabilityEstimation.cpp
)
ament_target_dependencies(${PROJECT_NAME}_library
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME}_library
  ${PROJECT_NAME}_map
)
rclcpp_components_register_nodes(${PROJECT_NAME}_library "traversability_estimation::TraversabilityEstimation")

## Declare a cpp executable
add_executable(${PROJECT_NAME}
  src/traversability_estimation_node.cpp
)

## Specify libraries to link a library or executable target against
ament_target_dependencies(${PROJECT_NAME}
  ${dependencies}
)
target_link_libraries(${PROJECT_NAME}
  ${PROJECT_NAME}_library
)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executables and/or libraries for installation
install(TARGETS ${PROJECT_NAME}_map ${PROJECT_NAME}_library ${PROJECT_NAME}
  ARCHIVE DESTINATION lib 
  LIBRARY DESTINATION lib 
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

## Mark cpp header files for installation
install(
  DIRECTORY include/${PROJECT_NAME}
  DESTINATION include
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY config maps #launch
    DESTINATION share/${PROJECT_NAME}
)

ament_export_dependencies(${AMENT_PACKAGE_DEPENDENCIES})
ament_export_include_directories(include ${Eigen_INCLUDE_DIRS})
ament_export_libraries(${PROJECT_NAME}_library)

ament_package()
