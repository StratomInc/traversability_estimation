cmake_minimum_required(VERSION 3.5)
project(traversability_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/FootprintPath.msg"
  "msg/TraversabilityResult.msg"
  "srv/CheckFootprintPath.srv"
  "srv/Overwrite.srv"
  DEPENDENCIES geometry_msgs builtin_interfaces
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
