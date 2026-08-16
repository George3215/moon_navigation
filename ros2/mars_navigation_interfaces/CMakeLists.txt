cmake_minimum_required(VERSION 3.8)
project(mars_navigation_interfaces)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(nav_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/GetGlobalMap.srv"
  "srv/InitializeMap.srv"
  "srv/InitializePath.srv"
  "srv/UpdateMap.srv"
  DEPENDENCIES nav_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
