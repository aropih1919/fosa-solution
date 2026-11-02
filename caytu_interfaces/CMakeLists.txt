cmake_minimum_required(VERSION 3.8)
project(caytu_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/DetectedEntity.msg"
  "msg/DetectedEntityArray.msg"
  "msg/DialogueState.msg"
  "msg/SemanticLocation.msg"
  "msg/SemanticLocationArray.msg"
  "msg/TagDetection.msg"
  "msg/TagDetectionArray.msg"
  "srv/GetLocationByTag.srv"
  "srv/ResolveLocation.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
