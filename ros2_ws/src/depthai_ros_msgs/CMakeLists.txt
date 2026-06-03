cmake_minimum_required(VERSION 3.8)
project(depthai_ros_msgs)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/SpatialDetection.msg"
  "msg/SpatialDetectionArray.msg"
  DEPENDENCIES std_msgs geometry_msgs vision_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
