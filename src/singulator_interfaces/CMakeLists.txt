cmake_minimum_required(VERSION 3.8)
project(singulator_interfaces)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/BoxObservation.msg"
  "msg/BoxObservationArray.msg"
  "msg/MatrixCommand.msg"
  "msg/MatrixState.msg"
  "srv/ResetScenario.srv"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
