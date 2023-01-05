cmake_minimum_required(VERSION 3.10)
project(oculus_interfaces VERSION 0.1)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

# Generate Messages/Services
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OculusHeader.msg"
  "msg/OculusVersionInfo.msg"
  "msg/OculusStatus.msg"
  "msg/OculusFireConfig.msg"
  "msg/OculusPing.msg"
  "msg/OculusStampedPing.msg"
  # "msg/Ping.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

# EXPORTS
ament_export_dependencies(rosidl_default_runtime)

ament_package()
