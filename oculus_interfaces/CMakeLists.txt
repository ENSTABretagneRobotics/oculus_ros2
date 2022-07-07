cmake_minimum_required(VERSION 3.10)
project(oculus_interfaces VERSION 0.1)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

if(NOT TARGET oculus_driver)
    find_package(oculus_driver REQUIRED)
endif()

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
  DEPENDENCIES builtin_interfaces std_msgs
)

# EXPORTS
ament_export_dependencies(rosidl_default_runtime)

ament_package()