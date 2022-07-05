cmake_minimum_required(VERSION 3.10)
project(oculus_sonar VERSION 0.1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)

if(NOT TARGET narval_oculus)
    find_package(narval_oculus REQUIRED)
endif()

# Generate Messages/Services
# find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/OculusHeader.msg"
  "msg/OculusVersionInfo.msg"
  "msg/OculusStatus.msg"
  "msg/OculusFireConfig.msg"
  "msg/OculusPing.msg"
  "msg/OculusStampedPing.msg"
  DEPENDENCIES std_msgs #builtin_interfaces
)

# replace with target include directories ?
include_directories(${rclcpp_INCLUDE_DIRS})
include_directories(${rclpy_INCLUDE_DIRS})
include_directories(${std_msgs_INCLUDE_DIRS})
include_directories(
  include
  ${ament_INCLUDE_DIRS}
)

add_executable(oculus_sonar_node
    src/oculus_sonar_node.cpp
)
# target_include_directories(oculus_sonar_node include)
target_link_libraries(oculus_sonar_node
    narval_oculus
)

# INSTALL
install(PROGRAMS scripts/bag_to_oculus
        DESTINATION bin)
install(DIRECTORY launch cfg DESTINATION share/${PROJECT_NAME})

# EXPORTS
ament_export_dependencies(rosidl_default_runtime)

# ament_target_dependencies(oculus_sonar_node
#   "rclcpp"
# )

rosidl_target_interfaces(oculus_sonar_node
  ${PROJECT_NAME} "rosidl_typesupport_cpp")

ament_package()

