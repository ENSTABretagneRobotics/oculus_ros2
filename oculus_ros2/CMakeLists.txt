cmake_minimum_required(VERSION 3.10)
project(oculus_ros2 VERSION 0.1)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(oculus_interfaces REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(Boost 1.71.0 COMPONENTS system thread REQUIRED)

if(NOT TARGET oculus_driver)
    find_package(oculus_driver REQUIRED)
endif()

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
# target_link_libraries(oculus_sonar_node
#     oculus_driver
# )

ament_target_dependencies(oculus_sonar_node
  rclcpp
  oculus_driver
  oculus_interfaces
  rcl_interfaces
)

# INSTALL
install(PROGRAMS scripts/bag_to_oculus
        DESTINATION bin)
install(DIRECTORY launch cfg DESTINATION share/${PROJECT_NAME})

install(TARGETS 
  oculus_sonar_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()

