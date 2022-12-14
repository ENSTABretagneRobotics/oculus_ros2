cmake_minimum_required(VERSION 3.16)
project(oculus_ros2 VERSION 1.0)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(oculus_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rcl_interfaces REQUIRED)

find_package(oculus_driver QUIET)
if(NOT TARGET oculus_driver)
    include(FetchContent)
    FetchContent_Declare(oculus_driver
        GIT_REPOSITORY   https://github.com/ENSTABretagneRobotics/oculus_driver.git
        GIT_TAG          v1.0
    )
    FetchContent_MakeAvailable(oculus_driver)
endif()

add_executable(oculus_sonar_node
    src/oculus_sonar_node.cpp
)
target_link_libraries(oculus_sonar_node PUBLIC
    ${ament_LIBRARIES}
    oculus_driver
)
target_compile_features(oculus_sonar_node PRIVATE cxx_std_17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(oculus_sonar_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

ament_target_dependencies(oculus_sonar_node PUBLIC
  rclcpp
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

