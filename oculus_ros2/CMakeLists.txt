cmake_minimum_required(VERSION 3.16)
project(oculus_ros2 VERSION 1.0)
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -ggdb3)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(oculus_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# find_package(rqt_reconfigure REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV 4.5.4 REQUIRED)
find_package(oculus_driver REQUIRED)

add_executable(oculus_sonar_node
    src/oculus_sonar_node.cpp
    src/sonar_viewer.cpp
)
target_include_directories(oculus_sonar_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(oculus_sonar_node PRIVATE
    oculus_driver
)
ament_target_dependencies(oculus_sonar_node PUBLIC
    rclcpp
    oculus_interfaces
    rcl_interfaces
    sensor_msgs
    OpenCV
    cv_bridge
)

add_executable(oculus_viewer_node
    src/oculus_viewer_node.cpp
    src/sonar_viewer.cpp
)
target_include_directories(oculus_viewer_node PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(oculus_viewer_node PRIVATE
    oculus_driver
)
ament_target_dependencies(oculus_viewer_node PUBLIC
    rclcpp
    oculus_interfaces
    rcl_interfaces
    sensor_msgs
    OpenCV
    cv_bridge
)

install(PROGRAMS scripts/display_oculus_file.py scripts/oculus_to_rosbag.py DESTINATION bin)
install(DIRECTORY launch cfg DESTINATION share/${PROJECT_NAME})
install(PROGRAMS scripts/oculus_subscriber_to_image.py DESTINATION lib/${PROJECT_NAME})
install(TARGETS oculus_sonar_node oculus_viewer_node DESTINATION lib/${PROJECT_NAME})

ament_package()
