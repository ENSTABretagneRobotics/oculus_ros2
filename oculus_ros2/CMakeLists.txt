cmake_minimum_required(VERSION 3.16)
project(oculus_ros2 VERSION 1.0)

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
find_package(OpenCV REQUIRED)

find_package(oculus_driver QUIET)

if(NOT TARGET oculus_driver)
    include(FetchContent)
    FetchContent_Declare(oculus_driver
        GIT_REPOSITORY https://github.com/ENSTABretagneRobotics/oculus_driver.git
        GIT_TAG master # tag for development : TODO handle the good version
    )
    FetchContent_MakeAvailable(oculus_driver)
endif()


add_executable(oculus_sonar_node
    src/oculus_sonar_node.cpp
    src/sonar_viewer.cpp
)
add_executable(oculus_viewer_node
    src/oculus_viewer_node.cpp
    src/sonar_viewer.cpp
)

# target_include_directories(oculus_sonar_node PRIVATE
#   ${CMAKE_CURRENT_SOURCE_DIR}/include
#   ${OpenCV_INCLUDE_DIRS})
include_directories( ${OpenCV_INCLUDE_DIRS} )

target_link_libraries(oculus_sonar_node PRIVATE
  ${OpenCV_LIBRARIES}
  ${ament_LIBRARIES}
    oculus_driver
)
target_link_libraries(oculus_viewer_node PRIVATE
  ${OpenCV_LIBRARIES}
  ${ament_LIBRARIES}
    oculus_driver
)
target_compile_features(oculus_sonar_node PRIVATE cxx_std_17)
target_compile_features(oculus_viewer_node PRIVATE cxx_std_17)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(oculus_sonar_node PRIVATE -Wall -Wextra -Wpedantic)
    target_compile_options(oculus_viewer_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

ament_target_dependencies(oculus_sonar_node PUBLIC
    rclcpp
    oculus_interfaces
    rcl_interfaces
    sensor_msgs
    OpenCV
    cv_bridge
)
ament_target_dependencies(oculus_viewer_node PUBLIC
    rclcpp
    oculus_interfaces
    rcl_interfaces
    sensor_msgs
    OpenCV
    cv_bridge
)


install(PROGRAMS
    scripts/oculus_subscriber_to_image.py
    DESTINATION  lib/${PROJECT_NAME}
)

install(PROGRAMS
    scripts/display_oculus_file.py
    RENAME display_oculus_file
    DESTINATION bin
)
install(PROGRAMS
    scripts/oculus_to_rosbag.py
    RENAME oculus_to_rosbag
    DESTINATION bin
)

install(DIRECTORY
    launch
    cfg
    DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
    oculus_sonar_node
    oculus_viewer_node
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
