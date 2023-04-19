cmake_minimum_required(VERSION 3.16)
project(oculus_ros2 VERSION 1.0)



# find dependencies
find_package(ament_cmake REQUIRED)
find_package(oculus_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(std_msgs REQUIRED)
# find_package(rqt_reconfigure REQUIRED)
find_package(rcl_interfaces REQUIRED)

# find_package(message_generation REQUIRED)
# find_package(dynamic_reconfigure REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set (CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

find_package(oculus_driver QUIET)

if(NOT TARGET oculus_driver)
    include(FetchContent)
    FetchContent_Declare(oculus_driver
        GIT_REPOSITORY https://github.com/ENSTABretagneRobotics/oculus_driver.git
        GIT_TAG master # tag for developpement : TODO handle the good version
    )
    FetchContent_MakeAvailable(oculus_driver)
endif()

add_executable(oculus_sonar_node
    src/oculus_sonar_node.cpp
)
target_link_libraries(oculus_sonar_node PRIVATE
    ${ament_LIBRARIES}
    oculus_driver
)
target_compile_features(oculus_sonar_node PRIVATE cxx_std_20)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    target_compile_options(oculus_sonar_node PRIVATE -Wall -Wextra -Wpedantic)
endif()

ament_target_dependencies(oculus_sonar_node PUBLIC
    rclcpp
    oculus_interfaces
    rcl_interfaces
)

#include_directories(include/ ${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})
include_directories(${catkin_INCLUDE_DIRS} ${OpenCV_INCLUDE_DIRS})


find_package(OpenCV REQUIRED)
include_directories( ${OpenCV_INCLUDE_DIRS} )
add_executable(oculus_viewer_node
    src/oculus_viewer_node.cpp
)
target_include_directories(oculus_viewer_node PRIVATE src)
target_link_libraries(oculus_viewer_node PRIVATE
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    #narval_display
)
add_dependencies(oculus_viewer_node
    oculus_sonar_generate_messages_cpp
    # ${PROJECT_NAME}_gencfg
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
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
