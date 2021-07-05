cmake_minimum_required(VERSION 3.10)
project(oculus_sonar VERSION 0.1)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    message_generation
    dynamic_reconfigure
)

generate_dynamic_reconfigure_options(${CMAKE_CURRENT_SOURCE_DIR}/cfg/OculusSonar.cfg)

if(NOT TARGET narval_oculus)
    find_package(narval_oculus REQUIRED)
endif()

add_message_files(FILES
   OculusHeader.msg
   OculusVersionInfo.msg
   OculusStatus.msg
   OculusFireConfig.msg
   OculusPing.msg
)

# add_service_files(FILES
#     AddTwoInts.srv
# )

## Generate added messages and services with any dependencies listed here
generate_messages(DEPENDENCIES
    std_msgs
)

catkin_package(CATKIN_DEPENDS roscpp rospy std_msgs)

# replace with target include directories ?
include_directories(${catkin_INCLUDE_DIRS})

add_executable(oculus_sonar_node
    src/oculus_sonar_node.cpp
)
target_include_directories(oculus_sonar_node PRIVATE src)
target_link_libraries(oculus_sonar_node PRIVATE
    ${catkin_LIBRARIES}
    narval_oculus
)
add_dependencies(oculus_sonar_node oculus_sonar_generate_messages_cpp ${PROJECT_NAME}_gencfg)

# find_package(narval_display REQUIRED)
# add_executable(oculus_sonar_display_node
#     src/oculus_sonar_display_node.cpp
# )
# target_include_directories(oculus_sonar_display_node PRIVATE src)
# target_link_libraries(oculus_sonar_display_node PRIVATE
#     ${catkin_LIBRARIES}
#     narval_oculus
#     narval_display
# )
# add_dependencies(oculus_sonar_display_node oculus_sonar_generate_messages_cpp ${PROJECT_NAME}_gencfg)

#############
## Install ##
#############

# install(TARGETS talker_ext
#     RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION} 
# )
# 
# install(TARGETS talker_ext
#     ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#     RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
# )

# ## Mark cpp header files for installation
# install(DIRECTORY include/beginner_tutorials_ext
#     DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#     FILES_MATCHING PATTERN "*.h"
#     PATTERN ".svn" EXCLUDE
# )
