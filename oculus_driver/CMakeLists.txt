cmake_minimum_required(VERSION 3.5)
project(oculus_driver)

add_subdirectory(oculus_driver)

add_subdirectory(oculus_driver/python)
# Run the pip3 command to install the package
execute_process(
    COMMAND pip3 install --user -e ${CMAKE_SOURCE_DIR}/oculus_driver/python
    RESULT_VARIABLE PIP_INSTALL_RESULT
)
# Check if the installation was successful
if(NOT PIP_INSTALL_RESULT EQUAL "0")
    message(FATAL_ERROR "Failed to install oculus_driver_python package with pip3")
endif()

find_package(ament_cmake REQUIRED) # TODO(abourgois, needed? Can't it be remove?)
ament_package() # TODO(abourgois, needed? Can't it be remove?)
