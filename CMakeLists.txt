cmake_minimum_required(VERSION 3.10)
project(narval_oculus VERSION 0.1)

find_package(Boost COMPONENTS system thread REQUIRED)

list(APPEND narval_oculus
    include/narval_oculus/Oculus.h
)

add_library(narval_oculus INTERFACE)
target_include_directories(narval_oculus INTERFACE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(narval_oculus INTERFACE
    Boost::system
    Boost::thread
)

add_subdirectory(tests)


