cmake_minimum_required(VERSION 3.10)
project(narval_oculus VERSION 0.1)

find_package(Boost COMPONENTS system thread REQUIRED)

list(APPEND narval_oculus
    include/narval_oculus/Oculus.h
    include/narval_oculus/print_utils.h
    include/narval_oculus/StatusListener.h
)

add_library(narval_oculus SHARED
    src/print_utils.cpp
    src/StatusListener.cpp
)
target_include_directories(narval_oculus PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(narval_oculus PUBLIC
    Boost::system
    Boost::thread
)

add_subdirectory(tests)


