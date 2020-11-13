cmake_minimum_required(VERSION 3.10)
project(narval_oculus VERSION 0.1)

find_package(Boost COMPONENTS system thread REQUIRED)
message(STATUS "Boost version  : ${Boost_VERSION}")
message(STATUS "Boost location : ${Boost_INCLUDE_DIRS}")

list(APPEND narval_oculus
    include/narval_oculus/Oculus.h
    include/narval_oculus/utils.h
    include/narval_oculus/print_utils.h
    include/narval_oculus/CallbackQueue.h
    include/narval_oculus/StatusListener.h
    include/narval_oculus/Client.h
)

add_library(narval_oculus SHARED
    # src/utils.cpp
    src/print_utils.cpp
    src/StatusListener.cpp
    src/Client.cpp
)
target_include_directories(narval_oculus PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(narval_oculus PUBLIC
    Boost::system
    Boost::thread
)
target_compile_features(narval_oculus PUBLIC cxx_std_11)

add_subdirectory(tools)
add_subdirectory(tests)


