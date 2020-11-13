cmake_minimum_required(VERSION 3.10)
project(narval_oculus_tools VERSION 0.1)

if(NOT TARGET narval_oculus)
    find_package(narval_oculus)
endif()

# attempting to find narval_display
if(NOT TARGET narval_display)
    find_package(narval_display)
endif()

# if found, build oculus_display
if(TARGET narval_display)

add_executable(oculus_display
    src/oculus_display.cpp
)
target_link_libraries(oculus_display PRIVATE
    narval_display
    narval_oculus
)

else()
    message(WARNING "Could not find narval_display package. oculus_display will not be built")
endif()

