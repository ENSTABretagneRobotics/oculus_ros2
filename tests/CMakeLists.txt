cmake_minimum_required(VERSION 3.10)
project(narval_oculus_tests VERSION 0.1)

list(APPEND test_names
    statusraw_test
    status_display
    client_test
)

list(APPEND test_deps
    narval_oculus
)

foreach(name ${test_names})
    add_executable(${name} src/${name}.cpp)
    target_link_libraries(${name} ${test_deps})
endforeach(name)



