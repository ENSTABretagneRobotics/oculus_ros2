cmake_minimum_required(VERSION 3.10)
project(narval_oculus_tests VERSION 0.1)

list(APPEND test_names
    statusraw_test
    status_display
    client_test
    async_client_test
    config_test
)

list(APPEND test_deps
    narval_oculus
)

# if(NOT TARGET narval_base)
#     find_package(narval_base QUIET)
# endif()
# if(TARGET narval_base)
#     list(APPEND test_names oculusfile_test)
#     list(APPEND test_deps narval_base)
# endif()


foreach(name ${test_names})
    add_executable(${name} src/${name}.cpp)
    target_link_libraries(${name} ${test_deps})
    target_compile_features(${name} PRIVATE cxx_std_14)
endforeach(name)



