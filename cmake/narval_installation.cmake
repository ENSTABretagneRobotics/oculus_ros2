function(narval_install_target TARGET_NAME)

	set(multiValueArgs
        ADDITIONAL_CONFIG_FILES
        ADDITIONAL_CONFIG_COMMANDS)
	cmake_parse_arguments(NARVAL_INSTALLATION "${options}" "${oneValueArgs}"
	                      "${multiValueArgs}" ${ARGN} )
    
    # message(STATUS "=== ${NARVAL_INSTALLATION_ADDITIONAL_CONFIG_COMMANDS}")
    set(NARVAL_INSTALLATION_ADDITIONAL_CONFIG_BODY "")
    foreach(command ${NARVAL_INSTALLATION_ADDITIONAL_CONFIG_COMMANDS})
        # message(STATUS "  === ${command}")
        set(NARVAL_INSTALLATION_ADDITIONAL_CONFIG_BODY 
            "${NARVAL_INSTALLATION_ADDITIONAL_CONFIG_BODY}${command}\n")
    endforeach()

    # RPATH related configuration (see https://gitlab.kitware.com/cmake/community/-/wikis/doc/cmake/RPATH-handling for details)
    set_target_properties(${TARGET_NAME} PROPERTIES
        SKIP_BUILD_RPATH FALSE
        BUILD_WITH_INSTALL_RPATH FALSE
        INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib"
        INSTALL_RPATH_USE_LINK_PATH TRUE
    )
    list(FIND CMAKE_PLATFORM_IMPLICIT_LINK_DIRECTORIES "${CMAKE_INSTALL_PREFIX}/lib" isSystemDir)
    if("${isSystemDir}" STREQUAL "-1")
        set_target_properties(${TARGET_NAME} PROPERTIES
            INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib" # redundant with above ??
        )
    endif()

	include(GNUInstallDirs)
	# Configuration
	set(VERSION_CONFIG "${CMAKE_CURRENT_BINARY_DIR}/generated/${TARGET_NAME}ConfigVersion.cmake")
	set(PROJECT_CONFIG "${CMAKE_CURRENT_BINARY_DIR}/generated/${TARGET_NAME}Config.cmake")
	set(TARGET_EXPORT_NAME "${TARGET_NAME}Targets")
	set(CONFIG_INSTALL_DIR "${CMAKE_INSTALL_LIBDIR}/cmake/${TARGET_NAME}")
	
	include(CMakePackageConfigHelpers)
	write_basic_package_version_file(
	    "${VERSION_CONFIG}" COMPATIBILITY SameMajorVersion
	)
	configure_package_config_file(
	    "cmake/Config.cmake.in"
	    "${PROJECT_CONFIG}"
	    INSTALL_DESTINATION "${CONFIG_INSTALL_DIR}"
	    PATH_VARS CMAKE_INSTALL_INCLUDEDIR
	)
    # get_target_property(HEADERS ${TARGET_NAME} PUBLIC_HEADER)
    # message(STATUS "Public headers : ${HEADERS}")

	
	# Installation
	install(
	    TARGETS "${TARGET_NAME}"
	    EXPORT "${TARGET_EXPORT_NAME}"
	    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}"
	    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}"
	    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}"
	    INCLUDES DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}"
	    # PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/${INSTALLATION_HEADER_PREFIX}"
	)
    list(APPEND CONFIG_FILES_TO_INSTALL
        ${PROJECT_CONFIG}
        ${VERSION_CONFIG}
        ${NARVAL_INSTALLATION_ADDITIONAL_CONFIG_FILES}
    )
	install(
	    FILES ${CONFIG_FILES_TO_INSTALL}
	    DESTINATION "${CONFIG_INSTALL_DIR}"
	)

    # Installing header files
    get_target_property(TARGET_TYPE_VALUE ${TARGET_NAME} TYPE)
    if(${TARGET_TYPE_VALUE} STREQUAL "INTERFACE_LIBRARY")
        get_target_property(HEADER_FILES ${TARGET_NAME} INTERFACE_PUBLIC_HEADER)
    else()
        get_target_property(HEADER_FILES ${TARGET_NAME} PUBLIC_HEADER)
    endif()
    if(NOT "${HEADER_FILES}" STREQUAL "HEADER_FILES-NOTFOUND")
        # message(STATUS "HEADER_FILES : ${HEADER_FILES}")
        foreach(header ${HEADER_FILES})
            get_filename_component(header_dir ${header} DIRECTORY)
            # message(STATUS "HEADER INSTALL : ${CMAKE_INSTALL_INCLUDEDIR}/../${header_dir}")
            install(FILES ${header}
                    DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/../${header_dir}")
        endforeach()
    endif()

    # Export ting target ?
	install(
	    EXPORT "${TARGET_EXPORT_NAME}"
	    # NAMESPACE "${PROJECT_NAME}::"
	    DESTINATION "${CONFIG_INSTALL_DIR}"
	)

endfunction()

