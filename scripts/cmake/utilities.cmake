function(create_export_name TARGET_NAME EXPORT_NAME)
	set(${EXPORT_NAME} ${TARGET_NAME}d PARENT_SCOPE)	
endfunction()

function(install_filter)
    set(PREFIX INSTALL)
    set(OPTIONS)
    set(SINGLE_ARGS TARGET DESTINATION)
    set(MULTI_ARGS)

    cmake_parse_arguments("${PREFIX}" "${OPTIONS}" "${SINGLE_ARGS}" "${MULTI_ARGS}" "${ARGN}")

    if(NOT DEFINED INSTALL_TARGET)
        message(FATAL_ERROR "install_filter called without TARGET.")
    endif()

    set(INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/${INSTALL_DESTINATION})

    install(TARGETS ${INSTALL_TARGET} DESTINATION ${INSTALL_DESTINATION})

    if(WIN32)
        get_target_property(NAME ${INSTALL_TARGET} INSTALL_NAME)

        if(NOT INSTALL_NAME)
            set(INSTALL_NAME ${INSTALL_TARGET})
        endif()

        set(PDB_FULL_PATH ${CMAKE_CURRENT_BINARY_DIR}/${${INSTALL_TARGET}_BUILD_NAME_}/${INSTALL_NAME}.pdb)

        install(FILES ${PDB_FULL_PATH} DESTINATION ${INSTALL_DESTINATION})
    endif()
endfunction()

function(install_library)
    set(PREFIX INSTALL)
    set(OPTIONS)
    set(SINGLE_ARGS TARGET DESTINATION)
    set(MULTI_ARGS)

    cmake_parse_arguments("${PREFIX}" "${OPTIONS}" "${SINGLE_ARGS}" "${MULTI_ARGS}" "${ARGN}")

    if(NOT DEFINED INSTALL_TARGET)
        message(FATAL_ERROR "install_library called without TARGET.")
    endif()
	
    set(RUNTIME_DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/${INSTALL_DESTINATION})
    set(LIBRARY_DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${INSTALL_DESTINATION})
    set(ARCHIVE_DESTINATION ${CMAKE_INSTALL_PREFIX}/lib/${INSTALL_DESTINATION})
    set(INCLUDES_DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${INSTALL_DESTINATION})
	
	# install for release 
    install(TARGETS ${INSTALL_TARGET}
			#CONFIGURATIONS Release RelWithDebInfo
            RUNTIME DESTINATION ${RUNTIME_DESTINATION}
            LIBRARY DESTINATION ${LIBRARY_DESTINATION}
            ARCHIVE DESTINATION ${ARCHIVE_DESTINATION}
            INCLUDES DESTINATION ${INCLUDES_DESTINATION}
			
    )

endfunction()

function(install_executable)
    set(PREFIX INSTALL)
    set(OPTIONS)
    set(SINGLE_ARGS TARGET DESTINATION)
    set(MULTI_ARGS)

    cmake_parse_arguments("${PREFIX}" "${OPTIONS}" "${SINGLE_ARGS}" "${MULTI_ARGS}" "${ARGN}")

    if(NOT DEFINED INSTALL_TARGET)
        message(FATAL_ERROR "install_executable called without TARGET.")
    endif()
	

    set(INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/bin/${INSTALL_DESTINATION})

    install(TARGETS ${INSTALL_TARGET} 
			EXPORT ${INSTALL_EXPORT_NAME}
			DESTINATION ${INSTALL_DESTINATION} 
	)
endfunction()

function(install_directory)
    set(PREFIX INSTALL)
    set(OPTIONS)
    set(SINGLE_ARGS DIRECTORY DESTINATION)
    set(MULTI_ARGS)

    cmake_parse_arguments("${PREFIX}" "${OPTIONS}" "${SINGLE_ARGS}" "${MULTI_ARGS}" "${ARGN}")

    if(NOT DEFINED INSTALL_DIRECTORY)
        message(FATAL_ERROR "install_directory called without DIRECTORY.")
    endif()

    if(NOT DEFINED INSTALL_DESTINATION)
        message(FATAL_ERROR "install_directory called without DESTINATION.")
    endif()

    if(NOT ${INSTALL_DIRECTORY} MATCHES "/$")
        set(INSTALL_DIRECTORY ${INSTALL_DIRECTORY}/)
    endif()

    install(DIRECTORY ${INSTALL_DIRECTORY} DESTINATION ${INSTALL_DESTINATION})
endfunction()

function(install_files)
    set(PREFIX INSTALL)
    set(OPTIONS)
    set(SINGLE_ARGS DESTINATION)
    set(MULTI_ARGS FILES CONFIGURATIONS)

    cmake_parse_arguments("${PREFIX}" "${OPTIONS}" "${SINGLE_ARGS}" "${MULTI_ARGS}" "${ARGN}")

    if(NOT DEFINED INSTALL_FILES)
        message(FATAL_ERROR "install_files called without FILES.")
    endif()

    if(NOT DEFINED INSTALL_DESTINATION)
        message(FATAL_ERROR "install_files called without DESTINATION.")
    endif()
	
	if(NOT DEFINED INSTALL_CONFIGURATIONS)
		message(FATAL_ERROR "install_files called without CONFIGURATIONS.")
    endif()
    install(FILES ${INSTALL_FILES}
			DESTINATION ${INSTALL_DESTINATION}
			CONFIGURATIONS ${INSTALL_CONFIGURATIONS}
			)
endfunction()

function(install_include)
    set(PREFIX INSTALL)
    set(OPTIONS)
    set(SINGLE_ARGS FILES DIRECTORY DESTINATION)
    set(MULTI_ARGS)

    cmake_parse_arguments("${PREFIX}" "${OPTIONS}" "${SINGLE_ARGS}" "${MULTI_ARGS}" "${ARGN}")

    if(NOT DEFINED INSTALL_FILES AND NOT DEFINED INSTALL_DIRECTORY)
        message(FATAL_ERROR "install_include called without FILES/DIRECTORY")
    endif()

    if(NOT ${INSTALL_DIRECTORY} MATCHES "/$")
        set(INSTALL_DIRECTORY ${INSTALL_DIRECTORY}/)
    endif()

    if(NOT ${INSTALL_DESTINATION} STREQUAL "")
        set(INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/include/${INSTALL_DESTINATION})
    else()
        set(INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX}/include)
    endif()

    if(DEFINED INSTALL_FILES)
        install_files(FILES ${INSTALL_FILES} DESTINATION ${INSTALL_DESTINATION})
    endif()

    if(DEFINED INSTALL_DIRECTORY)
        install_directory(DIRECTORY ${INSTALL_DIRECTORY} DESTINATION ${INSTALL_DESTINATION})
    endif()
endfunction()




macro(include_guard_iav)
	get_filename_component(NAME_GIT_SUBMODUL ${CMAKE_CURRENT_LIST_DIR} NAME)
	get_property(_INCLUDE_GUARD_LOCAL GLOBAL PROPERTY _INCLUDE_GUARD_${NAME_GIT_SUBMODUL})
	if(DEFINED _INCLUDE_GUARD_LOCAL)
		return()
	endif()
	set_property(GLOBAL PROPERTY _INCLUDE_GUARD_${NAME_GIT_SUBMODUL} TRUE)
endmacro()

macro(add_subdirectory_guarded LIB_TO_GUARD)
	get_filename_component(NAME_GIT_SUBMODUL ${CMAKE_CURRENT_LIST_DIR}/${LIB_TO_GUARD} NAME)
	get_property(_INCLUDE_GUARD_LOCAL GLOBAL PROPERTY _INCLUDE_GUARD_${NAME_GIT_SUBMODUL})
	if(NOT DEFINED _INCLUDE_GUARD_LOCAL)
		add_subdirectory(${LIB_TO_GUARD})
	endif()
endmacro()