set(TargetNameExe ${CMAKE_PROJECT_NAME}_filter_ekf_example)

add_executable(${TargetNameExe} ex_filter_ekf.cpp )

find_package(Python2 COMPONENTS Development)

target_include_directories(${TargetNameExe} PRIVATE ${Python2_INCLUDE_DIRS})
target_include_directories(${TargetNameExe} PUBLIC ${CMAKE_SOURCE_DIR}/3rdParty/matplotlib)
target_compile_definitions(${TargetNameExe} PUBLIC USE_MATPLOTLIB WITHOUT_NUMPY)

target_link_libraries(${TargetNameExe} PUBLIC
    Python2::Python
    estimator)
