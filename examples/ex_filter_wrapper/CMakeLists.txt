set(TargetNameExe ${CMAKE_PROJECT_NAME}_filter_wrapper_example)

add_executable(${TargetNameExe} ex_filter_wrapper.cpp )

target_link_libraries(${TargetNameExe} PUBLIC estimator)