set(TargetNameExe ${CMAKE_PROJECT_NAME}_filter_ekf_example)

add_executable(${TargetNameExe} ex_filter_ekf.cpp )

target_link_libraries(${TargetNameExe} PUBLIC estimator)