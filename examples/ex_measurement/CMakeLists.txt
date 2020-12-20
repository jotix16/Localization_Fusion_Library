set(TargetNameExe ${CMAKE_PROJECT_NAME}_measurement_example)

add_executable(${TargetNameExe} ex_measurement.cpp )

target_link_libraries(${TargetNameExe} PUBLIC estimator)