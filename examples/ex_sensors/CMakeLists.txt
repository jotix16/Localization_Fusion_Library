set(TargetNameExe1 ${CMAKE_PROJECT_NAME}_sensors_odom_example)
set(TargetNameExe2 ${CMAKE_PROJECT_NAME}_sensors_imu_example)
set(TargetNameExe3 ${CMAKE_PROJECT_NAME}_sensors_gps_example)

add_executable(${TargetNameExe1} ex_sensors_odom.cpp )
add_executable(${TargetNameExe2} ex_sensors_imu.cpp )
add_executable(${TargetNameExe3} ex_sensors_gps.cpp )

target_link_libraries(${TargetNameExe1} PUBLIC estimator)
target_link_libraries(${TargetNameExe2} PUBLIC estimator)
target_link_libraries(${TargetNameExe3} PUBLIC estimator)