set(TargetNameExe ${CMAKE_PROJECT_NAME}_exe)


add_executable(${TargetNameExe} main.cpp )

target_link_libraries(${TargetNameExe} PUBLIC ${CMAKE_PROJECT_NAME})