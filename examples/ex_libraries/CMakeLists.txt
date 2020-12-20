set(TargetNameExe ${CMAKE_PROJECT_NAME}_lib_exe)

add_executable(${TargetNameExe} ex_lib.cpp )

target_link_libraries(${TargetNameExe} PRIVATE eigen rapidjson)