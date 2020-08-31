set(TargetNameExe ${CMAKE_PROJECT_NAME}_filter_config_example)

add_executable(${TargetNameExe} ex_filter_config.cpp )

target_link_libraries(${TargetNameExe} PUBLIC estimator rapidjson)