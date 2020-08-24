set(TargetNameExe ${CMAKE_PROJECT_NAME}_motion_model_example)

add_executable(${TargetNameExe} ex_motion_mod.cpp )

target_link_libraries(${TargetNameExe} PUBLIC estimator)