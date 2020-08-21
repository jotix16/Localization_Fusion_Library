set(TargetName motion_model)

# set the relevant files
FILE(GLOB_RECURSE HEADERS include *.h *.hpp)
FILE(GLOB_RECURSE SOURCES src *.cpp)

# add library
add_library(${TargetName} ${HEADERS} ${SOURCES})

# autogen ros messages from idl's
#autogen_use_idl( 
#	BASE_PATH ROSInterfaces
#    IDL_LIST
#        std_msgs/msg/Time
#        std_msgs/msg/Header
#	TARGET_NAME ${TargetName}
#)

#target_precompile_headers(${PROJECT_NAME} PRIVATE include/mapgraph_lib/pch.h)
# target_include_directories(${TargetName} PUBLIC ${LIBRARIES_ROOT_PATH}/eigen/eigen_3_1_4)

#target_include_directories(${TargetName}
#        PUBLIC
 #       ${INTERFACES_PATH}/IDL/ROSInterfaces 
 #   )
target_include_directories( ${TargetName} PRIVATE include)
target_include_directories( ${TargetName} PUBLIC include)

 
# install
install_library(TARGET ${TargetName})
install_include(DIRECTORY include)
# install_library(TARGET ${TargetName} EXPORT_NAME  CONFIGURATIONS Debug)
