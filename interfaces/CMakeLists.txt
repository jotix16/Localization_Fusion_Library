
cmake_minimum_required(VERSION 3.17)
# project(ROS_INTERFACES VERSION 1.1)
set(TargetName ros_interfaces)

# set the relevant files
FILE(GLOB_RECURSE HEADERS include *.h *.hpp)
FILE(GLOB_RECURSE SOURCES src *.cpp)


# add library
add_library(${TargetName} ${HEADERS} ${SOURCES})

target_include_directories( ${TargetName} PRIVATE include)
target_include_directories( ${TargetName} PUBLIC include)

# target_link_libraries(${TargetName} eigen rapidjson)

add_executable(main main.cpp)
target_link_libraries(main ${TargetName})

# install
install_library(TARGET ${TargetName})
install_include(DIRECTORY include)