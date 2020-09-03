set(TargetName estimator)

# set the relevant files
FILE(GLOB_RECURSE HEADERS include *.h *.hpp)
FILE(GLOB_RECURSE SOURCES src *.cpp)

# add library
add_library(${TargetName} ${HEADERS} ${SOURCES})

target_include_directories( ${TargetName} PRIVATE include)
target_include_directories( ${TargetName} PUBLIC include)

target_link_libraries(${TargetName} eigen rapidjson ros_interfaces)

# install
install_library(TARGET ${TargetName})
install_include(DIRECTORY include)