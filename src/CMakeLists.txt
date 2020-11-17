set(TargetName estimator)

# set the relevant files
FILE(GLOB_RECURSE HEADERS include *.h *.hpp)
FILE(GLOB_RECURSE SOURCES src *.cpp)


# Geographiclib installs FindGeographicLib.cmake to this non-standard location
# required to be installed previously!!
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "/usr/share/cmake/geographiclib/")
find_package(GeographicLib REQUIRED COMPONENTS STATIC)

# add library
add_library(${TargetName} ${HEADERS} ${SOURCES})

target_include_directories( ${TargetName} PRIVATE include)
target_include_directories( ${TargetName} PUBLIC include)

target_link_libraries(${TargetName} eigen rapidjson ros_interfaces ${GeographicLib_LIBRARIES})

# install
install_library(TARGET ${TargetName})
install_include(DIRECTORY include)