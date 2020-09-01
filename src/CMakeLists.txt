set(TargetName estimator)

# set the relevant files
FILE(GLOB_RECURSE HEADERS include *.h *.hpp)
FILE(GLOB_RECURSE SOURCES src *.cpp)

# add library
add_library(${TargetName} ${HEADERS} ${SOURCES})

function(framework_use_idl) 
endfunction()

set(ROSInterfaces ROSInterfaces)
autogen_use_idl(
    BASE_PATH ${ROSInterfaces}
    IDL_LIST
        std_msgs/msg/Time
        std_msgs/msg/Header
)

target_include_directories( ${TargetName} PRIVATE include)
target_include_directories( ${TargetName} PUBLIC include)

target_link_libraries(${TargetName} eigen rapidjson)

# install
install_library(TARGET ${TargetName})
install_include(DIRECTORY include)
