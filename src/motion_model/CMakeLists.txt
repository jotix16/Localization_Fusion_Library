set(TargetName motion_model)

# set the relevant files
FILE(GLOB_RECURSE HEADERS include *.h *.hpp)
FILE(GLOB_RECURSE SOURCES src *.cpp)

# add library
add_library(${TargetName} ${HEADERS} ${SOURCES})

# include directories
target_include_directories( ${TargetName} PRIVATE include)
target_include_directories( ${TargetName} PUBLIC include)

# link
target_link_libraries(${TargetName} PUBLIC eigen)

# install
install_library(TARGET ${TargetName})
install_include(DIRECTORY include)
