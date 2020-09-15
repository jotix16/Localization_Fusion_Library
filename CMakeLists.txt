##########################################################################################
#                                                                                        #
#         This program is the confidential and proprietary product of IAV GmbH.          #
# Any unauthorised use, reproduction or transfer of this program is strictly prohibited. #
#                                  Copyright IAV GmbH.                                   #
#             (Subject to limited distribution and restricted disclosure only.)          #
#                                   All rights reserved.                                 #
#                                                                                        #
##########################################################################################

cmake_minimum_required(VERSION 3.13)

set(CMAKE_CXX_STANDARD 14)  # only because of chrono literals
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Setup root lib-name
set(NAME_ROOT "LocalizationFusionLibrary")

# Setup project name and version
set(${NAME_ROOT}_MAJOR_VERSION 0)
set(${NAME_ROOT}_MINOR_VERSION 1)
set(${NAME_ROOT}_PATCH_VERSION 0)
set(${NAME_ROOT}_VERSION "${${NAME_ROOT}_MAJOR_VERSION}.${${NAME_ROOT}_MINOR_VERSION}.${${NAME_ROOT}_PATCH_VERSION}")

project(${NAME_ROOT} VERSION ${${NAME_ROOT}_VERSION})

######################### R O S #################################
#################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  message_generation
)


################################################
## Declare ROS messages, services and actions ##
################################################
# add_message_files(
# 	 FILES
#  )

## Generate added messages and services with any dependencies listed here
#  generate_messages(
#    DEPENDENCIES
#    std_msgs
#  )

catkin_package(
  CATKIN_DEPENDS message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)
#################################################################
######################### R O S #################################

option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_EXAMPLES "Build examples, will link matplotlib. Therefore you will need a valid Python2 installation." ON)


# Add CMake module paths
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH};${CMAKE_CURRENT_LIST_DIR}/scripts/cmake CACHE PATH "Path for utility functions")
# Add CMake prefix path
set(CMAKE_PREFIX_PATH "${CMAKE_PREFIX_PATH};${CMAKE_CURRENT_LIST_DIR}/scripts/cmake")

# Include Functions
include(utilities)

# Import main CMake module
# include("${CMAKE_CURRENT_LIST_DIR}/.deps/cmake/ModuleCompilation.cmake")

enable_testing(True)

# Add compiler definitions
add_definitions(-D${NAME_ROOT}_VERSION=${${NAME_ROOT}_VERSION})
add_definitions(-D${NAME_ROOT}_MAJOR_VERSION=${${NAME_ROOT}_MAJOR_VERSION})
add_definitions(-D${NAME_ROOT}_MINOR_VERSION=${${NAME_ROOT}_MINOR_VERSION})
add_definitions(-D${NAME_ROOT}_PATCH_VERSION=${${NAME_ROOT}_PATCH_VERSION})

# Set basic paths
set(${NAME_ROOT}_ROOT_PATH           ${CMAKE_CURRENT_LIST_DIR})
set(${NAME_ROOT}_MODULE_PATH         ${CMAKE_CURRENT_LIST_DIR}/module)
set(${NAME_ROOT}_EXAMPLES_PATH       ${CMAKE_CURRENT_LIST_DIR}/examples)
set(${NAME_ROOT}_SRC_PATH            ${CMAKE_CURRENT_LIST_DIR}/src)
set(${NAME_ROOT}_TESTS_PATH          ${CMAKE_CURRENT_LIST_DIR}/test)
set(${NAME_ROOT}_TOOLS_PATH          ${CMAKE_CURRENT_LIST_DIR}/tools)
set(${NAME_ROOT}_THIRDPARTY_PATH     ${CMAKE_CURRENT_LIST_DIR}/3rdParty)
set(${NAME_ROOT}_INTERFACES          ${CMAKE_CURRENT_LIST_DIR}/interfaces)

# Set the default install prefix if it's not set by the user
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
    set(CMAKE_INSTALL_PREFIX ${CMAKE_SOURCE_DIR}/install CACHE PATH "Install directory used by install." FORCE)
endif()

include (GenerateExportHeader)
add_subdirectory(${${NAME_ROOT}_THIRDPARTY_PATH})
add_subdirectory(${${NAME_ROOT}_SRC_PATH})
add_subdirectory(${${NAME_ROOT}_EXAMPLES_PATH})
add_subdirectory(${${NAME_ROOT}_TESTS_PATH})
add_subdirectory(${${NAME_ROOT}_INTERFACES})

# Add a rosnode
add_executable(filter_ros_node examples/ex_filter_wrapper/ex_filter_wrapper.cpp)
target_link_libraries(filter_ros_node ${catkin_LIBRARIES} ${PROJECT_NAME})
# add_dependencies(filter_node beginner_tutorials_generate_messages_cpp)