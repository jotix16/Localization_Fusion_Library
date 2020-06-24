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

# This line is important for CMake
project(LocalizationFusionLibrary)
# Import main CMake module:
include("${CMAKE_CURRENT_LIST_DIR}/.deps/cmake/ModuleCompilation.cmake")
# Create module using Bonnie:
autogen_add_module(${PROJECT_NAME})



# List IDL files
autogen_use_idl( # automatically chooses IDLs from .deps/IDL/GlobalInterfaces if no BASE_PATH is given
    IDL_LIST
)
autogen_use_idl( BASE_PATH ROSInterfaces
    IDL_LIST
)

# Add all recursively found .c .cpp and .h files to the project:
autogen_get_source_files_from_main_dir()

# include eigen
target_include_directories(${TargetName} PUBLIC
		${LIBRARIES_ROOT_PATH}/eigen/eigen_3_1_4)

target_include_directories(${TargetName}
        PUBLIC
        ${INTERFACES_PATH}/IDL/ROSInterfaces  # For FWCompatible.h etc.
    )

