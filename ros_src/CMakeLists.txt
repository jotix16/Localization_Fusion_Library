project(LocalizationFusionLibrary)
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
# Add a rosnode
add_executable(filter_ros_node src/ex_filter_wrapper.cpp)
target_link_libraries(filter_ros_node ${catkin_LIBRARIES} estimator)
# add_dependencies(filter_node beginner_tutorials_generate_messages_cpp)