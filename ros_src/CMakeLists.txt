project(LocalizationFusionLibrary)
######################### R O S #################################
#################################################################
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  eigen_conversions
  nav_msgs
  sensor_msgs
  message_generation
  tf2
  tf2_geometry_msgs
  tf2_ros
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
# add_executable(filter_ros_node src/ex_filter_wrapper.cpp)
# target_link_libraries(filter_ros_node ${catkin_LIBRARIES} estimator)
# add_dependencies(filter_node beginner_tutorials_generate_messages_cpp)

add_definitions(-Dloqeee="${PROJECT_SOURCE_DIR}")
add_executable(filter_ros_node src/filter_node.cpp)
target_include_directories(filter_ros_node PRIVATE include)
target_link_libraries(filter_ros_node ${catkin_LIBRARIES} estimator)