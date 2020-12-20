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
add_executable(ex_timer_buffer src/ex_timer_buffer.cpp)
target_link_libraries(ex_timer_buffer ${catkin_LIBRARIES} estimator)

add_executable(test_euler src/test_euler.cpp)
target_link_libraries(test_euler ${catkin_LIBRARIES} estimator)

add_executable(filter_ros_node src/filter_node.cpp)
target_include_directories(filter_ros_node PRIVATE include)
target_link_libraries(filter_ros_node ${catkin_LIBRARIES} estimator)