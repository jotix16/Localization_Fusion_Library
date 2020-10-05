#include <iostream>
#include "ros/ros.h"

#include <XmlRpcException.h>
#include <filter_node.h>
#include <cstdlib>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "filter_node");  // creates the node
  ros::NodeHandle n;
  ros::NodeHandle n_param("~");

  // NodeCtraEKF3D node(n, n_param);
  NodeCtraEKF2D node(n, n_param);
  // NodeCtrvEKF2D node(n, n_param);
  node.init("filter_config.json");
  // node.init("filter_config_rosario.json");
  node.publish_current_state();
  ros::spin();

  return 0;
}
