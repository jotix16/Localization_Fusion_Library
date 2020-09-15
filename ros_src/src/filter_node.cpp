#include "ros/ros.h"
#include <iostream>

#include<filter_node.h>


int main(int argc, char **argv)
{
    ros::init(argc, argv, "filter_node"); // creates the node
    
    ros::NodeHandle n;

    FilterNode node(n);

    ros::spin();
    return 0;

}
