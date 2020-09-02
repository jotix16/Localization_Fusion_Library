#include<iostream>
#include<geometry_msgs/msg/PoseWithCovariance.h>
#include<geometry_msgs/msg/TwistWithCovariance.h>
#include<nav_msgs/msg/Odometry.h>
#include<sensor_msgs/msg/Imu.h>


int main()
{
    std::cout<<"Hello ROS"<<std::endl;

    geometry_msgs::msg::TwistWithCovariance my_twist_with_cov;
    my_twist_with_cov.twist.linear.x = 12.0;
    geometry_msgs::msg::PoseWithCovariance my_pose_with_cov;
    sensor_msgs::msg::Imu my_imu;
    nav_msgs::msg::Odometry my_odom;

    std::cout << my_twist_with_cov.twist.linear.y << std::endl;
    std::cout << my_odom.header.frame_id << " " << my_odom.header.stamp.sec << std::endl;


}