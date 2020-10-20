#include<sensors/odom.h>
#include<sensors/imu.h>
#include<iostream>
#include<Eigen/Dense>

#include<nav_msgs/msg/Odometry.h>


using namespace Eigen;
using OdomD = iav::state_predictor::sensors::OdomD;
using ImuD =  iav::state_predictor::sensors::ImuD;
using StateVector   = typename OdomD::StateVector;

int main()
{
    std::cout<< "EXAMPLE ODOM SENSOR"<<std::endl;
    bool update_vector[15] = { true,true,true,
                               true,true,true,
                               true,true,true,
                               true,true,true,
                               true,true,true};
    std::ostream* out_stream;
    OdomD odom("odom_topic", update_vector, 12.0, out_stream, false);


    StateVector state = StateVector::Zero();
    nav_msgs::msg::Odometry msg;
    msg.header.frame_id = "odom_0"; msg.header.stamp.sec = 1200;
    msg.pose.pose.position.x = 1.0; msg.pose.pose.position.y = 2.0; msg.pose.pose.position.z = 3.0;
    msg.pose.pose.orientation.x = 1.0; msg.pose.pose.orientation.y = 1.0; msg.pose.pose.orientation.z = 1.0; msg.pose.pose.orientation.w = 1.0;
    msg.twist.twist.linear.x = 10.0; msg.twist.twist.linear.y = 11.0; msg.twist.twist.linear.z = 12.0;
    msg.twist.twist.angular.x = 0.0; msg.twist.twist.angular.y = 0.0; msg.twist.twist.angular.z = 12.0;
    for (int i = 0; i < 6; i++)
    {
       msg.pose.covariance[i+i*6] = 1e-9; msg.twist.covariance[i+i*6]= 1e-9; 
    }
    odom.odom_callback(state, &msg, Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity());
    msg.header.stamp.sec = 3331;
    odom.odom_callback(state, &msg, Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity());
    odom.odom_callback(state, &msg, Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity());
    odom.odom_callback(state, &msg, Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity());
    odom.odom_callback(state, &msg, Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity());
}