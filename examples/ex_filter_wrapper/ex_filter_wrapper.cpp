#include<iostream>
#include<chrono>
#include<thread>

#include<filter/filter_wrapper.h>
#include<utilities/filter_utilities.h>

#include<geometry_msgs/msg/PoseWithCovariance.h>
#include<geometry_msgs/msg/TwistWithCovariance.h>
#include<nav_msgs/msg/Odometry.h>
#include<sensor_msgs/msg/Imu.h>

#include<eigen/Eigen>

class ClassWithClock {
    iav::state_predictor::Clock clock;
    public:
        ClassWithClock()
        {
            clock = iav::state_predictor::Clock();
        }
        void print()
        {
            std::cout<<(clock.now()) <<"\n";
        }
};

int main(){
    // 1. try out clock from utilities
    using namespace std::chrono;
    using namespace std::chrono_literals;
    ClassWithClock my_class;
    std::this_thread::sleep_for(0.1s);
    my_class.print();


    // 2. Try out filterWrapper

    // initiate filterWrapper
    const char * path = "config/filter_config.json";
    // const char * path = "../../config/filter_config.json";
    iav::state_predictor::filter::FilterCtrvEKF2D my_filter_wrapper(path);

    nav_msgs::msg::Odometry msg;
    msg.header.frame_id = "odom_0";
    msg.header.stamp.sec = 1200;
    msg.pose.pose.position.x = 1.0;
    msg.pose.pose.position.y = 2.0;
    msg.pose.pose.position.z = 3.0;
    for (int i = 0; i < 6; i++)
    {
       msg.pose.covariance[i+i*6] = 1e-9; 
       msg.twist.covariance[i+i*6]= 1e-9; 
    }
    msg.pose.pose.orientation.x = 1.0;
    msg.pose.pose.orientation.y = 1.0;
    msg.pose.pose.orientation.z = 1.0;
    msg.pose.pose.orientation.w = 1.0;
    msg.twist.twist.linear.x = 10.0;
    msg.twist.twist.linear.y = 11.0;
    msg.twist.twist.linear.z = 12.0;
    msg.twist.twist.angular.x = 0.0;
    msg.twist.twist.angular.y = 0.0;
    msg.twist.twist.angular.z = 12.0;


    my_filter_wrapper.odom_callback(&msg, Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity());
    msg.header.stamp.sec = 3331;
    my_filter_wrapper.odom_callback(&msg, Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity());
    my_filter_wrapper.odom_callback(&msg, Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity());
    my_filter_wrapper.odom_callback(&msg, Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity());
    my_filter_wrapper.odom_callback(&msg, Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity());
    std::cout<< "State: " <<my_filter_wrapper.get_state().transpose()<<"\n";
    std::cout<< "Covariance: \n" <<my_filter_wrapper.get_covariance().transpose()<<"\n";
    // geometry_msgs::msg::TwistWithCovariance twist;

    return 1;
}