#include<sensors/odom.h>
#include<sensors/imu.h>
#include<iostream>
#include<Eigen/Dense>

#include<sensor_msgs/msg/Imu.h>


using namespace Eigen;
using ImuD =  iav::state_predictor::sensors::ImuD;
using StateVector   = typename ImuD::StateVector;

int main()
{
    std::cout<< "EXAMPLE IMU SENSOR"<<std::endl;
    bool update_vector[15] = { true,true,true,
                               true,true,true,
                               true,true,true,
                               true,true,true,
                               true,true,true};
    std::ostream* out_stream;
    ImuD imu("imu_topic", update_vector, 12.0, out_stream, false);

    StateVector state = StateVector::Zero();
    sensor_msgs::msg::Imu msg;
    msg.header.frame_id = "imu_0"; msg.header.stamp.sec = 1200;
    msg.orientation.x = 1.0; msg.orientation.y = 1.0; msg.orientation.z = 1.0; msg.orientation.w = 1.0;
    msg.angular_velocity.x = 1.0; msg.angular_velocity.y = 2.0; msg.angular_velocity.z = 3.0;
    msg.linear_acceleration.x = 0.1; msg.linear_acceleration.y = 0.2; msg.linear_acceleration.z = 0.002;
    for (int i = 0; i < 3; i++)
    {
       msg.orientation_covariance[i+i*3] = 1e-9;
       msg.angular_velocity_covariance[i+i*3]= 1e-9;
       msg.linear_acceleration_covariance[i+i*3]= 1e-9;
    }
    imu.imu_callback(state, &msg, Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity());
    msg.header.stamp.sec = 3331;
    imu.imu_callback(state, &msg, Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity());
    imu.imu_callback(state, &msg, Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity());
    imu.imu_callback(state, &msg, Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity());
    imu.imu_callback(state, &msg, Eigen::Isometry3d::Identity(), Eigen::Isometry3d::Identity());
}