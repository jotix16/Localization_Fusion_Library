#include <array>
#include <string>
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>

#include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>

#include <filter/filter_wrapper.h>
#include <utilities/filter_utilities.h>

#include <Eigen/Eigen>

using FilterWrapper = iav::state_predictor::filter::FilterCtrvEKF2D;

class FilterNode
{
    private:
        ros::NodeHandle m_nh;
        std::vector<ros::Subscriber> m_sub_topics;
        FilterWrapper m_filter_wrapper;
        tf2::BufferCore m_tf_buffer;
        // tf2_ros::TransformListener m_tf_Listener(m_tf_buffer);
    
    public:
        FilterNode(ros::NodeHandle& nh): m_nh(nh)
        {
            std::string path = std::string(loqeee) + std::string("/config/filter_config.json");
            m_filter_wrapper.reset(path.c_str());

            init();
        };

    void my_odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        ROS_INFO("Odom Callback called!\n");
    }

    geometry_msgs::TransformStamped get_transform(
        const std_msgs::msg::Header & header)
    {
        // Get the transform between the msg and the output frame. We treat the
        // possible exceptions as unrecoverable and let them bubble up.
        std::string m_frame_id = "mikel";
        return m_tf_buffer.lookupTransform(m_frame_id, header.frame_id, ros::Time(0));
    }

    void init()
    {
        std::array<std::string,3> topics = {"/jackal_velocity_controller/odom", "odom_2", "odom_3"};

        for(auto odom_top:topics)
        {
            ROS_INFO_STREAM("Subscribing to: " << odom_top);
            m_sub_topics.push_back(
                m_nh.subscribe<nav_msgs::Odometry>(odom_top, 10, &FilterNode::my_odom_callback, this)
                );
        }
    }
};