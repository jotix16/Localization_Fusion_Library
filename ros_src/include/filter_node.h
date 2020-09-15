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
    public:
        using OdomMsg = nav_msgs::Odometry;
        using PoseWithCovStampedMsg = geometry_msgs::PoseWithCovarianceStamped;
        using TwistWithCovStampedMsg = geometry_msgs::TwistWithCovarianceStamped;
        using ImuMsg = sensor_msgs::Imu;
        using HeaderMsg = std_msgs::msg::Header;
        using TransformStamped = geometry_msgs::TransformStamped;
    private:
        // filter wrapper
        FilterWrapper m_filter_wrapper;

        // nod handler(needed for subscription and co)
        ros::NodeHandle m_nh;

        // Subscription vectors
        std::vector<ros::Subscriber> m_odom_sub_topics;
        std::vector<ros::Subscriber> m_imu_sub_topics;
        std::vector<ros::Subscriber> m_gps_sub_topics;

        // state publisher
         ros::Publisher m_position_publisher;

        // tf2
        tf2::BufferCore m_tf_buffer;
        tf2_ros::TransformListener m_tf_listener;
        tf2_ros::TransformBroadcaster m_tf_broadcaster;

        // frames
        std::string m_odom_frame_id;
        std::string m_baselink_child_frame_id;
        std::string m_map_frame_id;
        std::string m_output_baselink_frame_id;

        // time vs data triggered options
        bool m_data_triggered;
        double m_publish_frequency;


    public:
        FilterNode(ros::NodeHandle& nh): m_nh(nh), m_tf_listener(m_tf_buffer)
        {
            std::string path = std::string(NODE_PATH) + std::string("/config/filter_config.json");
            m_filter_wrapper.reset(path.c_str());
            init();
        };

        void init()
        {
            // initialize publisher
            m_position_publisher = m_nh.advertise<OdomMsg>("odometry/filtered", 20);

            std::array<std::string,3> topics = {"/jackal_velocity_controller/odom", "odom_2", "odom_3"};
            for(auto odom_top:topics)
            {
                ROS_INFO_STREAM("Subscribing to: " << odom_top);
                m_odom_sub_topics.push_back(m_nh.subscribe<OdomMsg>(odom_top, 10, &FilterNode::odom_callback, this));
            }
        }

        TransformStamped get_transform(const HeaderMsg & header)
        {
            std::string m_frame_id = "mikel"; //TO_DO
            return m_tf_buffer.lookupTransform(m_frame_id, header.frame_id, ros::Time(0));
        }

        void odom_callback(const OdomMsg::ConstPtr& msg)
        {
            ROS_INFO("Odom Callback called!\n");
        }

        void pose_callback(const PoseWithCovStampedMsg::ConstPtr& msg)
        {
            ROS_INFO("Pose Callback called!\n");
        }

        void twist_callback(const TwistWithCovStampedMsg::ConstPtr& msg)
        {
            ROS_INFO("Twist Callback called!\n");
        }

        void imu_callback(const ImuMsg::ConstPtr& msg)
        {
            ROS_INFO("IMU Callback called!\n");
        }

        void publish_current_state()
        {
            OdomMsg filteredPosition;
            m_position_publisher.publish(filteredPosition);
        }


};