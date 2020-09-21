#include <array>
#include <string>
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"


#include <nav_msgs/msg/Odometry.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>

// #include <tf2/buffer_core.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

#include <geometry_msgs/TransformStamped.h>

#include <filter/filter_wrapper.h>
#include <utilities/filter_utilities.h>

#include <Eigen/Eigen>

using FilterWrapper = iav::state_predictor::filter::FilterCtrvEKF2D;



template<class FilterT, typename T = double>
class FilterNode
{
    public:
        using OdomMsg = nav_msgs::Odometry;
        using OdomMsgLocFusLib = nav_msgs::msg::Odometry;

        using PoseWithCovStampedMsg = geometry_msgs::PoseWithCovarianceStamped;
        using TwistWithCovStampedMsg = geometry_msgs::TwistWithCovarianceStamped;
        using ImuMsg = sensor_msgs::Imu;
        using HeaderMsg = std_msgs::msg::Header;
        using TransformStamped = geometry_msgs::TransformStamped;
        using TransformationMatrix = typename Eigen::Transform<double, 3, Eigen::TransformTraits::Isometry>;

    private:
        // filter wrapper
        FilterT m_filter_wrapper;

        // nod handler(needed for subscription and co)
        ros::NodeHandle m_nh;
        ros::NodeHandle m_n_param;

        // Subscription vectors
        std::vector<ros::Subscriber> m_odom_sub_topics;
        std::vector<ros::Subscriber> m_imu_sub_topics;
        std::vector<ros::Subscriber> m_gps_sub_topics;

        // state publisher
         ros::Publisher m_position_publisher;

        // tf2
        tf2_ros::Buffer m_tf_buffer;
        tf2_ros::TransformListener m_tf_listener;
        tf2_ros::TransformBroadcaster m_tf_broadcaster;

        // frames
        std::string m_map_frame_id;
        std::string m_odom_frame_id;
        std::string m_baselink_frame_id;
        std::string m_output_baselink_frame_id;

        // time vs data triggered options
        bool m_data_triggered;
        double m_publish_frequency;


    public:
        FilterNode(ros::NodeHandle& nh, ros::NodeHandle& n_param): m_nh(nh), m_n_param(n_param), m_tf_listener(m_tf_buffer)
        {
            init();
        }

        void init()
        {
            // 1. initialize filter
            std::string path = std::string(NODE_PATH) + std::string("/config/filter_config.json");
            ROS_INFO_STREAM( "Initialize FilterWrapper from path: " << path << "\n");
            m_filter_wrapper.reset(path.c_str());

            // 2. initialize publisher
            m_position_publisher = m_nh.advertise<OdomMsg>("odometry/filtered", 20);

            // 3. initialize subscribers            
            int odom_nr_;
            m_n_param.param("odom_nr", odom_nr_, 0);

            for(int i=0; i < odom_nr_; i++)
            {
                std::stringstream ss;
                ss << "odom" << i ;

                std::string odom_top;
                m_n_param.getParam(ss.str(), odom_top);
                ROS_INFO_STREAM("Subscribing to: " << odom_top);
                m_odom_sub_topics.push_back(m_nh.subscribe<OdomMsg>(odom_top, 10, 
                boost::bind(&FilterNode::odom_callback, this, _1, odom_top)));
            }
            
            // 4. set the frames
            m_n_param.param("map_frame", m_map_frame_id, std::string("map"));
            m_n_param.param("odom_frame", m_odom_frame_id, std::string("odom"));
            m_n_param.param("base_link_frame", m_baselink_frame_id, std::string("base_link"));
            m_n_param.param("base_link_frame_output", m_output_baselink_frame_id, m_baselink_frame_id);

            ROS_FATAL_COND(m_map_frame_id == m_odom_frame_id ||
                        m_odom_frame_id == m_baselink_frame_id ||
                        m_map_frame_id == m_baselink_frame_id ||
                        m_odom_frame_id == m_output_baselink_frame_id ||
                        m_map_frame_id == m_output_baselink_frame_id,
                        "Invalid frame configuration! The values for map_frame, odom_frame, "
                        "and base_link_frame must be unique. If using a base_link_frame_output values, it "
                        "must not match the map_frame or odom_frame.");                     
        }

        TransformStamped get_transform(const HeaderMsg & header)
        {
            std::string m_frame_id = "mikel"; //TO_DO
            return m_tf_buffer.lookupTransform(m_frame_id, header.frame_id, ros::Time(0));
        }

        void odom_callback(const OdomMsg::ConstPtr& msg, std::string topic_name)
        {
            TransformationMatrix transform_to_world;
            TransformationMatrix transform_to_base_link;

            // msg->header.stamp

            // 1. get transformations from world and base_link to the sensor frame
            std::cout << "MSG FRAME ID: " << topic_name << "\n";
            // ROS_INFO(msg->header.frame_id);
            std::string msgFrame = (msg->header.frame_id == "" ? m_baselink_frame_id : msg->header.frame_id);
            geometry_msgs::TransformStamped transformStamped;
            try
            {
                // a. world frame to sensor frame for the pose part of msg
                transformStamped = m_tf_buffer.lookupTransform(m_map_frame_id, msgFrame ,ros::Time(0), ros::Duration(3.0));
                transform_to_world = tf2::transformToEigen(transformStamped);

                if (m_baselink_frame_id == msgFrame)
                {
                    transform_to_base_link.setIdentity();
                }
                else 
                {
                    // b. base_link frame to sensor frame for the twist part of msg
                    transformStamped = m_tf_buffer.lookupTransform(m_baselink_frame_id, msgFrame, ros::Time(0), ros::Duration(3.0));
                    transform_to_base_link = tf2::transformToEigen(transformStamped);
                }
            }
            catch (tf2::TransformException &ex)
            {
              ROS_INFO("%s",ex.what());
              ros::Duration(1.0).sleep();
              return;
            }

            // get msg in our local msg form.
            OdomMsgLocFusLib msg_loc;
            to_local_msg(msg, msg_loc);


            
            // 2. call filter's odom_callback
            ROS_INFO_STREAM( "Odom Callback called  msg time:" << ((double)msg->header.stamp.sec) << " now:" << ros::Time::now() <<"\n");
            // ros_info_msg(msg_loc);
            m_filter_wrapper.odom_callback(topic_name, &msg_loc, transform_to_world, transform_to_base_link);


            // std::cout << transform_to_world.matrix() << "\n";
            // std::cout << transform_to_base_link.matrix() << "\n";

            // 3. publish updated base_link frame
            publish_current_state();

        }
        
        void ros_info_msg(const OdomMsgLocFusLib& msg)
        {
            std::stringstream ss;
            ss << msg.pose.pose.position.x << "//"
               << msg.pose.pose.position.y << "//"
               << msg.pose.pose.position.z << "//"
               << msg.twist.twist.linear.x << "//"
               << msg.twist.twist.linear.y << "//"
               << msg.twist.twist.linear.z << "//"
               << msg.twist.twist.angular.x << "//"
               << msg.twist.twist.angular.y << "//"
               << msg.twist.twist.angular.z << "//";

            ROS_INFO_STREAM( "Measurement: " << ss.str() << "\n");
        }
        
        void to_local_msg(const OdomMsg::ConstPtr& msg, OdomMsgLocFusLib &msg_loc)
        {
            msg_loc.header.frame_id = msg->header.frame_id;
            msg_loc.header.stamp.sec = msg->header.stamp.sec;
            msg_loc.header.stamp.nanosec = msg->header.stamp.nsec;
            msg_loc.pose.pose.position.x = msg->pose.pose.position.x;
            msg_loc.pose.pose.position.y = msg->pose.pose.position.y;
            msg_loc.pose.pose.position.z = msg->pose.pose.position.z;
            for (int i = 0; i < 36; i++)
            {
                msg_loc.pose.covariance[i] = msg->pose.covariance[i]; 
                msg_loc.twist.covariance[i] = msg->twist.covariance[i]; 
            }
            msg_loc.pose.pose.orientation.x = msg->pose.pose.orientation.x;
            msg_loc.pose.pose.orientation.y = msg->pose.pose.orientation.y;
            msg_loc.pose.pose.orientation.z = msg->pose.pose.orientation.z;
            msg_loc.pose.pose.orientation.w = msg->pose.pose.orientation.w;
            msg_loc.twist.twist.linear.x = msg->twist.twist.linear.x;
            msg_loc.twist.twist.linear.y = msg->twist.twist.linear.y;
            msg_loc.twist.twist.linear.z = msg->twist.twist.linear.z;
            msg_loc.twist.twist.angular.x = msg->twist.twist.angular.x;
            msg_loc.twist.twist.angular.y = msg->twist.twist.angular.y;
            msg_loc.twist.twist.angular.z = msg->twist.twist.angular.z;
        }

        void to_ros_msg(OdomMsg& msg, const OdomMsgLocFusLib &msg_loc)
        {
            msg.header.frame_id =      msg_loc.header.frame_id;
            msg.header.stamp.sec =     msg_loc.header.stamp.sec;
            msg.header.stamp.nsec = msg_loc.header.stamp.nanosec;
            msg.pose.pose.position.x = msg_loc.pose.pose.position.x;
            msg.pose.pose.position.y = msg_loc.pose.pose.position.y;
            msg.pose.pose.position.z = msg_loc.pose.pose.position.z;
            for (int i = 0; i < 36; i++)
            {
                msg.pose.covariance[i] =  msg_loc.pose.covariance[i]; 
                msg.twist.covariance[i] = msg_loc.twist.covariance[i]; 
            }
            msg.pose.pose.orientation.x = msg_loc.pose.pose.orientation.x;
            msg.pose.pose.orientation.y = msg_loc.pose.pose.orientation.y;
            msg.pose.pose.orientation.z = msg_loc.pose.pose.orientation.z;
            msg.pose.pose.orientation.w = msg_loc.pose.pose.orientation.w;
            msg.twist.twist.linear.x =    msg_loc.twist.twist.linear.x;
            msg.twist.twist.linear.y =    msg_loc.twist.twist.linear.y;
            msg.twist.twist.linear.z =    msg_loc.twist.twist.linear.z;
            msg.twist.twist.angular.x =   msg_loc.twist.twist.angular.x;
            msg.twist.twist.angular.y =   msg_loc.twist.twist.angular.y;
            msg.twist.twist.angular.z =   msg_loc.twist.twist.angular.z;
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
            if (!m_filter_wrapper.is_initialized()) return;
            OdomMsgLocFusLib msg_loc;
            msg_loc = m_filter_wrapper.get_state_odom();

            // publish topic
            OdomMsg msg_pub;
            to_ros_msg(msg_pub, msg_loc);
            msg_pub.header.stamp = ros::Time(m_filter_wrapper.get_last_measurement_time());
            msg_pub.header.frame_id = m_map_frame_id;
            msg_pub.child_frame_id = m_output_baselink_frame_id;
            m_position_publisher.publish(msg_pub);

            // publish frame
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header = msg_pub.header;
            transformStamped.child_frame_id = msg_pub.child_frame_id;
            transformStamped.transform.translation.x = msg_pub.pose.pose.position.x;
            transformStamped.transform.translation.y = msg_pub.pose.pose.position.y;
            transformStamped.transform.translation.z = msg_pub.pose.pose.position.z;
            transformStamped.transform.rotation = msg_pub.pose.pose.orientation;
            m_tf_broadcaster.sendTransform(transformStamped);
        }
};

// explicit template initialization
using NodeCtrvEKF2D = FilterNode<iav::state_predictor::filter::FilterCtrvEKF2D, double>;
using NodeCtraEKF2D = FilterNode<iav::state_predictor::filter::FilterCtraEKF2D, double>;
using NodeCtraEKF3D = FilterNode<iav::state_predictor::filter::FilterCtraEKF3D, double>;