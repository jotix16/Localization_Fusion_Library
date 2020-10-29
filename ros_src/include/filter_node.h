
#pragma once

#include <array>
#include <string>
#include <iostream>

#include "ros/ros.h"
#include "std_msgs/String.h"
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <geometry_msgs/TwistWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/AccelWithCovarianceStamped.h>
#include <geometry_msgs/PoseWithCovariance.h>
#include <geometry_msgs/TwistWithCovariance.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/NavSatFix.h>

// #include <tf2/buffer_core.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_eigen/tf2_eigen.h>

#include <Eigen/Eigen>

#include <filter/filter_wrapper.h>
#include <utilities/filter_utilities.h>


using FilterWrapper = iav::state_predictor::filter::FilterCtrvEKF2D;



template<class FilterT, typename T = double>
class FilterNode
{
    public:
        using OdomMsg = nav_msgs::Odometry;
        using OdomMsgLocFusLib = nav_msgs::msg::Odometry;
        using ImuMsg = sensor_msgs::Imu;
        using ImuMsgLocFusLib = sensor_msgs::msg::Imu;
        using GpsMsg = sensor_msgs::NavSatFix;
        using GpsMsgLocFusLib = sensor_msgs::msg::NavSatFix;

        using PoseWithCovStampedMsg = geometry_msgs::PoseWithCovarianceStamped;
        using TwistWithCovStampedMsg = geometry_msgs::TwistWithCovarianceStamped;

        using HeaderMsg = std_msgs::msg::Header;
        using TransformStamped = geometry_msgs::TransformStamped;
        using TransformationMatrix = typename Eigen::Transform<double, 3, Eigen::TransformTraits::Isometry>;

    private:
        // filter wrapper
        FilterT m_filter_wrapper;

        // ros handles
        ros::NodeHandle m_nh; // node handler(needed for subscription and co)
        ros::NodeHandle m_nh_param;

        // Subscription vectors
        std::vector<ros::Subscriber> m_odom_sub_topics;
        std::vector<ros::Subscriber> m_imu_sub_topics;
        std::vector<ros::Subscriber> m_gps_sub_topics;

        // state publisher
         ros::Publisher m_position_publisher;
         ros::Publisher m_imu_publisher;

        // tf2
        tf2_ros::Buffer m_tf_buffer;
        tf2_ros::TransformListener m_tf_listener;
        tf2_ros::TransformBroadcaster m_tf_broadcaster;

        // frames
        std::string m_world_frame_id; // to be used for orientation part of IMU and GPS
        std::string m_map_frame_id;
        std::string m_odom_frame_id;
        std::string m_baselink_frame_id;
        std::string m_output_baselink_frame_id;

        // time vs data triggered options
        bool m_data_triggered;
        double m_publish_frequency;
        bool m_enu_published;

    public:
        /**
         * @brief Constructor that inizializes configuration related parameters and time-keeping
         * @param[in] nh - the main node handler needed for subscriptions and other ros related stuff
         * @param[in] nh_param - node handler needed for parameter reading
         */
        FilterNode(ros::NodeHandle& nh, ros::NodeHandle& nh_param): m_nh(nh), m_nh_param(nh_param), m_tf_listener(m_tf_buffer),m_enu_published(false)
        {}

        /**
         * @brief FilterNode: Function that inizializes the FilterWrapper from the .json config file and the
         * ros node parameters from the launch file. In addition to that it initializes the subscribers/publishers
         * and sets the initial frame transformations.
         * @param[in] config_path - path to .json configuration file
         */
        void init(std::string config_file2)
        {
            // 1. initialize filter
            std::string config_file;
            m_nh_param.param("config", config_file, std::string("WTF"));
            std::string path = std::string(NODE_PATH) + std::string("/ros_src/config/") + config_file;
            ROS_INFO_STREAM("conf file: " << config_file << "\n");
            ROS_INFO_STREAM("Initialize FilterWrapper from path: " << path << "\n");
            m_filter_wrapper.reset_config(path.c_str());

            // 2. initialize publisher
            m_position_publisher = m_nh.advertise<OdomMsg>("odometry/filtered", 20);
            m_imu_publisher = m_nh.advertise<OdomMsg>("pose/filtered", 20);

            // 3.a. initialize odom subscribers
            int odom_nr_;
            m_nh_param.param("odom_nr", odom_nr_, 0);

            for(int i=0; i < odom_nr_; i++)
            {
                std::stringstream ss;
                ss << "odom" << i ;
                std::string odom_topic;
                m_nh_param.getParam(ss.str(), odom_topic);
                ROS_INFO_STREAM("Subscribing to: " << odom_topic);
                m_odom_sub_topics.push_back(m_nh.subscribe<OdomMsg>(odom_topic, 10,
                boost::bind(&FilterNode::odom_callback, this, _1, odom_topic)));
            }

            // 3.b. initialize imu subscribers
            int imu_nr_;
            m_nh_param.param("imu_nr", imu_nr_, 0);

            for(int i=0; i < imu_nr_; i++)
            {
                std::stringstream ss;
                ss << "imu" << i ;
                std::string imu_topic;
                m_nh_param.getParam(ss.str(), imu_topic);
                ROS_INFO_STREAM("Subscribing to: " << imu_topic);
                m_imu_sub_topics.push_back(m_nh.subscribe<ImuMsg>(imu_topic, 10,
                boost::bind(&FilterNode::imu_callback, this, _1, imu_topic)));
            }

            // 3.c. initialize gps subscribers
            int gps_nr_;
            m_nh_param.param("gps_nr", gps_nr_, 0);

            for(int i=0; i < gps_nr_; i++)
            {
                std::stringstream ss;
                ss << "gps" << i ;
                std::string gps_topic;
                m_nh_param.getParam(ss.str(), gps_topic);
                ROS_INFO_STREAM("Subscribing to: " << gps_topic);
                m_gps_sub_topics.push_back(m_nh.subscribe<GpsMsg>(gps_topic, 10,
                boost::bind(&FilterNode::gps_callback, this, _1, gps_topic)));
            }

            // 4. set the frames
            m_nh_param.param("map_frame", m_map_frame_id, std::string("map"));
            m_nh_param.param("odom_frame", m_odom_frame_id, std::string("odom"));
            m_nh_param.param("base_link_frame", m_baselink_frame_id, std::string("base_link"));
            m_nh_param.param("base_link_frame_output", m_output_baselink_frame_id, m_baselink_frame_id);

            ROS_FATAL_COND(m_map_frame_id == m_odom_frame_id ||
                        m_odom_frame_id == m_baselink_frame_id ||
                        m_map_frame_id == m_baselink_frame_id ||
                        m_odom_frame_id == m_output_baselink_frame_id ||
                        m_map_frame_id == m_output_baselink_frame_id,
                        "Invalid frame configuration! The values for map_frame, odom_frame, "
                        "and base_link_frame must be unique. If using a base_link_frame_output values, it "
                        "must not match the map_frame or odom_frame.");
        }

        /**
         * @brief FilterNode: Callback for receiving all odom msgs. It extracts transformation matrixes
         * to the fusing frames and calls the corresponding callback from filter_wrapper.
         * @param[in] msg - reference to the odom msg of the measurement
         * @param[string] topic_name - name of the topic where we listened the message
         */
        void odom_callback(const OdomMsg::ConstPtr& msg, std::string topic_name)
        {
            TransformationMatrix transform_to_map;
            TransformationMatrix transform_to_base_link;

            // 1. get transformations from map and base_link to the sensor frame
            std::string msgFrame = (msg->header.frame_id == "" ? m_baselink_frame_id : msg->header.frame_id);
            std::string msgChildFrame = (msg->child_frame_id == "" ? m_baselink_frame_id : msg->child_frame_id);
            geometry_msgs::TransformStamped transformStamped1;
            geometry_msgs::TransformStamped transformStamped2;
            try
            {
                // a. map frame to sensor frame for the pose part of msg
                transformStamped1 = m_tf_buffer.lookupTransform(m_map_frame_id, msgFrame ,ros::Time(0), ros::Duration(1.0));
                transform_to_map = tf2::transformToEigen(transformStamped1);

                if (m_baselink_frame_id == msgFrame)
                {
                    transform_to_base_link.setIdentity();
                }
                else
                {
                    // b. base_link frame to sensor frame for the twist part of msg
                    transformStamped2 = m_tf_buffer.lookupTransform(m_baselink_frame_id, msgChildFrame, ros::Time(0), ros::Duration(1.0));
                    transform_to_base_link = tf2::transformToEigen(transformStamped2);
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
            to_local_odom_msg(msg, msg_loc);

            // 2. call filter's odom_callback
            // ros_info_msg(msg_loc);
            std::cout << "CHILD FRANE ID: " << msgChildFrame << "\n";
            std::cout << "MSG FRAME ID: " << msgFrame << "\n";
            m_filter_wrapper.odom_callback(topic_name, &msg_loc, transform_to_map, transform_to_base_link);

            // 3. publish updated base_link frame
            publish_current_state();
        }

        /**
         * @brief FilterNode: Callback for receiving all IMU msgs. It extracts transformation matrixes
         * to the fusing frames and calls the corresponding callback from filter_wrapper.
         * @param[in] msg - reference to the IMU msg of the measurement
         * @param[string] topic_name - name of the topic where we listened the message
         */
        void imu_callback(const ImuMsg::ConstPtr& msg, std::string topic_name)
        {
            // std::stringstream ss;
            // ss << "IMU MSG:\n" << *msg << "\n";
            // std::cout <<ss.str();

            TransformationMatrix transform_base_link_imu;
            TransformationMatrix transform_map_base_link;

            // 1. get transformations from base_link to the sensor frame
            std::string msgFrame = (msg->header.frame_id == "" ? m_baselink_frame_id : msg->header.frame_id);
            geometry_msgs::TransformStamped transform_stamped1;
            geometry_msgs::TransformStamped transform_stamped;
            try
            {
                // a. map_bl
                transform_stamped1 = m_tf_buffer.lookupTransform(m_map_frame_id, m_baselink_frame_id, ros::Time(0), ros::Duration(1.0));
                transform_map_base_link = tf2::transformToEigen(transform_stamped);

                // b. baselink frame to imu sensor frame for the velocity and acceleratation part of the imu msg
                if (m_baselink_frame_id == msgFrame)
                {
                    transform_base_link_imu.setIdentity();
                }
                else
                {
                    // bl_imu
                    transform_stamped = m_tf_buffer.lookupTransform(m_baselink_frame_id, msgFrame, ros::Time(0), ros::Duration(1.0));
                    transform_base_link_imu = tf2::transformToEigen(transform_stamped);

                }
            }
            catch (tf2::TransformException &ex)
            {
                ROS_INFO("%s",ex.what());
                ros::Duration(1.0).sleep();
                return;
            }

            // VISUALIZE IMU
            // visualize_imu(msg, msgFrame);

            // get msg in our local msg form.
            ImuMsgLocFusLib msg_loc;
            to_local_imu_msg(msg, msg_loc);

            // 2. call filter's imu_callback
            // ros_info_msg(msg_loc);
            if(m_filter_wrapper.imu_callback(topic_name, &msg_loc, transform_base_link_imu, transform_map_base_link))
            {
                // 3. publish updated base_link frame
                publish_current_state();
            }
        }

        void gps_callback(const GpsMsg::ConstPtr& msg, std::string topic_name)
        {
            // std::stringstream ss;
            // ss << "GPS MSG:\n" << *msg << "\n";
            // std::cout <<ss.str();

            // 1. get transformations from base_link to the sensor frame
            TransformationMatrix transform_to_base_link;
            std::string msgFrame = (msg->header.frame_id == "" ? m_baselink_frame_id : msg->header.frame_id);
            geometry_msgs::TransformStamped transform_stamped;
            try
            {
                // b. baselink frame to gps sensor frame for the velocity and acceleratation part of the gps msg
                if (m_baselink_frame_id == msgFrame)
                {
                    transform_to_base_link.setIdentity();
                }
                else
                {
                    transform_stamped = m_tf_buffer.lookupTransform(m_baselink_frame_id, msgFrame, ros::Time(0), ros::Duration(1.0));
                    transform_to_base_link = tf2::transformToEigen(transform_stamped);
                }
            }
            catch (tf2::TransformException &ex)
            {
                ROS_INFO("%s",ex.what());
                ros::Duration(1.0).sleep();
                return;
            }

            // 2. get msg in our local msg form.
            GpsMsgLocFusLib msg_loc;
            to_local_navsat_msg(msg, msg_loc);

            // 3. call filter's gps_callback
            if(m_filter_wrapper.gps_callback(topic_name, &msg_loc, transform_to_base_link))
            {
                // 3. publish updated base_link frame
                publish_current_state();
            }
        }



        void pose_callback(const PoseWithCovStampedMsg::ConstPtr& msg)
        {
            ROS_INFO("Pose Callback called!\n");
        }

        void twist_callback(const TwistWithCovStampedMsg::ConstPtr& msg)
        {
            ROS_INFO("Twist Callback called!\n");
        }

        /**
         * @brief FilterNode: Helper function tht publishes the estimated state and updates the respective tf2 frames.
         */
        void publish_current_state()
        {
            if (!m_filter_wrapper.is_initialized()) return;
            OdomMsgLocFusLib msg_loc;
            msg_loc = m_filter_wrapper.get_state_odom();

            // publish topic
            OdomMsg msg_pub;
            to_ros_odom_msg(msg_pub, msg_loc);
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

        /**
         * @brief FilterNode: Helper function to transform ros odom to idl(ros2)_odom
         * @param[in] msg - reference to the ROS msg to be transformed
         * @param[inout] msg_loc - reference to the msg[idl(ROS2)] to be filled and returned
         */
        void to_local_odom_msg(const OdomMsg::ConstPtr& msg, OdomMsgLocFusLib &msg_loc)
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

        /**
         * @brief FilterNode: Helper function to transform ros imu to idl(ros2)_imu
         * @param[in] msg - reference to the ROS msg to be transformed
         * @param[inout] msg_loc - reference to the msg[idl(ROS2)] to be filled and returned
         */
        void to_local_imu_msg(const ImuMsg::ConstPtr& msg, ImuMsgLocFusLib &msg_loc)
        {
            // header
            msg_loc.header.frame_id = msg->header.frame_id;
            msg_loc.header.stamp.sec = msg->header.stamp.sec;
            msg_loc.header.stamp.nanosec = msg->header.stamp.nsec;

            // orientation
            msg_loc.orientation.x = msg->orientation.x;
            msg_loc.orientation.y = msg->orientation.y;
            msg_loc.orientation.z = msg->orientation.z;
            msg_loc.orientation.w = msg->orientation.w;
            for (int i = 0; i < 9; i++)
            {
                msg_loc.orientation_covariance[i] = msg->orientation_covariance[i];
            }

            // angular velocity
            msg_loc.angular_velocity.x = msg->angular_velocity.x;
            msg_loc.angular_velocity.y = msg->angular_velocity.y;
            msg_loc.angular_velocity.z = msg->angular_velocity.z;
            for (int i = 0; i < 9; i++)
            {
                msg_loc.angular_velocity_covariance[i] = msg->angular_velocity_covariance[i];
            }

            // linear acceleration
            msg_loc.linear_acceleration.x = msg->linear_acceleration.x;
            msg_loc.linear_acceleration.y = msg->linear_acceleration.y;
            msg_loc.linear_acceleration.z = msg->linear_acceleration.z;
            for (int i = 0; i < 9; i++)
            {
                msg_loc.linear_acceleration_covariance[i] = msg->linear_acceleration_covariance[i];
            }
        }
        /**
         * @brief FilterNode: Helper function to transform ros NavSatFix to idl(ros2)_NavSatFix
         * @param[in] msg - reference to the ROS msg to be transformed
         * @param[inout] msg_loc - reference to the msg[idl(ROS2)] to be filled and returned
         */

        void to_local_navsat_msg(const GpsMsg::ConstPtr& msg, GpsMsgLocFusLib &msg_loc)
        {
            // status
            msg_loc.status.status = msg->status.status;
            msg_loc.status.service = msg->status.service;

            // header
            msg_loc.header.frame_id = msg->header.frame_id;
            msg_loc.header.stamp.sec = msg->header.stamp.sec;
            msg_loc.header.stamp.nanosec = msg->header.stamp.nsec;

            // position
            msg_loc.latitude = msg->latitude;
            msg_loc.longitude = msg->longitude;
            msg_loc.altitude = msg->altitude;
            for (int i = 0; i < 9; i++)
            {
                msg_loc.position_covariance[i] = msg->position_covariance[i];
            }
            msg_loc.position_covariance_type = msg->position_covariance_type;
        }

        /**
         * @brief FilterNode: Helper function to transform idl(ROS2) odom to ros odom
         * @param[in] msg_loc - reference to the msg[idl(ROS2)] to transformed
         * @param[inout] msg - reference to the ROS msg to be filled and returned
         */
        void to_ros_odom_msg(OdomMsg& msg, const OdomMsgLocFusLib &msg_loc)
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

        /**
         * @brief FilterNode: Helper function to print out a msg.
         * @param[in] msg - reference to the msg to be printed
         */
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

            std::cout << "Measurement: " << ss.str() << "\n";
        }

        /**
         * @brief FilterNode: Helper function to publish an odom msg from the imu for visualization
         * @param[in] msg - reference to the IMU msg to be printed
         * @param[in] msgFrame - transformation frame of the IMU sensor
         */
        void visualize_imu(const ImuMsg::ConstPtr& msg, std::string msgFrame)
        {
            // ------------------------------- Visualize IMU ------------------------------------ //
            tf2::Quaternion quat_map_enu, quat_enu_imu_meas, quat_bl_imu;
            geometry_msgs::Quaternion q_map_enu = m_tf_buffer.lookupTransform(m_map_frame_id, "enu", ros::Time(0), ros::Duration(1.0)).transform.rotation;
            geometry_msgs::Quaternion q_bl_imu = m_tf_buffer.lookupTransform(m_baselink_frame_id , msgFrame, ros::Time(0), ros::Duration(1.0)).transform.rotation;
            tf2::convert(q_map_enu , quat_map_enu);
            tf2::convert(msg->orientation , quat_enu_imu_meas);
            tf2::convert(q_bl_imu , quat_bl_imu);

            quat_enu_imu_meas = quat_map_enu * quat_enu_imu_meas * quat_bl_imu.inverse();
            quat_enu_imu_meas.normalize();

            geometry_msgs::PoseWithCovariance* posePtr = new geometry_msgs::PoseWithCovariance();
            posePtr->pose.orientation = tf2::toMsg(quat_enu_imu_meas);

            Eigen::Quaterniond q;
            Eigen::fromMsg(posePtr->pose.orientation, q);
            std::cout << "RIGHT1: " << q.toRotationMatrix().eulerAngles(0, 1, 2).transpose() << "\n";
            std::cout << "RIGHT2: " << q.vec().transpose() << " "  << q.w() << "\n";

            // Copy the covariance
            for (size_t i = 0; i < 3; i++)
            {
                for (size_t j = 0; j < 3; j++)
                {
                    posePtr->covariance[6*i + j] = 0.0;
                    posePtr->covariance[6*(i + 3) + j] = 0.0;
                    posePtr->covariance[6*i + (j + 3)] = 0.0;
                    posePtr->covariance[6 * (i + 3) + (j + 3)] =
                    msg->orientation_covariance[3 * i + j];
                }
            }
            OdomMsgLocFusLib msg_loc_tempo;
            msg_loc_tempo = m_filter_wrapper.get_state_odom(false);
            posePtr->pose.position.x = msg_loc_tempo.pose.pose.position.x;
            posePtr->pose.position.y = msg_loc_tempo.pose.pose.position.y;
            posePtr->pose.position.z = msg_loc_tempo.pose.pose.position.z;

            OdomMsg msg_pub;
            msg_pub.header.stamp = ros::Time::now();
            msg_pub.header.frame_id = m_map_frame_id;
            msg_pub.child_frame_id = m_baselink_frame_id;
            msg_pub.pose = *posePtr;
            m_imu_publisher.publish(msg_pub);
            // ------------------------------- Visualize IMU ------------------------------------ //
        }


        /**
         * @brief FilterNode: Helper function to calculate T_map_enu from the first IMU measurement.
         *        It is used to represent orientations from IMU in map frame: x_map = T_map_enu * x_enu
         * @param[in] msg - reference to the IMU msg to be printed
         * @param[in] msgFrame - transformation frame of the IMU sensor
         */
        void publish_enu_frame(TransformationMatrix trans_world_imu, const ImuMsg::ConstPtr& msg)
        {
            Eigen::Quaterniond q;
            Eigen::fromMsg(msg->orientation, q);
            std::cout << "EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEMIKEL: " << q.x() <<", "<< q.y() <<", "  << q.z() <<"\n";
            trans_world_imu = trans_world_imu * q.inverse();
            q = trans_world_imu.rotation();
            std::cout << "EEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEEMIKEL: " << q.x() <<", "<< q.y() <<", "  << q.z() <<"\n";
            // publish frame
            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header = msg->header;
            transformStamped.header.frame_id = m_map_frame_id;
            transformStamped.child_frame_id = "enu";
            transformStamped.transform.translation.x = 1;
            transformStamped.transform.translation.y = 1;
            transformStamped.transform.translation.z = 1;
            transformStamped.transform.rotation.x = q.x();
            transformStamped.transform.rotation.y = q.y();
            transformStamped.transform.rotation.z = q.z();
            transformStamped.transform.rotation.w = q.w();
            m_tf_broadcaster.sendTransform(transformStamped);
        }


};

// explicit template initialization
using NodeCtrvEKF2D = FilterNode<iav::state_predictor::filter::FilterCtrvEKF2D, double>;
using NodeCtraEKF2D = FilterNode<iav::state_predictor::filter::FilterCtraEKF2D, double>;
using NodeCtraEKF3D = FilterNode<iav::state_predictor::filter::FilterCtraEKF3D, double>;