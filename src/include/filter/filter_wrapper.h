
/*! \file
*
* Copyright (c) 1983-2020 IAV GmbH. All rights reserved.
*
* \authors Mikel Zhobro, Robert Treiber IAV GmbH
*
* Correspondence should be directed to IAV.
*
* IAV GmbH\n
* Carnotstraße 1\n
* 10587 Berlin
*
* GERMANY
*
* \note
* Neither IAV GmbH nor the authors admit liability
* nor provide any warranty for any of this software.
* Until the distribution is granted by IAV GmbH
* this source code is under non disclosure and must
* only be used within projects with controlling
* interest of IAV GmbH.
*/

#pragma once

#include <iomanip>
#include <unordered_map>
#include <vector>

#include <Eigen/Eigen>

#include <utilities/filter_utilities.h>
#include <buffer/filter_buffer.h>
#include <filter/filter_config.h>
#include <filter/filter_ekf.h>
#include <measurement/measurement_time_keeper.h>
#include <measurement/measurement.h>
#include <sensors/odom.h>
#include <sensors/imu.h>
#include <sensors/gps.h>

#include <geometry_msgs/msg/Vector3.h>
#include <geometry_msgs/msg/PoseWithCovariance.h>
#include <geometry_msgs/msg/TwistWithCovariance.h>
#include <nav_msgs/msg/Odometry.h>
#include <sensor_msgs/msg/Imu.h>
#include <sensor_msgs/msg/NavSatFix.h>

#include <mutex>
#include <thread>

namespace iav{ namespace state_predictor { namespace filter {

/**
 * @brief Class that wrapps the filter and takes care of timekeeping and measurement processing
 * @param<template> FilterT - The filter algorithm that we are usign for fusion
 * @param<template> T - Type that should be used for calculations(default is double, but float can be used too)
 */
template<class FilterT>
class FilterWrapper
{
public:
    using MeasurementTimeKeeper = measurement::MeasurementTimeKeeper;
    using T             = typename FilterT::T;
    using FilterConfig_ = FilterConfig<T>;
    using Measurement   = typename FilterT::Measurement;
    using MeasurementPtr   = typename std::shared_ptr<Measurement>;
    using States        = typename FilterT::States;
    using StateCovTime = typename FilterT::StateCovTime;
    using StateCovTimePtr = typename std::shared_ptr<StateCovTime>;
    using StateVector   = typename FilterT::StateVector;
    using StateMatrix   = typename FilterT::StateMatrix;
    using OdomT = typename sensors::Odom<T,States>;
    using ImuT  = typename sensors::Imu<T,States>;
    using GpsT  = typename sensors::Gps<T,States>;
    using BufferT = typename buffer::Buffer <Measurement, StateCovTime, T>;

    using AngleAxisT            = typename Eigen::AngleAxis<T>;
    using QuaternionT           = typename Eigen::Quaternion<T>;
    using TransformationMatrix  = typename Eigen::Transform<T, 3, Eigen::TransformTraits::Isometry>;
    using Vector3T              = typename Eigen::Matrix<T, 3, 1>;

private:
    FilterConfig_ m_config;
    FilterT m_filter;
    Clock m_wall_time;
    MeasurementTimeKeeper m_time_keeper;

    // hash maps with sensor objects
    std::unordered_map<std::string, OdomT> m_odom_sensors_hmap;
    std::unordered_map<std::string, ImuT> m_imu_sensors_hmap;
    std::unordered_map<std::string, GpsT> m_gps_sensors_hmap;

    // debuging variables
    std::ofstream m_debug_stream;
    bool m_debug;
    std::mutex m_callback_mutex;

    // functions from ros/adtf node
    std::function<tTime()> get_time_now;
    std::function<void()> publish_state;

    // options
    BufferT m_time_triggered_buffer;
    bool m_data_triggered;

public:
    FilterWrapper() = default;
    // FilterWrapper() : m_debug(false) {}

    /**
     * @brief Constructor that inizializes configuration related parameters and time-keeping
     * @param[in] config_path - path to .json configuration file
     */
    FilterWrapper(const char* config_path)
    {
        reset_config(config_path);
    }

    void set_time_callback(std::function<T()> func)
    {
        get_time_now = func;
    }

    void set_publish_state(std::function<void()> func)
    {
        publish_state = func;
    }

    /**
     * @brief FilterWrapper: Function that inizializes configuration related parameters and time-keeping
     * @param[in] config_path - path to .json configuration file
     */
    void reset_config(const char* config_path)
    {
        std::cout << "RESETING CONFIG\n";
        configure(config_path);
        m_wall_time = Clock();
        m_time_keeper = MeasurementTimeKeeper();
        m_debug = true;
        create_debug();
        if (!m_data_triggered) init_buffer();
    }

    void init_buffer()
    {
        int interval = 50;
        std::cout << "********* Initializing timed buffer with a period of " << interval << " milliseconds. *********\n";
        m_time_triggered_buffer.set_process_measurement_function([this](MeasurementPtr d) { return this->process_measurement(d);});
        m_time_triggered_buffer.set_predict_function([this](tTime t) { return this->temporal_update(t);});
        m_time_triggered_buffer.set_publish_function([this]() { this->publish_state();});
        m_time_triggered_buffer.set_get_state_ptr_function([this]() { return this->get_state_ptr();});
        m_time_triggered_buffer.set_get_time_now([this]() { return this->get_time_now();});
        m_time_triggered_buffer.start(interval);
    }

    /**
     * @brief FilterWrapper: Function that creates the debug stream if m_debug is set.
     */
    void create_debug()
    {
        if (m_debug)
        {
            std::string debugOutFile = std::string("DEBUG_localization_fusion.log");
            try
            {
                m_debug_stream.open(debugOutFile.c_str());

                // Make sure we succeeded
                if (m_debug_stream.is_open())
                {
                    DEBUG_W("\t\t\t-----------------------------------------\n");
                    DEBUG_W("\t\t\t----- /FilterWrapper::Debug is on!" << " ------\n");
                    DEBUG_W("\t\t\t-----------------------------------------\n");
                    m_filter.setDebug(&m_debug_stream);
                }
                else
                {
                    std::cout<< "RosFilter::loadParams() - unable to create debug output file " << debugOutFile;
                }
            }
            catch(const std::exception &e)
            {
                std::cout<<"RosFilter::loadParams() - unable to create debug output file" << debugOutFile
                                << ". Error was " << e.what() << "\n";
            }
        }
    }

    /**
     * @brief FilterWrapper: Callback for receiving all odom msgs. It processes all comming messages
     * by considering transforming them in the fusing frame, considering the update_vector to ignore parts
     * of the measurements and capturing matrixes with faulty values.
     * @param[in] topic_name - topic name of the odom sensor
     * @param[in] msg - pointer to the odom msg of the measurement
     * @param[in] transform_to_map - transf from sensor frame of msg to map frame where pose is fused
     * @param[in] transform_to_base_link - transf from sensor frame of msg to base_link frame where twist is fused
     */
    bool odom_callback(
        const std::string& topic_name,
        nav_msgs::msg::Odometry* msg,
        const TransformationMatrix& transform_to_base_link,
        bool odom_bl=false)
    {
        if (odom_bl)
        {
            MeasurementPtr m = m_odom_sensors_hmap[topic_name].odom_callback(get_state(), msg);
            return handle_measurement(m);
        }
        else
        {
            MeasurementPtr m = m_odom_sensors_hmap[topic_name].odom_callback(get_state(), msg, transform_to_base_link);
            return handle_measurement(m);
        }
    }

    /**
     * @brief FilterWrapper: Callback for receiving all imu msgs. It processes all comming messages
     * by considering transforming them in the fusing frame, considering the update_vector to ignore parts
     * of the measurements and capturing matrixes with faulty values.
     * @param[in] topic_name - topic name of the imu sensor
     * @param[in] msg - pointer to the imu msg of the measurement
     * @param[in] transform_to_base_link - transf from sensor frame to base_link frame where angular velocity and acceleration are fused
     */
    bool imu_callback(
        const std::string& topic_name,
        sensor_msgs::msg::Imu* msg,
        const TransformationMatrix& transform_base_link_imu,
        const TransformationMatrix& transform_map_base_link)
    {
        if (!is_initialized())
        {
            DEBUG_W("Got IMU but have to wait for odom first. Ignoring\n");
            return false;
        }
        MeasurementPtr m = m_imu_sensors_hmap[topic_name].imu_callback(get_state(), msg, transform_base_link_imu, transform_map_base_link);
        return handle_measurement(m);
    }

    /**
     * @brief FilterWrapper: Callback for receiving all gps msgs. It processes all comming messages
     * by considering transforming them in the fusing frame, considering the update_vector to ignore parts
     * of the measurements and capturing matrixes with faulty values.
     * @param[in] topic_name - topic name of the gps sensor
     * @param[in] msg - pointer to the gps msg of the measurement
     * @param[in] transform_to_base_link - transf from sensor frame to base_link frame
     */
    bool gps_callback(
        const std::string& topic_name,
        sensor_msgs::msg::NavSatFix* msg,
        const TransformationMatrix& transform_to_base_link)
    {
        // Make sure the GPS data is usable
        bool good_gps = (msg->status.status != sensor_msgs::msg::NavSatStatus__STATUS_NO_FIX &&
                            !std::isnan(msg->altitude) &&
                            !std::isnan(msg->latitude) &&
                            !std::isnan(msg->longitude));

        if(!is_initialized() || !good_gps) return false; // the filter is not yet initialized or bad gps reading.
        if(!m_gps_sensors_hmap[topic_name].ready())
        {
            // find one initialized imu and take its R_map_enu
            for(auto imu:m_imu_sensors_hmap)
            {
                if(imu.second.ready())
                {
                    auto R_map_enu = imu.second.get_R_map_enu();
                    m_gps_sensors_hmap[topic_name].initialize(get_state(), R_map_enu, msg->latitude, msg->longitude, msg->altitude, transform_to_base_link);
                    return false; // didn't went through but just initialized.
                }
            }
            return false; // no imu initialized yet so we have to wait.
        }
       MeasurementPtr m = m_gps_sensors_hmap[topic_name].gps_callback(get_state(), msg, transform_to_base_link);
       handle_measurement(m);
    }

    /**
     * @brief FilterWrapper: this function differentiates the data_triggered from the time_triggered option.
     *        it calls process_measurement immediately if data triggered and puts the measurement in the buffer otherwise .
     * @param[in] measurement - measurement to be handled
     * @return true if handling was sucessful
     */
    bool handle_measurement(MeasurementPtr measurement)
    {
        // TO_DO: this function differentiates the data_triggered and time_triggered option
        // it calls process_measurement imidiately if data triggered and otherwise puts the measurement in the buffer.
        if (m_data_triggered)
        {
            if (process_measurement(measurement))
            {
                publish_state();
                return true;
            }
        }
        else
        {
            //TO_DO: initialize the buffer
            m_time_triggered_buffer.enqueue_measurement(measurement);
        }
        return true;
    }

    /**
     * @brief FilterWrapper: Processes a measurement by feeding it to the filter's temporal & observation update.
     *        It initializes the time keeper and filter with the first measurement if they are not initialized.
     * @param[in] measurement - measurement to be processed
     * @return true if processing was sucessful( either filtered or reseted)
     */
    bool process_measurement(MeasurementPtr measurement)
    {
        DEBUG_W("\n\t\t--------------- Wrapper Process_Measurement: IN -------------------\n");
        // Get global time
        // tTime time_now = m_wall_time.now();
        tTime time_now = get_time_now();

        if (!is_initialized()) {
            DEBUG_W("Have to initialize!\n");
            DEBUG_W(std::fixed << std::setprecision(4) << " -> Innovation:  " << measurement->innovation.transpose() << "\n");
            DEBUG_W(std::fixed << std::setprecision(4) << " -> Measurement: " << measurement->z.transpose() << "\n");
            // TO_DO: this is not strictly correct, but should be good enough. If we get an observation
            // and the filter is not set to any state, we reset it.
            // We only consider the parts that are allowed by update_vector
            // TO_DO: any other way? -> If a part of the state is 0 and we get a measurement corresponding to eat we initialize
            // it with the measurement and the coresponding diagonal element with the variance of the measurement

            // Initialize the filter with the first measurement
            reset(*measurement, time_now);
            DEBUG_W("Reseting timer with: Meas time: " << measurement->m_time_stamp << " Time now:" << time_now <<"\n");
            return true;
        }
        else
        {
            DEBUG_W(std::fixed << std::setprecision(4) << " -> State:       " << get_state().transpose() << "\n");
            // 1. temporal update
            // auto dt = m_time_keeper.time_since_last_temporal_update(time_now);
            auto dt = m_time_keeper.time_since_last_update(measurement->m_time_stamp);
            DEBUG_W("\n--------------- Wrapper: Temporal update, dt = "<< dt << ", t = " << m_time_keeper.to_global_time(measurement->m_time_stamp)  <<" ---------------\n");
            DEBUG_W("\n--------------- Wrapper: Temporal update, now = "<< time_now << ", stamp = " << measurement->m_time_stamp  <<" ---------------\n");
            temporal_update(dt);

            // 2. observation update
            observation_update(measurement, time_now);
        }

        DEBUG_W("\t\t--------------- Wrapper Process_Measurement: OUT -------------------\n");
        return true;
    }

    bool temporal_update(tTime dt)
    {
        bool ret_val = false;
        // 1. temporal update
        if (dt < 0)
        {
            DEBUG_W("\n--------------- DELAYED MEASURMENT!! ---------------\n");
            ret_val = false;
        }
        else if (m_filter.temporal_update(dt))
        {
            m_time_keeper.update_after_temporal_update(dt);
            // DEBUG_W(" -> Covar temp: \n");
            // DEBUG_W(std::fixed << std::setprecision(4) << get_covariance() << "\n");
            DEBUG_W(std::fixed << std::setprecision(4) << " -> State temp: " << get_state().transpose() << "\n");
            ret_val = true;
        }
        else DEBUG_W("TEMPORAL UPDATE DIDNT HAPPEN\n");

        return ret_val;
    }

    bool observation_update(MeasurementPtr measurement, tTime time_now = 0)
    {
        bool ret_val = false;
        if (m_filter.observation_update(*measurement))
        {
            m_time_keeper.update_with_measurement(measurement->m_time_stamp, time_now);

            DEBUG_W("\n--------------- Wrapper: Observation update! ---------------\n");
            DEBUG_W(std::fixed << std::setprecision(4) << " -> Innovation:  " << measurement->innovation.transpose() << "\n");
            DEBUG_W(std::fixed << std::setprecision(4) << " -> Measurement: " << measurement->z.transpose() << "\n");
            DEBUG_W(std::fixed << std::setprecision(4) << " -> State obsv: " << get_state().transpose() << "\n");
            DEBUG_W(" -> Covar obsv: \n");
            DEBUG_W(std::fixed << std::setprecision(4) << get_covariance() << "\n");
            ret_val = true;
        }
        else DEBUG_W(" Mahalanobis failed\n");
        return ret_val;
    }

    /**
     * @brief FilterWrapper: Getter function for the state estimation
     * @return the state estimation vector
     */
    inline const StateVector get_state() const
    {
        return m_filter.get_state();
    }

    /**
     * @brief FilterWrapper: Getter function for the state estimation
     * @return the state estimation vector
     */
    inline StateCovTimePtr get_state_ptr()
    {
        return StateCovTimePtr(new StateCovTime(get_last_measurement_time(), get_state(), get_covariance()));
    }

    /**
     * @brief FilterWrapper: Getter function for the covariance estimation
     * @return the covariance estimation matrix
     */
    inline const StateMatrix get_covariance() const
    {
        return m_filter.get_covariance();
    }

    /**
     * @brief FilterWrapper: Checker function to see if everything is initialized.
     * @return true if everything is initialized properly
     */
    inline bool is_initialized() const
    {
        return (m_filter.is_initialized() && m_time_keeper.is_initialized());
    }

    /**
     * @brief FilterWrapper: Resets the time keeper and the filter
     * @param[in] measurement_vector - measurement vector to be used for initializing the filter's state
     * @param[in] mapping_matrix - state to measurement mapping
     * @param[in] time_now - global time of the measurement(from wall clock)
     * @param[in] time_stamp - time stamp of the measurement
     * @return true if reseting was successful
     */
    bool reset(const Measurement& measurement, tTime time_now)
    {
        // 1. reset filter
        StateVector x0;
        x0.setZero();
        // could use update_indeces to avoid matrix multiplication but since it only
        // happens once it not that big of a deal
        x0 += measurement.H.transpose() * measurement.z;

        // extract the part of init_estimation and process_noise covariances you need
        StateMatrix init_cov = StateMatrix::Identity()*1e-4;

        // init_cov.setIdentity();
        StateMatrix process_noise;
        process_noise.setIdentity();

        for (auto i:States::full_state_to_estimated_state)
        {
            if(i < STATE_SIZE) // check if this perticular state is not being estimated
            {
                for (auto j:States::full_state_to_estimated_state)
                {
                    if(j < STATE_SIZE) // check if this perticular state is not being estimated
                    {
                        init_cov(i, j) = m_config.m_init_estimation_covariance(i,j);
                        process_noise(i, j) = m_config.m_process_noise(i,j);
                    }
                }
            }
        }
        bool huge_noise = false;
        for ( uint i = 0; i < measurement.m_update_indices.size(); i++)
        {
            if(i < STATE_SIZE)
            {
                for ( uint j = 0; j < measurement.m_update_indices.size(); j++)
                {
                    if(j < STATE_SIZE)
                    {
                        init_cov(States::full_state_to_estimated_state[measurement.m_update_indices[i]], States::full_state_to_estimated_state[measurement.m_update_indices[j]]) = measurement.R(i,j);
                        if(measurement.R(i,j) > 60) huge_noise = true;
                    }
                }
            }
        }


        DEBUG_W("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n");
        if (huge_noise) DEBUG_W("VERY BIG INITIAL ESTIMATION COVARIANCE\n");
        DEBUG_W("RESETING\n" << "State\n" << x0.transpose() << std::setprecision(9) << "\nInit Covariance:\n" << init_cov << "\nProcess Noise\n" << process_noise << std::setprecision(4));
        DEBUG_W("\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
        m_filter.reset(x0, init_cov, process_noise);

        // 2. reset timekeeper
        m_time_keeper.reset(time_now, measurement.m_time_stamp);
        return true;
    }

    /**
     * @brief FilterWrapper: Function that creates an Odometry msg with data from the estimated state.
     * @return Odometry msg from the state. Not known values are set to 0
     */
    nav_msgs::msg::Odometry get_state_odom(bool debug=true)
    {
        const StateVector &state = m_filter.get_state();
        const StateMatrix &cov_mat = m_filter.get_covariance();
        nav_msgs::msg::Odometry msg;
        uint ix, iy, iz;

        // 1. Position
        ix = States::full_state_to_estimated_state[STATE_X];
        iy = States::full_state_to_estimated_state[STATE_Y];
        iz = States::full_state_to_estimated_state[STATE_Z];
        msg.pose.pose.position.x = ix < 15 ? m_filter.at(ix) : 0.0;
        msg.pose.pose.position.y = iy < 15 ? m_filter.at(iy) : 0.0;
        msg.pose.pose.position.z = iz < 15 ? m_filter.at(iz) : 0.0;

        // 2. Orientation
        ix = States::full_state_to_estimated_state[STATE_ROLL];
        iy = States::full_state_to_estimated_state[STATE_PITCH];
        iz = States::full_state_to_estimated_state[STATE_YAW];
        // EULER
        T roll =  ix < 15 ? m_filter.at(ix) : 0.0;
        T pitch = iy < 15 ? m_filter.at(iy) : 0.0;
        T yaw =   iz < 15 ? m_filter.at(iz) : 0.0;
        // euler to quaternion
        QuaternionT qq;
        qq = euler::get_quat_rpy(roll, pitch, yaw).normalized();
        msg.pose.pose.orientation.x = qq.x();
        msg.pose.pose.orientation.y = qq.y();
        msg.pose.pose.orientation.z = qq.z();
        msg.pose.pose.orientation.w = qq.w();

        // 3. Linear Twist
        ix = States::full_state_to_estimated_state[STATE_V_X];
        iy = States::full_state_to_estimated_state[STATE_V_Y];
        iz = States::full_state_to_estimated_state[STATE_V_Z];
        msg.twist.twist.linear.x = ix < 15 ? m_filter.at(ix) : 0.0;
        msg.twist.twist.linear.y = iy < 15 ? m_filter.at(iy) : 0.0;
        msg.twist.twist.linear.z = iz < 15 ? m_filter.at(iz) : 0.0;

        // 4. Angular Twist
        ix = States::full_state_to_estimated_state[STATE_V_ROLL];
        iy = States::full_state_to_estimated_state[STATE_V_PITCH];
        iz = States::full_state_to_estimated_state[STATE_V_YAW];
        msg.twist.twist.angular.x = ix < 15 ? m_filter.at(ix) : 0.0;
        msg.twist.twist.angular.y = iy < 15 ? m_filter.at(iy) : 0.0;
        msg.twist.twist.angular.z = iz < 15 ? m_filter.at(iz) : 0.0;

        // 6. Pose Covariance
        for (int i = 0; i < POSE_SIZE; i++)
        {
            ix = States::full_state_to_estimated_state[i];
            for(int j = 0; j < POSE_SIZE; j++)
            {
                iy = States::full_state_to_estimated_state[j];
                if ( iy > 14 || ix > 14)
                {
                     msg.pose.covariance[i + j*POSE_SIZE] = 1e-9;
                }
                else
                msg.pose.covariance[i + j*POSE_SIZE] = cov_mat(ix , iy);
            }
        }

        // 7. Twist Covariance
        for (int i = 0; i < TWIST_SIZE; i++)
        {
            ix = States::full_state_to_estimated_state[i+POSE_SIZE];
            for(int j = 0; j < TWIST_SIZE; j++)
            {
                iy = States::full_state_to_estimated_state[j+POSE_SIZE];
                if ( iy > 14 || ix > 14)
                msg.twist.covariance[i + j*TWIST_SIZE] = 1e-9;
                else
                msg.twist.covariance[i + j*TWIST_SIZE] = cov_mat(ix , iy);
            }
        }

        // 8. Debugg information
        if(!debug) return msg;
        DEBUG_W("\n******************** State msg to be published: ********************\n");
        DEBUG_W("orinetation-> roll: " << roll << ", pitch: " << pitch << ", yaw: " << yaw << "\n");
        DEBUG_W("pose: "
                << msg.pose.pose.position.x << " "
                << msg.pose.pose.position.y << " "
                << msg.pose.pose.position.z << " "
                << msg.pose.pose.orientation.x << " "
                << msg.pose.pose.orientation.y << " "
                << msg.pose.pose.orientation.z << " "
                << msg.pose.pose.orientation.w << " \n");

        DEBUG_W("twist: "
                << msg.twist.twist.linear.x << " "
                << msg.twist.twist.linear.y << " "
                << msg.twist.twist.linear.z << " "
                << msg.twist.twist.angular.x << " "
                << msg.twist.twist.angular.y << " "
                << msg.twist.twist.angular.z << " \n");

        DEBUG_W("pose cov:\n" << std::fixed << std::setprecision(4) << utilities::printtt(msg.pose.covariance, POSE_SIZE, POSE_SIZE));
        DEBUG_W("twist cov:\n" << std::fixed << std::setprecision(4) << utilities::printtt(msg.twist.covariance, TWIST_SIZE, TWIST_SIZE));
        DEBUG_W("********************************************************************\n\n");

        return msg;
    }

    /**
     * @brief FilterWrapper: Function that returns the latest timestamp the state changed
     * @return Latest timestamp the state changed which is the
     * max( temporal_update_timestamp, observation_update_timestamp).
     */
    tTime get_last_measurement_time()
    {
        return m_time_keeper.latest_timestamp();
    }

    /**
     * @brief FilterWrapper: Function that inizializes configuration related parameters of the class.
     * @return config_path - path to .json configuration file
     */
    void configure(const char* config_path)
    {
        std::cout << "CONFIG: " << config_path << "\n";
        m_config = FilterConfig_(config_path);
        m_data_triggered = m_config.m_data_triggered;
        for (auto x: m_config.m_sensor_configs)
        {
            std::cout<<x.first<<" ";
            switch(x.second.m_type) {
            case 0:
                std::cout << " ~~ m_update_vector: " << utilities::printtt(x.second.m_update_vector, 1, STATE_SIZE) << "\n";
                m_odom_sensors_hmap.emplace(x.first,
                    OdomT(x.first, x.second.m_update_vector, x.second.m_mahal_thresh, &m_debug_stream, m_debug));
              break;
            case 1:
              // code block
              break;
            case 2:
              // code block
              break;
            case 3:
                std::cout << " ~~ m_update_vector: " << utilities::printtt(x.second.m_update_vector, 1, STATE_SIZE) << "\n";
                m_imu_sensors_hmap.emplace(x.first,
                    ImuT(x.first, x.second.m_update_vector, x.second.m_mahal_thresh, &m_debug_stream, m_debug, x.second.m_remove_gravity));
              break;
            case 4:
                std::cout << " ~~ m_update_vector: " << utilities::printtt(x.second.m_update_vector, 1, STATE_SIZE) << "\n";
                m_gps_sensors_hmap.emplace(x.first,
                    GpsT(x.first, x.second.m_update_vector, x.second.m_mahal_thresh, &m_debug_stream, m_debug));
              break;
            default:
                // code block
                return;
            }
        }
        std::cout <<"\n";
    }

};

// explicit template initialization
using FilterCtrvEKF2D = FilterWrapper<Ctrv_EKF2D>;
using FilterCtraEKF2D = FilterWrapper<Ctra_EKF2D>;
using FilterCtraEKF3D = FilterWrapper<Ctra_EKF3D>;

} // end namespace filter
} // end namespace state_predictor
} // end namespace iav