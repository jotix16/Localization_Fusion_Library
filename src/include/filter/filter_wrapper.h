
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
#include <filter/filter_config.h>
#include <filter/filter_ekf.h>
#include <measurement/measurement_time_keeper.h>
#include <measurement/measurement.h>
#include <sensors/odom.h>
#include <sensors/imu.h>

#include <geometry_msgs/msg/Vector3.h>
#include <geometry_msgs/msg/PoseWithCovariance.h>
#include <geometry_msgs/msg/TwistWithCovariance.h>
#include <nav_msgs/msg/Odometry.h>
#include <sensor_msgs/msg/Imu.h>
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
    using States        = typename FilterT::States;
    using StateVector   = typename FilterT::StateVector;
    using StateMatrix   = typename FilterT::StateMatrix;
    using OdomT = typename sensors::Odom<T,States>;
    using ImuT  = typename sensors::Imu<T,States>;

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

    // debuging variables
    std::ofstream m_debug_stream;
    bool m_debug;
    std::mutex m_callback_mutex; 

public:
    FilterWrapper() = default;
    
    /**
     * @brief Constructor that inizializes configuration related parameters and time-keeping
     * @param[in] config_path - path to .json configuration file
     */
    FilterWrapper(const char* config_path)
    {
        reset_config(config_path);
    }

    /**
     * @brief FilterWrapper: Function that inizializes configuration related parameters and time-keeping
     * @param[in] config_path - path to .json configuration file
     */
    void reset_config(const char* config_path)
    {
        configure(config_path);
        m_wall_time = Clock();
        m_time_keeper = MeasurementTimeKeeper();
        m_debug = true;
        create_debug();
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
     * @brief FilterNode: Callback for receiving all odom msgs. It processes all comming messages
     * by considering transforming them in the fusing frame, considering the update_vector to ignore parts
     * of the measurements and capturing matrixes with faulty values.
     * @param[in] msg - pointer to the odom msg of the measurement
     * @param[in] transform_to_map - transf from sensor frame of msg to map frame where pose is fused
     * @param[in] transform_to_base_link - transf from sensor frame of msg to base_link frame where twist is fused
     */
    
    bool odom_callback(
        const std::string& topic_name,
        nav_msgs::msg::Odometry* msg,
        const TransformationMatrix& transform_to_map,
        const TransformationMatrix& transform_to_base_link)
    {
       Measurement m = m_odom_sensors_hmap[topic_name].odom_callback(get_state(), msg, transform_to_map, transform_to_base_link);
       handle_measurement(m);
    }

    /**
     * @brief FilterNode: Callback for receiving all odom msgs. It processes all comming messages
     * by considering transforming them in the fusing frame, considering the update_vector to ignore parts
     * of the measurements and capturing matrixes with faulty values.
     * @param[in] msg - pointer to the imu msg of the measurement
     * @param[in] transform_to_world - transf from sensor frame of msg to world frame where orientation is fused
     * @param[in] transform_to_base_link - transf from sensor frame to base_link frame where angular velocity and acceleration are fused
     */
    bool imu_callback(
        const std::string& topic_name,
        sensor_msgs::msg::Imu* msg,
        const TransformationMatrix& transform_to_world,
        const TransformationMatrix& transform_to_base_link)
    {
       Measurement m = m_imu_sensors_hmap[topic_name].imu_callback(get_state(), msg, transform_to_world, transform_to_base_link);
       handle_measurement(m);
    }

    /**
     * @brief FilterWrapper: this function differentiates the data_triggered and time_triggered option.
     *        it calls process_measurement immediately if data triggered and puts the measurement in the buffer otherwise .
     * @param[in] measurement - measurement to be handled
     * @return true if handling was sucessful
     */
    bool handle_measurement(Measurement& measurement)
    {
        // TO_DO: this function differentiates the data_triggered and time_triggered option
        // it calls process_measurement imidiately if data triggered and otherwise puts the measurement in the buffer.
        bool data_triggered = true;
        if (data_triggered)
        {
            return process_measurement(measurement);
        }
        return true;
    }

    /**
     * @brief FilterWrapper: Processes a measurement by feeding it to the filter's temporal & observation update.
     *        It initializes the time keeper and filter with the first measurement if they are not initialized.
     * @param[in] measurement - measurement to be processed
     * @return true if processing was sucessful
     */
    bool process_measurement(Measurement& measurement)
    {
        DEBUG_W("\n\t\t--------------- Wrapper Process_Measurement: IN -------------------\n");
        // Get global time
        tTime time_now = m_wall_time.now();

        if (!is_initialized()) {
            DEBUG_W("\n");
            DEBUG_W(std::fixed << std::setprecision(4) << " -> Innovation:  " << measurement.innovation.transpose() << "\n");
            DEBUG_W(std::fixed << std::setprecision(4) << " -> Measurement: " << measurement.z.transpose() << "\n");
            // TO_DO: this is not strictly correct, but should be good enough. If we get an observation
            // and the filter is not set to any state, we reset it.
            // We only consider the parts that are allowed by update_vector
            // TO_DO: any other way?

            // Initialize the filter with the first measurement
            reset(measurement, time_now);
            return false;
        }
        else
        {
            DEBUG_W(std::fixed << std::setprecision(4) << " -> State:       " << get_state().transpose() << "\n");
            // 1. temporal update
            auto dt = m_time_keeper.time_since_last_temporal_update(time_now);
            DEBUG_W("\n--------------- Wrapper: Temporal update, dt = "<< dt << " ---------------\n");
            if (dt < 0) return false;
            // std::lock_guard<std::mutex> guard(m_callback_mutex);
            if (m_filter.temporal_update(dt))
            {
                m_time_keeper.update_after_temporal_update(dt);
                
                DEBUG_W(" -> Covar temp: \n");
                DEBUG_W(std::fixed << std::setprecision(4) << get_covariance() << "\n");
                DEBUG_W(std::fixed << std::setprecision(4) << " -> State temp: " << get_state().transpose() << "\n");
            }
            // 2. observation update
            if (m_filter.observation_update(measurement))
            {
                m_time_keeper.update_with_measurement(measurement.m_time_stamp, time_now);

                DEBUG_W("\n--------------- Wrapper: Observation update! ---------------\n");
                DEBUG_W(std::fixed << std::setprecision(4) << " -> Innovation:  " << measurement.innovation.transpose() << "\n");
                DEBUG_W(std::fixed << std::setprecision(4) << " -> Measurement: " << measurement.z.transpose() << "\n");
                DEBUG_W(std::fixed << std::setprecision(4) << " -> State obsv: " << get_state().transpose() << "\n");
                DEBUG_W(" -> Covar obsv: \n");
                DEBUG_W(std::fixed << std::setprecision(4) << get_covariance() << "\n");
            }
            else DEBUG_W(" Mahalanobis failed\n");
        }

        DEBUG_W("\t\t--------------- Wrapper Process_Measurement: OUT -------------------\n");
        return true;
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
        //TO_DO: check if the measurement is stateful?
        // either odometry or pose

        // 1. reset filter
        StateVector x0;
        x0.setZero();
        // could use update_indeces to avoid matrix multiplication but since it only
        // happens once it not that big of a deal
        x0 += measurement.H.transpose() * measurement.z;

        // extract the part of init_estimation and process_noise covariances you need
        StateMatrix init_cov;
        init_cov.setIdentity();
        StateMatrix process_noise;
        process_noise.setIdentity();
        uint ind_temp1 = 0;
        uint ind_temp2 = 0;
        DEBUG_W("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<\n");
        DEBUG_W("Initial State Covariance\n"<< std::setprecision(9));
        for (auto i:States::full_state_to_estimated_state)
        {
            
            if(i < STATE_SIZE)
            {
                ind_temp2 = 0;
                for (auto j:States::full_state_to_estimated_state)
                {
                    if(j < STATE_SIZE)
                    {
                        init_cov(i, j) = m_config.m_init_estimation_covariance(ind_temp1,ind_temp2);
                        DEBUG_W(m_config.m_init_estimation_covariance(ind_temp1,ind_temp2) << " ");
                        process_noise(i, j) = m_config.m_process_noise(ind_temp1,ind_temp2);
                    }
                    ++ind_temp2;
                }
                DEBUG_W("\n");
            }
            ++ind_temp1;
        }
        DEBUG_W("RESETING\n" << "State\n" << x0.transpose() <<"\nInit Covariance:\n" << init_cov << "\nProcess Noise\n" << process_noise);
        DEBUG_W("\n>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>\n");
        m_filter.reset(x0, init_cov, process_noise);

        // 2. reset timekeeper
        m_time_keeper.reset(time_now, measurement.m_time_stamp);
        return true;
    }

    /**
     * @brief FilterWrapper: Function that inizializes configuration related parameters of the class.
     * @param[out] config_path - path to .json configuration file
     */
    void configure(const char* config_path)
    {
        m_config = FilterConfig_(config_path);
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
              m_imu_sensors_hmap.emplace(x.first,
                    ImuT(x.first, x.second.m_update_vector, x.second.m_mahal_thresh, &m_debug_stream, m_debug));
              break;
            default:
                // code block
                return;
            }
        }
        std::cout <<"\n";
    }

    /**
     * @brief FilterWrapper: Function that creates an Odometry msg with data from the estimated state.
     * @return Odometry msg from the state. Not known values are set to 0
     */
    nav_msgs::msg::Odometry get_state_odom()
    {
        const StateVector &state = m_filter.get_state();
        const StateMatrix &cov_mat = m_filter.get_covariance();
        nav_msgs::msg::Odometry msg;
        uint ix, iy, iz;

        // 1. Posisiton
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
        T roll =  ix < 15 ? m_filter.at(ix) : 0.0;
        T pitch = iy < 15 ? m_filter.at(iy) : 0.0;
        T yaw =   iz < 15 ? m_filter.at(iz) : 0.0;
        // euler to quaternion
        QuaternionT qq;
        qq = AngleAxisT(roll, Vector3T::UnitX())
        * AngleAxisT(pitch, Vector3T::UnitY())
        * AngleAxisT(yaw, Vector3T::UnitZ());
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
        DEBUG_W("\n******************** State msg to be published: ********************\n")
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
        DEBUG_W("********************************************************************\n\n")

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

};

// explicit template initialization
using FilterCtrvEKF2D = FilterWrapper<Ctrv_EKF2D>;
using FilterCtraEKF2D = FilterWrapper<Ctra_EKF2D>;
using FilterCtraEKF3D = FilterWrapper<Ctra_EKF3D>;

} // end namespace filter 
} // end namespace state_predictor
} // end namespace iav