
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

#include<vector>
#include<eigen/Eigen>

#include<utilities/filter_utilities.h>
#include<filter/filter_config.h>
#include<filter/filter_ekf.h>
#include<measurement/measurement_time_keeper.h>
#include<measurement/measurement.h>

#include<geometry_msgs/msg/PoseWithCovariance.h>
#include<geometry_msgs/msg/TwistWithCovariance.h>
#include<nav_msgs/msg/Odometry.h>
#include<sensor_msgs/msg/Imu.h>


namespace iav{ namespace state_predictor { namespace filter {

template<class FilterT, int num_state, typename T = double>
class FilterWrapper
{
public:
    using Measurement= typename measurement::Measurement<3,T>;
    using MeasurementTimeKeeper = typename measurement::MeasurementTimeKeeper;
    using FilterConfig_ = FilterConfig<num_state>;
    using StateVector = typename FilterT::StateVector;
    using StateMatrix = typename FilterT::StateMatrix;
    using Vector = typename FilterT::Vector;
    using Matrix = typename FilterT::Matrix;
    using MeasurementMatrix = typename Eigen::Matrix<T, num_state, -1>;
    using TransformationMatrix = typename Eigen::Matrix<T, 4, 4>;

private:
    FilterT m_filter;
    MeasurementTimeKeeper m_time_keeper;
    FilterConfig_ m_config;

public:
    FilterWrapper(const char* config_path)
    {
        configure(config_path);
    }

    
    void pose_callback(geometry_msgs::msg::PoseWithCovariance msg, Eigen::Isometry<T> transform)
    {
        Vector pose(POSITION_SIZE);
        pose << msg.pose.position.x, msg.pose.position.y
                msg.pose.position.z, msg.pose.orientation.x;
        Vector orientation_quaternion(4) << 
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w;
    }

    bool handle_measurement(Measurement measurement)
    {
        // 1.
        // Get update_vector and mahalanobis_threshold and noise convariance from m_config

        //2.
        // Transform, reduce measurements in unified format

        //3.
        // call temporial & observation update
        
        
        // TO_DO: this function differentiates the data_triggered and time_triggered option
        // it calls process_measurement imidiately if data triggered and otherwise puts the measurement in the buffer.

    }

    bool process_measurement(Measurement measurement)
    {
        if (!is_initialized()) {
            // TO_DO: this is not strictly correct, but should be good enough. If we get an observation
            // and the filter is not set to any state, we reset it.
            // We only consider the parts that are allowed by update_vector

            // tTime global_time_of_message_received = 1; // TO_DO: get global time
            // return reset(measurement, global_time_of_message_received);
        }

        return true;
    }

    bool temporal_update(const tTime& delta_t)
    {
        if (!is_initialized()) { return false};
        m_filter.tempoal_update(delta_t);
        m_time_keeper.update_after_temporal_update(delta_t);
        return true;
    }

    bool observation_update(Vector z, MeasurementMatrix H, MeasurementMatrix R, T mahalanobis_threshold)
    {
        if (!is_initialized())
        {
            tTime time_now = 12.0;
            reset(z, H, time_now);
            return true
        };
        m_filter.observation_update(z, H, R, mahalanobis_threshold);
        return true;
    }

    inline StateVector get_state() const
    {
        return m_filter.get_state();
    }

    inline StateMatrix get_covariance() const
    {
        return m_filter.get_covariance();
    }

    bool is_initialized() const
    {
        return (m_filter.is_initialized() && m_time_keeper.is_initialized());
    }

    bool reset(Vector measurement_vector, MeasurementMatrix mapping_matrix, tTime init_time)
    {
        //TO_DO: check if the measurement is stateful
        // either odometry or pose
        // Maybe need to template it according to the motionmodel used
        return true;
    }

    void configure(const char* config_path)
    {
        m_config = FilterConfig_(config_path);
    }

};

using FilterCtrvEKF2D = FilterWrapper<Ctrv_EKF2D, 8, double>;

} // end namespace filter 
} // end namespace state_predictor
} // end namespace iav