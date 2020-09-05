
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

    using States = typename FilterT::States;
    using StateVector = typename FilterT::StateVector;
    using StateMatrix = typename FilterT::StateMatrix;
    using Vector = typename FilterT::Vector;
    using Matrix = typename FilterT::Matrix;

    using Matrix4T = typename Eigen::Matrix<T, 4, 4>;
    using Matrix3T = typename Eigen::Matrix<T, 3, 3>;
    using Quaternion = typename Eigen::Quaternion<T>;
    using MeasurementMatrix = typename Eigen::Matrix<T, num_state, -1>;
    using TransformationMatrix = typename Eigen::Transform<T, 3, Eigen::TransformTraits::Isometry>;


private:
    FilterT m_filter;
    MeasurementTimeKeeper m_time_keeper;
    FilterConfig_ m_config;

public:
    FilterWrapper(const char* config_path)
    {
        configure(config_path);
    }

    // from array to Matrix, used when reading from msg
    void copy_covariance(Matrix& destination, T* source, uint dimension)
    {
        for (uint i = 0; i < dimension; i++)
        {
            for (uint j = 0; j < dimension; j++)
            {
                destination(i,j) = source[i*dimension + j];
            }
        }
    }
    
    // from Matrix to array, used to create msg
    void copy_covariance(T* destination, const Matrix& source, uint dimension)
    {
        for (uint i = 0; i < dimension; i++)
        {
            for (uint j = 0; j < dimension; j++)
            {
                destination[i*dimension + j](i,j) = source(i,j);
            }
        }
    }

    void pose_callback(nav_msgs::msg::Odometry* msg, TransformationMatrix transform)
    {
        int* update_vector = m_config.m_sensor_configs[msg->header.frame_id];

        // consider the update_vector
        std::vector<size_t> updateIndices_t;
        for (size_t i = 0; i < STATE_SIZE; ++i)
        {
            if (update_vector[i])
            {
                updateIndices_t.push_back(i);
            }
        }
        size_t updateSize_t = updateIndices_t.size();

        // consider only parts of measurement that are present on our state
        size_t updateSize = 0U;
        for (uint i = 0; i < updateSize_t; i++)
        {
            if(States::full_state_to_estimated_state[updateIndices_t[i]] < STATE_SIZE) 
                updateIndices.push_back(updateIndices_t[i]);
        }
        size_t updateSize = updateIndices.size();


        // transform orientation in a usefull form
        Quaternion orientation;
        // Handle bad (empty) quaternions and normalize
        if (msg->pose.pose.orientation.x == 0 && msg->pose.pose.orientation.y == 0 &&
            msg->pose.pose.orientation.z == 0 && msg->pose.pose.orientation.w == 0)
        {
            orientation = {1.0, 0.0, 0.0, 0.0};
        }
        else
        {
            orientation = {msg->pose.pose.orientation.w,
                           msg->pose.pose.orientation.x,
                           msg->pose.pose.orientation.y,
                           msg->pose.pose.orientation.z};

            if (orientation.norm()-1.0 > 0.01)
            {
                orientation.normalize();
            }
        }

        // 1. Rotate Pose
        // create pose transformation matrix which saves  |R R R T|
        // the orientation in form of a (R)otation-matrix |R R R T|
        // and position as (T)ranslation-vector.          |R R R T|
        //                                                |0 0 0 1|
        // consider update_vector
        Matrix4T pose_transf;
        pose_transf.block<3,3>(0,0) = 
        orientation.toRotationMatrix().array().rowwise() * Vector3d(
            {update_vector[STATE_ROLL]
             update_vector[STATE_PITCH]
             update_vector[STATE_YAW]}).transpose().array(); // consider update_vec for the orientation

        pose_transf.block<3,1>(0,3) = Eigen::Vector3d{
            msg->pose.pose.position.x * update_vector[STATE_X],
            msg->pose.pose.position.y * update_vector[STATE_Y],
            msg->pose.pose.position.z * update_vector[STATE_Z]};

        pose_transf = transform * pose_transf;

        Vector measurement(6);
        measurement.block<3>(0) = pose_transf.block<3,1>(0,3);
        measurement.block<3>(3) = pose_transf.block<3,3>(0,0).eulerAngles(0,1,2);

        // 2. Rotate Covariance
        Matrix rot6d(6,6);
        rot6d.setIdentity();
        rot6d.block<3,3>(0,0) = transform.rotation();
        rot6d.block<3,3>(3,3) = transform.rotation();

        Matrix covariance(6,6);
        covariance.setZero();
        covariance = rot6d * covariance * rot6d.transpose();

        // 3. Rotate update_vector --> like in robot localization
        
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



        Vector sub_measurement(updateSize);
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