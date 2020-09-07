
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
            // we dont ignore roll pitch yaw but rotations in certain directions in sensor frame
            orientation = {msg->pose.pose.orientation.w,
                           msg->pose.pose.orientation.x * update_vector[STATE_ROLL],
                           msg->pose.pose.orientation.y * update_vector[STATE_PITCH],
                           msg->pose.pose.orientation.z * update_vector[STATE_YAW]  };

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
        orientation.toRotationMatrix();
        // 
        // 
        // 
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

        // 3. Create a vector with same size as the measurement with 0 at components that are not in the state
        std::vector<size_t> updateIndices_t;
        for (size_t i = 0; i < POSE_SIZE; ++i)
        {
            if(States::full_state_to_estimated_state[i] < STATE_SIZE)
                updateIndices_t.push_back(i);
        }
        uint updateSize_t = updateIndices_t.size();
        
        // 4. fill sub_measurement vector and sub_covariance matrix and sub_inovation vector
        Vector sub_measurement(updateSize_t); // z
        Vector sub_innovation(updateSize_t);  // z'-z
        Matrix sub_covariance(updateSize_t, updateSize_t);
        for ( uint i = 0; i < updateSize_t; i++)
        {
            sub_measurement(i) = measurement(updateIndices_t[i]);
            sub_innovation(i) = m_filter.at(States::full_state_to_estimated_state[updateIndices_t[i]]) - measurement(updateIndices_t[i]);
            for (uint j = 0; j < updateSize_t; j++)
            {
                sub_covariance(i,j) = covariance(updateIndices_t[i], updateIndices_t[i]);
            }
        }

        // 5. Create state to measurement mapping and inovation
        // Matrix state_to_measurement_mapping(States::STATE_SIZE_M, updateSize_t);
        MeasurementMatrix state_to_measurement_mapping(updateSize_t);
        state_to_measurement_mapping.setZero();
        for (uint i = 0; i < updateSize_t; i++)
        {
            state_to_measurement_mapping(States::full_state_to_estimated_state[updateIndices_t[i]], updateIndices_t[i]);
        }

        // 6. Send measurement to be handled
        T mahalanobis_thresh = 0.4;
        tTime time_now = 0.2;
        MeasurementPtr meas = MeasurementPtr(new)
        handle_measurement(
            Measurement(time_now, sub_measurement, sub_covariance, sub_innovation, 
           state_to_measurement_mapping, msg->header.frame_id, mahalanobis_thresh))
    }

    bool handle_measurement(const Measurement& measurement)
    {
        // TO_DO: this function differentiates the data_triggered and time_triggered option
        // it calls process_measurement imidiately if data triggered and otherwise puts the measurement in the buffer.
        bool data_triggered = true;
        if (data_triggered)
        {
            process_measurement(measurement)
        }

    }

    bool process_measurement(const Measurement& measurement)
    {
        tTime time_now = 12.0;
        if (!is_initialized()) {
            // TO_DO: this is not strictly correct, but should be good enough. If we get an observation
            // and the filter is not set to any state, we reset it.
            // We only consider the parts that are allowed by update_vector

            // tTime global_time_of_message_received = 1; // TO_DO: get global time
            // return reset(measurement, global_time_of_message_received);
            reset(measurement.m_measurement_vector, measurement.m_state_to_measurement_mapping, time_now);
            return false;
        }
        else{

            // temporal update
            m_filter.tempoal_update(delta_t);
            m_time_keeper.update_after_temporal_update(delta_t);

            // observation update
            m_filter.observation_update(measurement.m_measurement_vector,measurement.m_innovation,
                    measurement.m_state_to_measurement_mapping, measurement.m_measurement_covariance,
                    measurement.m_mahalanobis_thresh);
                               //z, H, R, mahalanobis_threshold);
            m_time_keeper.update_with_measurement(measurement.m_time_stamp, time_now)
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

    bool observation_update(Vector z, Vector z_innovation, MeasurementMatrix H, Matrix R, T mahalanobis_threshold)
    {
        if (!is_initialized())
        {
            tTime time_now = 12.0;
            reset(z, H, time_now);
            return true
        };
        m_filter.observation_update(z, H, R, mahalanobis_threshold);
        tTime time_stamp = 0.1, global_time = 0.1;
    	m_time_keeper.update_with_measurement(time_stamp,  global_time)
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
        StateVector x0;
        x0 = mapping_matrix*measurement_vector;
        // x0.setZero();
        // x0 = mapping_matrix*measurement_vector;
        m_filter.reset(x0, m_config.init_estimate_covariance, m_config.process_noise);
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