
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
    using Measurement= typename measurement::Measurement<num_state,T>;
    using MeasurementTimeKeeper = typename measurement::MeasurementTimeKeeper;
    using FilterConfig_ = FilterConfig<num_state>;

    using States = typename FilterT::States;
    using StateVector = typename FilterT::StateVector;
    using StateMatrix = typename FilterT::StateMatrix;
    using Vector = typename FilterT::Vector;
    using Matrix = typename FilterT::Matrix;
    using MeasurementMatrix = typename FilterT::ObservationMatrix;

    using Matrix6T = typename Eigen::Matrix<T, 6, 6>;
    using Matrix4T = typename Eigen::Matrix<T, 4, 4>;
    using Matrix3T = typename Eigen::Matrix<T, 3, 3>;
    using Vector6T = typename Eigen::Matrix<T, 6, 1>;
    using Vector3T = typename Eigen::Matrix<T, 3, 1>;
    using Quaternion = typename Eigen::Quaternion<T>;
    using TransformationMatrix = typename Eigen::Transform<T, 3, Eigen::TransformTraits::Isometry>;


private:
    FilterT m_filter;
    MeasurementTimeKeeper m_time_keeper;
    FilterConfig_ m_config;
    Clock m_wall_time;

public:
    FilterWrapper(const char* config_path)
    {
        configure(config_path);
        m_wall_time = Clock();
        m_time_keeper = MeasurementTimeKeeper();
    }

    // from array to Matrix, used when reading from msg
    void copy_covariance(Matrix6T& destination, T* source, uint dimension)
    {
        for (uint i = 0; i < dimension; i++)
        {
            for (uint j = 0; j < dimension; j++)
            {
                destination(i,j) = source[i*dimension + j];
            }
        }
    }

    // from array to Matrix, used when reading from msg
    void copy_covariance(Matrix6T& destination, const std::array<double, 36>& source, uint dimension)
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
    void copy_covariance(T* destination, const Matrix6T& source, uint dimension)
    {
        for (uint i = 0; i < dimension; i++)
        {
            for (uint j = 0; j < dimension; j++)
            {
                destination[i*dimension + j](i,j) = source(i,j);
            }
        }
    }

    void odom_callback(
        nav_msgs::msg::Odometry* msg,
        const TransformationMatrix& transform_to_world,
        const TransformationMatrix& transform_to_base_link)
    {
        bool* update_vector = m_config.m_sensor_configs[msg->header.frame_id].m_update_vector;
        
        // 1. Create vector the same size as the submeasurement that will be used
        // - its elements are the corresponding parts of the state
        // - the size of this index-vector enables initializing of the submeasurement matrixes
        std::vector<uint> updateIndices_pose;
        for (uint i = 0; i < POSE_SIZE; ++i)
        {
            if(States::full_state_to_estimated_state[i] < STATE_SIZE)
                updateIndices_pose.push_back(i);
        }
        size_t updateSize_pose = updateIndices_pose.size();

        std::vector<uint> updateIndices_twist;
        for (uint i = 0; i < TWIST_SIZE; ++i)
        {
            if(States::full_state_to_estimated_state[i] < STATE_SIZE)
                updateIndices_twist.push_back(i);
        }
        size_t updateSize_twist = updateIndices_twist.size();

        // 2. Initialize submeasurement matrixes
        size_t updateSize = updateSize_pose + updateSize_twist; 
        Vector sub_measurement(updateSize); sub_measurement.setZero(); // z
        Vector sub_innovation(updateSize); sub_innovation.setZero(); // z'-z 
        Matrix sub_covariance(updateSize, updateSize);
        MeasurementMatrix state_to_measurement_mapping;
        state_to_measurement_mapping.resize(updateSize, States::STATE_SIZE_M);
        state_to_measurement_mapping.setZero();
        
        // 3. Fill the submeasurement matrixes
        prepare_pose(&msg->pose, transform_to_world, update_vector, sub_measurement,
                     sub_covariance, sub_innovation, state_to_measurement_mapping,
                     updateIndices_pose, 0, updateSize_pose);

        std::cout<<"Submeasurement with pose      : " << sub_measurement.transpose() << "\n";
        prepare_twist(&msg->twist, transform_to_base_link, update_vector, sub_measurement,
                      sub_covariance, sub_innovation, state_to_measurement_mapping,
                      updateIndices_pose, updateSize_pose, updateSize_twist);

        std::cout<<"Submeasurement with pose&twist: " << sub_measurement.transpose() << "\n";

        // 4. Send measurement to be handled
        // TO_DO: clarify how to determine the pose and twist mahalanobis thresholds
        T mahalanobis_thresh = 0.4;
        tTime stamp_sec = static_cast<tTime>(msg->header.stamp.nanosec/1000000000LL);
        Measurement meas(stamp_sec, sub_measurement, sub_covariance, sub_innovation, 
                         state_to_measurement_mapping, msg->header.frame_id, mahalanobis_thresh);
        handle_measurement(meas);
    }

    void prepare_twist(
        geometry_msgs::msg::TwistWithCovariance* msg, 
        const TransformationMatrix& transform,
        bool* update_vector,
        Vector& sub_measurement,
        Matrix& sub_covariance, 
        Vector& sub_innovation, 
        MeasurementMatrix& state_to_measurement_mapping,
        std::vector<uint>& updateIndices_t,
        size_t ix1, size_t updateSize_t)
    {

        // 1. Extract linear and angular velocities
        // - consider update_vector
        Vector3T linear_vel; 
        linear_vel <<
            msg->twist.linear.x * (int)update_vector[STATE_V_X],
            msg->twist.linear.y * (int)update_vector[STATE_V_Y],
            msg->twist.linear.z * (int)update_vector[STATE_V_Z];

        Vector3T angular_vel;
        angular_vel <<
            msg->twist.angular.x * (int)update_vector[STATE_V_ROLL],
            msg->twist.angular.y * (int)update_vector[STATE_V_PITCH],
            msg->twist.angular.z * (int)update_vector[STATE_V_YAW];

        // 2. Extract angular velocities from the estimated state
        // - needed to add the effect of angular velocities to the linear ones(look below).
        Vector3T angular_vel_state;
        uint vroll_ix = States::full_state_to_estimated_state[STATE_V_ROLL];
        uint vpitch_ix = States::full_state_to_estimated_state[STATE_V_PITCH];
        uint vyaw_ix = States::full_state_to_estimated_state[STATE_V_YAW];

        angular_vel_state(0) = vroll_ix < STATE_SIZE ? m_filter.at(vroll_ix) : 0;
        angular_vel_state(1) = vpitch_ix < STATE_SIZE ? m_filter.at(vpitch_ix) : 0;
        angular_vel_state(2) = vyaw_ix < STATE_SIZE ? m_filter.at(vyaw_ix) : 0;

        // 3. Transform measurement to fusion frame
        auto rot = transform.rotation();
        auto origin = transform.translation();
        linear_vel = rot * linear_vel + origin.cross(angular_vel_state); // add effects of angular velocitioes
                                                                         // v = rot*v + origin(x)w
        angular_vel = rot * angular_vel;

        // 4. Compute measurement vector
        Vector6T measurement;
        measurement.head<3>() = linear_vel;
        measurement.tail<3>() = angular_vel;

        // 5. Compute measurement covariance
        Matrix6T covariance;
        covariance.setIdentity();
        copy_covariance(covariance, msg->covariance, TWIST_SIZE);

        // 6. Rotate Covariance to fusion frame
        Matrix6T rot6d;
        rot6d.setIdentity();
        rot6d.block<3,3>(0,0) = rot;
        rot6d.block<3,3>(3,3) = rot;
        covariance = rot6d * covariance * rot6d.transpose();

        // 7. Fill sub_measurement vector and sub_covariance matrix and sub_inovation vector
        for ( uint i = 0; i < updateSize_t; i++)
        {
            sub_measurement(i + ix1) = measurement(updateIndices_t[i]);
            sub_innovation(i + ix1) = m_filter.at(States::full_state_to_estimated_state[updateIndices_t[i]]) - measurement(updateIndices_t[i]);
            for (uint j = 0; j < updateSize_t; j++)
            {
                sub_covariance(i + ix1, j + ix1) = covariance(updateIndices_t[i], updateIndices_t[j]);
            }
        }

        // 7. Fill state to measurement mapping and inovation
        for (uint i = 0; i < updateSize_t; i++)
        {
            if (States::full_state_to_estimated_state[updateIndices_t[i]] < STATE_SIZE)
            state_to_measurement_mapping(i + ix1, States::full_state_to_estimated_state[updateIndices_t[i]]) = 1.0;
        }
    }

    void prepare_pose(
        geometry_msgs::msg::PoseWithCovariance* msg, 
        const TransformationMatrix& transform,
        bool* update_vector,
        Vector& sub_measurement,
        Matrix& sub_covariance, 
        Vector& sub_innovation, 
        MeasurementMatrix& state_to_measurement_mapping,
        std::vector<uint>& updateIndices_t,
        uint ix1, size_t updateSize_t)
    {

        // 1. Write orientation in a useful form( Quaternion -> rotation matrix)
        // - Handle bad (empty) quaternions and normalize
        Quaternion orientation;
        if (msg->pose.orientation.x == 0 && msg->pose.orientation.y == 0 &&
            msg->pose.orientation.z == 0 && msg->pose.orientation.w == 0)
        {
            orientation = {1.0, 0.0, 0.0, 0.0};
        }
        else
        {
            // we dont ignore roll pitch yaw but rotations in certain directions in sensor frame
            orientation = {msg->pose.orientation.w,
                           msg->pose.orientation.x * (int)update_vector[STATE_ROLL],
                           msg->pose.orientation.y * (int)update_vector[STATE_PITCH],
                           msg->pose.orientation.z * (int)update_vector[STATE_YAW]  };

            if (orientation.norm()-1.0 > 0.01)
            {
                orientation.normalize();
            }
        }

        // 2. Write Pose as Transformation Matrix for easy transformations
        // - create pose transformation matrix which saves  |R R R T|
        // - the orientation in form of a (R)otation-matrix |R R R T|
        // - and position as (T)ranslation-vector.          |R R R T|
        //                                                  |0 0 0 1|
        // consider update_vector
        Matrix4T pose_transf;
        pose_transf.setIdentity();
        pose_transf.block<3,3>(0,0) = orientation.toRotationMatrix();
        pose_transf.block<3,1>(0,3) = Eigen::Vector3d{
            msg->pose.position.x * (int)update_vector[STATE_X],
            msg->pose.position.y * (int)update_vector[STATE_Y],
            msg->pose.position.z * (int)update_vector[STATE_Z]};

        // 3. Transform pose to fusion frame
        pose_transf = transform * pose_transf;

        // 4. Compute measurement vector
        Vector6T measurement;
        measurement.head<3>() = pose_transf.block<3,1>(0,3);
        measurement.tail<3>() = pose_transf.block<3,3>(0,0).eulerAngles(0,1,2);

        // 5. Rotate Covariance to fusion frame
        Matrix6T rot6d;
        rot6d.setIdentity();
        auto rot = transform.rotation();
        rot6d.block<3,3>(0,0) = rot;
        rot6d.block<3,3>(3,3) = rot;

        // 6. Compute measurement covariance
        Matrix6T covariance;
        covariance.setIdentity();
        copy_covariance(covariance, msg->covariance, POSE_SIZE);
        covariance = rot6d * covariance * rot6d.transpose();

        // 4. Fill sub_measurement vector and sub_covariance matrix and sub_inovation vector
        for ( uint i = 0; i < updateSize_t; i++)
        {
            sub_measurement(i + ix1) = measurement(updateIndices_t[i]);
            sub_innovation(i + ix1) = m_filter.at(States::full_state_to_estimated_state[updateIndices_t[i]]) - measurement(updateIndices_t[i]);
            for (uint j = 0; j < updateSize_t; j++)
            {
                sub_covariance(i + ix1, j + ix1) = covariance(updateIndices_t[i], updateIndices_t[j]);
            }
        }

        // 5. Fill state to measurement mapping and inovation
        for (uint i = 0; i < updateSize_t; i++)
        {
            if (States::full_state_to_estimated_state[updateIndices_t[i]]<15)
            state_to_measurement_mapping(i + ix1, States::full_state_to_estimated_state[updateIndices_t[i]]) = 1.0;
        }
    }

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

    bool process_measurement(Measurement& measurement)
    {
        // Get global time
        tTime time_now = m_wall_time.now();

        if (!is_initialized()) {
            // TO_DO: this is not strictly correct, but should be good enough. If we get an observation
            // and the filter is not set to any state, we reset it.
            // We only consider the parts that are allowed by update_vector

            // std::cout<<"Filter or timeMeasurement are not initialized. We are initializing!\n";
            reset(measurement.m_measurement_vector, measurement.m_state_to_measurement_mapping, time_now, measurement.m_time_stamp);
            return false;
        }
        else
        {
            // 1. temporal update
            auto dt = m_time_keeper.time_since_last_temporal_update(time_now);
            if (m_filter.temporal_update(dt))
                m_time_keeper.update_after_temporal_update(dt);

            // 2. observation update
            m_filter.observation_update(measurement.m_measurement_vector,measurement.m_innovation,
                    measurement.m_state_to_measurement_mapping, measurement.m_measurement_covariance,
                    measurement.m_mahalanobis_thresh);
                               //z, H, R, mahalanobis_threshold);
            m_time_keeper.update_with_measurement(measurement.m_time_stamp, time_now);
        }
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

    inline bool is_initialized() const
    {
        return (m_filter.is_initialized() && m_time_keeper.is_initialized());
    }

    bool reset(Vector measurement_vector, MeasurementMatrix mapping_matrix, tTime time_now, tTime time_stamp)
    {
        //TO_DO: check if the measurement is stateful
        // either odometry or pose
        // Maybe need to template it according to the motionmodel used

        // 1. reset filter
        StateVector x0;
        x0 = mapping_matrix.transpose()*measurement_vector;
        // std::cout << "Initializing with:" << x0.transpose()<<"\n" << mapping_matrix <<"\n" << "Measurement: "<< measurement_vector.transpose() <<"\n";
        m_filter.reset(x0, m_config.init_estimation_covariance, m_config.process_noise);

        // 2. reset timekeeper
        m_time_keeper.reset(time_now, time_stamp);
        return true;
    }

    void configure(const char* config_path)
    {
        m_config = FilterConfig_(config_path);
    }

};

using FilterCtrvEKF2D = FilterWrapper<Ctrv_EKF2D, 6, double>;

} // end namespace filter 
} // end namespace state_predictor
} // end namespace iav