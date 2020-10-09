
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

#include <vector>
#include <Eigen/Eigen>

#include <utilities/filter_utilities.h>
#include <filter/filter_config.h>
#include <filter/filter_ekf.h>
#include <measurement/measurement_time_keeper.h>
#include <measurement/measurement.h>

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
    using T = typename FilterT::T;
    using MeasurementTimeKeeper = measurement::MeasurementTimeKeeper;
    using Measurement = typename FilterT::Measurement;
    using MappingMatrix = typename Measurement::MappingMatrix;
    using FilterConfig_ = FilterConfig<T>;
    using States = typename FilterT::States;
    using StateVector = typename FilterT::StateVector;
    using StateMatrix = typename FilterT::StateMatrix;
    using Vector = typename FilterT::Vector;
    using Matrix = typename FilterT::Matrix;

    using AngleAxisT = typename Eigen::AngleAxis<T>;
    using QuaternionT = typename Eigen::Quaternion<T>;
    using TransformationMatrix = typename Eigen::Transform<T, 3, Eigen::TransformTraits::Isometry>;
    using Matrix6T = typename Eigen::Matrix<T, 6, 6>;
    using Matrix4T = typename Eigen::Matrix<T, 4, 4>;
    using Matrix3T = typename Eigen::Matrix<T, 3, 3>;
    using Vector6T = typename Eigen::Matrix<T, 6, 1>;
    using Vector3T = typename Eigen::Matrix<T, 3, 1>;

public:
    FilterConfig_ m_config;

private:
    FilterT m_filter;
    MeasurementTimeKeeper m_time_keeper;
    Clock m_wall_time;
    int debug = 1;
    bool m_debug;
    std::mutex m_callback_mutex; 
    std::ofstream m_debug_stream;

public:
    FilterWrapper() = default;
    
    /**
     * @brief Constructor that inizializes configuration related parameters and time-keeping
     * @param[in] config_path - path to .json configuration file
     */
    FilterWrapper(const char* config_path)
    {
        reset(config_path);
    }

    /**
     * @brief FilterWrapper: Function that inizializes configuration related parameters and time-keeping
     * @param[in] config_path - path to .json configuration file
     */
    void reset(const char* config_path)
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
     * @brief FilterWrapper: Helper function that copies a squared matrix from array to eigen matrix
     * @param[out] destination - Eigen 6x6 matrix that has to be filled
     * @param[in] source - source array
     */
    // from array to Matrix, used when reading from msg
    template<uint dim = 6>
    void copy_covariance(Matrix6T& destination, const std::array<double, dim*dim>& source)
    {
        for (uint i = 0; i < dim; i++)
        {
            for (uint j = 0; j < dim; j++)
            {
                destination(i,j) = source[i*dim + j];
            }
        }
    }

    /**
     * @brief FilterWrapper: Helper function that copies a squared from matrix to array used for the msgs.
     * @param[in] destination - destination array which should be filled
     * @param[in] index - source matrix 
     */
    template<uint dim = 6>
    void copy_covariance(std::array<double, dim*dim>& destination, const Matrix6T& source)
    {
        for (uint i = 0; i < dim; i++)
        {
            for (uint j = 0; j < dim; j++)
            {
                destination[i*dim+ j](i,j) = source(i,j);
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
    void odom_callback(
        const std::string& topic_name,
        nav_msgs::msg::Odometry* msg,
        const TransformationMatrix& transform_to_map,
        const TransformationMatrix& transform_to_base_link)
    {
        DEBUG_W("\n\t\t--------------- Wrapper Odom_callback: IN -------------------\n");
        bool* update_vector = m_config.m_sensor_configs[topic_name].m_update_vector;

        if (debug > -1)
        {
            DEBUG_W( "\n" << msg->header.frame_id <<" ~~ Update_vector: " << utilities::printtt(update_vector, 1,15));
        }

        // 1. Create vector the same size as the submeasurement
        // - its elements are the corresponding parts of the state we are estimating
        // - the size of this index-vector enables initializing of the submeasurement matrixes
        // TO_DO: we are not ignoring nan & inf measurements
        std::vector<uint> update_indices_pose;
        for (uint i = 0; i < POSE_SIZE; ++i)
        {
            if(States::full_state_to_estimated_state[i] < STATE_SIZE)
                update_indices_pose.push_back(i);
        }
        size_t update_size_pose = update_indices_pose.size();

        std::vector<uint> update_indices_twist;
        for (uint i = POSE_SIZE; i < TWIST_SIZE + POSE_SIZE; ++i)
        {
            if(States::full_state_to_estimated_state[i] < STATE_SIZE)
                update_indices_twist.push_back(i);
        }
        size_t update_size_twist = update_indices_twist.size();

        // 2. Initialize submeasurement related variables
        size_t update_size = update_size_pose + update_size_twist; 
        Vector sub_measurement = Vector::Zero(update_size); // z
        Vector sub_innovation = Vector::Zero(update_size); // z'-z 
        Matrix sub_covariance = Matrix::Zero(update_size, update_size);
        MappingMatrix state_to_measurement_mapping = MappingMatrix::Zero(update_size, States::STATE_SIZE_M);
        std::vector<uint> sub_u_indices;
        sub_u_indices.reserve( update_size ); // preallocate memory
        sub_u_indices.insert( sub_u_indices.end(), update_indices_pose.begin(), update_indices_pose.end() );
        sub_u_indices.insert( sub_u_indices.end(), update_indices_twist.begin(), update_indices_twist.end() );
        // if we just want to put update_indices_twist after update_indices_pose instaed of creating a new one
        // update_indices_pose.insert( update_indices_pose.end(), update_indices_twist.begin(), update_indices_twist.end() );
        
        // 3. Fill the submeasurement matrixes
        prepare_pose(&(msg->pose), transform_to_map, update_vector, sub_measurement,
                     sub_covariance, sub_innovation, state_to_measurement_mapping,
                     update_indices_pose, 0, update_size_pose);

        prepare_twist(&(msg->twist), transform_to_base_link, update_vector, sub_measurement,
                      sub_covariance, sub_innovation, state_to_measurement_mapping,
                      update_indices_twist, update_size_pose, update_size_twist);

        // 4. Send measurement to be handled
        // TO_DO: clarify how to determine the pose and twist mahalanobis thresholds
        T mahalanobis_thresh = 400;
        tTime stamp_sec = static_cast<tTime>(msg->header.stamp.sec + 1e-9*static_cast<double>(msg->header.stamp.nanosec));
        Measurement meas(stamp_sec, sub_measurement, sub_covariance, sub_innovation, state_to_measurement_mapping,
                        sub_u_indices, msg->header.frame_id, mahalanobis_thresh);
        handle_measurement(meas);
        DEBUG_W("\t\t--------------- Wrapper Odom_callback: OUT -------------------\n");

    }

    /**
     * @brief FilterNode: Callback for receiving all odom msgs. It processes all comming messages
     * by considering transforming them in the fusing frame, considering the update_vector to ignore parts
     * of the measurements and capturing matrixes with faulty values.
     * @param[in] msg - pointer to the imu msg of the measurement
     * @param[in] transform_to_world - transf from sensor frame of msg to world frame where orientation is fused
     * @param[in] transform_to_base_link - transf from sensor frame to base_link frame where angular velocity and acceleration are fused
     */
    void imu_callback(
        const std::string& topic_name,
        sensor_msgs::msg::Imu* msg,
        const TransformationMatrix& transform_to_world,
        const TransformationMatrix& transform_to_base_link)
    {
        DEBUG_W("\n\t\t--------------- Wrapper Imu_callback: IN -------------------\n");
        bool* update_vector = m_config.m_sensor_configs[topic_name].m_update_vector;

        if (debug > -1)
        {
            DEBUG_W( "\n" << msg->header.frame_id <<" ~~ Update_vector: " << utilities::printtt(update_vector, 1,15));
        }

        // 1. Create vector the same size as the submeasurement
        // - its elements are the corresponding parts of the state we are estimating
        // - the size of this index-vector enables initializing of the submeasurement matrixes
        // TO_DO: we are not ignoring nan & inf measurements
        std::vector<uint> update_indices_pose;
        for (uint i = POSITION_SIZE; i < POSE_SIZE; ++i)
        {
            if(States::full_state_to_estimated_state[i] < STATE_SIZE)
                update_indices_pose.push_back(i);
        }
        size_t update_size_pose = update_indices_pose.size();

        std::vector<uint> update_indices_twist;
        for (uint i = POSE_SIZE; i < POSE_SIZE + ORIENTATION_SIZE; ++i)
        {
            if(States::full_state_to_estimated_state[i] < STATE_SIZE)
                update_indices_twist.push_back(i);
        }
        size_t update_size_twist = update_indices_twist.size();

        // 2. Initialize submeasurement related variables
        size_t update_size = update_size_pose + update_size_twist; 
        Vector sub_measurement = Vector::Zero(update_size); // z
        Vector sub_innovation = Vector::Zero(update_size); // z'-z 
        Matrix sub_covariance = Matrix::Zero(update_size, update_size);
        MappingMatrix state_to_measurement_mapping = MappingMatrix::Zero(update_size, States::STATE_SIZE_M);
        std::vector<uint> sub_u_indices;
        sub_u_indices.reserve( update_size ); // preallocate memory
        sub_u_indices.insert( sub_u_indices.end(), update_indices_pose.begin(), update_indices_pose.end() );
        sub_u_indices.insert( sub_u_indices.end(), update_indices_twist.begin(), update_indices_twist.end() );
        // if we just want to put update_indices_twist after update_indices_pose instaed of creating a new one
        // update_indices_pose.insert( update_indices_pose.end(), update_indices_twist.begin(), update_indices_twist.end() );
        
        // 3. Fill the submeasurement matrixes
        // prepare_pose(&(msg->pose), transform_to_world, update_vector, sub_measurement,
        //              sub_covariance, sub_innovation, state_to_measurement_mapping,
        //              update_indices_pose, 0, update_size_pose);
        // Ignore rotational velocity if the first covariance value is -1
        if ((msg->angular_velocity_covariance[0] + 1) < 1e-9)
        {
            std::cout << "Received IMU message with -1 as its first covariance value for angular"
                      << "velocity. Ignoring angular velocity...\n";
        }
        else
        {
            // Repeat for velocity
            // geometry_msgs::TwistWithCovarianceStamped* twistPtr = new geometry_msgs::TwistWithCovarianceStamped();
            geometry_msgs::msg::TwistWithCovariance* twistPtr = new geometry_msgs::msg::TwistWithCovariance();
            // twistPtr->header = msg->header;
            twistPtr->twist.angular = msg->angular_velocity;

            // Copy the covariance
            std::cout << "Angular v covariance:\n";
            for (size_t i = 0; i < ORIENTATION_SIZE; i++)
            {
                for (size_t j = 0; j < ORIENTATION_SIZE; j++)
                {
                    twistPtr->covariance[TWIST_SIZE * (i + ORIENTATION_SIZE) + (j + ORIENTATION_SIZE)] =
                    msg->angular_velocity_covariance[ORIENTATION_SIZE * i + j];
                    std::cout <<twistPtr->covariance[TWIST_SIZE * (i + ORIENTATION_SIZE) + (j + ORIENTATION_SIZE)] << " ";
                }
                std::cout <<"\n";
            }

            std::cout << "Angular v:" << twistPtr->twist.angular.x << " "
                                      << twistPtr->twist.angular.y << " "
                                      << twistPtr->twist.angular.z << "\n";
            // geometry_msgs::TwistWithCovarianceStampedConstPtr tptr(twistPtr);
            // twistCallback(tptr, twistCallbackData, baseLinkFrameId_);
            prepare_twist(twistPtr, transform_to_base_link, update_vector, sub_measurement,
                          sub_covariance, sub_innovation, state_to_measurement_mapping,
                          update_indices_twist, update_size_pose, update_size_twist);


            // 4. Send measurement to be handled
            // TO_DO: clarify how to determine the pose and twist mahalanobis thresholds
            T mahalanobis_thresh = 400;
            tTime stamp_sec = static_cast<tTime>(msg->header.stamp.sec + 1e-9*static_cast<double>(msg->header.stamp.nanosec));
            Measurement meas(stamp_sec, sub_measurement, sub_covariance, sub_innovation, state_to_measurement_mapping,
                            sub_u_indices, msg->header.frame_id, mahalanobis_thresh);
            // handle_measurement(meas);
        }
        DEBUG_W("\t\t--------------- Wrapper Odom_callback: OUT -------------------\n");
    }

    /**
     * @brief FilterWrapper: Prepares a Twist msg and fills the corresponding part of the measurement
     * @param[in] msg - pointer to msg to be prepared
     * @param[in] transform - transformation matrix to the frame of fusion(base_link frame)
     * @param[in] update_vector - vector that specifies which parts of the measurement should be considered
     * @param[inout] sub_measurement - measurement vector to be filled
     * @param[inout] sub_covariance - covariance matrix to be filled
     * @param[inout] sub_innovation - innovation vector to be filled
     * @param[inout] state_measurement_mapping - state to measurement mapping matrix to be filled
     * @param[inout] update_indices - holds the respective indexes of the measurement's component to the full state's components
     * @param[in] ix1 - starting index from which the sub_measurement should be filled 
     * @param[in] update_size - size of the measurement to be filled
     */
    void prepare_twist(
        geometry_msgs::msg::TwistWithCovariance* msg, 
        const TransformationMatrix& transform,
        bool* update_vector,
        Vector& sub_measurement,
        Matrix& sub_covariance, 
        Vector& sub_innovation, 
        MappingMatrix& state_to_measurement_mapping,
        const std::vector<uint>& update_indices,
        size_t ix1, size_t update_size)
    {
        DEBUG_W("\n\t\t--------------- Wrapper Prepare_Twist: IN -------------------\n");

        // 1. Extract angular velocities
        // - consider update_vector
        Vector3T angular_vel;
        angular_vel <<
            msg->twist.angular.x * (int)update_vector[STATE_V_ROLL],
            msg->twist.angular.y * (int)update_vector[STATE_V_PITCH],
            msg->twist.angular.z * (int)update_vector[STATE_V_YAW];
        // - Transform measurement to fusion frame
        auto rot = transform.rotation();
        angular_vel = rot * angular_vel;

        // 2. Extract linear velocities
        // - consider update_vector
        Vector3T linear_vel; 
        linear_vel <<
            msg->twist.linear.x * (int)update_vector[STATE_V_X],
            msg->twist.linear.y * (int)update_vector[STATE_V_Y],
            msg->twist.linear.z * (int)update_vector[STATE_V_Z];
        // - Extract angular velocities from the estimated state
        // - needed to add the effect of angular velocities to the linear ones(look below).
        Vector3T angular_vel_state;
        uint vroll_ix = States::full_state_to_estimated_state[STATE_V_ROLL];
        uint vpitch_ix = States::full_state_to_estimated_state[STATE_V_PITCH];
        uint vyaw_ix = States::full_state_to_estimated_state[STATE_V_YAW];

        angular_vel_state(0) = vroll_ix < STATE_SIZE ? m_filter.at(vroll_ix) : 0.0;
        angular_vel_state(1) = vpitch_ix < STATE_SIZE ? m_filter.at(vpitch_ix) : 0.0;
        angular_vel_state(2) = vyaw_ix < STATE_SIZE ? m_filter.at(vyaw_ix) : 0.0;
        // - Transform measurement to fusion frame
        auto origin = transform.translation();
        linear_vel = rot * linear_vel + origin.cross(angular_vel_state); // add effects of angular velocitioes
                                                                         // v = rot*v + origin(x)w
        // DEBUG_W("Origin\n" << origin.transpose() << "\n");
        // DEBUG_W("Angular veloc\n" << angular_vel_state.transpose() << "\n");
        // DEBUG_W("Effects of angular veloc\n" << origin.cross(angular_vel_state).transpose() << "\n");

        // 4. Compute measurement vector
        Vector6T measurement;
        measurement.template head<3>() = linear_vel;
        measurement.template tail<3>() = angular_vel;

        // 5. Compute measurement covariance
        Matrix6T covariance;
        covariance.setZero();
        copy_covariance<TWIST_SIZE>(covariance, msg->covariance);

        // 6. Rotate Covariance to fusion frame
        Matrix6T rot6d;
        rot6d.setZero();
        rot6d.template block<3,3>(0,0) = rot;
        rot6d.template block<3,3>(3,3) = rot;
        covariance = rot6d * covariance * rot6d.transpose();

        // 7. Fill sub_measurement vector and sub_covariance matrix and sub_inovation vector
        uint meas_index = 0U;
        uint meas_index2 = 0U;
        for ( uint i = 0; i < update_size; i++)
        {
            meas_index = update_indices[i] - POSE_SIZE;
            sub_measurement(i + ix1) = measurement(meas_index);
            sub_innovation(i + ix1) = measurement(meas_index) - m_filter.at(States::full_state_to_estimated_state[update_indices[i]]);
            for (uint j = 0; j < update_size; j++)
            {
                meas_index2 = update_indices[j] - POSE_SIZE;
                sub_covariance(i + ix1, j + ix1) = covariance(meas_index, meas_index2);
            }
        }

        // 7. Fill state to measurement mapping and inovation
        for (uint i = 0; i < update_size; i++)
        {
            if (States::full_state_to_estimated_state[update_indices[i]] < STATE_SIZE)
            state_to_measurement_mapping(i + ix1, States::full_state_to_estimated_state[update_indices[i]]) = 1.0;
        }
        
        DEBUG_W("\t\t--------------- Wrapper Prepare_Twist: OUT -------------------\n");
    }

    /**
     * @brief FilterWrapper: Prepares a Pose msg and fills the corresponding part of the measurement
     * @param[in] msg - pointer to msg to be prepared
     * @param[in] transform - transformation matrix to the frame of fusion(world frame)
     * @param[in] update_vector - vector that specifies which parts of the measurement should be considered
     * @param[inout] sub_measurement - measurement vector to be filled
     * @param[inout] sub_covariance - covariance matrix to be filled
     * @param[inout] sub_innovation - innovation vector to be filled
     * @param[inout] state_measurement_mapping - state to measurement mapping matrix to be filled
     * @param[inout] update_indices - holds the respective indexes of the measurement's component to the full state's components
     * @param[in] ix1 - starting index from which the sub_measurement should be filled 
     * @param[in] update_size - size of the measurement to be filled
     */
    void prepare_pose(
        geometry_msgs::msg::PoseWithCovariance* msg, 
        const TransformationMatrix& transform,
        bool* update_vector,
        Vector& sub_measurement,
        Matrix& sub_covariance, 
        Vector& sub_innovation, 
        MappingMatrix& state_to_measurement_mapping,
        const std::vector<uint>& update_indices,
        uint ix1, size_t update_size)
    {
        DEBUG_W("\n\t\t--------------- Wrapper Prepare_Pose: IN -------------------\n");
        DEBUG_W("\n");
        // 1. Write orientation in a useful form( Quaternion -> rotation matrix)
        // - Handle bad (empty) quaternions and normalize
        QuaternionT orientation;
        if (msg->pose.orientation.x == 0 && msg->pose.orientation.y == 0 &&
            msg->pose.orientation.z == 0 && msg->pose.orientation.w == 0)
        {
            DEBUG_W("Invalid orientation quaternion. Orientation is set to all 0!\n");
            orientation = {1.0, 0.0, 0.0, 0.0};
        }
        else
        {
            orientation = {msg->pose.orientation.w,
                           msg->pose.orientation.x,
                           msg->pose.orientation.y,
                           msg->pose.orientation.z};
            if (orientation.norm()-1.0 > 0.01)
            {
                orientation.normalize();
            }

            // - consider update_vector
            // -- extract roll pitch yaw 
            auto rpy = orientation.toRotationMatrix().eulerAngles(0, 1, 2);
            // -- ignore roll pitch yaw according to update_vector 
            rpy[0] *= (int)update_vector[STATE_ROLL];
            rpy[1] *= (int)update_vector[STATE_PITCH];
            rpy[2] *= (int)update_vector[STATE_YAW];
            orientation = AngleAxisT(rpy[0], Vector3T::UnitX())
                        * AngleAxisT(rpy[1], Vector3T::UnitY())
                        * AngleAxisT(rpy[2], Vector3T::UnitZ());

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
        pose_transf.template block<3,3>(0,0) = orientation.toRotationMatrix();
        pose_transf.template block<3,1>(0,3) = Eigen::Vector3d{
            msg->pose.position.x * (int)update_vector[STATE_X],
            msg->pose.position.y * (int)update_vector[STATE_Y],
            msg->pose.position.z * (int)update_vector[STATE_Z]};

        // 3. Transform pose to fusion frame
        pose_transf = transform * pose_transf;
        DEBUG_W( " -> Pose transformed:\n" << pose_transf << "\n");
        // 4. Compute measurement vector
        Vector6T measurement;
        measurement.template head<3>() = pose_transf.template block<3,1>(0,3);
        measurement.template tail<3>() = pose_transf.template block<3,3>(0,0).eulerAngles(0,1,2);

        // 5. Rotate Covariance to fusion frame
        Matrix6T rot6d;
        rot6d.setZero();
        auto rot = transform.rotation();
        rot6d.template block<3,3>(0,0) = rot;
        rot6d.template block<3,3>(3,3) = rot;

        // 6. Compute measurement covariance
        Matrix6T covariance;
        covariance.setZero();
        copy_covariance<POSE_SIZE>(covariance, msg->covariance);
        covariance = rot6d * covariance * rot6d.transpose();

        // 4. Fill sub_measurement vector and sub_covariance matrix and sub_inovation vector
        for ( uint i = 0; i < update_size; i++)
        {
            sub_measurement(i + ix1) = measurement(update_indices[i]);
            sub_innovation(i + ix1) = measurement(update_indices[i]) - m_filter.at(States::full_state_to_estimated_state[update_indices[i]]);
            for (uint j = 0; j < update_size; j++)
            {
                sub_covariance(i + ix1, j + ix1) = covariance(update_indices[i], update_indices[j]);
            }
                // if(sub_covariance(i,i) < 0.0) sub_covariance(i,i) = -sub_covariance(i,i);
                // if(sub_covariance(i,i) < 1e-9) sub_covariance(i,i) = 1e-9;
        }

        // 5. Fill state to measurement mapping and inovation
        for (uint i = 0; i < update_size; i++)
        {
            if (States::full_state_to_estimated_state[update_indices[i]]<15)
            state_to_measurement_mapping(i + ix1, States::full_state_to_estimated_state[update_indices[i]]) = 1.0;
        }
        
        DEBUG_W(" -> Noise:\n" << std::fixed << std::setprecision(4) << covariance << "\n");
        DEBUG_W("\t\t--------------- Wrapper Prepare_Pose: OUT -------------------\n");
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
            if(debug > 1)
            {
                DEBUG_W(" -> Noise temp:\n");
                DEBUG_W(std::fixed << std::setprecision(4) <<  measurement.R << "\n");
            }
            DEBUG_W(std::fixed << std::setprecision(4) << " -> Innovation:  " << measurement.innovation.transpose() << "\n");
            DEBUG_W(std::fixed << std::setprecision(4) << " -> Measurement: " << measurement.z.transpose() << "\n");
            DEBUG_W(std::fixed << std::setprecision(4) << " -> State:       " << get_state().transpose() << "\n");

            // 1. temporal update
            auto dt = m_time_keeper.time_since_last_temporal_update(time_now);
            DEBUG_W("\n--------------- Wrapper: Temporal update, dt = "<< dt << " ---------------\n");
            if (dt < 0) return false;
            // std::lock_guard<std::mutex> guard(m_callback_mutex);
            if (m_filter.temporal_update(dt))
            {
                m_time_keeper.update_after_temporal_update(dt);
                
                DEBUG_W(std::fixed << std::setprecision(4) << " -> State temp: " << get_state().transpose() << "\n");
                if(debug > 1) DEBUG_W(" -> Covar temp: \n");
                if(debug > 1) DEBUG_W(std::fixed << std::setprecision(4) << get_covariance() << "\n");
            }
            // 2. observation update
            if (m_filter.observation_update(measurement))
            {
                m_time_keeper.update_with_measurement(measurement.m_time_stamp, time_now);

                DEBUG_W("\n--------------- Wrapper: Observation update! ---------------\n");
                DEBUG_W(std::fixed << std::setprecision(4) << " -> State obsv: " << get_state().transpose() << "\n");
                if(debug > 1) DEBUG_W(" -> Covar obsv: \n");
                if(debug > 1) DEBUG_W(std::fixed << std::setprecision(4) << get_covariance() << "\n");
            }
            else DEBUG_W(" Mahalanobis failed" << "\n");
        }

        DEBUG_W("\t\t--------------- Wrapper Process_Measurement: OUT -------------------\n");
        return true;
    }

    /**
     * @brief FilterWrapper: Getter function for the state estimation
     * @return the state estimation vector
     */
    inline const StateVector& get_state() const
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
        // could use update_indeces to avoid matrix multiplication but since it only
        // happens once it not that big of a deal
        x0 = measurement.H.transpose() * measurement.z;
        if(debug > 1) DEBUG_W("<---RESET--->\n Initializing with:" << x0.transpose()<<"\n");
        if(debug > 1) DEBUG_W("Mapping Matrix:\n" << measurement.H << "\n");

        // extract the part of init_estimation and process_noise covariances you need
        StateMatrix init_cov;
        init_cov.setIdentity();
        StateMatrix process_noise;
        process_noise.setIdentity();
        uint ind_temp1;
        uint ind_temp2;
        for (auto i:measurement.m_update_indices)
        {
            ind_temp1 = States::full_state_to_estimated_state[i];

            for (auto j:measurement.m_update_indices)
            {
                ind_temp2 = States::full_state_to_estimated_state[j];

                init_cov(ind_temp1, ind_temp2) = m_config.m_init_estimation_covariance(i,j);
                process_noise(ind_temp1, ind_temp2) = m_config.m_process_noise(i,j);
            }
        }
        
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
        if(debug >-1)
        {
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
        }

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