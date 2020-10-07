
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

namespace iav{ namespace state_predictor { namespace filter {

/**
 * @brief Class that wrapps the filter and takes care of timekeeping and measurement processing
 * @param<template> FilterT - The filter algorithm that we are usign for fusion
 * @param<template> num_state - Size of the state
 * @param<template> T - Type that should be used for calculations(default is double, but float can be used too)
 */
template<class FilterT, int num_state, typename T = double>
class FilterWrapper
{
public:
    using Measurement= typename measurement::Measurement<num_state,T>;
    using MeasurementTimeKeeper = measurement::MeasurementTimeKeeper;
    using FilterConfig_ = FilterConfig<T>;

    using States = typename FilterT::States;
    using StateVector = typename FilterT::StateVector;
    using StateMatrix = typename FilterT::StateMatrix;
    using Vector = typename FilterT::Vector;
    using Matrix = typename FilterT::Matrix;
    using MeasurementMatrix = typename FilterT::ObservationMatrix;

    using AngleAxisT = typename Eigen::AngleAxis<T>;
    using Matrix6T = typename Eigen::Matrix<T, 6, 6>;
    using Matrix4T = typename Eigen::Matrix<T, 4, 4>;
    using Matrix3T = typename Eigen::Matrix<T, 3, 3>;
    using Vector6T = typename Eigen::Matrix<T, 6, 1>;
    using Vector3T = typename Eigen::Matrix<T, 3, 1>;
    using Quaternion = typename Eigen::Quaternion<T>;
    using TransformationMatrix = typename Eigen::Transform<T, 3, Eigen::TransformTraits::Isometry>;

public:
    FilterConfig_ m_config;

private:
    FilterT m_filter;
    MeasurementTimeKeeper m_time_keeper;
    Clock m_wall_time;
    int debug = 1;
    bool m_debug;
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
            std::string debugOutFile = std::string("DEBUG_localization_fusion.txt");
            try
            {
                m_debug_stream.open(debugOutFile.c_str());

                // Make sure we succeeded
                if (m_debug_stream.is_open())
                {
                    DEBUG_W("-----------------------------------------\n");
                    DEBUG_W("----- /FilterWrapper::Debug is on!" << " ------\n");
                    DEBUG_W("-----------------------------------------\n");
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
     * @brief FilterWrapper: Callback for receiving all odom msgs
     * @param[in] msg - pointer to the msg of the measurement
     * @param[in] transform_to_world - transf from sensor frame of msg to the world frame where the pose part is fused
     * @param[in] transform_to_base_link - transf from sensor frame of msg to the base_link frame where the twist part is fused
     */
    void odom_callback(
        const std::string& topic_name,
        nav_msgs::msg::Odometry* msg,
        const TransformationMatrix& transform_to_world,
        const TransformationMatrix& transform_to_base_link)
    {
        if(debug > 1) std::cout << "---------------Wrapper Odom_callback: IN-------------------"<< std::endl;
        bool* update_vector = m_config.m_sensor_configs[topic_name].m_update_vector;

        if (debug > 1)
        {
            std::cout << msg->header.frame_id <<" ~~ Update_vector: ";
            for(int i = 0; i < 15; ++i)
            {
                std::cout << 2.0 * update_vector[i] << " ";
            }
            std::cout << "\n";
        }

        // 1. Create vector the same size as the submeasurement that will be used
        // - its elements are the corresponding parts of the state
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

        // 2. Initialize submeasurement matrixes
        size_t update_size = update_size_pose + update_size_twist; 
        Vector sub_measurement(update_size); sub_measurement.setZero(); // z
        Vector sub_innovation(update_size); sub_innovation.setZero(); // z'-z 
        Matrix sub_covariance(update_size, update_size);
        sub_covariance.setZero();
        MeasurementMatrix state_to_measurement_mapping;
        state_to_measurement_mapping.resize(update_size, States::STATE_SIZE_M);
        state_to_measurement_mapping.setZero();
        
        // 3. Fill the submeasurement matrixes
        prepare_pose(&(msg->pose), transform_to_world, update_vector, sub_measurement,
                     sub_covariance, sub_innovation, state_to_measurement_mapping,
                     update_indices_pose, 0, update_size_pose);

        prepare_twist(&(msg->twist), transform_to_base_link, update_vector, sub_measurement,
                      sub_covariance, sub_innovation, state_to_measurement_mapping,
                      update_indices_twist, update_size_pose, update_size_twist);

        // 4. Send measurement to be handled
        // TO_DO: clarify how to determine the pose and twist mahalanobis thresholds
        T mahalanobis_thresh = 400;
        // TO_DO: not sure about the time, should try out.
        // tTime stamp_sec = (static_cast<tTime>(msg->header.stamp.nanosec)/1000000000LL);
        tTime stamp_sec = static_cast<tTime>(msg->header.stamp.sec + 1e-9*static_cast<double>(msg->header.stamp.nanosec));
        Measurement meas(stamp_sec, sub_measurement, sub_covariance, sub_innovation, 
                         state_to_measurement_mapping, msg->header.frame_id, mahalanobis_thresh);
        handle_measurement(meas);
        if(debug > 0) std::cout << "---------------Wrapper Odom_callback: OUT-------------------\n";
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
        MeasurementMatrix& state_to_measurement_mapping,
        std::vector<uint>& update_indices,
        size_t ix1, size_t update_size)
    {
        if(debug > 1) std::cout << "---------------Wrapper Prepare_Twist: IN-------------------\n";
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

        angular_vel_state(0) = vroll_ix < STATE_SIZE ? m_filter.at(vroll_ix) : 0.0;
        angular_vel_state(1) = vpitch_ix < STATE_SIZE ? m_filter.at(vpitch_ix) : 0.0;
        angular_vel_state(2) = vyaw_ix < STATE_SIZE ? m_filter.at(vyaw_ix) : 0.0;

        // 3. Transform measurement to fusion frame
        auto rot = transform.rotation();
        auto origin = transform.translation();
        linear_vel = rot * linear_vel + origin.cross(angular_vel_state); // add effects of angular velocitioes
                                                                         // v = rot*v + origin(x)w
        // std::cout << "Origin\n" << origin.transpose() << "\n";
        // std::cout << "Angular veloc\n" << angular_vel_state.transpose() << "\n";
        // std::cout << "Effects of angular veloc\n" << origin.cross(angular_vel_state).transpose() << "\n";
        angular_vel = rot * angular_vel;

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
        // std::cout << "Covariance\n";
        // std::cout << std::fixed << std::setprecision(4) << covariance;
        covariance = rot6d * covariance * rot6d.transpose();
        // std::cout << "Covariance after\n";
        // std::cout << std::fixed << std::setprecision(4) << covariance;

        // 7. Fill sub_measurement vector and sub_covariance matrix and sub_inovation vector
        uint meas_index = 0U;
        uint meas_index2 = 0U;
        std::cout << "Twist\n";
        for ( uint i = 0; i < update_size; i++)
        {
            meas_index = update_indices[i] - POSE_SIZE;
            sub_measurement(i + ix1) = measurement(meas_index);
            sub_innovation(i + ix1) = measurement(meas_index) - m_filter.at(States::full_state_to_estimated_state[update_indices[i]]);
            for (uint j = 0; j < update_size; j++)
            {
                meas_index2 = update_indices[j] - POSE_SIZE;
                std::cout << "(" << i + ix1 << ", " << j + ix1 << ") ";
                sub_covariance(i + ix1, j + ix1) = covariance(meas_index, meas_index2);
            }
            std::cout << "\n";
        }

        // 7. Fill state to measurement mapping and inovation
        for (uint i = 0; i < update_size; i++)
        {
            if (States::full_state_to_estimated_state[update_indices[i]] < STATE_SIZE)
            state_to_measurement_mapping(i + ix1, States::full_state_to_estimated_state[update_indices[i]]) = 1.0;
        }
        
        if(debug > 1) std::cout << "---------------Wrapper Prepare_Twist: OUT-------------------\n";
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
        MeasurementMatrix& state_to_measurement_mapping,
        std::vector<uint>& update_indices,
        uint ix1, size_t update_size)
    {
        if(debug > 1) std::cout << "---------------Wrapper Prepare_Pose: IN-------------------\n";

        // 1. Write orientation in a useful form( Quaternion -> rotation matrix)
        // - Handle bad (empty) quaternions and normalize
        Quaternion orientation;
        if (msg->pose.orientation.x == 0 && msg->pose.orientation.y == 0 &&
            msg->pose.orientation.z == 0 && msg->pose.orientation.w == 0)
        {
            if(debug > 0) std::cout << "---------------Wrapper Prepare_Pose: Orientation is all 0 -------------------\n";
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
        if(debug > 1) std::cout << "---------------Wrapper Prepare_Pose: -------------------\n";
        if(debug > 1) std::cout << " -> Pose transformed:\n" << pose_transf << "\n";
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
        // std::cout << "Rot6d\n";
        // std::cout << std::fixed << std::setprecision(4) << rot6d;

        // 4. Fill sub_measurement vector and sub_covariance matrix and sub_inovation vector
        std::cout << "Pose:\n";
        for ( uint i = 0; i < update_size; i++)
        {
            sub_measurement(i + ix1) = measurement(update_indices[i]);
            sub_innovation(i + ix1) = measurement(update_indices[i]) - m_filter.at(States::full_state_to_estimated_state[update_indices[i]]);
            for (uint j = 0; j < update_size; j++)
            {
                sub_covariance(i + ix1, j + ix1) = covariance(update_indices[i], update_indices[j]);
                std::cout << "(" << i + ix1 << ", " <<  j + ix1 << ") ";
            }
            std::cout << "\n";
                // if(sub_covariance(i,i) < 0.0) sub_covariance(i,i) = -sub_covariance(i,i);
                // if(sub_covariance(i,i) < 1e-9) sub_covariance(i,i) = 1e-9;
        }
        std::cout <<"\n";

        // 5. Fill state to measurement mapping and inovation
        for (uint i = 0; i < update_size; i++)
        {
            if (States::full_state_to_estimated_state[update_indices[i]]<15)
            state_to_measurement_mapping(i + ix1, States::full_state_to_estimated_state[update_indices[i]]) = 1.0;
        }
        
        if(debug > 1) std::cout << " -> Noise:\n";
        if(debug > 1) std::cout << std::fixed << std::setprecision(4) << covariance << "\n";
        if(debug > 1) std::cout << "---------------Wrapper Prepare_Pose: OUT-------------------\n";
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
        if(debug > 1) std::cout << "---------------Wrapper Process_Measurement: IN-------------------\n";
        // Get global time
        tTime time_now = m_wall_time.now();

        if (!is_initialized()) {
            if(debug > 0) std::cout << std::fixed << std::setprecision(4) << " -> Innovation:  " << measurement.m_innovation.transpose() << "\n";
            if(debug > 0) std::cout << std::fixed << std::setprecision(4) << " -> Measurement: " << measurement.m_measurement_vector.transpose() << "\n";
            // TO_DO: this is not strictly correct, but should be good enough. If we get an observation
            // and the filter is not set to any state, we reset it.
            // We only consider the parts that are allowed by update_vector
            // TO_DO: any other way?

            // Initialize the filter with the first measurement
            // std::cout<<"Filter or timeMeasurement are not initialized. We are initializing!\n";
            reset(measurement.m_measurement_vector, measurement.m_state_to_measurement_mapping, time_now, measurement.m_time_stamp);
            return false;
        }
        else
        {

            if(debug > 1)
            {
                std::cout << " -> Noise temp:\n";
                std::cout << std::fixed << std::setprecision(4) <<  measurement.m_measurement_covariance << "\n";
            }
            if(debug > 0) std::cout << std::fixed << std::setprecision(4) << " -> Innovation:  " << measurement.m_innovation.transpose() << "\n";
            if(debug > 0) std::cout << std::fixed << std::setprecision(4) << " -> Measurement: " << measurement.m_measurement_vector.transpose() << "\n";
            if(debug > 0) std::cout << std::fixed << std::setprecision(4) << " -> State:       " << get_state().transpose() << "\n";

            // 1. temporal update
            auto dt = m_time_keeper.time_since_last_temporal_update(time_now); // PROBLEM: is calculated wrong.
            if(debug > 0) std::cout << "---------------Wrapper: Temporal update, dt = "<< dt << "---------------\n";
            if (dt < 0) return false;
            if (m_filter.temporal_update(dt))
            {
                m_time_keeper.update_after_temporal_update(dt);
                
                if(debug > 0) std::cout << std::fixed << std::setprecision(4) << " -> State temp: " << get_state().transpose() << "\n";
                if(debug > 1) std::cout << " -> Covar temp: \n";
                if(debug > 1) std::cout << std::fixed << std::setprecision(4) << get_covariance() << "\n";
            }
            // 2. observation update
            if (m_filter.observation_update(measurement.m_measurement_vector,measurement.m_innovation,
                    measurement.m_state_to_measurement_mapping, measurement.m_measurement_covariance,
                    measurement.m_mahalanobis_thresh))
            {
                m_time_keeper.update_with_measurement(measurement.m_time_stamp, time_now);

                if(debug > 0) std::cout << "---------------Wrapper: Observation update!---------------\n";
                if(debug > 0) std::cout << std::fixed << std::setprecision(4) << " -> State obsv: " << get_state().transpose() << "\n";
                if(debug > 1) std::cout << " -> Covar obsv: \n";
                if(debug > 1) std::cout << std::fixed << std::setprecision(4) << get_covariance() << "\n";
            }
            else if(debug > 0) std::cout << " Mahalanobis failed" << "\n";
        }

        if(debug > 1) std::cout << "---------------Wrapper Process_Measurement: OUT-------------------\n";
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
    inline StateMatrix get_covariance() const
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
    bool reset(Vector measurement_vector, MeasurementMatrix mapping_matrix, tTime time_now, tTime time_stamp)
    {
        //TO_DO: check if the measurement is stateful?
        // either odometry or pose
        // Maybe need to template it according to the motionmodel used

        // 1. reset filter
        StateVector x0;
        x0 = mapping_matrix.transpose()*measurement_vector;
        if(debug > 1) std::cout << "Initializing with:" << x0.transpose()<<"\n" << "Measurement: "<< measurement_vector.transpose() <<"\n";
        if(debug > 1) std::cout << mapping_matrix << "\n";
        // std::cout << "Init cov:\n" << m_config.m_init_estimation_covariance<<"\n";

        // extract the part of init_estimation and process_noise covariances you need
        // TO_DO: maybe better send some indices_vector and have only one loop? 
        //        But should be fine as it is called only once in the beginning.
        StateMatrix init_cov;
        init_cov.setIdentity();
        StateMatrix process_noise;
        process_noise.setIdentity();
        uint ind_temp1;
        uint ind_temp2;
        for (uint i = 0; i < STATE_SIZE; ++i)
        {
            ind_temp1 = States::full_state_to_estimated_state[i];
            if( ind_temp1 < STATE_SIZE)
            {
                for (uint j = 0; j < STATE_SIZE; j++)
                {
                    ind_temp2 = States::full_state_to_estimated_state[j];
                    if( ind_temp2< STATE_SIZE)
                    {
                        init_cov(ind_temp1, ind_temp2) = m_config.m_init_estimation_covariance(i,j);
                        process_noise(ind_temp1, ind_temp2) = m_config.m_process_noise(i,j);
                    }
                }
            }
        }
        
        m_filter.reset(x0, init_cov, process_noise);

        // 2. reset timekeeper
        m_time_keeper.reset(time_now, time_stamp);
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

    // get state in odom form
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
        // // euler to quaternion
        Eigen::Quaterniond qq;
        qq = AngleAxisT(roll, Vector3T::UnitX())
        * AngleAxisT(pitch, Vector3T::UnitY())
        * AngleAxisT(yaw, Vector3T::UnitZ());
        // qq.normalize();
        // std::cout << qq.x() << " " << qq.y() << " "  << qq.z() << " "  << qq.w() << " "  <<"NORMMMM: " << qq.norm() <<"\n";
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
            // if ( ix > 14) continue;
            for(int j = 0; j < POSE_SIZE; j++)
            {
                iy = States::full_state_to_estimated_state[j];
                if ( iy > 14 || ix > 14)
                {
                     msg.pose.covariance[i + j*6] = 1e-9;
                    // continue;
                }
                else
                msg.pose.covariance[i + j*6] = cov_mat(ix , iy);
            }
        }

        // 7. Twist Covariance
        for (int i = 0; i < TWIST_SIZE; i++)
        {
            ix = States::full_state_to_estimated_state[i+POSE_SIZE];
            // if ( ix > 14) continue;
            for(int j = 0; j < TWIST_SIZE; j++)
            {
                iy = States::full_state_to_estimated_state[j+POSE_SIZE];
                if ( iy > 14 || ix > 14) 
                msg.twist.covariance[i + j*6] = 1e-9;
                else
                msg.twist.covariance[i + j*6] = cov_mat(ix , iy);
            }
        }

        // 8. Debugg information
        if(debug >2)
        {
            std::cout << "pose: ";
            std::cout << msg.pose.pose.position.x << " ";
            std::cout << msg.pose.pose.position.y << " ";
            std::cout << msg.pose.pose.position.z << " ";
            std::cout << msg.pose.pose.orientation.x << " ";
            std::cout << msg.pose.pose.orientation.y << " ";
            std::cout << msg.pose.pose.orientation.z << " ";
            std::cout << msg.pose.pose.orientation.w << " ";
            std::cout << "\n";

            std::cout << "twist: ";
            std::cout << msg.twist.twist.linear.x << " ";
            std::cout << msg.twist.twist.linear.y << " ";
            std::cout << msg.twist.twist.linear.z << " ";
            std::cout << msg.twist.twist.angular.x << " ";
            std::cout << msg.twist.twist.angular.y << " ";
            std::cout << msg.twist.twist.angular.z << " ";
            std::cout << "\n";
            
            std::cout << "pose cov:\n";
            for (int i = 0; i < 6; i++)
            {
                for (int j = 0; j < 6; j++)
                {
                    std::cout << msg.pose.covariance[i*6+j] << " ";
                }
                std::cout << "\n";
            }
            std::cout << "\n";

            std::cout << "twist cov:\n";
            for (int i = 0; i < 6; i++)
            {
                for (int j = 0; j < 6; j++)
                {
                    std::cout << msg.twist.covariance[i*6+j] << " ";
                }
                std::cout << "\n";
            }
            std::cout << "\n";
        }

        return msg;
    }
    
    tTime get_last_measurement_time()
    {
        return m_time_keeper.latest_timestamp();
    }

};

// explicit template initialization
using FilterCtrvEKF2D = FilterWrapper<Ctrv_EKF2D, 6, double>;
using FilterCtraEKF2D = FilterWrapper<Ctra_EKF2D, 8, double>;
using FilterCtraEKF3D = FilterWrapper<Ctra_EKF3D, 15, double>;

} // end namespace filter 
} // end namespace state_predictor
} // end namespace iav