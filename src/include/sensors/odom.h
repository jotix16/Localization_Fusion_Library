
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
#include <motion_model/motion_model.h>

#include <geometry_msgs/msg/Vector3.h>
#include <geometry_msgs/msg/PoseWithCovariance.h>
#include <geometry_msgs/msg/TwistWithCovariance.h>
#include <nav_msgs/msg/Odometry.h>
#include <sensor_msgs/msg/Imu.h>

namespace iav{ namespace state_predictor { namespace sensors {

template<typename T, typename States>
class Odom
{
public:
    static constexpr int num_state = States::STATE_SIZE_M;

    using Measurement = typename measurement::Measurement<num_state, T>;

    using StateVector =   typename Eigen::Matrix<T, num_state, 1>;
    using MappingMatrix = typename Eigen::Matrix<T, Eigen::Dynamic, num_state>;
    using Vector =        typename Eigen::Matrix<T, Eigen::Dynamic, 1>;
    using Matrix =        typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
    using Matrix6T =      typename Eigen::Matrix<T, 6, 6>;
    using Matrix4T =      typename Eigen::Matrix<T, 4, 4>;
    using Vector6T =      typename Eigen::Matrix<T, 6, 1>;
    using Vector3T =      typename Eigen::Matrix<T, 3, 1>;

    using TransformationMatrix = typename Eigen::Transform<T, 3, Eigen::TransformTraits::Isometry>;
    using QuaternionT = typename Eigen::Quaternion<T>;
    using AngleAxisT = typename Eigen::AngleAxis<T>;

    private:
    bool m_debug;
    std::ostream* m_debug_stream;

    public:
    Odom(): m_debug(true)
    { }

    void setDebug(std::ostream* out_stream)
    {
            m_debug_stream = out_stream;
            DEBUG("\t\t\t-----------------------------------------\n");
            DEBUG("\t\t\t----- /SensorOdom::Odom is on!" << " ------\n");
            DEBUG("\t\t\t-----------------------------------------\n");
    }

    Measurement odom_callback(
        const StateVector& state,
        bool* update_vector,
        nav_msgs::msg::Odometry* msg,
        const TransformationMatrix& transform_to_map,
        const TransformationMatrix& transform_to_base_link)
    {
        DEBUG("\n\t\t--------------- Wrapper Odom_callback: IN -------------------\n");
        // bool* update_vector = m_config.m_sensor_configs[topic_name].m_update_vector; //// 1111111111111111111111111111111111111111111111111111111111111
        DEBUG( "\n" << msg->header.frame_id <<" ~~ Update_vector: " << utilities::printtt(update_vector, 1, STATE_SIZE));

        // If a whole position, orientation or linear_velocity, angular_velocity should be ignored
        // we cannot assume them to be zeros as in the case where we ignore only a component of each of them.
        // In such a case we make sure that the update_indeces don't include these parts at all.
        bool valid_position, valid_orientation, valid_angular_velocity, valid_linear_velocity;
        valid_position = update_vector[STATE_X] || update_vector[STATE_Y] || update_vector[STATE_Z];
        valid_orientation = update_vector[STATE_ROLL] || update_vector[STATE_PITCH] || update_vector[STATE_YAW];
        valid_linear_velocity = update_vector[STATE_V_X] || update_vector[STATE_V_Y] || update_vector[STATE_V_Z];
        valid_angular_velocity = update_vector[STATE_V_ROLL] || update_vector[STATE_V_PITCH] || update_vector[STATE_V_YAW];

        // 1. Create vector the same size as the submeasurement
        // - its elements are the corresponding parts of the state we are estimating
        // - the size of this index-vector enables initializing of the submeasurement matrixes
        // TO_DO: we are not ignoring nan & inf measurements

        // a. POSE PART
        uint start_index, end_index;
        start_index = valid_position ?  0 : POSITION_SIZE;
        end_index = valid_orientation ?  POSE_SIZE : POSE_SIZE - ORIENTATION_SIZE;
        std::vector<uint> update_indices_pose;
        for (uint i = start_index; i < end_index; ++i)
        {
            if(States::full_state_to_estimated_state[i] < STATE_SIZE)
                update_indices_pose.push_back(i);
        }
        size_t update_size_pose = update_indices_pose.size();

        // a. TWIST PART
        start_index = valid_linear_velocity ?  POSE_SIZE : POSE_SIZE + ORIENTATION_SIZE;
        end_index = valid_angular_velocity ?  TWIST_SIZE + POSE_SIZE : TWIST_SIZE + POSE_SIZE - ORIENTATION_SIZE;
        std::vector<uint> update_indices_twist;
        for (uint i = start_index; i < end_index; ++i)
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
        MappingMatrix state_to_measurement_mapping = MappingMatrix::Zero(update_size, num_state);
        std::vector<uint> sub_u_indices;
        sub_u_indices.reserve(update_size); // preallocate memory
        sub_u_indices.insert(sub_u_indices.end(), update_indices_pose.begin(), update_indices_pose.end());
        sub_u_indices.insert(sub_u_indices.end(), update_indices_twist.begin(), update_indices_twist.end());
        // if we just want to put update_indices_twist after update_indices_pose instaed of creating a new one
        // update_indices_pose.insert(update_indices_pose.end(), update_indices_twist.begin(), update_indices_twist.end());
        
        // 3. Fill the submeasurement matrixes
        prepare_pose(state, &(msg->pose), transform_to_map, update_vector, sub_measurement,
                     sub_covariance, sub_innovation, state_to_measurement_mapping,
                     update_indices_pose, 0, update_size_pose);
                     
        if(valid_angular_velocity || valid_linear_velocity)
        prepare_twist(state, &(msg->twist), transform_to_base_link, update_vector, sub_measurement,
                      sub_covariance, sub_innovation, state_to_measurement_mapping,
                      update_indices_twist, update_size_pose, update_size_twist, false);

        // 4. Send measurement to be handled
        // TO_DO: clarify how to determine the pose and twist mahalanobis thresholds
        T mahalanobis_thresh = 400;
        tTime stamp_sec = static_cast<tTime>(msg->header.stamp.sec + 1e-9*static_cast<double>(msg->header.stamp.nanosec));
        Measurement meas(stamp_sec, sub_measurement, sub_covariance, sub_innovation, state_to_measurement_mapping,
                        sub_u_indices, msg->header.frame_id, mahalanobis_thresh);
        DEBUG("\t\t--------------- Wrapper Odom_callback: OUT -------------------\n");
        return meas;
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
        const StateVector& state,
        geometry_msgs::msg::PoseWithCovariance* msg, 
        const TransformationMatrix& transform,
        bool* update_vector,
        Vector& sub_measurement,
        Matrix& sub_covariance, 
        Vector& sub_innovation, 
        MappingMatrix& state_to_sub_measurement_mapping,
        const std::vector<uint>& update_indices,
        uint ix1, size_t update_size)
    {
        DEBUG("\n\t\t--------------- Wrapper Prepare_Pose: IN -------------------\n");
        DEBUG("\n");
        if(update_size == 0) return;

        bool ignore_orientation, ignore_position;
        ignore_position = update_indices[0] >= POSITION_SIZE;
        ignore_orientation = update_indices[update_size-1] < POSITION_SIZE;

        // 1. Write orientation in a useful form( Quaternion -> rotation matrix)
        // - Handle bad (empty) quaternions and normalize
        QuaternionT orientation;
        if (msg->pose.orientation.x == 0 && msg->pose.orientation.y == 0 &&
            msg->pose.orientation.z == 0 && msg->pose.orientation.w == 0)
        {
            DEBUG("Invalid orientation quaternion. Orientation is set to all 0!\n");
            orientation = {1.0, 0.0, 0.0, 0.0};
        }
        else if (ignore_orientation)
        {
            DEBUG("Orientation is being ignored according to update_vector!\n");
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
        if(!ignore_orientation)
        pose_transf.template block<3,3>(0,0) = orientation.toRotationMatrix(); // orientation part

        if(!ignore_position)
        pose_transf.template block<3,1>(0,3) = Eigen::Vector3d{
            msg->pose.position.x * (int)update_vector[STATE_X],
            msg->pose.position.y * (int)update_vector[STATE_Y],
            msg->pose.position.z * (int)update_vector[STATE_Z]}; // position part

        // 3. Transform pose to fusion frame
        pose_transf = transform * pose_transf;
        DEBUG( " -> Pose transformed:\n" << pose_transf << "\n");
        // 4. Compute measurement vector
        Vector6T measurement;
        measurement.setZero();
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
        for (uint i = 0; i < TWIST_SIZE; i++)
        {
            for (uint j = 0; j < TWIST_SIZE; j++)
            {
                covariance(i,j) = msg->covariance[i*TWIST_SIZE + j];
            }
        }
        covariance = rot6d * covariance * rot6d.transpose();

        // 4. Fill sub_measurement vector and sub_covariance matrix and sub_inovation vector
        for ( uint i = 0; i < update_size; i++)
        {
            sub_measurement(i + ix1) = measurement(update_indices[i]);
            sub_innovation(i + ix1) = measurement(update_indices[i]) - state(States::full_state_to_estimated_state[update_indices[i]]);
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
            state_to_sub_measurement_mapping(i + ix1, States::full_state_to_estimated_state[update_indices[i]]) = 1.0;
        }
        
        DEBUG(" -> Noise pose:\n" << std::fixed << std::setprecision(4) << sub_covariance  << "\n");
        DEBUG("\t\t--------------- Wrapper Prepare_Pose: OUT -------------------\n");
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
        const StateVector& state,
        geometry_msgs::msg::TwistWithCovariance* msg, 
        const TransformationMatrix& transform,
        bool* update_vector,
        Vector& sub_measurement,
        Matrix& sub_covariance, 
        Vector& sub_innovation, 
        MappingMatrix& state_to_sub_measurement_mapping,
        const std::vector<uint>& update_indices,
        size_t ix1, size_t update_size, bool is_imu)
    {
        DEBUG("\n\t\t--------------- Wrapper Prepare_Twist: IN -------------------\n");

        // 4. Create measurement vector (corresponds to the twist part)
        Vector6T measurement;
        measurement.setZero(); // should not be impo

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
        measurement.template tail<3>() = angular_vel;

        if(!is_imu)
        {
            // 2. Extract linear velocities if it is not imu
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

            angular_vel_state(0) = vroll_ix < STATE_SIZE ? state(vroll_ix) : 0.0;
            angular_vel_state(1) = vpitch_ix < STATE_SIZE ? state(vpitch_ix) : 0.0;
            angular_vel_state(2) = vyaw_ix < STATE_SIZE ? state(vyaw_ix) : 0.0;
            // - Transform measurement to fusion frame
            auto origin = transform.translation();
            linear_vel = rot * linear_vel + origin.cross(angular_vel_state); // add effects of angular velocitioes
                                                                            // v = rot*v + origin(x)w
            // DEBUG("Origin\n" << origin.transpose() << "\n");
            // DEBUG("Angular veloc\n" << angular_vel_state.transpose() << "\n");
            // DEBUG("Effects of angular veloc\n" << origin.cross(angular_vel_state).transpose() << "\n");
            measurement.template head<3>() = linear_vel;
        }

        // 5. Compute measurement covariance
        Matrix6T covariance;
        covariance.setZero();
        for (uint i = 0; i < TWIST_SIZE; i++)
        {
            for (uint j = 0; j < TWIST_SIZE; j++)
            {
                covariance(i,j) = msg->covariance[i*TWIST_SIZE + j];
            }
        }
        // 6. Rotate Covariance to fusion frame
        Matrix6T rot6d;
        rot6d.setZero();
        rot6d.template block<3,3>(0,0) = rot;
        rot6d.template block<3,3>(3,3) = rot;
        covariance = rot6d * covariance * rot6d.transpose();

        // 7. Fill sub_measurement vector and sub_covariance matrix and sub_inovation vector
        uint meas_index = 0U;
        uint meas_index2 = 0U;
        std::stringstream sss;
        sss<<"INDEXES: "<< ix1 <<"\n";
        for ( uint i = 0; i < update_size; i++)
        {
            meas_index = update_indices[i] - POSE_SIZE;
            sub_measurement(i + ix1) = measurement(meas_index);
            sub_innovation(i + ix1) = measurement(meas_index) - state(States::full_state_to_estimated_state[update_indices[i]]);
            for (uint j = 0; j < update_size; j++)
            {
                meas_index2 = update_indices[j] - POSE_SIZE;
                sss << "(" << meas_index << "," <<meas_index2 <<"->"<< covariance(meas_index, meas_index2)  <<") ";
                sub_covariance(i + ix1, j + ix1) = covariance(meas_index, meas_index2);
            }
            sss<<"\n";
        }
            sss<<"\n";

        // 7. Fill state to measurement mapping and inovation
        for (uint i = 0; i < update_size; i++)
        {
            state_to_sub_measurement_mapping(i + ix1, States::full_state_to_estimated_state[update_indices[i]]) = 1.0;
        }

        DEBUG(" -> ROT twist: "<<update_size<<"\n" << std::fixed << std::setprecision(4) << rot << "\n");
        DEBUG(" -> Noise from twist msg:\n" << std::fixed << std::setprecision(4) << utilities::printtt(msg->covariance, 6, 6));
        DEBUG(" -> Noise twist:\n" << std::fixed << std::setprecision(9) << sub_covariance << "\n");
        DEBUG(" -> SubNoise twist\n" <<sss.str());
        DEBUG("\t\t--------------- Wrapper Prepare_Twist: OUT -------------------\n");
    }

};

using OdomD = Odom<double, motion_model::Ctrv2D>;

} // end namespace sensors 
} // end namespace state_predictor
} // end namespace iav