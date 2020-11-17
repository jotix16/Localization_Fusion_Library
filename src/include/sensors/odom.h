
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

#include <sensors/sensor_base.h>
#include <geometry_msgs/msg/PoseWithCovariance.h>
#include <geometry_msgs/msg/TwistWithCovariance.h>
#include <nav_msgs/msg/Odometry.h>

namespace iav{ namespace state_predictor { namespace sensors {

template<typename T, typename States>
class Odom : public SensorBase<T, States>
{
public:
    using SensorBaseT   = SensorBase<T, States>;
    using Measurement   = typename SensorBaseT::Measurement;
    using StateVector   = typename SensorBaseT::StateVector;
    using MappingMatrix = typename SensorBaseT::MappingMatrix;
    using Vector        = typename SensorBaseT::Vector;
    using Matrix        = typename SensorBaseT::Matrix;
    using Matrix6T      = typename SensorBaseT::Matrix6T;
    using Matrix4T      = typename SensorBaseT::Matrix4T;
    using Matrix3T      = typename SensorBaseT::Matrix3T;
    using Vector6T      = typename SensorBaseT::Vector6T;
    using Vector3T      = typename SensorBaseT::Vector3T;

    using TransformationMatrix = typename SensorBaseT::TransformationMatrix;
    using QuaternionT          = typename SensorBaseT::QuaternionT;

private:
    using SensorBaseT::num_state;
    // sensor config
    using SensorBaseT::m_topic_name;
    using SensorBaseT::m_update_vector;
    using SensorBaseT::m_mahalanobis_threshold;

    // debugging
    using SensorBaseT::m_debug;
    using SensorBaseT::m_debug_stream;

public:
    Odom(){};

    /**
     * @brief Odom: Constructor to initialize the odom object.
     * @param[in] topic_name - vector for the state we are estimating, needed to compute T_map_bl(could use ros too, but first step to independence)
     * @param[in] update_vector - pointer to the odom msg of the measurement
     * @param[in] mahalanobis_threshold - transf from sensor frame to base_link frame where angular velocity and acceleration are fused
     * @param[in] out_stream - transf from sensor frame to base_link frame where angular velocity and acceleration are fused
     * @param[in] debug - transf from sensor frame to base_link frame where angular velocity and acceleration are fused
     */
    Odom(const std::string topic_name, const bool* update_vector,
         const T mahalanobis_threshold, std::ostream* out_stream, bool debug)
        : SensorBaseT(topic_name, update_vector, mahalanobis_threshold, out_stream, debug)
        { }

    /**
     * @brief Odom: Callback for receiving all odom msgs. It processes all comming messages
     * by considering transforming them in the fusing frame, considering the m_update_vector to ignore parts
     * of the measurements and capturing matrixes with faulty values. It is a special case of the default odom_callback
     * as it only hanldes odom messages that have frame_id=m_odom_frame and child_frame_id=m_base_link_frame
     * @param[in] state - vector for the state we are estimating, needed to compute T_map_bl(could use ros too, but first step to independence)
     * @param[in] msg - odom msg
     * @param[out] transformed and processed measurement corersponding to the msg
     */
    Measurement odom_callback(
        const StateVector& state,
        nav_msgs::msg::Odometry* msg)
    {
        DEBUG("\n\t\t--------------- Odom[" << m_topic_name<< "] Odom_callback_identity: IN -------------------\n");
        DEBUG( "\n" << msg->header.frame_id <<" ~~ m_update_vector: " << utilities::printtt(m_update_vector, 1, STATE_SIZE));

        // 1.a. POSE PART
        Vector6T pose_measurement;
        pose_measurement[0] = msg->pose.pose.position.x;
        pose_measurement[1] = msg->pose.pose.position.y,
        pose_measurement[2] = msg->pose.pose.position.z;
        pose_measurement.template tail<3>() = euler::get_euler_rpy(QuaternionT(
            msg->pose.pose.orientation.w,
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z
            ));

        std::vector<uint> update_indices_pose;
        for (uint i = 0; i < POSE_SIZE; ++i)
        {
            if(States::full_state_to_estimated_state[i] < STATE_SIZE && m_update_vector[i])
            {
                update_indices_pose.push_back(i);
            }
        }
        size_t update_size_pose = update_indices_pose.size();

        // 1.b. TWIST PART
        Vector6T twist_measurement;
        twist_measurement[0] = msg->twist.twist.linear.x;
        twist_measurement[1] = msg->twist.twist.linear.y,
        twist_measurement[2] = msg->twist.twist.linear.z;
        twist_measurement[3] = msg->twist.twist.angular.x;
        twist_measurement[4] = msg->twist.twist.angular.y;
        twist_measurement[5] = msg->twist.twist.angular.z;

        std::vector<uint> update_indices_twist;
        for (uint i = POSE_SIZE; i < POSE_SIZE + TWIST_SIZE; ++i)
        {
            if(States::full_state_to_estimated_state[i] < STATE_SIZE && m_update_vector[i])
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

        // 3.a. Fill the submeasurement for POSE
        for (uint i = 0; i < update_size_pose; i++)
        {
            sub_measurement[i] = pose_measurement[update_indices_pose[i]];
            sub_innovation[i] = sub_measurement[i] - state(States::full_state_to_estimated_state[update_indices_pose[i]]);
            for (uint j = 0; j < update_size_pose; j++)
            {
                sub_covariance(i, j) = msg->pose.covariance[POSE_SIZE* update_indices_pose[i] + update_indices_pose[j]];
            }
            state_to_measurement_mapping(i , States::full_state_to_estimated_state[update_indices_pose[i]]) = 1.0;
        }

        // 3.b. Fill the submeasurement for TWIST
        for (uint i = 0; i < update_size_twist; i++)
        {
            uint id_i = i + update_size_pose;
            sub_measurement[id_i] = twist_measurement[update_indices_twist[i]-POSE_SIZE];
            sub_innovation[id_i] = sub_measurement[id_i] - state(States::full_state_to_estimated_state[update_indices_twist[i]]);
            for (uint j = 0; j < update_size_twist; j++)
            {
                sub_covariance(id_i, j + update_size_pose) = msg->pose.covariance[TWIST_SIZE * update_indices_twist[i] + update_indices_twist[j]];
            }
            state_to_measurement_mapping(id_i, States::full_state_to_estimated_state[update_indices_twist[i]]) = 1.0;
        }

        // 4. Create measurement to be handled
        tTime stamp_sec = static_cast<tTime>(msg->header.stamp.sec + 1e-9*static_cast<double>(msg->header.stamp.nanosec));
        Measurement meas(stamp_sec, sub_measurement, sub_covariance, sub_innovation, state_to_measurement_mapping,
                        sub_u_indices, msg->header.frame_id, m_mahalanobis_threshold);

        DEBUG(" -> Odom " << meas.print());
        debug_msg(msg, TransformationMatrix::Identity());
        DEBUG("\t\t--------------- Odom[" << m_topic_name<< "] Odom_callback_identity: OUT -------------------\n");
        return meas;
    }

    /**
     * @brief Odom: Callback for receiving all odom msgs. It processes all comming messages
     * by considering transforming them in the fusing frame, considering the m_update_vector to ignore parts
     * of the measurements and capturing matrixes with faulty values.
     * @param[in] state - vector for the state we are estimating, needed to compute T_map_bl(could use ros too, but first step to independence)
     * @param[in] msg - odom msg
     * @param[in] transform_to_base_link - transf from sensor frame to base_link frame where angular velocity and acceleration are fused
     * @param[out] transformed and processed measurement corersponding to the msg
     */
    Measurement odom_callback(
        const StateVector& state,
        nav_msgs::msg::Odometry* msg,
        const TransformationMatrix& transform_to_base_link)
    {
        DEBUG("\n\t\t--------------- Odom[" << m_topic_name<< "] Odom_callback: IN -------------------\n");
        DEBUG( "\n" << msg->header.frame_id <<" ~~ m_update_vector: " << utilities::printtt(m_update_vector, 1, STATE_SIZE));

        // If a whole position, orientation or linear_velocity, angular_velocity should be ignored
        // we cannot assume them to be zeros as in the case where we ignore only a component of each of them.
        // In such a case we make sure that the update_indeces don't include these parts at all.
        bool valid_position, valid_orientation, valid_angular_velocity, valid_linear_velocity;
        valid_position = m_update_vector[STATE_X] || m_update_vector[STATE_Y] || m_update_vector[STATE_Z];
        valid_orientation = m_update_vector[STATE_ROLL] || m_update_vector[STATE_PITCH] || m_update_vector[STATE_YAW];
        valid_linear_velocity = m_update_vector[STATE_V_X] || m_update_vector[STATE_V_Y] || m_update_vector[STATE_V_Z];
        valid_angular_velocity = m_update_vector[STATE_V_ROLL] || m_update_vector[STATE_V_PITCH] || m_update_vector[STATE_V_YAW];

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

        // 3. Fill the submeasurement matrixes
        prepare_pose(state, &(msg->pose), transform_to_base_link, sub_measurement,
                    sub_covariance, sub_innovation, state_to_measurement_mapping,
                    update_indices_pose, 0, update_size_pose,
                    valid_position, valid_orientation);

        if(valid_angular_velocity || valid_linear_velocity)
        prepare_twist(state, &(msg->twist), transform_to_base_link, sub_measurement,
                    sub_covariance, sub_innovation, state_to_measurement_mapping,
                    update_indices_twist, update_size_pose, update_size_twist,
                    valid_angular_velocity, valid_linear_velocity);

        // 4. Create measurement to be handled
        tTime stamp_sec = static_cast<tTime>(msg->header.stamp.sec + 1e-9*static_cast<double>(msg->header.stamp.nanosec));
        Measurement meas(stamp_sec, sub_measurement, sub_covariance, sub_innovation, state_to_measurement_mapping,
                        sub_u_indices, msg->header.frame_id, m_mahalanobis_threshold);
        DEBUG(" -> Odom " << meas.print());
        debug_msg(msg, transform_to_base_link);
        DEBUG("\t\t--------------- Odom[" << m_topic_name<< "] Odom_callback: OUT -------------------\n");
        return meas;
    }

    /**
     * @brief Odom: Prepares a Pose msg and fills the corresponding part of the measurement
     * @param[in] msg - pointer to msg to be prepared
     * @param[in] transform - transformation matrix to the frame of fusion(world frame)
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
        Vector& sub_measurement,
        Matrix& sub_covariance,
        Vector& sub_innovation,
        MappingMatrix& state_to_sub_measurement_mapping,
        const std::vector<uint>& update_indices,
        uint ix1, size_t update_size,
        const bool& valid_position, const bool& valid_orientation)
    {
        DEBUG("\n\t\t--------------- Odom[" << m_topic_name<< "] Prepare_Pose: IN -------------------\n");
        DEBUG("\n");
        if(update_size == 0) return;

        // 2. Write Pose as Transformation Matrix for easy transformations
        // - create pose transformation matrix which saves  |R R R T|
        // - the orientation in form of a (R)otation-matrix |R R R T|
        // - and position as (T)ranslation-vector.          |R R R T|
        //                                                  |0 0 0 1|
        auto state_rot = this->get_rotation_from_state(state);
        auto state_translation =  this->get_translation_from_state(state);
        // consider m_update_vector
        Matrix3T rot_mat;
        Vector3T position;
        // 1. Write orientation in a useful form( Quaternion -> rotation matrix)
        // - Handle bad (empty) quaternions and normalize
        QuaternionT orientation;
        if (valid_orientation)
        {
            if (msg->pose.orientation.x == 0 && msg->pose.orientation.y == 0 &&
            msg->pose.orientation.z == 0 && msg->pose.orientation.w == 0)
            {
                DEBUG("Invalid orientation quaternion. Orientation is set to all 0!\n");
                orientation = {1.0, 0.0, 0.0, 0.0};
            }
            orientation = {msg->pose.orientation.w,
                        msg->pose.orientation.x,
                        msg->pose.orientation.y,
                        msg->pose.orientation.z};
            if (orientation.norm()-1.0 > 0.01)
            {
                DEBUG("Normalizing quaternion that should have already been normalized!\n");
                orientation.normalize();
            }

            // - consider m_update_vector
            // -- extract roll pitch yaw
            auto rpy = euler::get_euler_rpy(orientation);
            auto rpy_state = euler::get_euler_rpy(Matrix3T(state_rot * transform.rotation())); // transform rpy in sensor frame R_map_bl * R_bl_sensor
            // // -- ignore roll pitch yaw according to m_update_vector
            rpy[0] = m_update_vector[STATE_ROLL] ?  rpy[0] : rpy_state[0];
            rpy[1] = m_update_vector[STATE_PITCH] ? rpy[1] : rpy_state[1];
            rpy[2] = m_update_vector[STATE_YAW] ?   rpy[2] : rpy_state[2];
            rot_mat = euler::quat_to_rot(euler::get_quat_rpy(rpy[0], rpy[1], rpy[2]).normalized()); // get rpy in rotation matrix form: R_map_sensor
        }
        else DEBUG("Orientation is being ignored according to m_update_vector!\n");

        if(valid_position)
        {
            auto position_state = state_translation + state_rot * transform.translation(); // transform position from state in sensor frame P_map_bl + R_map_bl*P_bl_sensor
            position[0] = m_update_vector[STATE_X] ? msg->pose.position.x : position_state[0];
            position[1] = m_update_vector[STATE_Y] ? msg->pose.position.y : position_state[1];
            position[2] = m_update_vector[STATE_Z] ? msg->pose.position.z : position_state[2]; // position P_map_sensor
        }
        else DEBUG("Position is being ignored according to m_update_vector!\n");

        // 3. Transform pose to fusion frame
        auto transform_inv = transform.inverse(); // T_sensor_bl
        auto rot = transform_inv.rotation(); // R_sensor_bl
        position += rot_mat *  transform_inv.translation(); // position in map frame: P_map_bl = P_map_sensor + R_map_sensor*P_sensor_bl
        rot_mat *= rot; // R_map_bl = R_map_sensor * R_sensor_bl

        // 4. Compute measurement vector
        Vector6T measurement;
        measurement.setZero();
        measurement.template head<3>() = position;
        measurement.template tail<3>() = euler::get_euler_rpy(rot_mat);

        // 5. Rotate Covariance to fusion frame
        Matrix6T rot6d;
        rot6d.setZero();
        rot6d.template block<3,3>(0,0) = rot;
        rot6d.template block<3,3>(3,3) = rot;

        // 6. Compute measurement covariance
        Matrix6T covariance;
        covariance.setZero();
        for (uint i = 0; i < POSE_SIZE; i++)
        {
            for (uint j = 0; j < POSE_SIZE; j++)
            {
                covariance(i,j) = msg->covariance[i*POSE_SIZE + j];
            }
        }
        covariance = rot6d * covariance * rot6d.transpose();

        // 4. Fill sub_measurement vector and sub_covariance matrix, sub_inovation vector
        //  - and state to measurement mapping
        for ( uint i = 0; i < update_size; i++)
        {
            sub_measurement(i + ix1) = measurement(update_indices[i]);
            sub_innovation(i + ix1) = measurement(update_indices[i]) - state(States::full_state_to_estimated_state[update_indices[i]]);
            for (uint j = 0; j < update_size; j++)
            {
                sub_covariance(i + ix1, j + ix1) = covariance(update_indices[i], update_indices[j]);
            }
            state_to_sub_measurement_mapping(i + ix1, States::full_state_to_estimated_state[update_indices[i]]) = 1.0;
        }

        // DEBUG(" -> Noise pose:\n" << std::fixed << std::setprecision(4) << sub_covariance  << "\n");
        DEBUG(" -> ROT pose: "<<update_size<<"\n" << std::fixed << std::setprecision(4) << rot << "\n");
        DEBUG("\t\t--------------- Odom[" << m_topic_name<< "] Prepare_Pose: OUT -------------------\n");
    }

    /**
     * @brief Odom: Prepares a Twist msg and fills the corresponding part of the measurement
     * @param[in] msg - pointer to msg to be prepared
     * @param[in] transform - transformation matrix to the frame of fusion(base_link frame)
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
        Vector& sub_measurement,
        Matrix& sub_covariance,
        Vector& sub_innovation,
        MappingMatrix& state_to_sub_measurement_mapping,
        const std::vector<uint>& update_indices,
        size_t ix1, size_t update_size,
        const bool& valid_angular_velocity, const bool& valid_linear_velocity)
    {
        DEBUG("\n\t\t--------------- Odom[" << m_topic_name<< "] Prepare_Twist: IN -------------------\n");

        // 1. Create measurement vector (corresponds to the twist part) and get rotation matrix
        Vector6T measurement;
        measurement.setZero(); // should not be impo
        auto rot = transform.rotation();

        if(valid_angular_velocity)
        {
            // 2. Extract angular velocities
            // - consider m_update_vector
            Vector3T angular_vel;
            angular_vel[0] = m_update_vector[STATE_V_ROLL] ?  msg->twist.angular.x : state(States::full_state_to_estimated_state[STATE_V_ROLL]);
            angular_vel[1] = m_update_vector[STATE_V_PITCH] ? msg->twist.angular.y : state(States::full_state_to_estimated_state[STATE_V_PITCH]);
            angular_vel[2] = m_update_vector[STATE_V_YAW] ?   msg->twist.angular.z : state(States::full_state_to_estimated_state[STATE_V_YAW]);
            // - Transform measurement to fusion frame
            angular_vel = rot * angular_vel;
            measurement.template tail<3>() = angular_vel;
        }
        else DEBUG("Angular velocities are being ignored according to m_update_vector!\n");

        if(valid_linear_velocity)
        {
            // 3. Extract linear velocities
            // - consider m_update_vector
            Vector3T linear_vel;
            linear_vel[0] = m_update_vector[STATE_V_X] ? msg->twist.linear.x : state(States::full_state_to_estimated_state[STATE_V_X]);
            linear_vel[1] = m_update_vector[STATE_V_Y] ? msg->twist.linear.y : state(States::full_state_to_estimated_state[STATE_V_Y]);
            linear_vel[2] = m_update_vector[STATE_V_Z] ? msg->twist.linear.z : state(States::full_state_to_estimated_state[STATE_V_Z]);
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
            linear_vel = rot * linear_vel + origin.cross(angular_vel_state); // add effects of angular velocities
                                                                            // v = rot*v + origin(x)w
            measurement.template head<3>() = linear_vel;
            // DEBUG("Origin\n" << origin.transpose() << "\n");
            // DEBUG("Angular veloc\n" << angular_vel_state.transpose() << "\n");
            // DEBUG("Effects of angular veloc\n" << origin.cross(angular_vel_state).transpose() << "\n");
        }
        else DEBUG("Linear velocities are being ignored according to m_update_vector!\n");

        // 4. Compute measurement covariance
        Matrix6T covariance;
        covariance.setZero();
        for (uint i = 0; i < TWIST_SIZE; i++)
        {
            for (uint j = 0; j < TWIST_SIZE; j++)
            {
                covariance(i,j) = msg->covariance[i*TWIST_SIZE + j];
            }
        }
        // 5. Rotate Covariance to fusion frame
        Matrix6T rot6d;
        rot6d.setZero();
        rot6d.template block<3,3>(0,0) = rot;
        rot6d.template block<3,3>(3,3) = rot;
        covariance = rot6d * covariance * rot6d.transpose();

        // 6. Fill sub_measurement vector and sub_covariance matrix, sub_inovation vector
        //  - and state to measurement mapping
        uint meas_index = 0U;
        uint meas_index2 = 0U;
        for ( uint i = 0; i < update_size; i++)
        {
            meas_index = update_indices[i] - POSE_SIZE;
            sub_measurement(i + ix1) = measurement(meas_index);
            sub_innovation(i + ix1) = measurement(meas_index) - state(States::full_state_to_estimated_state[update_indices[i]]);
            for (uint j = 0; j < update_size; j++)
            {
                meas_index2 = update_indices[j] - POSE_SIZE;
                sub_covariance(i + ix1, j + ix1) = covariance(meas_index, meas_index2);
            }
            state_to_sub_measurement_mapping(i + ix1, States::full_state_to_estimated_state[update_indices[i]]) = 1.0;
        }

        DEBUG(" -> ROT twist: "<<update_size<<"\n" << std::fixed << std::setprecision(4) << rot << "\n");
        DEBUG("\t\t--------------- Odom[" << m_topic_name<< "] Prepare_Twist: OUT -------------------\n");
    }

    /**
     * @brief Odom: Helper function that adds odom msg in the debug stream.
     * @param[in] msg - odom msg
     * @param[in] transform_to_bl - transf from sensor frame to base_link frame where angular velocity and acceleration are fused
     */
    void debug_msg(const nav_msgs::msg::Odometry* msg, const TransformationMatrix transform_to_bl)
    {
        QuaternionT orientation;
        orientation = {msg->pose.pose.orientation.w,
                       msg->pose.pose.orientation.x,
                       msg->pose.pose.orientation.y,
                       msg->pose.pose.orientation.z};

        DEBUG("\n------------------------ Message in: ------------------------\n");
        DEBUG("MSG frame: " << msg->header.frame_id << ", child_frame id: " << msg->child_frame_id << "\n");
        DEBUG("R_sens_to_bl:\n" << transform_to_bl.rotation() << "\n");
        DEBUG("rpy: " <<euler::get_euler_rpy(orientation.normalized()).transpose() << "\n");
        DEBUG("pose: "
                << msg->pose.pose.position.x << " "
                << msg->pose.pose.position.y << " "
                << msg->pose.pose.position.z << " "
                << orientation.vec().transpose() << " " << orientation.w() << "\n");

        DEBUG("twist: "
                << msg->twist.twist.linear.x << " "
                << msg->twist.twist.linear.y << " "
                << msg->twist.twist.linear.z << " "
                << msg->twist.twist.angular.x << " "
                << msg->twist.twist.angular.y << " "
                << msg->twist.twist.angular.z << " \n");
        DEBUG("---------------------------------------------------------------\n\n");
    }
};

using OdomD = Odom<double, motion_model::Ctrv2D::States>;

} // end namespace sensors
} // end namespace state_predictor
} // end namespace iav