
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
#include <sensors/sensor_base.h>

#include <geometry_msgs/msg/Vector3.h>
#include <geometry_msgs/msg/PoseWithCovariance.h>
#include <geometry_msgs/msg/TwistWithCovariance.h>
#include <sensor_msgs/msg/Imu.h>

namespace iav{ namespace state_predictor { namespace sensors {

template<typename T, typename States>
class Imu : public SensorBase<T, States>
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
    using Vector6T      = typename SensorBaseT::Vector6T;
    using Vector3T      = typename SensorBaseT::Vector3T;
    using Matrix3T      = typename SensorBaseT::Matrix3T;

    using TransformationMatrix = typename SensorBaseT::TransformationMatrix;
    using QuaternionT          = typename SensorBaseT::QuaternionT;
    using AngleAxisT           = typename SensorBaseT::AngleAxisT;


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
    using SensorBaseT::SensorBase; // inherite constructor
    Imu(){}; // default constructor


    /**
     * @brief FilterNode: Callback for receiving all odom msgs. It processes all comming messages
     * by considering transforming them in the fusing frame, considering the m_update_vector to ignore parts
     * of the measurements and capturing matrixes with faulty values.
     * @param[in] msg - pointer to the imu msg of the measurement
     * @param[in] transform_to_world - transf from sensor frame of msg to world frame where orientation is fused
     * @param[in] transform_to_base_link - transf from sensor frame to base_link frame where angular velocity and acceleration are fused
     */
    Measurement imu_callback(
        const StateVector& state,
        sensor_msgs::msg::Imu* msg,
        const TransformationMatrix& transform_to_world,
        const TransformationMatrix& transform_to_base_link)
    {
        DEBUG("\n\t\t--------------- Wrapper Imu_callback: IN -------------------\n");
        //TO_DO:s
        // if (!is_initialized()) 
        // {
        //     DEBUG("Got IMU but not initialized. Ignoring");
        //     return;
        // }
        DEBUG(msg->header.frame_id <<" ~~ m_update_vector: " << utilities::printtt(m_update_vector, 1, STATE_SIZE)<< "\n");

        bool valid_orientation, valid_angular_velocity, valid_linear_acceleration;
        valid_orientation = std::abs(msg->orientation_covariance[0] + 1.0) > 1e-9;
        valid_angular_velocity = std::abs(msg->angular_velocity_covariance[0] + 1) > 1e-9;
        valid_linear_acceleration = std::abs(msg->linear_acceleration_covariance[0] + 1) > 1e-9;


        // 1. Create vector the same size as the submeasurement
        // - its elements are the corresponding parts of the state we are estimating
        // - the size of this index-vector enables initializing of the submeasurement matrixes
        // TO_DO: we are not ignoring nan & inf measurements

        // a- update_indeces for the ORIENTATION in the measurement
        size_t update_size_orientation = 0;
        std::vector<uint> update_indices_orientation;
        if (valid_orientation)
        {
            for (uint i = POSITION_SIZE; i < POSE_SIZE; ++i)
            {
                if(States::full_state_to_estimated_state[i] < STATE_SIZE)
                    update_indices_orientation.push_back(i);
            }
            update_size_orientation = update_indices_orientation.size();
        }

        // b- update_indeces for the ANGULAR VELOCITIES in the TWIST measurement
        size_t update_size_twist = 0;
        std::vector<uint> update_indices_twist;
        if (valid_angular_velocity)
        {
            for (uint i = POSE_SIZE + POSITION_SIZE; i < POSE_SIZE + POSITION_SIZE + ORIENTATION_SIZE; ++i)
            {
                if(States::full_state_to_estimated_state[i] < STATE_SIZE)
                    update_indices_twist.push_back(i);
            }
            update_size_twist = update_indices_twist.size();
        }

        // c- update_indeces for the LINEAR ACCELERATION
        size_t update_size_acceleration = 0;
        std::vector<uint> update_indices_acceleration;
        if (valid_linear_acceleration)
        {
            for (uint i = POSE_SIZE + TWIST_SIZE; i < STATE_SIZE; ++i)
            {
                if(States::full_state_to_estimated_state[i] < STATE_SIZE)
                    update_indices_acceleration.push_back(i);
            }
            update_size_acceleration = update_indices_acceleration.size();
        }

        // 2. Initialize submeasurement related variables
        size_t update_size = update_size_orientation + update_size_twist + update_size_acceleration; 
        Vector sub_measurement = Vector::Zero(update_size); // z
        Vector sub_innovation = Vector::Zero(update_size); // z'-z 
        Matrix sub_covariance = Matrix::Zero(update_size, update_size);
        MappingMatrix state_to_measurement_mapping = MappingMatrix::Zero(update_size, States::STATE_SIZE_M);
        std::vector<uint> sub_u_indices;
        sub_u_indices.reserve(update_size); // preallocate memory
        sub_u_indices.insert(sub_u_indices.end(), update_indices_orientation.begin(), update_indices_orientation.end());
        sub_u_indices.insert(sub_u_indices.end(), update_indices_twist.begin(), update_indices_twist.end());
        sub_u_indices.insert(sub_u_indices.end(), update_indices_acceleration.begin(), update_indices_acceleration.end());
        
        // 3. Fill the submeasurement matrixes
        // a- fill the ORIENTATION part
        // REQUIRES DISCUSSION on how to transform the covariance matrix
        if (valid_orientation)
        {
            prepare_imu_orientation(state, msg->orientation, msg->orientation_covariance,
                          transform_to_world, transform_to_base_link, sub_measurement,
                          sub_covariance, sub_innovation, state_to_measurement_mapping,
                          update_indices_orientation, 0, update_size_orientation);
        }
        else
        {
            DEBUG("Received IMU message with -1 as its first covariance value for orientation."
                      << " Ignoring the orientation...\n");
        }

        // b- fill the ANGULAR VELOCITY part
        // - the idea is to create a twist message and to normally call prepare_twist
        if (valid_angular_velocity)
        {
            // create twist msg
            geometry_msgs::msg::TwistWithCovariance* twistPtr = new geometry_msgs::msg::TwistWithCovariance();
            twistPtr->twist.angular = msg->angular_velocity;

            // Copy the covariance
            for (size_t i = 0; i < ORIENTATION_SIZE; i++)
            {
                for (size_t j = 0; j < ORIENTATION_SIZE; j++)
                {
                    twistPtr->covariance[TWIST_SIZE*i + j] = 0.0;
                    twistPtr->covariance[TWIST_SIZE*(i + ORIENTATION_SIZE) + j] = 0.0;
                    twistPtr->covariance[TWIST_SIZE*i + (j + ORIENTATION_SIZE)] = 0.0;

                    twistPtr->covariance[TWIST_SIZE * (i + ORIENTATION_SIZE) + (j + ORIENTATION_SIZE)] =
                    msg->angular_velocity_covariance[ORIENTATION_SIZE * i + j];
                }
            }
            prepare_twist(state, twistPtr, transform_to_base_link, sub_measurement,
                          sub_covariance, sub_innovation, state_to_measurement_mapping,
                          update_indices_twist, update_size_orientation, update_size_twist, true);
        }
        else
        {
            DEBUG("Received IMU message with -1 as its first covariance value for angular"
                      << "velocity. Ignoring angular velocity...\n");
        }

        // c- fill the LINEAR ACCELERATION part
        if (valid_linear_acceleration)
        {
            prepare_acceleration(state, msg->linear_acceleration, msg->linear_acceleration_covariance,
                          transform_to_base_link, sub_measurement,
                          sub_covariance, sub_innovation, state_to_measurement_mapping,
                          update_indices_acceleration, update_size_orientation + update_size_twist, update_size_acceleration);
        }
        else
        {
            DEBUG("Received IMU message with -1 as its first covariance value for linear"
                      << "acceleration. Ignoring linear acceleration...\n");
        }

        // 4. Create measurement to be handled
        // TO_DO: clarify how to determine the pose and twist mahalanobis thresholds
        T mahalanobis_thresh = 400;
        tTime stamp_sec = static_cast<tTime>(msg->header.stamp.sec + 1e-9*static_cast<double>(msg->header.stamp.nanosec));
        Measurement meas(stamp_sec, sub_measurement, sub_covariance, sub_innovation, state_to_measurement_mapping,
                        sub_u_indices, msg->header.frame_id, mahalanobis_thresh);
        DEBUG("\t\t--------------- Wrapper Imu_callback: OUT -------------------\n");
        return meas;
    }

    /**
     * @brief FilterWrapper: Prepares a Twist msg and fills the corresponding part of the measurement
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
        size_t ix1, size_t update_size, bool is_imu)
    {
        DEBUG("\n\t\t--------------- Wrapper Prepare_Twist: IN -------------------\n");

        // 4. Create measurement vector (corresponds to the twist part)
        Vector6T measurement;
        measurement.setZero(); // should not be impo

        // 1. Extract angular velocities
        // - consider m_update_vector
        Vector3T angular_vel;
        angular_vel <<
            msg->twist.angular.x * (int)m_update_vector[STATE_V_ROLL],
            msg->twist.angular.y * (int)m_update_vector[STATE_V_PITCH],
            msg->twist.angular.z * (int)m_update_vector[STATE_V_YAW];
        // - Transform measurement to fusion frame
        auto rot = transform.rotation();
        angular_vel = rot * angular_vel;
        measurement.template tail<3>() = angular_vel;

        if(!is_imu)
        {
            // 2. Extract linear velocities if it is not imu
            // - consider m_update_vector
            Vector3T linear_vel; 
            linear_vel <<
                msg->twist.linear.x * (int)m_update_vector[STATE_V_X],
                msg->twist.linear.y * (int)m_update_vector[STATE_V_Y],
                msg->twist.linear.z * (int)m_update_vector[STATE_V_Z];
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

    /**
     * @brief FilterWrapper: Prepares a Pose msg and fills the corresponding part of the measurement
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
    void prepare_acceleration(
        const StateVector& state,
        geometry_msgs::msg::Vector3& meas_msg,
        std::array<double, 9> covariance_array, 
        const TransformationMatrix& transform,
        Vector& sub_measurement,
        Matrix& sub_covariance, 
        Vector& sub_innovation, 
        MappingMatrix& state_to_measurement_mapping,
        const std::vector<uint>& update_indices,
        uint ix1, size_t update_size)
    {
        DEBUG("\n\t\t--------------- Wrapper Prepare_Acceleration: IN -------------------\n");
        DEBUG("\n");
        // 1. Write orientation in a useful form( Quaternion -> rotation matrix)
        // - Handle bad (empty) quaternions and normalize

        Vector3T measurement;
        Matrix3T covariance;
        for(uint i = 0; i<3; ++i)
        {
            for(uint j = 0; j<3; ++j)
            {
                covariance(i, j) = covariance_array[i + j*3];
            }
        }
        measurement[0] = meas_msg.x * (int)m_update_vector[STATE_A_X];
        measurement[1] = meas_msg.y * (int)m_update_vector[STATE_A_Y];
        measurement[2] = meas_msg.z * (int)m_update_vector[STATE_A_Z]; // remove_grav --- wrong


        // std::cout << "G: " << measurement[2]<< "\n";

        // - Transform measurement to fusion frame
        auto rot = transform.rotation();

        Vector3T gravity_acc;
        gravity_acc << 0.0 , 0.0 , 9.8;
        gravity_acc = rot.transpose() * gravity_acc; // R^T = R^^1 for rotation matrixes (transpose instead of inverse)
        measurement -= gravity_acc;

        measurement = rot * measurement;
        covariance = rot * covariance * rot.transpose();

        // 4. Fill sub_measurement vector and sub_covariance matrix and sub_inovation vector
        auto offset = POSE_SIZE + TWIST_SIZE;
        for ( uint i = 0; i < update_size; i++)
        {
            sub_measurement(i + ix1) = measurement(update_indices[i] - offset);
            sub_innovation(i + ix1) = sub_measurement(i + ix1) - state(States::full_state_to_estimated_state[update_indices[i]]);
            for (uint j = 0; j < update_size; j++)
            {
                sub_covariance(i + ix1, j + ix1) = covariance(update_indices[i] - offset, update_indices[j] - offset);
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

        DEBUG(" -> Noise from acceleration msg:\n" << std::fixed << std::setprecision(4) << utilities::printtt(covariance_array, 3, 3));
        DEBUG(" -> Noise acceleration:\n" << std::fixed << std::setprecision(4) << sub_covariance << "\n");
        DEBUG("\t\t--------------- Wrapper Prepare_Acceleration: OUT -------------------\n");
    }

    /**
     * @brief FilterWrapper: Prepares an orientation msg and fills the corresponding part of the measurement
     * @param[in] msg - pointer to msg to be prepared
     * @param[in] transform_map_enu - transformation matrix T_map_enu (gives enu in map_frame)
     * @param[in] transform_bl_imu - transformation matrix T_bl_imu (gives imu in base_link frame)
     * @param[inout] sub_measurement - measurement vector to be filled
     * @param[inout] sub_covariance - covariance matrix to be filled
     * @param[inout] sub_innovation - innovation vector to be filled
     * @param[inout] state_measurement_mapping - state to measurement mapping matrix to be filled
     * @param[inout] update_indices - holds the respective indexes of the measurement's component to the full state's components
     * @param[in] ix1 - starting index from which the sub_measurement should be filled 
     * @param[in] update_size - size of the measurement to be filled
     */
    void prepare_imu_orientation(
        const StateVector& state,
        geometry_msgs::msg::Quaternion& meas_msg,
        std::array<double, 9> covariance_array, 
        const TransformationMatrix& transform_map_enu,
        const TransformationMatrix& transform_bl_imu,
        Vector& sub_measurement,
        Matrix& sub_covariance, 
        Vector& sub_innovation, 
        MappingMatrix& state_to_measurement_mapping,
        const std::vector<uint>& update_indices,
        uint ix1, size_t update_size)
    {
        DEBUG("\n\t\t--------------- Wrapper Prepare_Orientation: IN -------------------\n");
        DEBUG("\n");

        // 1. Read orientation and consider m_update_vector
        // - Handle bad (empty) quaternions and normalize
        QuaternionT orientation;
        orientation =  {meas_msg.w,
                        meas_msg.x,
                        meas_msg.y,
                        meas_msg.z};
                        
        if (orientation.norm()-1.0 > 0.01)
        {
            orientation.normalize();
        }

        // - consider m_update_vector
        // -- extract roll pitch yaw 
        auto rpy = orientation.toRotationMatrix().eulerAngles(0, 1, 2);
        // -- ignore roll pitch yaw according to m_update_vector 
        rpy[0] *= (int)m_update_vector[STATE_ROLL];
        rpy[1] *= (int)m_update_vector[STATE_PITCH];
        rpy[2] *= (int)m_update_vector[STATE_YAW];
        orientation = AngleAxisT(rpy[0], Vector3T::UnitX())
                    * AngleAxisT(rpy[1], Vector3T::UnitY())
                    * AngleAxisT(rpy[2], Vector3T::UnitZ());
        if (orientation.norm()-1.0 > 0.01)
        {
            orientation.normalize();
        }

        auto rot_meas = orientation.toRotationMatrix();
        auto rot_map_enu = transform_map_enu.rotation();
        auto rot_imu_bl = transform_bl_imu.rotation().transpose(); // transpose instead of inverse for rotation matrixes

        // - Transform measurement to fusion frame
        rot_meas = rot_map_enu * rot_meas; // R_map_imu      
        rot_meas = rot_meas * rot_imu_bl; // R_map_bl
        
        Vector3T measurement;
        measurement = rot_meas.eulerAngles(0, 1, 2);
        std::cout << "FUSE RPY: " << measurement.transpose() << "\n";
        orientation = rot_meas;
        std::cout << "FUSE QUAT: " << orientation.vec().transpose() << " " << orientation.w()<< "\n";

        // Transform covariance, not sure for the second transformation
        Matrix3T covariance;
        for(uint i = 0; i<3; ++i)
        {
            for(uint j = 0; j<3; ++j)
            {
                covariance(i, j) = covariance_array[i + j*3];
            }
        }
        covariance = rot_map_enu * covariance * rot_map_enu.transpose();
        covariance = rot_imu_bl * covariance * rot_imu_bl.transpose(); // not sure if this is right

        // 4. Fill sub_measurement vector and sub_covariance matrix and sub_inovation vector
        auto offset = POSITION_SIZE;
        for ( uint i = 0; i < update_size; i++)
        {
            sub_measurement(i + ix1) = measurement(update_indices[i] - offset);
            sub_innovation(i + ix1) = sub_measurement(i + ix1) - state(States::full_state_to_estimated_state[update_indices[i]]);
            for (uint j = 0; j < update_size; j++)
            {
                sub_covariance(i + ix1, j + ix1) = covariance(update_indices[i] - offset, update_indices[j] - offset);
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

        DEBUG(" -> Noise from orientation msg:\n" << std::fixed << std::setprecision(4) << utilities::printtt(covariance_array, 3, 3));
        DEBUG(" -> Noise orientation:\n" << std::fixed << std::setprecision(4) << sub_covariance << "\n");
        DEBUG("\t\t--------------- Wrapper Prepare_Orientation: OUT -------------------\n");
    }

};

using ImuD = Imu<double, motion_model::Ctrv2D::States>;

} // end namespace sensors 
} // end namespace state_predictor
} // end namespace iav