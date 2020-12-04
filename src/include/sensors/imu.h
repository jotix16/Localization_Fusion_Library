
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

#include <geometry_msgs/msg/Vector3.h>
#include <sensor_msgs/msg/Imu.h>

namespace iav{ namespace state_predictor { namespace sensors {

template<typename T, typename States>
class Imu : public SensorBase<T, States>
{
public:
    using SensorBaseT   = SensorBase<T, States>;
    using Measurement   = typename SensorBaseT::Measurement;
    using MeasurementPtr   = typename SensorBaseT::MeasurementPtr;
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
    using AngleAxisT           = typename SensorBaseT::AngleAxisT;


private:
    using SensorBaseT::num_state;
    // sensor config
    using SensorBaseT::m_topic_name;
    using SensorBaseT::m_update_vector;
    using SensorBaseT::m_mahalanobis_threshold;
    bool m_remove_gravity;

    // debugging
    using SensorBaseT::m_debug;
    using SensorBaseT::m_debug_stream;

    bool m_init_orientation;
    Matrix3T m_R_map_enu;
    Matrix3T m_R_bl_enu;
    // such as in robot_loc
    Matrix3T m_bl_imu_rot_yaw;
    Vector3T m_bl_imu_offset_vector;

public:
    Imu(){}; // default constructor

    /**
     * @brief Imu: Constructor to initialize the imu object.
     * @param[in] topic_name - vector for the state we are estimating, needed to compute T_map_bl(could use ros too, but first step to independence)
     * @param[in] update_vector - pointer to the imu msg of the measurement
     * @param[in] mahalanobis_threshold - transf from sensor frame to base_link frame where angular velocity and acceleration are fused
     * @param[in] out_stream - transf from sensor frame to base_link frame where angular velocity and acceleration are fused
     * @param[in] debug - transf from sensor frame to base_link frame where angular velocity and acceleration are fused
     * @param[in] remove_gravity - option to remove gravity from the acceleration part
     */
    Imu(const std::string topic_name, const bool* update_vector,
         const T mahalanobis_threshold, std::ostream* out_stream, bool debug, bool remove_gravity)
        : SensorBaseT(topic_name, update_vector, mahalanobis_threshold, out_stream, debug), m_init_orientation(false), m_remove_gravity(remove_gravity)
        { }

    /**
     * @brief Imu: Funciton that initializes the IMU orientation after the first measurement(only if we are not ignoring orientation)
     * @param[in] state - vector for the state we are estimating
     * @param[in] msg - pointer to the imu msg of the measurement
     * @param[in] T_bl_imu - transf from base_link frame to the sensor frame of msg
     * @param[in] T_map_bl - transf from sensor frame to base_link frame where angular velocity and acceleration are fused
     */
    void initialize(const StateVector& state,
         sensor_msgs::msg::Imu* msg,
         const TransformationMatrix& T_bl_imu,
         const TransformationMatrix& T_map_bl)
    {
        DEBUG("\n\t\t--------------- IMU[" << m_topic_name<< "] INITIALIZING: IN -------------------\n");

        // --------------------- CREATE TRANSFORMATION ---------------------
        // as in rob_loc
        m_bl_imu_offset_vector = euler::get_euler_rpy(T_bl_imu.rotation());
        m_bl_imu_rot_yaw = euler::quat_to_rot(euler::get_quat_rpy(0.0, 0.0, m_bl_imu_offset_vector[2]).normalized());
        m_init_orientation = true;
        std::cout << "HEY\n";

        // ------------- DEBUG
        DEBUG("Initialized at\n"
            << "--Rotation R_bl_imu_yaw:\n" << m_bl_imu_rot_yaw << std::endl
            << "--Euler Offset: "<< m_bl_imu_offset_vector.transpose() << std::endl);
        DEBUG("\n\t\t--------------- IMU[" << m_topic_name<< "] INITIALIZING: OUT -------------------" << std::endl);
        // -------------
    }

    /**
     * @brief Imu: Callback for receiving all odom msgs. It processes all comming messages
     * by considering transforming them in the fusing frame, considering the m_update_vector to ignore parts
     * of the measurements and capturing matrixes with faulty values.
     * @param[in] state - vector for the state we are estimating
     * @param[in] msg - pointer to the imu msg of the measurement
     * @param[in] transform_to_world - transf from sensor frame of msg to world frame where orientation is fused
     * @param[in] transform_to_base_link - transf from sensor frame to base_link frame where angular velocity and acceleration are fused
     */
    MeasurementPtr imu_callback(
        const StateVector& state,
        sensor_msgs::msg::Imu* msg,
        const TransformationMatrix& transform_base_link_imu,
        const TransformationMatrix& transform_map_base_link)
    {
        DEBUG("\n\t\t--------------- Imu[" << m_topic_name<< "] Imu_callback: IN -------------------\n");
        DEBUG(msg->header.frame_id <<" ~~ m_update_vector: " << utilities::printtt(m_update_vector, 1, STATE_SIZE)<< "\n");

        bool valid_orientation, valid_angular_velocity, valid_linear_acceleration;
        valid_orientation = std::abs(msg->orientation_covariance[0] + 1.0) > 1e-9
                            && (m_update_vector[STATE_ROLL] || m_update_vector[STATE_PITCH] || m_update_vector[STATE_YAW]);
        valid_angular_velocity = std::abs(msg->angular_velocity_covariance[0] + 1) > 1e-9
                            && (m_update_vector[STATE_V_ROLL] || m_update_vector[STATE_V_PITCH] || m_update_vector[STATE_V_YAW]);
        valid_linear_acceleration = std::abs(msg->linear_acceleration_covariance[0] + 1) > 1e-9
                            && (m_update_vector[STATE_A_X] || m_update_vector[STATE_A_Y] || m_update_vector[STATE_A_Z]);


        // 1. Create vector the same size as the submeasurement
        // - its elements are the corresponding parts of the state we are estimating
        // - the size of this index-vector enables initializing of the submeasurement matrixes
        // a- update_indeces for the ORIENTATION in the measurement
        size_t update_size_orientation = 0;
        std::vector<uint> update_indices_orientation;
        if (valid_orientation)
        {
            if(!m_init_orientation)
            {
                initialize(state, msg, transform_base_link_imu, transform_map_base_link);
            }

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
        MappingMatrix state_to_measurement_mapping = MappingMatrix::Zero(update_size, num_state);
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
                          transform_map_base_link, transform_base_link_imu, sub_measurement,
                          sub_covariance, sub_innovation, state_to_measurement_mapping,
                          update_indices_orientation, 0, update_size_orientation);
        }
        else
        {
            DEBUG("Received IMU message with -1 as its first covariance value for orientation."
                      << " Ignoring the orientation...\n");
        }

        // b- fill the ANGULAR VELOCITY part
        if (valid_angular_velocity)
        {
            prepare_angular_velocity(state, msg->angular_velocity, msg->angular_velocity_covariance, transform_base_link_imu,
                          sub_measurement, sub_covariance, sub_innovation, state_to_measurement_mapping,
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
                          transform_base_link_imu, sub_measurement,
                          sub_covariance, sub_innovation, state_to_measurement_mapping,
                          update_indices_acceleration, update_size_orientation + update_size_twist, update_size_acceleration);
        }
        else
        {
            DEBUG("Received IMU message with -1 as its first covariance value for linear"
                      << "acceleration. Ignoring linear acceleration...\n");
        }

        // 4. Create measurement to be handled
        tTime stamp_sec = static_cast<tTime>(msg->header.stamp.sec + 1e-9*static_cast<double>(msg->header.stamp.nanosec));
        MeasurementPtr meas(new Measurement(stamp_sec, sub_measurement, sub_covariance, sub_innovation, state_to_measurement_mapping,
                        sub_u_indices, msg->header.frame_id, m_mahalanobis_threshold));

        DEBUG(" -> IMU " << meas->print());
        DEBUG("\t\t--------------- Imu[" << m_topic_name<< "] Imu_callback: OUT -------------------\n");
        return meas;
    }

    /**
     * @brief Imu: Prepares an orientation msg and fills the corresponding part of the measurement
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
        const TransformationMatrix& transform_map_base_link,
        const TransformationMatrix& transform_bl_imu,
        Vector& sub_measurement,
        Matrix& sub_covariance,
        Vector& sub_innovation,
        MappingMatrix& state_to_measurement_mapping,
        const std::vector<uint>& update_indices,
        uint ix1, size_t update_size)
    {
        DEBUG("\n\t\t--------------- Imu[" << m_topic_name<< "] Prepare_Orientation: IN -------------------" << std::endl);
        DEBUG("\n");
        if(update_size == 0)
        {
            DEBUG("Got IMU orientation but just initialized and update_size=0. Ignoring" << std::endl;);
            return;
        }
        // 1. Read orientation and consider m_update_vector
        // - Handle bad (empty) quaternions and normalize
        QuaternionT orientation;
        orientation =  {meas_msg.w,
                        meas_msg.x,
                        meas_msg.y,
                        meas_msg.z};
        // - consider m_update_vector
        // -- extract roll pitch yaw
        auto rpy = euler::get_euler_rpy(orientation.normalized());

        // -- ignore roll pitch yaw according to m_update_vector
        rpy[0] = m_update_vector[STATE_ROLL] ? rpy[0] : state(States::full_state_to_estimated_state[STATE_ROLL]);
        rpy[1] = m_update_vector[STATE_PITCH] ? rpy[1] : state(States::full_state_to_estimated_state[STATE_PITCH]);
        rpy[2] = m_update_vector[STATE_YAW] ? rpy[2] : state(States::full_state_to_estimated_state[STATE_YAW]);

        DEBUG("ORIENTATION: " << rpy.transpose() << std::endl;);
        orientation = euler::get_quat_rpy(rpy[0], rpy[1], rpy[2]).normalized();

        Vector3T measurement;
        measurement = rpy - m_bl_imu_offset_vector;
        measurement[0] = utilities::normalize_angle(measurement[0]);
        measurement[1] = utilities::normalize_angle(measurement[1]);
        measurement[2] = utilities::normalize_angle(measurement[2]);
        measurement = m_bl_imu_rot_yaw * measurement;
        std::cout << "orient:" << measurement.transpose() << std::endl;

        // 3. Transform covariance, not sure for the second transformation
        Matrix3T covariance;
        for(uint i = 0; i<3; ++i)
        {
            for(uint j = 0; j<3; ++j)
            {
                covariance(i, j) = covariance_array[i + j*3];
            }
        }
        covariance = m_bl_imu_rot_yaw * covariance * m_bl_imu_rot_yaw.transpose();

        // 4. Fill sub_measurement vector and sub_covariance matrix, sub_inovation vector
        //  - and state to measurement mapping
        auto offset = POSITION_SIZE;
        for ( uint i = 0; i < update_size; i++)
        {
            sub_measurement(i + ix1) = measurement(update_indices[i] - offset);
            sub_innovation(i + ix1) = sub_measurement(i + ix1) - state(States::full_state_to_estimated_state[update_indices[i]]);
            for (uint j = 0; j < update_size; j++)
            {
                sub_covariance(i + ix1, j + ix1) = covariance(update_indices[i] - offset, update_indices[j] - offset);
            }
            state_to_measurement_mapping(i + ix1, States::full_state_to_estimated_state[update_indices[i]]) = 1.0;
        }

        DEBUG(" -> Noise from orientation msg:\n" << std::fixed << std::setprecision(4) << utilities::printtt(covariance_array, 3, 3));
        DEBUG("\t\t--------------- Imu[" << m_topic_name<< "] Prepare_Orientation: OUT -------------------\n");
    }

    /**
     * @brief Imu: Prepares a Twist msg and fills the corresponding part of the measurement
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
    void prepare_angular_velocity(
        const StateVector& state,
        geometry_msgs::msg::Vector3& meas_msg,
        std::array<double, 9> covariance_array,
        const TransformationMatrix& transform,
        Vector& sub_measurement,
        Matrix& sub_covariance,
        Vector& sub_innovation,
        MappingMatrix& state_to_sub_measurement_mapping,
        const std::vector<uint>& update_indices,
        size_t ix1, size_t update_size, bool is_imu)
    {
        DEBUG("\n\t\t--------------- Imu[" << m_topic_name<< "] Prepare_Angular_Velocity: IN -------------------\n");

        // 1. Extract angular velocities
        // - consider m_update_vector
        Vector3T angular_vel;
        angular_vel[0] = m_update_vector[STATE_V_ROLL] ? meas_msg.x : state(States::full_state_to_estimated_state[STATE_V_ROLL]);
        angular_vel[1] = m_update_vector[STATE_V_PITCH] ? meas_msg.y : state(States::full_state_to_estimated_state[STATE_V_PITCH]);
        angular_vel[2] = m_update_vector[STATE_V_YAW] ? meas_msg.z : state(States::full_state_to_estimated_state[STATE_V_YAW]);

        // - Transform measurement to fusion frame
        auto rot = transform.rotation();
        angular_vel = rot * angular_vel;

        // 2. Compute measurement covariance
        Matrix3T covariance;
        covariance.setZero();
        for (uint i = 0; i < ORIENTATION_SIZE; i++)
        {
            for (uint j = 0; j < ORIENTATION_SIZE; j++)
            {
                covariance(i,j) = covariance_array[i*ORIENTATION_SIZE + j];
            }
        }
        // - Rotate Covariance to fusion frame
        covariance = rot * covariance * rot.transpose();

        // 3. Fill sub_measurement vector and sub_covariance matrix, sub_inovation vector
        //  - and state to measurement mapping
        uint meas_index = 0U;
        uint meas_index2 = 0U;
        for ( uint i = 0; i < update_size; i++)
        {
            meas_index = update_indices[i] - POSE_SIZE - POSITION_SIZE;
            sub_measurement(i + ix1) = angular_vel(meas_index);
            sub_innovation(i + ix1) = angular_vel(meas_index) - state(States::full_state_to_estimated_state[update_indices[i]]);
            for (uint j = 0; j < update_size; j++)
            {
                meas_index2 = update_indices[j] - POSE_SIZE - POSITION_SIZE;
                sub_covariance(i + ix1, j + ix1) = covariance(meas_index, meas_index2);
            }
            state_to_sub_measurement_mapping(i + ix1, States::full_state_to_estimated_state[update_indices[i]]) = 1.0;
        }

        DEBUG(" -> ROT angular_velocity: "<<update_size<<"\n" << std::fixed << std::setprecision(4) << rot << "\n");
        DEBUG(" -> Noise from angular_velocity msg:\n" << std::fixed << std::setprecision(4) << utilities::printtt(covariance_array, 3, 3));
        DEBUG("\t\t--------------- Imu[" << m_topic_name<< "] Prepare_Angular_Velocity: OUT -------------------\n");
    }

    /**
     * @brief Imu: Prepares a Pose msg and fills the corresponding part of the measurement
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
        DEBUG("\n\t\t--------------- Imu[" << m_topic_name<< "] Prepare_Acceleration: IN -------------------\n");
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

        measurement[0] = m_update_vector[STATE_A_X] ? meas_msg.x : state(States::full_state_to_estimated_state[STATE_A_X]);
        measurement[1] = m_update_vector[STATE_A_Y] ? meas_msg.y : state(States::full_state_to_estimated_state[STATE_A_Y]);
        measurement[2] = m_update_vector[STATE_A_Z] ? meas_msg.z : state(States::full_state_to_estimated_state[STATE_A_Z]);
        // std::cout << "G: " << measurement[2]<< "\n";

        // 2. Transform measurement to fusion frame
        auto rot = transform.rotation();

        // 3. Remove gravity from the accelerations
        if (m_remove_gravity)
        {
            Vector3T gravity_acc;
            gravity_acc << 0.0 , 0.0 , 9.8;
            gravity_acc = rot.transpose() * gravity_acc; // R^T = R^^1 for rotation matrixes (transpose instead of inverse)
            gravity_acc[0] = m_update_vector[STATE_A_X] ? gravity_acc[0] : 0;
            gravity_acc[1] = m_update_vector[STATE_A_Y] ? gravity_acc[1] : 0;
            gravity_acc[2] = m_update_vector[STATE_A_Z] ? gravity_acc[2] : 0;
            measurement -= gravity_acc;
        }
        measurement = rot * measurement;
        covariance = rot * covariance * rot.transpose();

        // 4. Fill sub_measurement vector and sub_covariance matrix, sub_inovation vector
        //  - and state to measurement mapping
        auto offset = POSE_SIZE + TWIST_SIZE;
        for ( uint i = 0; i < update_size; i++)
        {
            sub_measurement(i + ix1) = measurement(update_indices[i] - offset);
            sub_innovation(i + ix1) = sub_measurement(i + ix1) - state(States::full_state_to_estimated_state[update_indices[i]]);
            for (uint j = 0; j < update_size; j++)
            {
                sub_covariance(i + ix1, j + ix1) = covariance(update_indices[i] - offset, update_indices[j] - offset);
            }
            state_to_measurement_mapping(i + ix1, States::full_state_to_estimated_state[update_indices[i]]) = 1.0;
        }

        DEBUG(" -> Noise from acceleration msg:\n" << std::fixed << std::setprecision(4) << utilities::printtt(covariance_array, 3, 3));
        DEBUG("\t\t--------------- Imu[" << m_topic_name<< "] Prepare_Acceleration: OUT -------------------\n");
    }

    /**
     * @brief Imu: Getter function that returns the rotation from map to enu frame.
     * @return R_map_enu (3x3Matrix)
     */
    Matrix3T get_R_map_enu()
    {
        return m_R_map_enu;
    }

    /**
     * @brief Imu: Helper function that checks if IMU orientation is initialized.
     * @return true/false
     */
    bool ready()
    {
        return m_init_orientation;
    }
};

using ImuD = Imu<double, motion_model::Ctrv2D::States>;

} // end namespace sensors
} // end namespace state_predictor
} // end namespace iav