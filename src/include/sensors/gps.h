
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
#include <sensor_msgs/msg/NavSatFix.h>

#include <GeographicLib/Geocentric.hpp>
#include <GeographicLib/LocalCartesian.hpp>

namespace iav{ namespace state_predictor { namespace sensors {

template<typename T, typename States>
class Gps : public SensorBase<T, States>
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

    //! @brief Local Cartesian projection around gps origin
    GeographicLib::LocalCartesian m_gps_local_cartesian;
    TransformationMatrix m_T_map_utm;
    bool m_initialized;

public:
    Gps(){}; // default constructor

    Gps(const std::string topic_name, const bool* update_vector,
         const T mahalanobis_threshold, std::ostream* out_stream, bool debug)
        : SensorBaseT(topic_name, update_vector, mahalanobis_threshold, out_stream, debug)
        { }

    inline bool ready() const
    {
        return m_initialized;
    }

    void initialize(const StateVector& state,
                    const Matrix3T R_map_enu,
                    T latitude, T longitude, T hae_altitude,
                    const TransformationMatrix& T_bl_gps)
    {
        DEBUG("\n\t\t--------------- Gps[" << m_topic_name<< "] INITIALIZING: IN -------------------\n");
        //--------------------- CAN GET THIS FROM ROS TOO!! ---------------------
            // 10. create transformation matrix T_map_bl
            TransformationMatrix T_map_bl;
            // 10a- R_map_bl Rotation part
            constexpr uint roll_ix = States::full_state_to_estimated_state[STATE_ROLL],
                        pitch_ix = States::full_state_to_estimated_state[STATE_PITCH],
                        yaw_ix = States::full_state_to_estimated_state[STATE_YAW];
            T roll = roll_ix < STATE_SIZE ? state[roll_ix] : 0,
            pitch = pitch_ix < STATE_SIZE ? state[pitch_ix] : 0,
            yaw = yaw_ix < STATE_SIZE ? state[yaw_ix] : 0;

            T_map_bl = AngleAxisT(roll, Vector3T::UnitX())
                    * AngleAxisT(pitch, Vector3T::UnitY())
                    * AngleAxisT(yaw, Vector3T::UnitZ());
            std::cout << "R_map_bl\n" << T_map_bl.rotation() <<"\n";

            // 10b- P_map_bl Translation part
            constexpr uint x_ix = States::full_state_to_estimated_state[STATE_X],
                        y_ix = States::full_state_to_estimated_state[STATE_Y],
                        z_ix = States::full_state_to_estimated_state[STATE_Z];
            T_map_bl.translation() << (x_ix < STATE_SIZE ? state[x_ix] : 0),
                                    (y_ix < STATE_SIZE ? state[y_ix] : 0),
                                    (z_ix < STATE_SIZE ? state[z_ix] : 0);
            std::cout << "P_map_bl\n" << T_map_bl.translation().transpose() <<"\n";
        // --------------------- CREATE TRANSFORMATION ---------------------

        // R_map_utm = R_map_en; P_map_utm = T_map_bl * P_bl_gps
        // 1. Fill the transformation matrix T_map_utm
        m_T_map_utm = R_map_enu; // a) same orientation as enu, TO_DO: magnetic_declination_ + yaw_offset_ + utm_meridian_convergence_
        m_T_map_utm.translation() = T_map_bl * T_bl_gps.translation(); // b) same position as first gps (P_map_gps)

        // 2. Initialize the local cartesian system,
        // i.e. the origin is at the position of gps sensor and the orientation is ENU
        m_gps_local_cartesian.Reset(latitude, longitude, 0);
        m_initialized = true;

        DEBUG("Initialized at \n"
            << "Rotation: " << std::endl << m_T_map_utm.rotation() << std::endl
            << "Translation: " << std::endl << m_T_map_utm.translation().transpose() << std::endl);
        DEBUG("\n\t\t--------------- Gps[" << m_topic_name<< "] INITIALIZING: OUT -------------------\n");
    }
    /**
     * @brief Gps: Callback for receiving all gps msgs. It processes all comming messages
     * by considering transforming them in the fusing frame, considering the m_update_vector to ignore parts
     * of the measurements and capturing matrixes with faulty values.
     * @param[in] state - vector for the state we are estimating, needed to compute T_map_bl(could use ros too, but first step to independence)
     * @param[in] msg - pointer to the gps msg of the measurement
     * @param[in] transform_to_base_link - transf from sensor frame to base_link frame where angular velocity and acceleration are fused
     */
    Measurement gps_callback(
        const StateVector& state,
        sensor_msgs::msg::NavSatFix* msg,
        const TransformationMatrix& transform_to_base_link)
    {
        DEBUG("\n\t\t--------------- Gps[" << m_topic_name<< "] Gps_callback: IN -------------------\n");
        DEBUG(msg->header.frame_id <<" ~~ m_update_vector: " << utilities::printtt(m_update_vector, 1, STATE_SIZE) << "\n");

        //--------------------- CAN GET THIS FROM ROS TOO!! ---------------------
            // 10. create transformation matrix T_map_bl
            TransformationMatrix T_map_bl;
            // 10a- R_map_bl rotation part
            constexpr uint roll_ix = States::full_state_to_estimated_state[STATE_ROLL],
                        pitch_ix = States::full_state_to_estimated_state[STATE_PITCH],
                        yaw_ix = States::full_state_to_estimated_state[STATE_YAW];
            T roll = roll_ix < STATE_SIZE ? state[roll_ix] : 0,
            pitch = pitch_ix < STATE_SIZE ? state[pitch_ix] : 0,
            yaw = yaw_ix < STATE_SIZE ? state[yaw_ix] : 0;
            T_map_bl = AngleAxisT(roll, Vector3T::UnitX())
                            * AngleAxisT(pitch, Vector3T::UnitY())
                            * AngleAxisT(yaw, Vector3T::UnitZ());

            // 10b- P_map_bl translation part
            constexpr uint x_ix = States::full_state_to_estimated_state[STATE_X],
                        y_ix = States::full_state_to_estimated_state[STATE_Y],
                        z_ix = States::full_state_to_estimated_state[STATE_Z];
            T_map_bl.translation() << (x_ix < STATE_SIZE ? state[x_ix] : 0),
                                    (y_ix < STATE_SIZE ? state[y_ix] : 0),
                                    (z_ix < STATE_SIZE ? state[z_ix] : 0);
        // --------------------- CREATE TRANSFORMATION ---------------------

        // can check but it doesnt make sense if we ignore x,y or z of a gps sensor
        // bool valid_position = m_update_vector[STATE_X] || m_update_vector[STATE_Y] || m_update_vector[STATE_Z];

        // 0. Get POSITION indices
        std::vector<uint> update_indices;
        for (uint i = 0; i < POSITION_SIZE; ++i)
        {
            if(States::full_state_to_estimated_state[i] < STATE_SIZE)
                update_indices.push_back(i);
        }
        size_t update_size = update_indices.size();

        // 1. Get gps POSITION in local UTM cartesian coordinates (P_utm_gps_meas)
        Vector3T measurement = Vector3T::Zero();
        m_gps_local_cartesian.Forward(msg->latitude, msg->longitude, msg->altitude,
                                      measurement[0], measurement[1], measurement[2]);
        std::cout << "P_MAP_GPS_MEASUREMENT: " << measurement.transpose() << "\n";
        DEBUG("P_MAP_GPS_MEASUREMENT: "  << measurement.transpose() << "\n");

        // 2. Get gps POSITION in map frame (P_map_gps_meas)
        measurement = m_T_map_utm * measurement;
        // - rotate the covariance with the same rotation
        Matrix3T covariance;
        for (size_t i = 0; i < POSITION_SIZE; i++)
        {
            for (size_t j = 0; j < POSITION_SIZE; j++)
            {
                covariance(i, j) = msg->position_covariance[POSITION_SIZE * i + j];
            }
        }
        covariance = m_T_map_utm.rotation() * covariance * m_T_map_utm.rotation().transpose();

        // 3. Get base_link POSITION in map frame corresponding to the gps measurement (P_map_bl_meas)
        // In this step no covariance has to be rotated since we are adding only an offset.
        // Only the translation: P_map_bl_meas = P_map_gps_meas - (R_map_bl*P_bl_gps)
        //                                                                └──> bl->gps in map frame
        measurement.noalias() -= T_map_bl.rotation() * transform_to_base_link.translation(); // R_map_bl * P_bl_gps is the translation vector from bl to gps in map frame
        std::cout << "P_MAP_BL_MEASUREMENT:" << measurement.transpose() << "\n";
        DEBUG("P_MAP_BL_MEASUREMENT: "  << measurement.transpose() << "\n");

        // 4. Fill sub_measurement vector and sub_covariance matrix, sub_inovation vector
        //  - and state to measurement mapping
        Vector sub_measurement = Vector::Zero(update_size); // z
        Vector sub_innovation = Vector::Zero(update_size); // z'-z
        Matrix sub_covariance = Matrix::Zero(update_size, update_size);
        MappingMatrix state_to_measurement_mapping = MappingMatrix::Zero(update_size, num_state);
        for ( uint i = 0; i < update_size; i++)
        {
            sub_measurement(i) = measurement(update_indices[i]);
            sub_innovation(i) = sub_measurement(i) - state(States::full_state_to_estimated_state[update_indices[i]]);
            for (uint j = 0; j < update_size; j++)
            {
                sub_covariance(i, j) = covariance(update_indices[i], update_indices[j]);
            }
            state_to_measurement_mapping(i, States::full_state_to_estimated_state[update_indices[i]]) = 1.0;
                // if(sub_covariance(i,i) < 0.0) sub_covariance(i,i) = -sub_covariance(i,i);
                // if(sub_covariance(i,i) < 1e-9) sub_covariance(i,i) = 1e-9;
        }

        // 6. Create measurement to be handled
        tTime stamp_sec = static_cast<tTime>(msg->header.stamp.sec + 1e-9*static_cast<double>(msg->header.stamp.nanosec));
        Measurement meas(stamp_sec, sub_measurement, sub_covariance, sub_innovation, state_to_measurement_mapping,
                        update_indices, msg->header.frame_id, m_mahalanobis_threshold);
        DEBUG(" -> GPS " << meas.print());
        DEBUG("\n\t\t--------------- Gps[" << m_topic_name<< "] Gps_callback: OUT -------------------\n");
        return meas;
    }
};

using GpsD = Gps<double, motion_model::Ctra2D::States>;

} // end namespace sensors
} // end namespace state_predictor
} // end namespace iav