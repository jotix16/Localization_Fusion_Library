
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
#include <geometry_msgs/msg/Vector3.h>
#include <geometry_msgs/msg/PoseWithCovariance.h>
#include <geometry_msgs/msg/TwistWithCovariance.h>
// #include <sensor_msgs/msg/Gps.h>

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
                    TransformationMatrix R_map_enu,
                    T latitude, T longitude, T hae_altitude,
                    const TransformationMatrix& transform_to_base_link)
    {

        // 1. create transformation matrix T_map_bl
        TransformationMatrix T_map_bl;
        // - R_map_bl rotation part
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

        // - P_map_bl translation part
        constexpr uint x_ix = States::full_state_to_estimated_state[STATE_X],
                       y_ix = States::full_state_to_estimated_state[STATE_Y],
                       z_ix = States::full_state_to_estimated_state[STATE_Z];
        T_map_bl.translation() << (x_ix < STATE_SIZE ? state[x_ix] : 0),
                                  (y_ix < STATE_SIZE ? state[y_ix] : 0),
                                  (z_ix < STATE_SIZE ? state[z_ix] : 0);
        std::cout << "P_map_bl\n" << T_map_bl.translation().transpose() <<"\n";


        // R_map_utm = R_map_en; P_map_utm = T_map_bl * P_bl_gps
        // Fill the transformation matrix T_map_utm
        m_T_map_utm = R_map_enu; // same orientation as enu
        m_T_map_utm.translation() = T_map_bl * transform_to_base_link.translation(); // same position as first gps

        m_gps_local_cartesian.Reset(latitude, longitude, hae_altitude);
        m_initialized = true;
        std::cout << "Initialized at \n";
        std::cout << "Rotation: " << std::endl << m_T_map_utm.rotation() << std::endl;
        std::cout << "Translation: " << std::endl << m_T_map_utm.translation() << std::endl;
    }
    /**
     * @brief Gps: Callback for receiving all gps msgs. It processes all comming messages
     * by considering transforming them in the fusing frame, considering the m_update_vector to ignore parts
     * of the measurements and capturing matrixes with faulty values.
     * @param[in] state - vector for the state we are estimating, needed to compute T_map_bl(could use ros too, but first step to independence)
     * @param[in] msg - pointer to the gps msg of the measurement
     * @param[in] transform_to_base_link - transf from sensor frame to base_link frame where angular velocity and acceleration are fused
     */
    //TO_DO: have to use local msg instead of latitude, longitude and hae_altitude
    Measurement gps_callback(
        const StateVector& state,
        T latitude, T longitude, T hae_altitude,
        const TransformationMatrix& transform_to_base_link)
    {
        DEBUG("\n\t\t--------------- Gps[" << m_topic_name<< "] Gps_callback: IN -------------------\n");
        // DEBUG(msg->header.frame_id <<" ~~ m_update_vector: " << utilities::printtt(m_update_vector, 1, STATE_SIZE) << "\n");

        // can check but it doesnt make sense if we ignore xyz of a gps sensor
        // bool valid_position = m_update_vector[STATE_X] || m_update_vector[STATE_Y] || m_update_vector[STATE_Z];

        // 1. get gps position(P_utm_gps) in local UTM cartesian coordinates
        Vector3T measurement = Vector3T::Zero();
        m_gps_local_cartesian.Forward(latitude, longitude, hae_altitude,
                                      measurement[0], measurement[1], measurement[2]);

        // 2. transform gps position in map frame P_map_gps 
        std::cout << "WOW we got " << measurement.transpose() << "\n";
        measurement = m_T_map_utm * measurement;
        
        // 3. create transformation matrix T_map_bl
        TransformationMatrix T_map_bl;
        // 3a- R_map_bl rotation part
        constexpr uint roll_ix = States::full_state_to_estimated_state[STATE_ROLL],
                       pitch_ix = States::full_state_to_estimated_state[STATE_PITCH],
                       yaw_ix = States::full_state_to_estimated_state[STATE_YAW];
        T roll = roll_ix < STATE_SIZE ? state[roll_ix] : 0,
          pitch = pitch_ix < STATE_SIZE ? state[pitch_ix] : 0,
          yaw = yaw_ix < STATE_SIZE ? state[yaw_ix] : 0;
        T_map_bl = AngleAxisT(roll, Vector3T::UnitX()) 
                         * AngleAxisT(pitch, Vector3T::UnitY()) 
                         * AngleAxisT(yaw, Vector3T::UnitZ());

        // 3b- P_map_bl translation part
        constexpr uint x_ix = States::full_state_to_estimated_state[STATE_X],
                       y_ix = States::full_state_to_estimated_state[STATE_Y],
                       z_ix = States::full_state_to_estimated_state[STATE_Z];
        T_map_bl.translation() << (x_ix < STATE_SIZE ? state[x_ix] : 0),
                                  (y_ix < STATE_SIZE ? state[y_ix] : 0),
                                  (z_ix < STATE_SIZE ? state[z_ix] : 0);

        // 4. get base_link position(P_map_bl_meas) in map frame corresponding to the gps measurement
        // - P_map_bl_meas = P_map_gps_meas * P_bl_gps^-1
        measurement.noalias() -= T_map_bl * transform_to_base_link.translation(); // T_map_bl * P_bl_gps is the translation vector from bl to gps in map frame
        std::cout << "WOW transformed:" << measurement.transpose() << "\n";

        // TO_DO:
        // 5. Transorm covariance of the gps measurment
        // ---

        // 6. Create measurement to be handled
        // TO_DO: clarify how to determine the pose and twist mahalanobis thresholds
        // T mahalanobis_thresh = 400;
        // tTime stamp_sec = static_cast<tTime>(msg->header.stamp.sec + 1e-9*static_cast<double>(msg->header.stamp.nanosec));
        // Measurement meas(stamp_sec, sub_measurement, sub_covariance, sub_innovation, state_to_measurement_mapping,
        //                 sub_u_indices, msg->header.frame_id, mahalanobis_thresh);
        // DEBUG(" -> GPS " << meas.print());                
        DEBUG("\n\t\t--------------- Gps[" << m_topic_name<< "] Gps_callback: OUT -------------------\n");
    }
};

using GpsD = Gps<double, motion_model::Ctra2D::States>;

} // end namespace sensors 
} // end namespace state_predictor
} // end namespace iav