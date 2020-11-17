
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
#include <utilities/filter_euler_zyx.h>
#include <measurement/measurement.h>
#include <motion_model/motion_model.h>

namespace iav{ namespace state_predictor { namespace sensors {

template<typename T, typename States>
class SensorBase
{
public:
    static constexpr uint num_state = States::STATE_SIZE_M;

    using Measurement = typename measurement::Measurement<num_state, T>;

    using StateVector =   typename Eigen::Matrix<T, num_state, 1>;
    using MappingMatrix = typename Eigen::Matrix<T, Eigen::Dynamic, num_state>;
    using Vector =        typename Eigen::Matrix<T, Eigen::Dynamic, 1>;
    using Matrix =        typename Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;
    using Matrix6T =      typename Eigen::Matrix<T, 6, 6>;
    using Matrix4T =      typename Eigen::Matrix<T, 4, 4>;
    using Matrix3T =      typename Eigen::Matrix<T, 3, 3>;
    using Vector6T =      typename Eigen::Matrix<T, 6, 1>;
    using Vector3T =      typename Eigen::Matrix<T, 3, 1>;

    using TransformationMatrix = typename Eigen::Transform<T, 3, Eigen::TransformTraits::Isometry>;
    using QuaternionT = typename Eigen::Quaternion<T>;
    using AngleAxisT = typename Eigen::AngleAxis<T>;

protected:
    // sensor config
    const std::string m_topic_name;
    bool m_update_vector[STATE_SIZE];
    T m_mahalanobis_threshold;

    // debugging
    bool m_debug;
    std::ostream* m_debug_stream;

public:
    SensorBase(){};

    SensorBase(const std::string topic_name, const bool* update_vector,
               const T mahalanobis_threshold, std::ostream* out_stream, bool debug):
    m_debug(debug),
    m_topic_name(topic_name),
    m_mahalanobis_threshold(mahalanobis_threshold)
    {
        for(uint i=0; i<STATE_SIZE; ++i) m_update_vector[i] = update_vector[i];
        if(debug) setDebug(out_stream);
    }

    void setDebug(std::ostream* out_stream)
    {
            m_debug_stream = out_stream;
            DEBUG("\t\t\t-----------------------------------------\n");
            DEBUG("\t\t\t----- /Sensor::"<< m_topic_name <<" is on!" << " ------\n");
            DEBUG("\t\t\t-----------------------------------------\n");
    }
    Matrix3T get_rotation_from_state(const StateVector& state)
    {
        T r ,p ,y;
        uint roll_ix = States::full_state_to_estimated_state[STATE_ROLL];
        uint pitch_ix = States::full_state_to_estimated_state[STATE_PITCH];
        uint yaw_ix = States::full_state_to_estimated_state[STATE_YAW];
        r = roll_ix < STATE_SIZE ? state(roll_ix) : 0.0;
        p = pitch_ix < STATE_SIZE ? state(pitch_ix) : 0.0;
        y = yaw_ix < STATE_SIZE ? state(yaw_ix) : 0.0;
        return euler::quat_to_rot(euler::get_quat_rpy(r, p, y).normalized());
    }

    Vector3T get_translation_from_state(const StateVector& state)
    {
        T x ,y ,z;
        uint x_ix = States::full_state_to_estimated_state[STATE_X];
        uint y_ix = States::full_state_to_estimated_state[STATE_Y];
        uint z_ix = States::full_state_to_estimated_state[STATE_Z];
        x = x_ix < STATE_SIZE ? state(x_ix) : 0.0;
        y = y_ix < STATE_SIZE ? state(y_ix) : 0.0;
        z = z_ix < STATE_SIZE ? state(z_ix) : 0.0;
        return {x, y, z};
    }

};

template<typename T, typename States>
constexpr uint SensorBase<T, States>::num_state;

using SensorD = SensorBase<double, motion_model::Ctrv2D::States>;

} // end namespace sensors
} // end namespace state_predictor
} // end namespace iav