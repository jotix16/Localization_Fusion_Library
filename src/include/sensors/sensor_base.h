
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
#include <measurement/measurement.h>
#include <motion_model/motion_model.h>

#include <geometry_msgs/msg/Vector3.h>
#include <geometry_msgs/msg/PoseWithCovariance.h>
#include <geometry_msgs/msg/TwistWithCovariance.h>
#include <nav_msgs/msg/Odometry.h>
#include <sensor_msgs/msg/Imu.h>

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
    const bool* m_update_vector;
    const T m_mahalanobis_threshold;

    // debugging
    bool m_debug;
    std::ostream* m_debug_stream;

public:
    SensorBase(const std::string topic_name, const bool* update_vector, const T mahalanobis_threshold, std::ostream* out_stream, bool debug): 
    m_debug(debug),
    m_topic_name(topic_name),
    m_update_vector(update_vector),
    m_mahalanobis_threshold(mahalanobis_threshold)
    { 
        if(debug) setDebug(out_stream);
    }

    void setDebug(std::ostream* out_stream)
    {
            m_debug_stream = out_stream;
            DEBUG("\t\t\t-----------------------------------------\n");
            DEBUG("\t\t\t----- /Sensor::"<< m_topic_name <<" is on!" << " ------\n");
            DEBUG("\t\t\t-----------------------------------------\n");
    }

};

template<typename T, typename States>
constexpr uint SensorBase<T, States>::num_state;

} // end namespace sensors 
} // end namespace state_predictor
} // end namespace iav