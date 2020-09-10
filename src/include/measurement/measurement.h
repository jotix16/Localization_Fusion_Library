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

#include<string>
#include<array>

#include<eigen/Eigen>

#include<utilities/filter_utilities.h>

namespace iav{ namespace state_predictor { namespace measurement{

template<uint num_state, typename T = double>
class Measurement
{
    using MappingMatrix = Eigen::Matrix<T, -1, num_state>;
    using MeasurementVector = Eigen::Matrix<T, -1, 1>;
    using CovarianceMatrix = Eigen::Matrix<T, -1, -1>;

public:
    tTime m_time_stamp;
    std::string m_sensor_id;
    T m_mahalanobis_thresh;
    MeasurementVector m_measurement_vector;
    MeasurementVector m_innovation;
    CovarianceMatrix m_measurement_covariance;
    MappingMatrix m_state_to_measurement_mapping;

public:
    Measurement(const tTime time_stamp, const MeasurementVector& measurement,
                const CovarianceMatrix& covariance, const MeasurementVector& innovation, 
                const MappingMatrix mapp_mat, const std::string& sensor_id, const T& mahalanobis_thresh):
                m_time_stamp{time_stamp}, m_sensor_id{sensor_id}, m_measurement_vector{measurement},
                m_measurement_covariance{covariance}, m_state_to_measurement_mapping{mapp_mat},
                m_innovation{innovation}, m_mahalanobis_thresh{mahalanobis_thresh}

    { }

    inline const MeasurementVector get_measurement() { return m_measurement_vector; }
    inline const CovarianceMatrix get_covariance() { return m_measurement_covariance; }
    inline const MappingMatrix get_mapping() { return m_state_to_measurement_mapping; }
  
    template<uint num_state, typename T> // for the case we keep members private and use getters
    friend bool operator<(Measurement<num_state, T> m1, Measurement<num_state, T> m2);

};

template<uint num_state, typename T = double>
bool operator<(Measurement<num_state, T> m1, Measurement<num_state, T> m2)
{
    return m1.m_time_stamp < m2.m_time_stamp;
}

}  // namespace measurement
}  // namespace state_predictor
}  // namespace iav