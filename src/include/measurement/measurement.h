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

template<typename T = double, uint... measurement_modalities>
class Measurement
{
    static constexpr std::int32_t EntriesCount = sizeof...(measurement_modalities);
    using ModalityArray = std::array<uint, static_cast<std::size_t>(EntriesCount)>;
    using MeasurementVector = Eigen::Matrix<T, EntriesCount, 1>;
    using CovarianceMatrix = Eigen::Matrix<T, EntriesCount, EntriesCount>;
    template<uint kNumOfCols>
    using MapMatrix = Eigen::Matrix<T, EntriesCount, kNumOfCols>;

public:
    tTime m_time_stamp;
    std::string m_sensor_id;
    static constexpr ModalityArray m_modality{measurement_modalities ...};
    MeasurementVector m_measurement_vector;
    CovarianceMatrix m_measurement_covariance;

public:
    Measurement(tTime time_stamp, std::string sensor_id): m_time_stamp(time_stamp), m_sensor_id(sensor_id)
    {
    }

    inline const MeasurementVector get_measurement() { return m_measurement_vector; }
    inline const CovarianceMatrix get_covariance() { return m_measurement_covariance; }
    template<uint num_state>
    inline const CovarianceMatrix get_mapping() 
    { 
        using MappingMatrix = MapMatrix<num_state>;
        return m_measurement_to_state_mapping; 
    }

    bool operator<(Measurement m2) { return m_time_stamp < m2.m_time_stamp; }

};

template<typename T = double, uint... measurement_modalities>
constexpr typename Measurement<T, measurement_modalities...>::ModalityArray
Measurement<T, measurement_modalities...>::m_modality;

}  // namespace measurement
}  // namespace state_predictor
}  // namespace iav