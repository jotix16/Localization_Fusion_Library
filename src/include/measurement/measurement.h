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

#include <string>
#include <array>

#include <Eigen/Eigen>

#include <utilities/filter_utilities.h>

namespace iav{ namespace state_predictor { namespace measurement{

/**
 * @brief Measurement class that aims to unify all times of measurements
 *        in order for them to be treated equally afterwards.
 */
// forward declaration for compatibility with gcc in Linux
template<uint num_state, typename T = double> 
class Measurement;

template<uint num_state, typename T> 
bool operator< (Measurement<num_state, T> m1, Measurement<num_state, T> m2);
// --------------------------------------------------------

template<uint num_state, typename T>
class Measurement
{
    using MappingMatrix = Eigen::Matrix<T, -1, num_state>;
    using MeasurementVector = Eigen::Matrix<T, -1, 1>;
    using CovarianceMatrix = Eigen::Matrix<T, -1, -1>;

public:
    // Time stamp of the measurement
    tTime m_time_stamp;
    // Sensor id from where the measurement came.
    std::string m_sensor_id;
    // Mahalanobis threshold for the measurement.
    T m_mahalanobis_thresh;
    // Measurement vector to be considered 
    // MeasurementVector m_measurement_vector;
    MeasurementVector z;
    // The precalculated innovation(spares computations, instead of H*x)
    // MeasurementVector m_innovation;
    MeasurementVector innovation;
    // Measurement covariance matrix
    // CovarianceMatrix m_measurement_covariance;
    CovarianceMatrix R;
    // Maps the state to the measurement, in literature known as matrix H
    // MappingMatrix m_state_to_measurement_mapping;
    MappingMatrix H;

    std::vector<uint> m_update_indices;

public:
/**
 * @brief Measurement: Constructor tha creates the measurement.
 * @param[in] time_stamp - Time stamp of the measurement
 * @param[in] measurement - Measurement vector to be considered 
 * @param[in] covariance - Measurement covariance matrix
 * @param[in] innovation - The precalculated innovation(spares computations, instead of H*x)
 * @param[in] map_matrix - Maps the state to the measurement, in literature known as matrix H
 * @param[in] sensor_id - Sensor id from where the measurement came.
 * @param[in] mahalanobis_thresh - Mahalanobis threshold for the measurement.
 */
    Measurement(const tTime time_stamp, const MeasurementVector& measurement, const CovarianceMatrix& covariance,
                const MeasurementVector& innovation, const MappingMatrix map_matrix, 
                const std::vector<uint> update_indices, const std::string& sensor_id, const T& mahalanobis_thresh):
                m_time_stamp{time_stamp}, m_sensor_id{sensor_id}, z{measurement},
                R{covariance}, H{map_matrix}, m_update_indices{update_indices},
                innovation{innovation}, m_mahalanobis_thresh{mahalanobis_thresh}

    { }

/**
 * @brief Measurement: Getter function for the measurement vector.
 * @return Measurement vector.
 */
    inline const MeasurementVector get_measurement() { return z; }

/**
 * @brief Measurement: Getter function for the measurement covariance matrix.
 * @return Measurement covariance matrix.
 */
    inline const CovarianceMatrix get_covariance() { return R; }

/**
 * @brief Measurement: Getter function for the mapping matrix.
 * @return State to measurement mapping matrix.
 */
    inline const MappingMatrix get_mapping() { return H; }
  
/**
 * @brief Measurement: Defines the operator < as friend of the class. In order to be able to compare measurements.
 * @param[in] m1 - First measurement
 * @param[in] m2 - Second measurement
 * @return returns true if m1 id older than m2.
 */
    // template<uint num_state, typename T> // for the case we keep members private and use getters
    friend bool operator< <>(Measurement<num_state, T> m1, Measurement<num_state, T> m2);
};

/**
 * @brief Measurement: Overrides the operator "<" in order to be able to compare measurements.
 * @param[in] m1 - First measurement
 * @param[in] m2 - Second measurement
 * @return returns true if m1 id older than m2.
 */
template<uint num_state, typename T = double>
bool operator<(Measurement<num_state, T> m1, Measurement<num_state, T> m2)
{
    return m1.m_time_stamp < m2.m_time_stamp;
}

}  // namespace measurement
}  // namespace state_predictor
}  // namespace iav