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

#include<iostream>
#include<iomanip>
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
public:
    using MappingMatrix = Eigen::Matrix<T, Eigen::Dynamic, num_state>;
    using Vector = Eigen::Matrix<T, Eigen::Dynamic, 1>;
    using Matrix = Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>;

public:
    // Time stamp of the measurement
    tTime m_time_stamp;
    // Sensor id from where the measurement came.
    std::string m_sensor_id;
    // Mahalanobis threshold for the measurement.
    T m_mahalanobis_thresh;
    // Measurement vector to be considered
    Vector z;
    // The precalculated innovation(spares computations, instead of H*x)
    Vector innovation;
    // Measurement covariance matrix
    Matrix R;
    // Maps the state to the measurement, in literature known as matrix H
    MappingMatrix H;
    // corresponging full-state-indexes of the measurement vector z
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
    Measurement(const tTime& time_stamp, const Vector& measurement, const Matrix& covariance,
                const Vector& innovation, const MappingMatrix& map_matrix,
                const std::vector<uint>& update_indices, const std::string& sensor_id, const T& mahalanobis_thresh):
                m_time_stamp{time_stamp}, m_sensor_id{sensor_id}, z{measurement},
                H{map_matrix}, m_update_indices{update_indices},
                innovation{innovation}, m_mahalanobis_thresh{mahalanobis_thresh}
    {
        R = covariance;
        for (uint i = 0; i < R.rows(); i++)
        {
            if(R(i,i) < 0.0)
            {
                R(i,i) = std::fabs(R(i,i));
                // std::cout << "covariance smaller than 0\n";
            }
            if(R(i,i) < 1e-9)
            {
                R(i,i) = 1e-9;
                // std::cout << "covariance near 0\n";
            }
        }
    }

/**
 * @brief Measurement: Getter function for the measurement vector.
 * @return Measurement vector.
 */
    inline const Vector get_measurement() { return z; }

/**
 * @brief Measurement: Getter function for the measurement covariance matrix.
 * @return Measurement covariance matrix.
 */
    inline const Matrix get_covariance() { return R; }

/**
 * @brief Measurement: Getter function for the mapping matrix.
 * @return State to measurement mapping matrix.
 */
    inline const MappingMatrix get_mapping() { return H; }

/**
 * @brief Measurement: Helping function to print measurement information.
 * @return String with measurement information.
 */
    std::string print()
    {
        std::ostringstream out;
        out << "SUBMEASUREMENT ->time: " << m_time_stamp << ", ->frame: " <<m_sensor_id<< ", ->mahal_thresh: " <<m_mahalanobis_thresh << "\n";
        out << " ----> Update indices: "; for(auto i: m_update_indices) out << i <<" "; out << "\n";
        out << " ----> Submeasurement:\n"  << std::fixed << std::setprecision(4) << z.transpose() << "\n";
        out << " ----> Subinnovation:\n"  << std::fixed << std::setprecision(4) << innovation.transpose() << "\n";
        out << " ----> Submapping H:\n"  << std::fixed << std::setprecision(0) << H << "\n";
        out << " ----> Subnoise R:\n"  << std::fixed << std::setprecision(4) << R << "\n";
        return out.str();
    }

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