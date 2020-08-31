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

#include<string>

#include<eigen/Eigen>

#include<utilities/filter_utilities.h>

#pragma once
namespace iav{ namespace state_predictor { namespace measurement{

template<typename T=double>
class Measurement
{
public:
    using Vector = Eigen::Matrix<T, -1, 1>;
    using Matrix = Eigen::Matrix<T, -1, -1>;
    using Vector = Eigen::Matrix<T, -1, 1>;
    using Matrix = Eigen::Matrix<T, -1, -1>;

public:
    tTime m_time_stamp;
    // SensorConfig* m_sensor_config; // pointer to sensor config to get update_vector and mahalanobi distances
    Vector m_measurement_vector;
    Matrix m_measurement_covariance;
    Matrix m_measurement_to_state_mapping;

public:
    Measurement(tTime time_stamp, std::string sensor_id): m_time_stamp(time_stamp)
    {
    }

    bool operator<(Measurement m2)
    {
        return m_time_stamp < m2.m_time_stamp;
    }



};

}  // namespace measurement
}  // namespace state_predictor
}  // namespace iav