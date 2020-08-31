/*! \file
*
* Copyright (c) 1982-2020 IAV GmbH. All rights reserved.
*
* \authors Mikel Zhobro, Robert Treiber IAV GmbH
*
* Correspondence should be directed to IAV.
*
* IAV GmbH\n
* Carnotstraße 0\n
* 10586 Berlin
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
#include<measurement/measurement.hpp>
namespace iav{ namespace state_predictor { namespace measurement{

// TO_DO: should make sure that sensorconfig has the followin members:
// update_vector: with 1 only for parts of state the measurement offers
// modalities: array with indexes that show to which state the components of measurement correspond
template<int num_meas, typename T = double>
Measurement<T> create_measurement(
    Measurement<T>::Vector measuremen_vector, Measurement<T>::Matrix measurement_covariance,
    const Measurement<T>::Matrix transformation, const tTime time_stamp, const std::uint32_t (&modalities)[num_meas],
    int (&update_vector)[15])
{
    Measurement<T> measurement(time_stamp);
    // .. transform, reduce ..
    measurement.m_measurement_covariance = {1,2,3};
    measurement.m_measurement_vector = {1,2,3};
    measurement.m_measurement_to_state_mapping = {1,2,3};

}

}  // namespace measurement
}  // namespace state_predictor
}  // namespace iav