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

#include<eigen/Eigen>

#include<motion_model/motion_model.h>


namespace iav{ namespace state_predictor { namespace filter {

template<class MotionModelT, int num_state, typename T = double>
class FilterBase
{
public:
    using StateVector = typename MotionModelT::StateVector;
    using StateMatrix = typename MotionModelT::StateMatrix;
    using ObservationVector = typename Eigen::Matrix<T, -1, 1>;
    using ObservationMatrix = typename Eigen::Matrix<T, -1, num_state>;
    using MeasurementNoisematrix = typename Eigen::Matrix<T, -1, -1>;

protected:
    bool m_initialized;
    MotionModelT m_motion_model;
    StateVector m_state;
    StateMatrix m_covariance;
    StateMatrix m_process_noise;

    /* data */
public:
    FilterBase(/* args */):m_initialized(0){};

    bool is_initialized()
    {
        return m_initialized
    }
    void reset(StateVector x0, StateMatrix P0)
    {
        m_state = x0;
        m_covariance = P0;
    }
    StateVector get_state()
    {
        return m_state;
    }
    StateMatrix get_covariance()
    {
        return m_covariance;
    }
    virtual bool passes_mahalanobis(ObservationVector innovation, MeasurementNoisematrix hph_t_r, T mahalanobis_threshold)=0;
    virtual bool temporal_update(tTime dt)=0;
    virtual bool observation_update(ObservationVector z, ObservationMatrix H, MeasurementNoisematrix R)=0;
};
} // end namespace filter 
} // end namespace state_predictor
} // end namespace iav