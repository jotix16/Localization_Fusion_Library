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
    using States = typename MotionModelT::States;
    using StateVector = typename MotionModelT::StateVector;
    using StateMatrix = typename MotionModelT::StateMatrix;
    using Matrix = typename Eigen::Matrix<T, -1, -1>;
    using Vector = typename Eigen::Matrix<T, -1, 1>;
    using ObservationVector = Vector;
    using ObservationMatrix = typename Eigen::Matrix<T, -1, num_state>;

protected:
    bool m_initialized;
    MotionModelT m_motion_model;
    StateVector m_state;
    StateMatrix m_covariance;
    StateMatrix m_process_noise;

    const StateMatrix m_identity ; // needed for a few operations.

public:
    FilterBase():m_initialized(0),m_identity(StateMatrix::Identity())
    {
    }

    inline bool is_initialized() const
    {
        return m_initialized;
    }
    void reset(const StateVector& x0, const StateMatrix& P0, const StateMatrix& Q)
    {
        m_state = x0;
        m_covariance = P0;
        m_process_noise = Q;
        m_initialized = true;
    }
    inline StateVector get_state() const
    {
        return m_state;
    }
    inline StateMatrix get_covariance() const
    {
        return m_covariance;
    }
    virtual bool passes_mahalanobis(const ObservationVector& innovation, const Matrix& hph_t_r_inv, const T& mahalanobis_threshold)=0;
    virtual bool temporal_update(const tTime& dt)=0;
    virtual bool observation_update(const ObservationVector& z, const ObservationMatrix& H, const Matrix& R, const T& mahalanobis_threshold)=0;
};
} // end namespace filter 
} // end namespace state_predictor
} // end namespace iav