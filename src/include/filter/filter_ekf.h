
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

#include <filter/filter_base.h>
#include <utilities/filter_utilities.h>


namespace iav{ namespace state_predictor { namespace filter {

//TO_DO: we can use dynamic process noise covariance. Especially for cases when
// we don't want to increase the covariance estimation of the state when the vehicle is not moving

/**
 * @brief Filter class that inherits from FilterBase class
 * @param<template> MotionModelT - The motion model used. This defines the state to be estimated too.
 * @param<template> num_state - Size of the state
 * @param<template> T - Type that should be used for calculations(default is double, but float can be used too)
 */
//TO_DO: we can use dynamic process noise covariance. Especially for cases when
// we don't want to increase the covariance estimation of the state when the vehicle is not moving
template<class MotionModelT, int num_state, typename T = double>
class FilterEkf : public FilterBase<MotionModelT, num_state, T>
{
public:
    using FilterBase_ = FilterBase<MotionModelT, num_state, T>;
    using StateVector = typename FilterBase_::StateVector;
    using StateMatrix = typename FilterBase_::StateMatrix;
    using ObservationVector = typename FilterBase_::ObservationVector;
    using ObservationMatrix = typename FilterBase_::ObservationMatrix;
    using Matrix = typename FilterBase_::Matrix;
    using Vector = typename FilterBase_::Vector;
    using States = typename FilterBase_::States;

public:
    /**
     * @brief FilterBase: Default constructor
     */
    FilterEkf(){};

    bool passes_mahalanobis(const ObservationVector& innovation, const Matrix& hph_t_r_inv, const T& mahalanobis_threshold)
    {
        T sq_measurement_mahalanobis = innovation.dot(hph_t_r_inv * innovation);
        T threshold = mahalanobis_threshold * mahalanobis_threshold;
        if(sq_measurement_mahalanobis <= threshold) return true;
        return false;
    }

    bool temporal_update(const tTime& dt)
    {
        if(!this->m_initialized) return false;

        // get jacobian matrix
        StateMatrix jacobian;
		this->m_motion_model.compute_jacobian_and_predict(jacobian,this->m_state, dt);

        // wrap angles of state
        for (uint i = States::ORIENTATION_OFFSET_M; i < States::POSITION_V_OFFSET_M; i++)
        {
           this->m_state[i] = utilities::clamp_rotation(this->m_state[i]);
        }
        
        // update the covariance: P = J * P * J' + Q
        this->m_covariance = jacobian *this->m_covariance * jacobian.transpose();
        this->m_covariance.noalias() += dt *this->m_process_noise;
        return true;
    }

    bool observation_update(const ObservationVector& z, ObservationVector& innovation, const ObservationMatrix& H, const Matrix& R, const T& mahalanobis_threshold)
    {
        // Check if initialized
        if(!this->m_initialized) return false;
        Matrix ph_t =this->m_covariance * H.transpose();
        Matrix hph_t_r_inv = (H * ph_t + R).inverse();

        // wrap angles of innovation
        for (uint j = States::ORIENTATION_OFFSET_M; j < States::POSITION_V_OFFSET_M; j++)
        {
            for (uint i = 0; i < H.rows(); i++)
            {
                if(H(i,j)>0.5)
                {
                    innovation[i] = utilities::clamp_rotation(innovation[i]);
                    break;
                }
            }
        }
        // check mahalanobis distance
        if(!passes_mahalanobis(innovation, hph_t_r_inv, mahalanobis_threshold)) return false;

        Matrix K(num_state, z.rows());
        K.setZero();
        K.noalias() = ph_t * hph_t_r_inv;
        this->m_state.noalias() += K * innovation;

        // wrap angles of state
        for (uint i = States::ORIENTATION_OFFSET_M; i < States::POSITION_V_OFFSET_M; i++)
        {
           this->m_state[i] = utilities::clamp_rotation(this->m_state[i]);
        }

        // Correct the covariance using Joseph form for stability. This is the
        // same as P = P - K * H * P, but presents less of a problem in the
        // presense of floating point roundoff.
        // The Joseph Update has the form: P = (I - KH)P(I - KH)' + KRK'
        // TO_DO: if covariance diagonal elements are near 0 give them a small value 
        StateMatrix I_K_H =this->m_identity;
        I_K_H.noalias() -= K * H;
        this->m_covariance = I_K_H *this->m_covariance * I_K_H.transpose();
        this->m_covariance.noalias() += K * R * K.transpose();
        return true;
    }
};

// explicit template initialization
using Ctrv_EKF2D = FilterEkf<motion_model::Ctrv2D, 6, double>;
using Ctra_EKF2D = FilterEkf<motion_model::Ctra2D, 6, double>;
using Ctra_EKF3D = FilterEkf<motion_model::Ctra3D, 6, double>;

} // end namespace filter 
} // end namespace state_predictor
} // end namespace iav