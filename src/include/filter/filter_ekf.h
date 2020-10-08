
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
#include <Eigen/Eigenvalues>

namespace iav{ namespace state_predictor { namespace filter {

//TO_DO: we can use dynamic process noise covariance. Especially for cases when
// we don't want to increase the covariance estimation of the state when the vehicle is not moving

/**
 * @brief Filter class that inherits from FilterBase class
 * @param<template> MotionModelT - The motion model used. This defines the state to be estimated too.
 * @param<template> T - Type that should be used for calculations(default is double, but float can be used too)
 */
//TO_DO: we can use dynamic process noise covariance. Especially for cases when
// we don't want to increase the covariance estimation of the state when the vehicle is not moving
template<class MotionModelT>
class FilterEkf : public FilterBase<MotionModelT>
{
public:
    using FilterBaseT = FilterBase<MotionModelT>;
    using T = typename FilterBaseT::T;
    using Measurement = typename FilterBaseT::Measurement;
    using StateVector = typename FilterBaseT::StateVector;
    using StateMatrix = typename FilterBaseT::StateMatrix;
    using Matrix = typename FilterBaseT::Matrix;
    using Vector = typename FilterBaseT::Vector;
    using States = typename FilterBaseT::States;

public:
    /**
     * @brief FilterBase: Default constructor
     */
    int debug = 1;
    FilterEkf(){};

    bool passes_mahalanobis(const Vector& innovation, const Matrix& hph_t_r_inv, const T& mahalanobis_threshold)
    {
        T sq_measurement_mahalanobis = innovation.dot(hph_t_r_inv * innovation);
        T threshold = mahalanobis_threshold * mahalanobis_threshold;
        // Eigen::EigenSolver<Matrix> eigensolver;
        // eigensolver.compute(hph_t_r_inv);
        // DEBUG("--------------- FilterEKF Mahalanobis: hph_t_r_inv: ---------------\n" << hph_t_r_inv << "\n");
        // DEBUG("--------------- FilterEKF Mahalanobis: Eigenvalues: ---------------\n " << eigensolver.eigenvalues().real().transpose() << "\n\n");
        
        if(sq_measurement_mahalanobis > threshold) 
        {
            DEBUG("--------------- FilterEKF Mahalanobis:" << "threshold: " << threshold << " value: " << sq_measurement_mahalanobis << " ---------------\n");
            return false;
        }
        return true;
    }

    bool temporal_update(const tTime& dt)
    {
        DEBUG("\n\t\t--------------- FilterEKF Temporal_Update: IN ---------------\n");
        if(!this->m_initialized) return false;

        // get jacobian matrix
        StateMatrix jacobian;
		this->m_motion_model.compute_jacobian_and_predict(jacobian,this->m_state, dt);

        // wrap angles of state
        for (uint i = States::ORIENTATION_OFFSET_M; i < States::POSITION_V_OFFSET_M; i++)
        {
           if(debug > 1) DEBUG("--------------- FilterEKF before clamp, yaw: " << this->m_state[i] <<"\n");
           this->m_state[i] = utilities::clamp_rotation(this->m_state[i]);
           if(debug > 1) DEBUG("--------------- FilterEKF after clamp, yaw: " << this->m_state[i] <<"\n");
        }
        
        // update the covariance: P = J * P * J' + Q
        this->m_covariance = jacobian *this->m_covariance * jacobian.transpose();
        this->m_covariance.noalias() += this->m_process_noise;

        DEBUG("\t\t--------------- FilterEKF Temporal_Update: OUT ---------------\n");
        return true;
    }

    bool observation_update(const Measurement& m) 
    {
        DEBUG("\n\t\t--------------- FilterEKF Observation_Update: IN ---------------\n");
        
        // Check if initialized
        if(!this->m_initialized)
        {
            if(debug > 1) DEBUG("\n---------------FilterEKF: Filter not initialized ---------------\n");
            return false;
        }
        if(debug > 1) DEBUG("H matrix: \n" << m.H << "\n");
        Matrix ph_t = this->m_covariance * m.H.transpose();
        Matrix hph_t_r_inv = (m.H * ph_t + m.R).inverse();

        // Compute innovation 
        // avoid calculating innovation directly like
        // innov = z - Hx, instead calculate it with a loop and update_indices 
        // O(n) complexity instead of O(mn)
        // wrap angles of innovation
        uint meas_index = 0U;
        Vector innovation(m.m_update_indices.size());
        for ( uint i = 0; i < m.m_update_indices.size(); i++)
        {
            meas_index = m.m_update_indices[i];
            innovation[i] = m.z(i) - this->m_state(States::full_state_to_estimated_state[meas_index]);
            // clamp
            if(meas_index < STATE_V_X && meas_index > STATE_Z) innovation[i] = utilities::clamp_rotation(innovation[i]);
        }

        // check mahalanobis distance
        if(!passes_mahalanobis(innovation, hph_t_r_inv, m.m_mahalanobis_thresh))
        {
            return false;
        }

        Matrix K(this->num_state, m.z.rows());
        K.setZero();
        K.noalias() = ph_t * hph_t_r_inv;
        DEBUG(std::fixed << std::setprecision(4) << "ph_t:\n" << ph_t << "\n");

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
        // if covariance diagonal elements are near 0 or negative we give them a small value 
        StateMatrix I_K_H = this->m_identity;
        I_K_H.noalias() -= K * m.H;
        this->m_covariance = I_K_H *this->m_covariance * I_K_H.transpose();
        this->m_covariance.noalias() += K * m.R * K.transpose();

        for (uint i = 0; i < States::STATE_SIZE_M; i++)
        {
            if(this->m_covariance(i,i) < 0.0) this->m_covariance(i,i) = -this->m_covariance(i,i);
            if(this->m_covariance(i,i) < 1e-9) this->m_covariance(i,i) = 1e-9;
            if(this->m_covariance(i,i) > 2.0) this->m_covariance(i,i) = 2.0;
        }
        DEBUG("\t\t--------------- FilterEKF Observation_Update: OUT ---------------\n");
        return true;
    }
};

// explicit template initialization
using Ctrv_EKF2D = FilterEkf<motion_model::Ctrv2D>;
using Ctra_EKF2D = FilterEkf<motion_model::Ctra2D>;
using Ctra_EKF3D = FilterEkf<motion_model::Ctra3D>;

} // end namespace filter 
} // end namespace state_predictor
} // end namespace iav