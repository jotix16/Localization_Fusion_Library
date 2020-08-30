
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

#include<filter/filter_base.h>
#include<utilities/filter_utilities.h>


namespace iav{ namespace state_predictor { namespace filter {

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
    using States = typename FilterBase<MotionModelT, num_state, T>::States;

public:
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
        StateMatrix jacobian;
		this->m_motion_model.compute_jacobian_and_predict(jacobian,this->m_state, dt);
        // wrap angles of state
        for (auto i:States::ANGLEidx)
        {
           this->m_state[i] = utilities::clamp_rotation(this->m_state[i]);
        }
        
        // update the covariance: P = J * P * J' + Q
       this->m_covariance = jacobian *this->m_covariance * jacobian.transpose();
       this->m_covariance.noalias() += dt *this->m_process_noise;
        return true;
    }
    bool observation_update(const ObservationVector& z, const ObservationMatrix& H, const Matrix& R, const T& mahalanobis_threshold)
    {
        // Check if initialized
        if(!this->m_initialized) return false;
        Matrix ph_t =this->m_covariance * H.transpose();
        Matrix hph_t_r_inv = (H * ph_t + R).inverse();
        ObservationVector innovation = z - H *this->m_state;
        // wrap angles of innovation( NOT SURE )
        // We could either allow to take an array with the indexes to the angles as input parameter()
        // or do the one below where I search for 1.0 in matrix H which come only in the columns that correspond to state angle indexes.
        // The corresponding rows' indexes should then be measurement/innovaion angles' indexes
        for (auto j:States::ANGLEidx)
        {
            for (uint i = 0; i < H.rows(); i++)
            {
                if(H(i,j)==1.0)
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
        for (auto i:States::ANGLEidx)
        {
           this->m_state[i] = utilities::clamp_rotation(this->m_state[i]);
        }

        // Correct the covariance using Joseph form for stability. This is the
        // same as P = P - K * H * P, but presents less of a problem in the
        // presense of floating point roundoff.
        // The Joseph Update has the form: P = (I - KH)P(I - KH)' + KRK'
        StateMatrix I_K_H =this->m_identity;
        I_K_H.noalias() -= K * H;
       this->m_covariance = I_K_H *this->m_covariance * I_K_H.transpose();
       this->m_covariance.noalias() += K * R * K.transpose();
        return true;
    }
};

using Ctrv_EKF2D = FilterEkf<motion_model::Ctrv2D, 6, double>;
using Ctra_EKF2D = FilterEkf<motion_model::Ctra2D, 6, double>;
using Ctra_EKF3D = FilterEkf<motion_model::Ctra3D, 6, double>;

} // end namespace filter 
} // end namespace state_predictor
} // end namespace iav