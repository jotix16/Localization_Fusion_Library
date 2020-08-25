
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


namespace iav{ namespace state_predictor { namespace filter {

template<class MotionModelT, int num_state, typename T = double>
class FilterEkf : public FilterBase<MotionModelT, num_state, T>
{
public:
    using StateVector = typename FilterBase::StateVector;
    using StateMatrix = typename FilterBase::StateMatrix;
    using ObservationVector = typename FilterBase::ObservationVector;
    using ObservationMatrix = typename FilterBase::ObservationMatrix;
    using MeasurementNoisematrix = typename FilterBase::MeasurementNoisematrix;

public:
    FilterEkf(){};
    bool passes_mahalanobis(ObservationVector innovation, MeasurementNoisematrix hph_t_r, T mahalanobis_threshold){return true;}
    bool temporal_update(tTime dt){return true;}
    bool observation_update(ObservationVector z, ObservationMatrix H, MeasurementNoisematrix R){return true;}

};

using Ctrv_EKF2D = FilterEkf<motion_model::Ctrv2D, 6, double>;
using Ctra_EKF2D = FilterEkf<motion_model::Ctra2D, 6, double>;
// using Ctra_EKF3D = FilterEkf<motion_model::Ctrv2D, 6, double>

} // end namespace filter 
} // end namespace state_predictor
} // end namespace iav