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
#include<measurement/measurement.h>
#include<motion_model/modtion_model.h>

namespace iav{ namespace state_predictor { namespace measurement{

// General message_to_measurement to capture not implemented scenarious
template<typename MsgT, typename T = double>
MeasurementT<T> message_to_measurement( const MsgT &)
{
  static_assert(sizeof(MsgT) == -1,
    "You have to have a specialization for message_to_measurement() function!");
  // We throw here to make linter happy as this is a non-void function.
  throw std::runtime_error("You have to have a specialization for message_to_measurement()!");
}

/// SPECIALIZATIONS
// Specialization of message_to_measurement for odometry pose.
template<typename MsgT, typename T = double>
Measurement<T> message_to_measurement(MsgT msg) 
{
  using MeasurementVector = typename Measurement<T>::MeasurementVector;
  using MeasurementMatrix = typename Measurement<T>::MeasurementMatrix;
  MeasurementPose2D<T> measurement(msg.header.stamp, msg.header.frame_id);
  // .. transform, reduce ..
  // TO_DO: from quaternion to rollpitchyaw
  measurement.m_measurement_vector<<{msg.pose.position.x, msg.pose.position.y, msg.orientation.x};
  measurement.m_measurement_matrix<<{2};
}

}  // namespace measurement
}  // namespace state_predictor
}  // namespace iav