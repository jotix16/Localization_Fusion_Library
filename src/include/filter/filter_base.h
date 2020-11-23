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

#include <Eigen/Eigen>

#include <motion_model/motion_model.h>
#include <measurement/measurement.h>

namespace iav{ namespace state_predictor { namespace filter {

/**
 * @brief Base class that every Filtering(Fusing) algorithms should inherit from.
 * @param<template> MotionModelT - The motion model used. This defines the state to be estimated too.
 */
template<class MotionModelT>
class FilterBase
{
public:
    static constexpr uint num_state = MotionModelT::number_states;
    using T = typename MotionModelT::ValueType;
    using Measurement = typename measurement::Measurement<num_state, T>;
    using States = typename MotionModelT::States;
    using StateVector = typename MotionModelT::StateVector;
    using StateMatrix = typename MotionModelT::StateMatrix;
    using Matrix = typename Measurement::Matrix;
    using Vector = typename Measurement::Vector;

public:
    class StateCovTime
    {
        public:
            tTime m_time_stamp;
            StateVector m_state;
            StateMatrix m_covariance;
            StateCovTime(const tTime time_stamp, const StateVector state, const StateMatrix covariance)
            : m_time_stamp{time_stamp}, m_state{state}, m_covariance{covariance}
            {}
    };

protected:
    bool m_initialized;
    MotionModelT m_motion_model;
    StateVector m_state;
    StateMatrix m_covariance;
    StateMatrix m_process_noise;
    bool m_debug;
    std::ostream* m_debug_stream;

    const StateMatrix m_identity ; // needed for a few operations.

public:
    /**
     * @brief FilterBase: Default constructor
     */
    FilterBase():m_initialized(0), m_identity(StateMatrix::Identity()), m_debug(true)
    { }

    /**
     * @brief FilterBase: Initializes dhe debug
     */
    void setDebug(std::ostream* out_stream)
    {
            m_debug_stream = out_stream;
            DEBUG("\t\t\t-----------------------------------------\n");
            DEBUG("\t\t\t----- /FilterBase::Debug is on!" << " ------\n");
            DEBUG("\t\t\t-----------------------------------------\n");
    }

    /**
     * @brief FilterBase: Check if filter is initialized.
     * @return returns true if filter is initialized, otherwise false.
     */
    inline bool is_initialized() const
    {
        return m_initialized;
    }

    /**
     * @brief FilterBase: Resets/initializes the filter
     * @param[in] x0 - initial state estimation
     * @param[in] P0 - initial covariance estimation
     * @param[in] Q - process noise
     */
    // TO_DO: check out if valuies make sin and has to return bool
    void reset(const StateVector& x0, const StateMatrix& P0, const StateMatrix& Q)
    {
        m_state = x0;
        m_covariance = P0;
        m_process_noise = Q;
        m_initialized = true;
    }

    /**
     * @brief FilterBase: Getter function for the state estimation vector
     * @return the state estimation vector
     */
    // not necessary const reference since problems with thread.
    inline const StateVector get_state() const
    {
        return m_state;
    }

    /**
     * @brief FilterBase: Getter function for the covariance estimation matrix
     * @return the covariance estimation matrix
     */
    inline const StateMatrix get_covariance() const
    {
        return m_covariance;
    }

    /**
     * @brief FilterBase: Getter function for indexing the state estimation matrix
     * @param[in] index - index
     * @return the value of state at index
    */
    inline T at(int index) const
    {
        return m_state(index);
    }

    /**
     * @brief FilterBase: Function that has to be implemented by every filter/fusion algorithm
     *        Checks if measurement is an outlier by checking the mahalanobis distance
     * @param[in] innovation - Innovation
     * @param[in] hph_t_r_inv - (H P H^t)^-1
     * @param[in] mahalanobis_threshold - Mahalanobis threshold
     * @return true if measurement(innovation) is not an outlier, otherwise false
     */
    // virtual bool passes_mahalanobis(const Vector& innovation, const Matrix& hph_t_r_inv, const T& mahalanobis_threshold)=0;

    /**
     * @brief FilterBase: Function that has to be implemented by every filter/fusion algorithm
     *        Predicts the state with a dt step
     * @param[in] dt - time difference since temporal update in seconds
     * @return returns true if temporal update is successful
     */
    virtual bool temporal_update(const tTime& dt)=0;

    /**
     * @brief FilterBase: Function that has to be implemented by every filter/fusion algorithm.
     *        Corrects the prediction exploiting the measurement
     * @param[in] z - measurement vector
     * @param[in] innovation - innovation vector(Hx-z)
     * @param[in] H - state to measurement mapping matrix
     * @param[in] R - measurement covariance
     * @param[in] mahalanobis_threshold - Mahalanobis threshold
     * @return returns true observation update is successful
     */
    virtual bool observation_update(const Measurement& measurement)=0;
};
} // end namespace filter
} // end namespace state_predictor
} // end namespace iav