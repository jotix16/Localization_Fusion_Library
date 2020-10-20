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

#include <utilities/filter_utilities.h>
namespace iav{ namespace state_predictor { namespace measurement{

/**
 * @brief Class that does time keeping of fusion filters.
 * I.e it keeps the times of last temporal & observation update in measurement-time-frame.
 */
class MeasurementTimeKeeper
{
private:		
    bool m_inizialized;
    // time (timestamp) when the last temporal update took place
    tTime m_t_last_temporal_update;
    // time difference between measurement time(bag..) and wall time(computer time, akka global time)
    tTime m_dt_to_global;
    // measurement time (timestamp) of the last measurement used for observation update
    tTime m_t_last_observation_update;
    bool debug = true;

public:
    MeasurementTimeKeeper(): m_inizialized(false)
    {}

/** 
 * @brief MeasurementTimeKeeper: 
 *        Synces last_temp_update and last_obs_update to the time_stamp of first measurement
 * @param[in] global_time_now - The global time from the WallTime when the measurement arrived.
 * @param[in] measurement_time_stamp - The time stamp of the measurement.
 */
    void reset(tTime global_time_now, tTime measurement_time_stamp)
    {
        m_dt_to_global = global_time_now - measurement_time_stamp;
        m_t_last_temporal_update = measurement_time_stamp;
        m_t_last_observation_update = measurement_time_stamp;
        if(debug) std::cout << "---------------Measurement Time Keeper: dt_global: " << m_dt_to_global 
                            << ", last_temp: " <<m_t_last_temporal_update<<" -------------------\n";
        m_inizialized = true;
    }

/**
 * @brief MeasurementTimeKeeper: Check if object of this class is initialized. 
 * @return true if the Object from MeasurementTimeKeeper is initialized, otherwise false.
 */
    inline bool is_initialized() const
    {
       return m_inizialized; 
    }

/**
 * @brief MeasurementTimeKeeper: Updates the time keeper after a temporal update.
 * @param[in] dt - The time-difference for which the prediction was be performed.
 */
    void update_after_temporal_update(tTime dt)
    {
        m_t_last_temporal_update += dt;
    }

/**
 * @brief MeasurementTimeKeeper: Updates the time keeper after an observation update.
 * @param[in] time_stamp - The time stamp of the measurement processed.
 * @param[in] global_time - The wall time when the measurement arrived. 
 */
    void update_with_measurement(tTime time_stamp, tTime global_time)
    {
        m_t_last_observation_update = time_stamp;
        m_dt_to_global = global_time - time_stamp;
        if(debug) std::cout << "---------------Measurement Time Keeper: dt_global: " << m_dt_to_global<<"\n";
    }

/**
 * @brief MeasurementTimeKeeper: Calculates the time difference since last temporal update.
 * @param[in] gloal_time_now - The wall time from which the time difference should be calculated.
 * @return The time difference from the last temporal update to global_time_now.
 *         dt = (global_time_now - m_dt_to_global) - m_t_last_temp_update
 */
    inline tTime time_since_last_temporal_update(tTime global_time_now) const
    {
        return global_time_now - m_dt_to_global - m_t_last_temporal_update;
    }

/**
 * @brief MeasurementTimeKeeper: Returns the latest time_stamp.
 *        Normally used to check if a new measurement comes from the past.
 * @return The latest time in meassurement-time-frame when either a temporal
 *         or an observation update took place.
 */
    inline tTime latest_timestamp() const
    {
        return m_t_last_observation_update < m_t_last_temporal_update ? m_t_last_temporal_update : m_t_last_observation_update;
    }

    inline tTime to_measurement_time(tTime global_time_now) const
    {
        return global_time_now - m_dt_to_global;
    }
};

}  // namespace measurement
}  // namespace state_predictor
}  // namespace iav