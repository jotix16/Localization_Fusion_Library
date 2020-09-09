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

#include<utilities/filter_utilities.h>
namespace iav{ namespace state_predictor { namespace measurement{

class MeasurementTimeKeeper
{
private:		
    bool m_inizialized;
    // measurement_time when the last measurement processed arrived
    tTime m_t_last_temporal_update;
    // the time difference between measurement_time(bag..) and computer_time(computer time)
    tTime m_dt_to_global;
    //measurement_time (timestamp) of the last measurement used for observation update
    tTime m_t_last_observation_update;

public:
    MeasurementTimeKeeper(): m_inizialized(false)
    {}

    // last_temp_update and last_obs_update are now sinced to the time_stamp of first measurement
    void reset(tTime global_time_now, tTime measurement_time_stamp)
    {
        m_dt_to_global = global_time_now - measurement_time_stamp;
        m_t_last_temporal_update = measurement_time_stamp;
        m_t_last_observation_update = measurement_time_stamp;
        m_inizialized = true;
    }

    inline bool is_initialized() const
    {
       return m_inizialized; 
    }

    void update_after_temporal_update(tTime dt)
    {
        m_t_last_temporal_update += dt;
    }

    void update_with_measurement(tTime time_stamp, tTime global_time)
    {
        m_t_last_observation_update = time_stamp;
        m_dt_to_global = global_time - time_stamp;
    }

    // dt = m_t_last_temp_update -(global_time_now - m_dt_to_global) 
    inline tTime time_since_last_temporal_update(tTime global_time_now) const
    {
        return m_t_last_temporal_update + m_dt_to_global - global_time_now;
    }

    inline tTime latest_timestamp() const
    {
        return m_t_last_observation_update < m_t_last_temporal_update ? m_t_last_temporal_update : m_t_last_observation_update;
    }
};

}  // namespace measurement
}  // namespace state_predictor
}  // namespace iav