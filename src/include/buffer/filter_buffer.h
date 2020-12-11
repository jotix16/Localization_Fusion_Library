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
#include <functional>
#include <deque>
#include <queue>
#include <vector>
#include <iostream>
#include <memory>
#include <mutex>
#include <assert.h>
#include <buffer/filter_timer.h>
#include <utilities/filter_utilities.h>

namespace iav{ namespace state_predictor { namespace buffer {
template<class Datatype, class Statetype, typename T=double>
class Buffer : public CallBackTimer
{
    public:
        using CallBackTimer::end;
        using CallBackTimer::start;
        using CallBackTimer::is_running;

        // Data types
        typedef Datatype DataT; // time, data, operator(data1, data2) return data1<data2,
        typedef Statetype StateT; // stamp, data
        typedef std::shared_ptr<DataT> DataTPtr;
        typedef std::shared_ptr<StateT> StateTPtr;

        typedef std::deque<DataTPtr> MeasurementHistoryDeque;
        typedef std::deque<StateTPtr> FilterStateHistoryDeque;
        typedef std::priority_queue<DataTPtr, std::vector<DataTPtr>, DataT> MeasurementQueue;

        // Callback types
        typedef std::function<bool(DataTPtr)> ProcessMeasCallback;
        typedef std::function<bool(tTime)> PredictCallback;
        typedef std::function<void()> PublishCallback;
        typedef std::function<StateTPtr()> GetStateCallback;
        typedef std::function<tTime()> GetTimeNowCallback;
        typedef std::function<void(StateTPtr)> ResetFilterStateCallback;

    public:
        Buffer() : CallBackTimer() {}
        void set_process_measurement_function(ProcessMeasCallback f){ m_process_measurement_callback = f;}
        void set_predict_function(PredictCallback f){ m_predict_callback = f;}
        void set_publish_function(PublishCallback f){ m_publish_callback = f;}
        void set_get_state_ptr_function(GetStateCallback f){ m_get_state_callback = f;}
        void set_reset_filter_state(ResetFilterStateCallback f){ m_reset_filter_state = f;}
        void set_get_time_now(GetTimeNowCallback f){ m_get_time_now = f;}

        /**
         * @brief Buffer: Inserts data in the m_measurement_raw queue
         * @param[in] data - measurement to be queued
         */
        void enqueue_measurement(DataTPtr data)
        {
            // std::cout << "Inserting: "; data->print(); std::cout<<"\n";
            std::lock_guard<std::mutex> guard(m_meas_raw_mutex);
            m_measurement_raw.push(data);
        }

        /**
         * @brief Buffer: It is a function triggered periodically that integrates functions up to the current time. It is responsible to
         *  - call **integrate_function**(Time t) that integrates all measurements up to current time _t_
         *  - publish latest estimated state and the map_bl transformation
         *  - publish the accelerometer if required
         *  - clear out expired history data
         * @param[in] data - measurement to be queued
         */
        void periodic_update(Event event=Event())
        {
            int time_needed = std::chrono::duration_cast<std::chrono::milliseconds>(event.current_expected-event.last_real).count();
            if(time_needed - m_update_interval > 100 )
                std::cout << "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!      Late: " << time_needed - m_update_interval << " !!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n";

            swap_n_push();
            T time = m_get_time_now();
            if(m_measurement_queue.empty() & m_state_history.empty())
            {
                // std::cout << " No Measurements yet\n";
            }
            else if(m_measurement_queue.empty() & !m_state_history.empty())
            {
                tTime dt= time - m_state_history.back()->m_time_stamp;
                if(dt > 0.4 && dt < 10000)
                {
                    std::cout << "DT: " << dt <<"\n";
                    m_predict_callback( time - m_state_history.back()->m_time_stamp);
                    m_publish_callback();
                }
            }
            else
            {
                integrate_measurement(time);
                m_publish_callback();
            }
        }

       /**
        * @brief Buffer: locks m_measurement_raw and inserts all its measurements to m_measurement_queue
        */
        void swap_n_push()
        {
            std::lock_guard<std::mutex> guard(m_meas_raw_mutex);
            if(m_measurement_queue.empty() && !m_measurement_raw.empty())
            {
                std::swap(m_measurement_raw, m_measurement_queue);
            }
            else
            {
                while(!m_measurement_raw.empty())
                {
                    m_measurement_queue.push(m_measurement_raw.top());
                    m_measurement_raw.pop();
                }
            }
        }

        /**
         * @brief Buffer: It integrates measurements up to time t
         *  - integrate all measurements with older timestamp than t
         *  - if any of the measurements is older than the current time of the filter -> go back to the first 'state S' with timestamp older than that of the measurement
         *  - relocate all measurement in the history with timestamp bigger than that of 'state S' to the m_measurement_queue so that they can be integrated again
         *  - if no measurement the last m_timeout seconds update up to the current time t
         *  - if filter not yet initialized don't do anything
         * @param[in] time - time up to which to integrate measurements
         */
        void integrate_measurement(const T time)
        {
            // std::cout <<"Integrate measurements up to time: " << time << "\n";
            if(m_state_history.empty())
            {
                m_delayed_nr = 0;
                std::cout << "State_History is empty\n";
                m_state_history.push_back(m_get_state_callback());
            }
            // 1. find the first VALID measurement that:
            // - either happens after the current state(bigger timestamp than the current state)
            // - or for which there is a state in the state_history that happened before(this state in the history has smaller time_stamp than the meas)
            DataTPtr d;// = m_measurement_queue.top();

            while(!m_measurement_queue.empty())
            {
                d = m_measurement_queue.top();
                if(d->m_time_stamp < m_state_history.front()->m_time_stamp)
                {
                    std::cout<<" Our State_History doesn't hold any state that dates before time: " << d->m_time_stamp <<", so IGNORING the measurement.\n";
                    m_measurement_queue.pop();
                }
                else if (d->m_time_stamp < m_state_history.back()->m_time_stamp)
                {
                    uint initial_size = static_cast<uint>(m_measurement_queue.size());
                    go_back_to_time(d->m_time_stamp);
                    // std::cout << "Reverted to time: " << d->m_time_stamp << " that required the revertion of "
                    // << static_cast<uint>(m_measurement_queue.size()) - initial_size<<" measurements.\n";
                    m_delayed_nr += m_measurement_queue.size() - initial_size;
                    // std::cout << "DELAYED NR: " << m_delayed_nr << "\n";
                    break;
                }
                else break;
            }

            // 2. Integrate measurements in the que since we made sure they are younger than the last measurement integrated
            while(!m_measurement_queue.empty())
            {
                if(m_measurement_queue.top()->m_time_stamp <= time)
                {
                    d = m_measurement_queue.top();
                    m_measurement_queue.pop();

                    // integrate
                    m_process_measurement_callback(d);
                    m_state_history.push_back(m_get_state_callback());
                    m_measurement_history.push_back(d);
                }
                else
                {
                    // std::cout << "Last integrated measurement is: ";
                    // d->print();
                    // std:: cout <<"\n";
                    break;
                }
            }
        }

        /**
         * @brief Buffer: Functions that implements the go back to time logic required when facing delayed measurementss
         *  - Finds the latest filter state before the given timestamp and makes it the current state again and discards the later states
         *  - insert all measurements between the older filter timestamp and now into the measurements queue so that they can be integrated again.
         * @param[in] time - time up to which we go back in time
         */
        void go_back_to_time(T time)
        {
            bool ret_val = false;
            // search the right state
            auto old_time = m_state_history.back()->m_time_stamp;
            StateTPtr last_history_state;
            while (!m_state_history.empty())
            {
                last_history_state = m_state_history.back();
                if(last_history_state->m_time_stamp >= time)
                {
                    m_state_history.pop_back();
                }
                else
                {
                    ret_val = true;
                    break;
                }
            }
            assert(ret_val == true); // should not hapen, since we make sure in integrate_measurement before calling this function
            //TO_DO: reset filter
            std::cout << " reseting filter dt: " << old_time-last_history_state->m_time_stamp << ", t_meas-t_state: " <<time-last_history_state->m_time_stamp << " in the past.\n";
            m_reset_filter_state(last_history_state);

            // search the right state
            DataTPtr last_history_measurement;
            while (!m_measurement_history.empty())
            {
                last_history_measurement = m_measurement_history.back();
                if(last_history_measurement->m_time_stamp >= last_history_state->m_time_stamp)
                {
                    m_measurement_queue.push(last_history_measurement);
                    m_measurement_history.pop_back();
                }
                else break;
            }
        }

        /**
         * @brief Buffer: Functions that state and measurement histories
         * @param[in] up_to_time - time up to which we clear the histories
         */
        void clear_expired_history(const T up_to_time)
        {
            std::cout << "\n----- Buffer::clear_expired_history() -----"
            << "\n Clear history up to(not including) time " << up_to_time << "\n";

            int poppedMeasurements = 0;
            int poppedStates = 0;

            while (!m_measurement_history.empty() && m_measurement_history.front()->m_time_stamp < up_to_time)
            {
                m_measurement_history.pop_front();
                poppedMeasurements++;
            }

            while (!m_state_history.empty() && m_state_history.front()->m_time_stamp < up_to_time)
            {
                m_state_history.pop_front();
                poppedStates++;
            }

            std::cout << "Popped " << poppedMeasurements << " measurements and " <<
                poppedStates << " states from their respective queues." <<
                    "\n---- Buffer::clear_expired_history() ----\n";
        }

        /**
         * @brief Buffer: Functions that clears the measurement queue
         */
        void clear_measurement_queues()
        {
            while (!m_measurement_queue.empty()) //  && ros::ok()
            {
                m_measurement_queue.pop();
            }
            return;
        }

        /**
         * @brief Buffer: Starts the thread that periodically (every 'intervall' seconds) calls periodic_update()
         *  @param[in] intervall - time intervall which decides how often we integrate measurements
         */
        void start(int intervall)
        {
            m_update_interval = intervall;
            auto f = [this](Event event) {this->periodic_update(event);};
            CallBackTimer::start(intervall, f);
        }

    public:
        MeasurementQueue get_measurement_queue() const { return m_measurement_queue; }
        MeasurementQueue get_measurement_raw() const { return m_measurement_raw; }
        MeasurementHistoryDeque get_measurement_history() const { return m_measurement_history; }
        FilterStateHistoryDeque get_state_history() const { return m_state_history; }

    private:
        ProcessMeasCallback m_process_measurement_callback;
        PredictCallback m_predict_callback;
        PublishCallback m_publish_callback;
        GetStateCallback m_get_state_callback;
        GetTimeNowCallback m_get_time_now;
        ResetFilterStateCallback m_reset_filter_state;

        std::mutex m_meas_raw_mutex;
        MeasurementQueue m_measurement_raw;
        MeasurementQueue m_measurement_queue;
        MeasurementHistoryDeque m_measurement_history;
        FilterStateHistoryDeque m_state_history;
        int m_update_interval;
        int m_delayed_nr;
};

} // end namespace buffer
} // end namespace state_predictor
} // end namespace iav