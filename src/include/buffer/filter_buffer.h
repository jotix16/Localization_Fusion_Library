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
        typedef std::function<bool(DataT)> ProcessMeasCallback;
        typedef std::function<bool(T)> PredictCallback;
        typedef std::function<void()> PublishCallback;
        typedef std::function<StateTPtr()> GetStateCallback;
        typedef std::function<T()> GetTimeNowCallback;

    public:
        Buffer() : CallBackTimer() {}
        void set_process_measurement_function(ProcessMeasCallback f){ m_process_measurement_callback = f;}
        void set_predict_function(PredictCallback f){ m_predict_callback = f;}
        void set_publish_function(PublishCallback f){ m_publish_callback = f;}
        void set_get_state_function(GetStateCallback f){ m_get_state_callback = f;}
        void set_get_time_now(GetTimeNowCallback f){ m_get_time_now = f;}

        void enqueue_measurement(DataTPtr data)
        {
            // std::cout << "Inserting: "; data->print(); std::cout<<"\n";
            m_measurement_raw.push(data);
        }

        /**
         * - [x]call integrate_function(Time t) that integrates all measurements up to time t
         * - [x]publish latest estimated state and the map_bl transformation
         * - [x]publish the accelerometer if required
         * - []clear out expired history data
         **/
        void periodic_update(Event event=Event())
        {
            T time = m_get_time_now();
            std::cout <<"Periodic update with time: " << time << "\n";
            swap_n_push();
            integrate_measurement(time);
            // m_publish_callback();
        }

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
         * - [x]integrate all measurements with older timestamp than t
         * - [x]if any of the measurements is older than the current time of the filter -> go back to the time of the measurement
         * - [x]after any measurement integration add measurement and state in their respective history_dequeues
         * - []if no measurement for still keep updating up to the current time t
         * - [x]if filter not yet initialized assume the process measurement will take care of it(so do nothing)
        **/
        void integrate_measurement(const T time)
        {
            std::cout <<"Integrate measurements up to time: " << time << "\n";
            if(m_state_history.empty())
            {
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
                if(d->stamp < m_state_history.front()->time)
                {
                    std::cout<<" Our State_History doesn't hold any state that dates before time: " << d->stamp <<", so IGNORING the measurement.\n";
                    m_measurement_queue.pop();
                }
                else if (d->stamp < m_state_history.back()->time)
                {
                    uint initial_size = static_cast<uint>(m_measurement_queue.size());
                    go_back_to_time(d->stamp);
                    std::cout << "Reverted to time: " << d->stamp << " that required the revertion of "
                    << static_cast<uint>(m_measurement_queue.size()) - initial_size<<" measurements.\n";
                    break;
                }
                else break;
            }

            // 2. Integrate measurements in the que since we made sure they are younger than the last measurement integrated
            while(!m_measurement_queue.empty())
            {
                if(m_measurement_queue.top()->stamp <= time)
                {
                    d = m_measurement_queue.top();
                    m_measurement_queue.pop();

                    // integrate
                    m_process_measurement_callback(*d);
                    m_state_history.push_back(m_get_state_callback());
                    m_measurement_history.push_back(d);
                }
                else
                {
                    std::cout << "Last integrated measurement is: ";
                    d->print();
                    std:: cout <<"\n";
                    break;
                }
            }
        }

        /**
         * - [x]Finds the latest filter state before the given timestamp and makes it the current state again and discards the later states
         * - [x]insert all measurements between the older filter timestamp and now into the measurements queue.
        **/
        void go_back_to_time(T time)
        {
            bool ret_val = false;
            // search the right state
            StateTPtr last_history_state;
            while (!m_state_history.empty())
            {
                last_history_state = m_state_history.back();
                if(last_history_state->time > time)
                {
                    m_state_history.pop_back();
                }
                else
                {
                    ret_val = true;
                    break;
                }
            }
            assert(ret_val == true);

            // search the right state
            DataTPtr last_history_measurement;
            while (!m_measurement_history.empty())
            {
                last_history_measurement = m_measurement_history.back();
                if(last_history_measurement->stamp > last_history_state->time)
                {
                    m_measurement_queue.push(last_history_measurement);
                    m_measurement_history.pop_back();
                }
                else break;
            }
        }

        void clear_expired_history(const T up_to_time)
        {
            std::cout << "\n----- Buffer::clear_expired_history() -----"
            << "\n Clear history up to(not including) time " << up_to_time << "\n";

            int poppedMeasurements = 0;
            int poppedStates = 0;

            while (!m_measurement_history.empty() && m_measurement_history.front()->stamp < up_to_time)
            {
                m_measurement_history.pop_front();
                poppedMeasurements++;
            }

            while (!m_state_history.empty() && m_state_history.front()->time < up_to_time)
            {
                m_state_history.pop_front();
                poppedStates++;
            }

            std::cout << "Popped " << poppedMeasurements << " measurements and " <<
                poppedStates << " states from their respective queues." <<
                    "\n---- Buffer::clear_expired_history() ----\n";
        }

        void clear_measurement_queue()
        {
            while (!m_measurement_queue.empty()) //  && ros::ok()
            {
                m_measurement_queue.pop();
            }
            return;
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

        MeasurementQueue m_measurement_raw;
        std::mutex m_meas_raw_mutex;
        MeasurementQueue m_measurement_queue;
        MeasurementHistoryDeque m_measurement_history;
        FilterStateHistoryDeque m_state_history;
};