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
#include <chrono>
#include <atomic>
#include <thread>
#include <functional>
#include <iostream>

class Event
{
        public:
            std::chrono::system_clock::time_point current_real;     ///< In a perfect world, this is when the current callback should be happening
            std::chrono::system_clock::time_point current_expected; ///< This is when the current callback was actually called (Time::now() as of the beginning of the callback)
            std::chrono::system_clock::time_point last_real;     ///< In a perfect world, this is when the last callback should have happened
            std::chrono::system_clock::time_point last_expected; ///< When the last callback actually happened
};

class CallBackTimer
{
public:
    /**
     * @brief CallBackTimer: Default constructor
     */
    CallBackTimer()
    :_execute(false)
    {}

    /**
     * @brief CallBackTimer: Destructor
     */
    ~CallBackTimer() {
        if( _execute.load(std::memory_order_acquire) ) {
            end();
        };
    }

    /**
     * @brief CallBackTimer: Methods that periodic calling
     */
    void stop()
    {
        _execute.store(false, std::memory_order_release);
    }

    /**
     * @brief CallBackTimer: Method that stops the periodic calling and stops(joins) the thread
     */
    void end()
    {
        stop();
        if( _thd.joinable() )
            _thd.join();
    }

    /**
     * @brief CallBackTimer: Method that starts the thread that periodically calls func
     * @param[in] interval - time duration in seconds which decides the rate we publish the estimated state.
     * @param[in] func - function that will be called periodically
     */
    void start(int interval, std::function<void(Event)> func)
    {
        if( _execute.load(std::memory_order_acquire) )
        {
            stop();
        };
        _execute.store(true, std::memory_order_release);
        _thd = std::thread([this, interval, func]()
        {
            std::cout << "--> Desired Period = " << interval << " milliseconds" << std::endl;
            event.current_expected = std::chrono::system_clock::now();
            event.current_real = event.current_expected;
            event.last_expected = event.current_expected;
            event.last_real = event.current_expected;

            std::chrono::milliseconds intervalPeriodMillis{interval};

            while (_execute.load(std::memory_order_acquire))
            {
                event.current_real = std::chrono::system_clock::now();
                func(event);
                event.last_expected = event.current_expected;
                event.last_real = event.current_real;
                event.current_expected = event.current_real + intervalPeriodMillis;
                std::this_thread::sleep_until(event.current_expected);
            }
        });
    }

    /**
     * @brief CallBackTimer: Checks if thread is running
     */
    bool is_running() const noexcept {
        return ( _execute.load(std::memory_order_acquire) &&
                 _thd.joinable() );
    }

public:
    Event event;

private:
    std::atomic<bool> _execute;
    std::thread _thd;
};