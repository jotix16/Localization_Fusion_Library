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
    CallBackTimer()
    :_execute(false)
    {}

    ~CallBackTimer() {
        if( _execute.load(std::memory_order_acquire) ) {
            end();
        };
    }

    void stop()
    {
        _execute.store(false, std::memory_order_release);
    }

    void end()
    {
        stop();
        if( _thd.joinable() )
            _thd.join();
    }

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