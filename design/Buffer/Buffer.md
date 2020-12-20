# Handling delayed measurement with a time_triggered approach

## Parameters
- **m_time_out** time duration in seconds after which we start to predict when no measurement are coming
- **intervall**  time duration in seconds which decides the rate we publish the estimated state. every intervall second the periodic_update(Time t) function is called
- **m_history_length** duration in seconds(or nanosec?) that we keep history of measurements and estimated filter states [not yet implemented but straight forward with the function ```clear_expired_history(const T up_to_time)``` ]
- **m_timeout** time duration after which we start doing temporal_updates if no measurement arrives

## Data structures
- **m_measurement_history** -> deque where we keep the integrated measurements, time ordered (front()-> measurment with earliest timestamp, back()-> measurement with oldest timestamp)
- **m_state_history** -> deque where we keep the estimated states, time ordered (front()-> filter state with earliest timestamp, back()-> filter state with oldest timestamp) (we can decide to use history_time in seconds instead of M&N)
- **m_meas_raw_mutex** -> mutex that protects m_measurement_raw
- **m_measurement_raw** -> place where all sensors insert their measurements directly.
- **m_measurement_queue** -> place where we buffer measurements before starting to integrate, so that we dont have to keep m_measurement_raw locked.

## Callbacks
- ProcessMeasCallback **m_process_measurement_callback** -> callback function that calls ```process_measurement(MeasurementPtr measurement)``` from filter_wrapper
    - Processes a measurement by feeding it to the filter's temporal & observation update.
    - It initializes the time keeper and filter with the first measurement if they are not initialized.
- PredictCallback **m_predict_callback** -> callback function that calls ```temporal_update(tTime dt)``` from filter_wrapper
    - it is called when no measurements come for a time longeth than m_time_out duration
- PublishCallback **m_publish_callback** -> function pointer that saves the member method ```publish_state()```  from FilterWrapper that publishes the current estimated state
- GetStateCallback **m_get_state_callback** -> funciton pointer that saves the member method ```get_state()``` from FilterWrapper that gets the current estimated state
- GetTimeNowCallback **m_get_time_now** -> callback function to get the time that calls ```get_time_now()``` from filter_wrapper (normally a callback from ros that is saved in filter_wrapper)
- ResetFilterStateCallback **m_reset_filter_state** -> callback functiont that resets the filter state to a certain state which calls ```reset_filter_state(const StateCovTimePtr& state)``` from filter_wrapper

## Functions

- **start(int intervall)** -> starts the thread that periodically (every 'intervall' seconds) calls periodic_update()

- **enqueue_measurement(Measurement m)** -> enques measurement where measurements with earlier times have priority

- **periodic_update(event)** -> it is triggered every **intevall** seconds and integrates functions up to the current time. So it is responsible to
     - call **integrate_function**(Time t) that integrates all measurements up to current time _t_
     - publish latest estimated state and the map_bl transformation
     - publish the accelerometer if required
     - clear out expired history data

- ***swap_n_push()** -> locks m_measurement_raw and inserts all its measurements to m_measurement_queue

- **integrate_measurement(Time t)** -> integrates measurements up to time t
     - integrate all measurements with older timestamp than t
     - if any of the measurements is older than the current time of the filter -> go back to the first 'state S' with timestamp older than that of the measurement
     - relocate all measurement in the history with timestamp bigger than that of 'state S' to the m_measurement_queue so that they can be integrated again
     - if no measurement the last m_timeout seconds update up to the current time t
     - if filter not yet initialized don't do anything

- **go_back_to_time(Time t)** ->
     - Finds the latest filter state before the given timestamp and makes it the current state again and discards the later states
     - insert all measurements between the older filter timestamp and now into the measurements queue so that they can be integrated again.

- **clear_expired_history(const T up_to_time)** -> clears histories up to a certain time

- **clear_measurement_queues()** -> clear the histories and the measurement queues


## Defined Data-Types
```c++
typedef std::priority_queue<MeasurementPtr, std::vector<MeasurementPtr>, Measurement> MeasurementQueue;
typedef std::deque<MeasurementPtr> MeasurementHistoryDeque;
typedef std::deque<FilterStatePtr> FilterStateHistoryDeque;
```