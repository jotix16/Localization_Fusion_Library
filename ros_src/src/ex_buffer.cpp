#include <functional>
#include <deque>
#include <queue>
#include <vector>
#include <iostream>
#include <memory>
#include <assert.h>


template<typename T=double>
class Statetype
{
public:
    T data;
    T time;
public:
    Statetype() : data(0), time(0) {}

    Statetype(const T m_data, const T m_stamp)
    {
        data = m_data;
        time = m_stamp;
    }

    void print()
    {
        std::cout << "(" << data << ", " << time << ") ";
    }
};
typedef Statetype<double> StateD;
typedef std::shared_ptr<StateD> StateDPtr;

template<typename T=double>
class Datatype
{
public:
    T data;
    T stamp;
public:
    Datatype(/* args */) : data(0), stamp(0)
    {}
    Datatype(T d, T t) : data(d), stamp(t)
    {}

    bool operator()(const std::shared_ptr<Datatype> m1, const std::shared_ptr<Datatype> m2)
    {
        return m1->stamp > m2->stamp;
    }
    bool operator()(const Datatype& m1, const Datatype& m2)
    {
        return m1.stamp > m2.stamp;
    }

    void print()
    {
        std::cout << "(" << data << ", "<< stamp << ") ";
    }
};
typedef Datatype<double> DataD;
typedef std::shared_ptr<DataD> DataDPtr;

template<typename T=double>
class Filter
{
    typedef Datatype<T> DataT;
    typedef Statetype<T> StateT;
    typedef std::shared_ptr<StateT> StateTPtr;

private:
    StateT state;

public:
    Filter(){};

    Filter(DataT d) : state(d.data, d.stamp){}

    bool predict(T dt)
    {
        state.data += state.data*dt;
        state.time += dt;
        return true;
    }

    bool update(DataT measurement)
    {
        state.data = measurement.data;
        state.time = measurement.stamp;
        return true;
    }

    bool process_measurement(DataT measurement)
    {
        predict(measurement.stamp - state.time);
        update(measurement);
    }

    StateDPtr get_state()
    {
        return StateDPtr(new StateT(state.data, state.time));
    }

    void print()
    {
        std::cout << "Filte_: " << "<<" << state.data << ", " << state.time << ">>\n";
    }
};
typedef Filter<double> FilterD;

template<class Datatype, class Statetype, typename T=double>
class Buffer
{
    public:
        typedef Datatype DataT; // time, data, operator(data1, data2) return data1<data2,
        typedef Statetype StateT; // stamp, data
        typedef std::shared_ptr<DataT> DataTPtr;
        typedef std::shared_ptr<StateT> StateTPtr;

        typedef std::deque<DataTPtr> MeasurementHistoryDeque;
        typedef std::deque<StateTPtr> FilterStateHistoryDeque;
        typedef std::priority_queue<DataTPtr, std::vector<DataTPtr>, DataT> MeasurementQueue;
        // top(): returns the top element                                               // empty(): returns true if queue empty
        // size(): returns size of queue                                                // push(DataDPtr): inserts element and sorts the underlying container,
        // emplace(): constructs element in-place and sorts the underlying container    // pop(): removes the top element
        typedef std::function<bool(DataT)> ProcessMeasCallback;
        typedef std::function<bool(T)> PredictCallback;
        typedef std::function<void()> PublishCallback;
        typedef std::function<StateTPtr()> GetStateCallback;

    public:
        ProcessMeasCallback m_process_measurement_callback;
        PredictCallback m_predict_callback;
        PublishCallback m_publish_callback;
        GetStateCallback m_get_state_callback;

        MeasurementQueue m_measurement_queue;
        MeasurementHistoryDeque m_measurement_fifo;
        MeasurementHistoryDeque m_measurement_history;
        FilterStateHistoryDeque m_state_history;

    public:
        Buffer(){};

        void set_process_measurement_function(ProcessMeasCallback f){ m_process_measurement_callback = f;}
        void set_predict_function(PredictCallback f){ m_predict_callback = f;}
        void set_publish_function(PublishCallback f){ m_publish_callback = f;}
        void set_get_state_function(GetStateCallback f){ m_get_state_callback = f;}

        void enqueue_measurement(DataTPtr data)
        {
            std::cout << "Inserting: "; data->print(); std::cout<<"\n";
            m_measurement_queue.push(data);
            // m_measurement_fifo.push(data);
        }

        /**
         * - [x]call integrate_function(Time t) that integrates all measurements up to time t
         * - [x]publish latest estimated state and the map_bl transformation
         * - [x]publish the accelerometer if required
         * - []clear out expired history data
         **/
        void periodic_update(const T time)
        {
            // auto temp_fifo = m_measurement_fifo;
            std::cout <<"Periodic update with time: " << time << "\n";
            integrate_measurement(time);
            // m_publish_callback();
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
            // m_state_history is empty
            if(m_state_history.empty())
            {
                std::cout << "State_History is empty\n";
                m_state_history.push_back(m_get_state_callback());
            }


            // 1. find the first VALID measurement that:
            // - either happens after the current state(bigger timestamp than the current state)
            // - or for which there is a state in the state_history that happened before(this state in the history has smaller time_stamp than the meas)
            DataTPtr d = m_measurement_queue.top();

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

        MeasurementQueue get_measurement_queue()
        {
            return m_measurement_queue;
        }

        MeasurementHistoryDeque get_measurement_history()
        {
            return m_measurement_history;
        }

        FilterStateHistoryDeque get_state_history()
        {
            return m_state_history;
        }

};
typedef Buffer<Datatype<double>, Statetype<double>, double> BufferD;


template<class Container>
void print(const Container& container, std::string name)
{
    std::cout << name << ": ";
    for(auto n : container)
    {
        n->print();
    }
    std::cout << '\n';
}

template<class MeasurementQueue>
void print(MeasurementQueue que)
{
    std::cout << "Meas queue: ";
    while(!que.empty())
    {
        auto t = que.top(); t->print();
        que.pop();
    }
    std::cout << "\n";
}

void print(BufferD b)
{
    print(b.get_measurement_queue());
    print(b.get_measurement_history(), "Meas-history");
    print(b.get_state_history(), "Stat-history");
    std::cout << "----------------------------------------\n";
}


int main()
{
    FilterD my_filter(DataD(0,0));

    BufferD b;
    b.set_process_measurement_function([&my_filter](DataD d) { return my_filter.process_measurement(d);});
    b.set_predict_function([&my_filter](double t) { return my_filter.predict(t);});
    b.set_publish_function([&my_filter]() { my_filter.print();});
    b.set_get_state_function([&my_filter]() { return my_filter.get_state();});

    // FilterStatePtr(new FilterState()) StateDPtr
    b.enqueue_measurement(DataDPtr(new DataD(1,0)));
    b.enqueue_measurement(DataDPtr(new DataD(2,1)));
    b.enqueue_measurement(DataDPtr(new DataD(3,2)));
    b.enqueue_measurement(DataDPtr(new DataD(4,3)));
    b.enqueue_measurement(DataDPtr(new DataD(5,4)));

    print(b);
    b.periodic_update(3);
    print(b);

    b.enqueue_measurement(DataDPtr(new DataD(6,2.5)));
    print(b);
    b.periodic_update(3);
    print(b);

    // Edge case 1: Delayed measurement
    b.enqueue_measurement(DataDPtr(new DataD(7,1.5)));
    print(b);
    b.periodic_update(3);
    print(b);

    b.clear_expired_history(2);
    print(b);
    b.clear_measurement_queue();
    print(b);

    // Edge case 2: Our history doesnt go as back in history as required by measurement
    b.enqueue_measurement(DataDPtr(new DataD(7,1.5)));
    b.enqueue_measurement(DataDPtr(new DataD(7,2)));
    print(b);
    b.periodic_update(4);
    print(b);
}

    // two possible ways for callbacks: lambdas & std::bind
    // int x  = 2;
    // std::function<bool(Datatype)> f = [&state1](Datatype d) { return state1.update(d);};
    // std::function<bool(Datatype)> f2 = std::bind(&Statetype<Datatype>::update, &state1, std::placeholders::_1, std::cref(x));