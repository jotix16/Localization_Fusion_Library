#include <functional>
#include <deque>
#include <queue>
#include <vector>
#include <iostream>
#include <buffer/filter_buffer.h>
#include "ros/ros.h"


template<typename T=double>
class Statetype
{
public:
    T data;
    T m_time_stamp;
public:
    Statetype() : data(0), m_time_stamp(0) {}

    Statetype(const T m_data, const T m_stamp)
    {
        data = m_data;
        m_time_stamp = m_stamp;
    }

    void print()
    {
        std::cout << "(" << data << ", " << m_time_stamp << ") ";
    }
};
typedef Statetype<double> StateD;
typedef std::shared_ptr<StateD> StateDPtr;

template<typename T=double>
class Datatype
{
public:
    T m_time_stamp;
    T data;
public:
    Datatype(/* args */) : data(0), m_time_stamp(0)
    {}
    Datatype(T d, T t) : data(d), m_time_stamp(t)
    {}

    bool operator()(const std::shared_ptr<Datatype> m1, const std::shared_ptr<Datatype> m2)
    {
        return m1->m_time_stamp > m2->m_time_stamp;
    }
    bool operator()(const Datatype& m1, const Datatype& m2)
    {
        return m1.m_time_stamp > m2.m_time_stamp;
    }

    void print()
    {
        std::cout << "(" << data << ", "<< m_time_stamp << ") ";
    }
};
typedef Datatype<double> DataD;
typedef std::shared_ptr<DataD> DataDPtr;

typedef iav::state_predictor::buffer::Buffer<Datatype<double>, Statetype<double>, double> BufferD; // define BufferD

template<class Container>
void print(Container container, std::string name)
{
    std::cout << name << ": ";
    for(auto n : container)
    {
        n->print();
    }
    std::cout << '\n';
}

template<class MeasurementQueue>
void print(std::string name , MeasurementQueue que)
{
    std::cout << name << ": ";
    while(!que.empty())
    {
        auto t = que.top(); t->print();
        que.pop();
    }
    std::cout << "\n";
}

void print(const BufferD& b)
{
    print("Meas-buffer", b.get_measurement_raw());
    print("Meas-queue", b.get_measurement_queue());
    print(b.get_measurement_history(), "Meas-history");
    print(b.get_state_history(), "Stat-history");
    std::cout << "----------------------------------------\n";
}


template<class Buffer, typename T=double>
class Filter
{
    typedef Datatype<T> DataT;
    typedef std::shared_ptr<DataT> DataTPtr;
    typedef Statetype<T> StateT;
    typedef std::shared_ptr<StateT> StateTPtr;

private:
    StateT state;
    Buffer b;

public:
    Filter(){};

    Filter(const DataT& d) : state(d.data, d.m_time_stamp){}

    bool predict(iav::state_predictor::tTime dt)
    {
        state.data += state.data*dt;
        state.m_time_stamp += dt;
        return true;
    }

    bool update(const DataTPtr& measurement)
    {
        state.data = measurement->data;
        state.m_time_stamp = measurement->m_time_stamp;
        return true;
    }

    bool process_measurement(const DataTPtr& measurement)
    {
        predict(measurement->m_time_stamp - state.m_time_stamp);
        update(measurement);
    }

    StateDPtr get_state()
    {
        return StateDPtr(new StateT(state.data, state.m_time_stamp));
    }

    void print()
    {
        std::cout << "Filte_: " << "<<" << state.data << ", " << state.m_time_stamp << ">>\n";
    }

    void test()
    {
        iav::state_predictor::tTime time = 0;
        b.set_process_measurement_function([this](DataTPtr d) { return this->process_measurement(d);});
        b.set_predict_function([this](iav::state_predictor::tTime t) { return this->predict(t);});
        b.set_publish_function([this]() { this->print();});
        b.set_get_state_ptr_function([this]() { return this->get_state();});
        b.set_get_time_now([&time]() { return time;});

        // FilterStatePtr(new FilterState()) StateDPtr
        b.enqueue_measurement(DataDPtr(new DataD(1,0)));
        b.enqueue_measurement(DataDPtr(new DataD(2,1)));
        b.enqueue_measurement(DataDPtr(new DataD(3,2)));
        b.enqueue_measurement(DataDPtr(new DataD(4,3)));
        b.enqueue_measurement(DataDPtr(new DataD(5,4)));

        time = 3;
        ::print(b);
        b.periodic_update();
        ::print(b);

        b.enqueue_measurement(DataDPtr(new DataD(6,2.5)));
        ::print(b);
        b.periodic_update();
        ::print(b);

        // Edge case 1: Delayed measurement
        b.enqueue_measurement(DataDPtr(new DataD(7,1.5)));
        ::print(b);
        b.periodic_update();
        ::print(b);

        b.clear_expired_history(2);
        ::print(b);
        b.clear_measurement_queues();
        ::print(b);

        // Edge case 2: Our history doesnt go as back in history as required by measurement
        time = 4;
        b.enqueue_measurement(DataDPtr(new DataD(7,1.5)));
        b.enqueue_measurement(DataDPtr(new DataD(7,2)));
        ::print(b);
        b.periodic_update();
        ::print(b);
    }

    void test2()
    {
        b.set_process_measurement_function([this](DataTPtr d) { return this->process_measurement(d);});
        b.set_predict_function([this](iav::state_predictor::tTime t) { return this->predict(t);});
        b.set_publish_function([this]() { this->print();});
        b.set_get_state_ptr_function([this]() { return this->get_state();});

        ros::Time::init();
        T init_time =  ros::Time::now().toSec();
        b.set_get_time_now([init_time]() { return ros::Time::now().toSec() - init_time + 3.0;});

        // initialize timer
        auto f = [this](Event event) {::print(this->b); this->b.periodic_update(event);};

        int intervall = 100;
        b.start(intervall, f);
        // b.start(intervall);
        auto th = std::thread([this, init_time, intervall]()
            {
                while(ros::Time::now().toSec() - init_time < 2)
                {
                    b.enqueue_measurement(DataDPtr(new DataD(5, ros::Time::now().toSec() - init_time + 3.4)));
                    std::this_thread::sleep_for(std::chrono::milliseconds(intervall+60));
                }
            });
        th.join();
        b.end();
    }

};
typedef Filter<BufferD, double> FilterD;

int main()
{
    std::cout << "START\n";
    FilterD my_filter(DataD(0,0));
    my_filter.test();
    my_filter.test2();
    std::cout << "END\n";

    char ch{};
    std::cin >> ch;
}
