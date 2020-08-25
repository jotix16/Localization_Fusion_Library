#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <sstream>

#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

namespace iav{ namespace state_predictor { namespace filter {

#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()


template<typename T>
void printt(T* arr, int a, int b){
    for (int i = 0; i < a; i++)
    {
        for (int j = 0; j < b; j++)
        {
            std::cout<<" "<<arr[i*a+j];
        }
        std::cout<<"\n";
    }
}

struct SensorConfig{
    using Document = rapidjson::Document;
    public:
    bool m_update_vector[15];
    double m_pose_mahalanobis_thresh[6];    
    double m_acc_mahalanobis_thresh[3];
    double m_speed_mahalanobis_thresh[6];

    SensorConfig(const Document& doc, const char* sensor_name, const int type)
    {
        for (int i = 0; i < 15; i++)
        {
            m_update_vector[i] = doc[sensor_name]["update_vector"][i].GetBool();
        }

        if(type <= 1) // only for odom and pose
        {
            for (int i = 0; i < 6; i++)
            {
                m_pose_mahalanobis_thresh[i] = doc[sensor_name]["pose_mahalanobis_thresh"][i].GetDouble();
            }
        }

        if(type > 1 || type ==0) // only for twist and imu
        {
            for (int i = 0; i < 6; i++)
            {
                m_speed_mahalanobis_thresh[i] = doc[sensor_name]["speed_mahalanobis_thresh"][i].GetDouble();
            }
        }

        if(type > 12) // only for imu
        {
            for (int i = 0; i < 3; i++)
            {
                m_acc_mahalanobis_thresh[i] = doc[sensor_name]["acc_mahalanobis_thresh"][i].GetDouble();
            }
        }
    }

    void print()
    {
        std::cout<<"\n Update_vector \n";
        printt(m_update_vector, 1, 15);

        std::cout<<"\n Pose_mahalanobis_thresh \n";
        printt(m_pose_mahalanobis_thresh, 1, 6);

        std::cout<<"\n Speed_mahalanobis_thresh \n";
        printt(m_speed_mahalanobis_thresh, 1, 6);
    }
};

template<int num_state>
struct FilterConfig{
    using Document = rapidjson::Document;
    using IStreamWrapper = rapidjson::IStreamWrapper;
    double m_init_estimate_covariance[num_state*num_state];
    double m_process_noise[num_state*num_state];
    std::map<std::string, struct SensorConfig> m_sensor_configs;

    FilterConfig(const char* path){
        // Create stream and check if valid
        std::ifstream ifs {path};
        if ( !ifs.is_open() )
        {
            std::cerr << "Could not open file for reading!\n";
            return;
        }

        // Create Document and parse
        IStreamWrapper isw { ifs };
        Document doc {};
        doc.ParseStream( isw );

        // Filter parameters
        // Init Estimate Covariance & Process Noise
        for (int i = 0; i < num_state; i++)
        {
            for (int j = 0; j < num_state; j++)
            {
                m_init_estimate_covariance[i*num_state+j] = doc["init_estimate_covariance"][i*num_state+j].GetDouble();
                m_process_noise[i*num_state+j] = doc["process_noise"][i*num_state+j].GetDouble();
            }
        }

        // Parse Sensors parameters
        const char *sensor_types[4] = {"odom_", "pose_", "twist_", "imu_"};
        for (int i = 0; i < 4; i++)
        {
            std::string sensor_type = sensor_types[i];
            for (int  j = 0; j < doc["sensor_inputs"][i].GetInt(); j++)
            {
                std::string str = sensor_type+ SSTR(j);
                m_sensor_configs.emplace(str, SensorConfig(doc, str.data(), i));
            }
        }
    }

    void print()
    {
        std::cout<<"\nInit_estimate_covariance\n";
        printt(m_init_estimate_covariance, num_state, num_state);

        std::cout<<"\nInit_process_noise\n";
        printt(m_process_noise, num_state, num_state);

        std::cout<<"\nSENSOR CONFIGS\n";
        for (auto x: m_sensor_configs)
        {
            std::cout<<x.first<<"";
            x.second.print();
        }
    }
};


using FilterConfig2D = FilterConfig<8>;

} // end namespace motion_model 
} // end namespace state_predictor
} // end namespace iav