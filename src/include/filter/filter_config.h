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

#include <iostream>
#include <fstream>
#include <map>
#include <string>
#include <sstream>

#include<eigen/Eigen>

#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include<utilities/filter_utilities.h>

namespace iav{ namespace state_predictor { namespace filter {

  /**
  * @brief Templated function to print out arrays of different types T
  * @param[in] arr - array to be printed
  * @param[in] a - nr of rows
  * @param[in] b - nr of columns
  */
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

/**
* @brief A class that holds information about one single sensor
*/
struct SensorConfig{
    using Document = rapidjson::Document;
    public:
    bool m_update_vector[15];
    double m_pose_mahalanobis_thresh[6];    
    double m_acc_mahalanobis_thresh[3];
    double m_speed_mahalanobis_thresh[6];

    /**
     * @brief Constructor that parses the sensor parameters.
     * @param[in] doc - Document that holds the informations to be parsed
     * @param[in] sensor_name - The name of the sensor, e.g. odom_1, imu_0 or ..
     * @param[in] type - One of the four types{1:"odom_", 2:"pose_", 3:"twist_", 4:"imu_"}
     * @return SensorConfig object
     */
    SensorConfig(const Document& doc, const char* sensor_name, const int type)
    {
        // make sure that components that are not part of each type of measurements
        // are 0s.
        for (int i = 0; i < STATE_SIZE; i++)
        {
            if(type == 1)
            {
                if(i < POSE_SIZE + TWIST_SIZE)
                {
                    m_update_vector[i] = doc[sensor_name]["update_vector"][i].GetBool();
                }
            }
            else if(type == 2)
            {
                if(i < POSE_SIZE)
                {
                    m_update_vector[i] = doc[sensor_name]["update_vector"][i].GetBool();
                }
            }
            else if(type == 3)
            {
                if(i >= POSE_SIZE && i < POSE_SIZE + TWIST_SIZE)
                {
                    m_update_vector[i] = doc[sensor_name]["update_vector"][i].GetBool();
                }
            }
            else if(type == 4)
            {
                if(i >= POSITION_SIZE && i < POSITION_SIZE + TWIST_SIZE || i >= POSITION_SIZE + POSE_SIZE)
                {
                    m_update_vector[i] = doc[sensor_name]["update_vector"][i].GetBool();
                }
            }
            m_update_vector[i] = 0;
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

  /**
  * @brief Prints out SensorConfig information
  */
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

/**
* @brief A class that holds information about a filter instance and all sensors it uses.
*/
template<int num_state, typename T = double>
struct FilterConfig{
    using StateMatrix = Eigen::Matrix<T, num_state, num_state>;
    using Document = rapidjson::Document;
    using IStreamWrapper = rapidjson::IStreamWrapper;
    
    StateMatrix init_estimation_covariance;
    StateMatrix process_noise;
    double m_init_estimate_covariance[num_state*num_state];
    double m_process_noise[num_state*num_state];
    std::map<std::string, struct SensorConfig> m_sensor_configs;


    /**
     * @brief Default Constructor
     */
    FilterConfig() = default;
    /**
     * @brief Constructor that parses the filter parameters and all of its sensors.
     * @param[in] path - Path to the .json file normally in the folder config/.
     * @return FilterConfig object
     */
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
                init_estimation_covariance(i,j) = doc["init_estimate_covariance"][i*num_state+j].GetDouble();
                m_init_estimate_covariance[i*num_state+j] = doc["init_estimate_covariance"][i*num_state+j].GetDouble();
                process_noise(i,j) = doc["process_noise"][i*num_state+j].GetDouble();
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

  /**
  * @brief Prints out SensorConfig information
  */
    void print()
    {
        std::cout<<"\nInit_estimate_covariance\n";
        std::cout<<"\n"<< init_estimation_covariance <<"\n";
        // printt(m_init_estimate_covariance, num_state, num_state);

        std::cout<<"\nInit_process_noise\n";
        std::cout<<"\n"<< process_noise <<"\n";
        // printt(m_process_noise, num_state, num_state);

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