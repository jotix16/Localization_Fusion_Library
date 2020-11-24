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

#include <Eigen/Eigen>

#include "rapidjson/document.h"
#include "rapidjson/istreamwrapper.h"
#include "rapidjson/writer.h"
#include "rapidjson/stringbuffer.h"

#include <utilities/filter_utilities.h>

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
    uint m_type;
    bool m_update_vector[STATE_SIZE];
    double m_mahal_thresh;

    // IMUS
    bool m_remove_gravity;
    bool m_enu; // TO_DO: if given in NED we have to make up for it

    /**
     * @brief Default constructor
     */
    SensorConfig() {};

    /**
     * @brief Constructor that parses the sensor parameters.
     * @param[in] doc - Document that holds the informations to be parsed
     * @param[in] sensor_name - The name of the sensor, e.g. odom_1, imu_0 or ..
     * @param[in] type - One of the four types{0:"odom_", 1:"pose_", 2:"twist_", 3:"imu_", 4:"gps_"}
     * @return SensorConfig object
     */
    SensorConfig(const Document& doc, const char* sensor_name, const uint type)
    {
        // make sure that components that are not part of each type of measurements
        // are 0s.
        m_type = type;
        m_mahal_thresh = doc[sensor_name]["mahalanobis_thresh"].GetDouble();
        for (int i = 0; i < STATE_SIZE; i++)
        {
            if(type == 0)
            {
                if(i < POSE_SIZE + TWIST_SIZE)
                {
                    m_update_vector[i] = doc[sensor_name]["update_vector"][i].GetBool();
                    continue;
                }
            }
            else if(type == 1)
            {
                if(i < POSE_SIZE)
                {
                    m_update_vector[i] = doc[sensor_name]["update_vector"][i].GetBool();
                    continue;
                }
            }
            else if(type == 2)
            {
                if(i >= POSE_SIZE && i < POSE_SIZE + TWIST_SIZE)
                {
                    m_update_vector[i] = doc[sensor_name]["update_vector"][i].GetBool();
                    continue;
                }
            }
            else if(type == 3)
            {
                m_remove_gravity = doc[sensor_name]["remove_gravity"].GetBool();
                if(i >= POSITION_SIZE && i < POSITION_SIZE + 3 || i >= POSITION_SIZE + POSE_SIZE)
                {
                    m_update_vector[i] = doc[sensor_name]["update_vector"][i].GetBool();
                    continue;
                }
            }
            else if(type == 4)
            {
                if(i < POSITION_SIZE)
                {
                    m_update_vector[i] = doc[sensor_name]["update_vector"][i].GetBool();
                    continue;
                }
            }
            m_update_vector[i] = 0;
        }
     }

    /**
     * @brief Prints out SensorConfig information
     */
    void print()
    {
        std::cout<<"\n Update_vector \n";
        printt(m_update_vector, 1, STATE_SIZE);

        std::cout<<"\n Mahalanobis_thresh: " << m_mahal_thresh << "\n";
    }
};

/**
* @brief A class that holds information about a filter instance and all sensors it uses.
*/
template<typename T = double>
struct FilterConfig{
    using FullStateMatrix = Eigen::Matrix<T, STATE_SIZE, STATE_SIZE>;
    using Document = rapidjson::Document;
    using IStreamWrapper = rapidjson::IStreamWrapper;

    FullStateMatrix m_init_estimation_covariance;
    FullStateMatrix m_process_noise;
    bool m_data_triggered;
    std::map<std::string, struct SensorConfig> m_sensor_configs;

    /**
     * @brief Default Constructor
     */
    FilterConfig() {}//= default;

    /**
     * @brief Constructor that parses the filter parameters and all of its sensors.
     * @param[in] path - Path to the .json file normally in the folder config/.
     * @return FilterConfig object
     */
    FilterConfig(const char* path)
    {
        reset(path);
    }

    /**
     * @brief Parses the filter parameters and all of its sensors.
     * @param[in] path - Path to the .json file normally in the folder config/.
     */
    void reset(const char* path)
    {
        // Create stream and check if valid
        std::ifstream ifs {path};
        if ( !ifs.is_open() )
        {
            std::cout << "Could not open file for reading!\n";
            return;
        }

        // Create Document and parse
        IStreamWrapper isw { ifs };
        Document doc {};
        doc.ParseStream( isw );

        // Filter parameters
        // data-or-time triggered
        m_data_triggered = !doc.HasMember("data_triggered") ? true : doc["data_triggered"].GetBool();

        // Init Estimate Covariance & Process Noise
        for (int i = 0; i < STATE_SIZE; i++)
        {
            for (int j = 0; j < STATE_SIZE; j++)
            {
                m_init_estimation_covariance(i,j) = doc["init_estimate_covariance"][i*STATE_SIZE+j].GetDouble();
                m_process_noise(i,j) = doc["process_noise"][i*STATE_SIZE+j].GetDouble();
            }
        }

        // Parse Sensors parameters
        const char *sensor_types[5] = {"odom_", "pose_", "twist_", "imu_", "gps_"};
        for (uint i = 0; i < 5; i++)
        {
            std::string sensor_type = sensor_types[i];
            std::cout << "Configuring: " << sensor_type << std::endl;
            for (int  j = 0; j < doc["sensor_inputs"][i].GetInt(); j++)
            {
                std::string sensor_name = sensor_type + SSTR(j);
                std::string topic = doc[sensor_name.data()]["topic_name"].GetString();
                std::cout << "Added from config topic: " << topic << std::endl;
                m_sensor_configs.emplace(topic, SensorConfig(doc, sensor_name.data(), i));
            }
        }
        std::cout << "Finished adding sensors!" << std::endl;

    }

    /**
     * @brief Prints out SensorConfig information
     */
    void print()
    {
        std::cout<<"\nInit_estimate_covariance\n";
        std::cout<<"\n"<< m_init_estimation_covariance <<"\n";

        std::cout<<"\nInit_process_noise\n";
        std::cout<<"\n"<< m_process_noise <<"\n";

        std::cout<<"\nSENSOR CONFIGS\n";
        for (auto x: m_sensor_configs)
        {
            std::cout<<x.first<<"";
            x.second.print();
        }
    }
};


using FilterConfig2D = FilterConfig<double>;

} // end namespace motion_model
} // end namespace state_predictor
} // end namespace iav