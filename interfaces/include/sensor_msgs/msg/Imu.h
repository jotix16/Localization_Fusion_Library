////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                        //
//         This program is the confidential and proprietary product of IAV GmbH.          //
// Any unauthorised use, reproduction or transfer of this program is strictly prohibited. //
//                                  Copyright IAV GmbH.                                   //
//             (Subject to limited distribution and restricted disclosure only.)          //
//                                   All rights reserved.                                 //
//                                                                                        //
////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                        //
// This file is generated by IDLCodeGenPy 1.11.1                                          //
//                                                                                        //
// Please do not modify this file by hand!                                                //
//                                                                                        //
////////////////////////////////////////////////////////////////////////////////////////////


#ifndef __IMU_DECL_H__
#define __IMU_DECL_H__


#include "topic_includes.h"

#include "geometry_msgs/msg/Quaternion.h"
#include "geometry_msgs/msg/Vector3.h"
#include "std_msgs/msg/Header.h"

#define __sensor_msgs__msg__Imu__idl 

#define IDL_HASH_IMU 0x364e21f1b86fae1cULL


namespace sensor_msgs {

    namespace msg {

        typedef std::array<double, 9> sensor_msgs__Imu__double_array_9;


#define sensor_msgs_msg_Imu_isPODType 1

//---------------------------------------------------------------------------
//! \class Imu
//!
//! \brief 
//
class Imu
{
public:
    //! \brief Constructor
    Imu();
    //! \brief Destructor
    ~Imu();

    //! \brief Copy constructor
    Imu(const sensor_msgs::msg::Imu &other);

    //! \brief Assignment operator
    sensor_msgs::msg::Imu& operator=(const sensor_msgs::msg::Imu &other);
#ifndef IDL_NO_PTR_ASSIGNMENT_OP
    //! \brief Assignment operator for pointer source
    inline sensor_msgs::msg::Imu& operator=(const sensor_msgs::msg::Imu *other) { *this = *other; return *this; }
#endif

    //! \brief Calculates how many bytes will be needed to serialize the object in its current state.
    std::size_t getSerializedSize() const;

    //! \brief Serialize data into 'buffer'. The buffer needs to have at least the size that getSerializedSize() returns.
    std::size_t serialize(void *buffer) const;

    //! \brief Deserialize data from buffer. The buffer needs to have at least the size that getSerializedSize() returns.
    std::size_t deserialize(const void *buffer);

    //! \brief Indicator function to signal that the class is purely made of simple data types
    static bool isPODType() { return true; }


public:
    std_msgs::msg::Header header;              
    geometry_msgs::msg::Quaternion orientation;
    sensor_msgs__Imu__double_array_9 orientation_covariance;
    geometry_msgs::msg::Vector3 angular_velocity;
    sensor_msgs__Imu__double_array_9 angular_velocity_covariance;
    geometry_msgs::msg::Vector3 linear_acceleration;
    sensor_msgs__Imu__double_array_9 linear_acceleration_covariance;
}; // struct: Imu

} // ns: msg

} // ns: sensor_msgs


#endif // __IMU_DECL_H__

