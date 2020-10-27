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


#ifndef __NAVSATSTATUS_DECL_H__
#define __NAVSATSTATUS_DECL_H__


#include "topic_includes.h"


#define __sensor_msgs__msg__NavSatStatus__idl 

#define IDL_HASH_NAVSATSTATUS 0x7f06007499e56087ULL


namespace sensor_msgs {

    namespace msg {
        const uint8_t NavSatStatus__STATUS_NO_FIX = 255;
        const uint8_t NavSatStatus__STATUS_FIX = 0;
        const uint8_t NavSatStatus__STATUS_SBAS_FIX = 1;
        const uint8_t NavSatStatus__STATUS_GBAS_FIX = 2;
        const uint16_t NavSatStatus__SERVICE_GPS = 1;
        const uint16_t NavSatStatus__SERVICE_GLONASS = 2;
        const uint16_t NavSatStatus__SERVICE_COMPASS = 4;
        const uint16_t NavSatStatus__SERVICE_GALILEO = 8;


#define sensor_msgs_msg_NavSatStatus_isPODType 1

//---------------------------------------------------------------------------
//! \class NavSatStatus
//!
//! \brief
//
class NavSatStatus
{
public:
    //! \brief Constructor
    NavSatStatus();
    //! \brief Destructor
    ~NavSatStatus();

    //! \brief Copy constructor
    NavSatStatus(const sensor_msgs::msg::NavSatStatus &other);

    //! \brief Assignment operator
    sensor_msgs::msg::NavSatStatus& operator=(const sensor_msgs::msg::NavSatStatus &other);
#ifndef IDL_NO_PTR_ASSIGNMENT_OP
    //! \brief Assignment operator for pointer source
    inline sensor_msgs::msg::NavSatStatus& operator=(const sensor_msgs::msg::NavSatStatus *other) { *this = *other; return *this; }
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
    uint8_t status;
    uint16_t service;
}; // struct: NavSatStatus

} // ns: msg

} // ns: sensor_msgs


#endif // __NAVSATSTATUS_DECL_H__
