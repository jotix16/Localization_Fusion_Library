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


#include "sensor_msgs/msg/NavSatStatus.h"
#include "std_msgs/msg/Header.h"
#include "sensor_msgs/msg/NavSatFix.h"


namespace sensor_msgs
{
namespace msg
{

std::size_t NavSatFix::serialize(void *pOutput) const
{
    if (!pOutput)
        return 0;
    uint8_t *buf = (uint8_t*)pOutput;

    buf += header.serialize(buf); // Header
    buf += status.serialize(buf); // NavSatStatus
    *(double*)buf = latitude;
    buf += 8;
    *(double*)buf = longitude;
    buf += 8;
    *(double*)buf = altitude;
    buf += 8;
    *(sensor_msgs__NavSatFix__double_array_9*)buf = position_covariance;
    // buf += None;
    *(uint8_t*)buf = position_covariance_type;
    buf += 1;

    return static_cast<std::size_t>(buf - (uint8_t*)pOutput);
}

std::size_t NavSatFix::deserialize(const void *pInput)
{
    if (!pInput)
        return 0;

    const uint8_t *buf = (const uint8_t*)pInput;


    buf += header.deserialize(buf); // Simple member header: Header
    buf += status.deserialize(buf); // Simple member status: NavSatStatus
    latitude = *reinterpret_cast<const double*>(buf);
    buf += 8;
    longitude = *reinterpret_cast<const double*>(buf);
    buf += 8;
    altitude = *reinterpret_cast<const double*>(buf);
    buf += 8;
    position_covariance = *reinterpret_cast<const sensor_msgs__NavSatFix__double_array_9*>(buf);
    // buf += None;
    position_covariance_type = *reinterpret_cast<const uint8_t*>(buf);
    buf += 1;
    return static_cast<uint32_t>(buf - (uint8_t*)pInput);
}

//! \brief default ctor
NavSatFix::NavSatFix()
{
    latitude = 0;
    longitude = 0;
    altitude = 0;
    // position_covariance = 0; // error: cannot initialize std::array like this
    position_covariance.fill(0); // to initialize a std::array instead
    position_covariance_type = 0;
}

//! \brief dtor
NavSatFix::~NavSatFix()
{







}

//! \brief Copy constructor
NavSatFix::NavSatFix(const sensor_msgs::msg::NavSatFix &other)
{

    header = other.header;
    status = other.status;
    latitude = other.latitude;
    longitude = other.longitude;
    altitude = other.altitude;
    position_covariance = other.position_covariance;
    position_covariance_type = other.position_covariance_type;}

std::size_t NavSatFix::getSerializedSize() const
{
	std::size_t _size_ = 0;

    _size_ += header.getSerializedSize();
    _size_ += status.getSerializedSize();
    _size_ += 8;
    _size_ += 8;
    _size_ += 8;
    // _size_ += None;
    _size_ += 1;
    return _size_;
}

#ifndef IDL_NO_PTR_ASSIGNMENT_OP
sensor_msgs::msg::NavSatFix& NavSatFix::operator=(const sensor_msgs::msg::NavSatFix &other)
{
    if (this != &other)
    {
        header = other.header;
        status = other.status;
        latitude = other.latitude;
        longitude = other.longitude;
        altitude = other.altitude;
        position_covariance = other.position_covariance;
        position_covariance_type = other.position_covariance_type;
    }
    return *this;
}
#endif //IDL_NO_PTR_ASSIGNMENT_OP
} // ns: msg

} // ns: sensor_msgs
