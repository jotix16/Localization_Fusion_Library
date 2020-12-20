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


#include "geometry_msgs/msg/Point.h"


namespace geometry_msgs
{
namespace msg
{

std::size_t Point::serialize(void *pOutput) const
{
    if (!pOutput)
        return 0;
    uint8_t *buf = (uint8_t*)pOutput;

    *(double*)buf = x;
    buf += 8;
    *(double*)buf = y;
    buf += 8;
    *(double*)buf = z;
    buf += 8;

    return static_cast<std::size_t>(buf - (uint8_t*)pOutput);
}

std::size_t Point::deserialize(const void *pInput)
{
    if (!pInput)
        return 0;

    const uint8_t *buf = (const uint8_t*)pInput;


    x = *reinterpret_cast<const double*>(buf);
    buf += 8;
    y = *reinterpret_cast<const double*>(buf);
    buf += 8;
    z = *reinterpret_cast<const double*>(buf);
    buf += 8;
    return static_cast<uint32_t>(buf - (uint8_t*)pInput);
}

//! \brief default ctor
Point::Point()
{

    x = 0;
    y = 0;
    z = 0;
}

//! \brief dtor
Point::~Point()
{



}

//! \brief Copy constructor
Point::Point(const geometry_msgs::msg::Point &other)
{

    x = other.x;
    y = other.y;
    z = other.z;}

std::size_t Point::getSerializedSize() const
{
	std::size_t _size_ = 0;

    _size_ += 8;
    _size_ += 8;
    _size_ += 8;
    return _size_;
}

#ifndef IDL_NO_PTR_ASSIGNMENT_OP
geometry_msgs::msg::Point& Point::operator=(const geometry_msgs::msg::Point &other)
{
    if (this != &other)
    {
        x = other.x;
        y = other.y;
        z = other.z;
    }
    return *this;
}
#endif //IDL_NO_PTR_ASSIGNMENT_OP
} // ns: msg

} // ns: geometry_msgs

