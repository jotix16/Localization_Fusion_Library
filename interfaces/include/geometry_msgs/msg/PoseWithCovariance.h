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


#ifndef __POSEWITHCOVARIANCE_DECL_H__
#define __POSEWITHCOVARIANCE_DECL_H__


#include "topic_includes.h"

#include "Pose.h"

#define __geometry_msgs__msg__PoseWithCovariance__idl 

#define IDL_HASH_POSEWITHCOVARIANCE 0x8990e6c33aa71aa6ULL


namespace geometry_msgs {

    namespace msg {

        typedef std::array<double, 36> geometry_msgs__PoseWithCovariance__double_array_36;


#define geometry_msgs_msg_PoseWithCovariance_isPODType 1

//---------------------------------------------------------------------------
//! \class PoseWithCovariance
//!
//! \brief 
//
class PoseWithCovariance
{
public:
    //! \brief Constructor
    PoseWithCovariance();
    //! \brief Destructor
    ~PoseWithCovariance();

    //! \brief Copy constructor
    PoseWithCovariance(const geometry_msgs::msg::PoseWithCovariance &other);

    //! \brief Assignment operator
    geometry_msgs::msg::PoseWithCovariance& operator=(const geometry_msgs::msg::PoseWithCovariance &other);
#ifndef IDL_NO_PTR_ASSIGNMENT_OP
    //! \brief Assignment operator for pointer source
    inline geometry_msgs::msg::PoseWithCovariance& operator=(const geometry_msgs::msg::PoseWithCovariance *other) { *this = *other; return *this; }
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
    geometry_msgs::msg::Pose pose;             
    geometry_msgs__PoseWithCovariance__double_array_36 covariance;
}; // struct: PoseWithCovariance

} // ns: msg

} // ns: geometry_msgs


#endif // __POSEWITHCOVARIANCE_DECL_H__

