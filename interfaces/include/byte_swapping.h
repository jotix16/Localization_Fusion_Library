////////////////////////////////////////////////////////////////////////////////////////////
//                                                                                        //
//         This program is the confidential and proprietary product of IAV GmbH.          //
// Any unauthorised use, reproduction or transfer of this program is strictly prohibited. //
//                                  Copyright IAV GmbH.                                   //
//             (Subject to limited distribution and restricted disclosure only.)          //
//                                   All rights reserved.                                 //
//                                                                                        //
////////////////////////////////////////////////////////////////////////////////////////////

#ifndef __BYTE_SWAPPING_H__
#define __BYTE_SWAPPING_H__

#include <stdlib.h>

/**
* This header provides byte swapping functions generated by IDLCodeGenPy.
* They are needed when using the annotation @SerializedByteOrder(BE/LE).
*/


static inline uint16_t fw_byteswap_u16(uint16_t Data)
{
#ifdef _WINDOWS
    return _byteswap_ushort(Data);
#else
    return __builtin_bswap16(Data);
#endif
}

static inline uint32_t fw_byteswap_u32(uint32_t Data)
{
#ifdef _WINDOWS
    return _byteswap_ulong(Data);
#else
    return __builtin_bswap32(Data);
#endif
}

static inline uint64_t fw_byteswap_u64(uint64_t Data)
{
#ifdef _WINDOWS
    return _byteswap_uint64(Data);
#else
    return __builtin_bswap64(Data);
#endif
}

#endif