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

#include <utilities/filter_utilities.h>

namespace iav{ namespace state_predictor{ namespace utilities{
    double clamp_rotation(tTime rotation)
    {
        while (rotation > PI)
        {
            rotation -= TAU;
        }

        while (rotation < -PI)
        {
            rotation += TAU;
        }

        return rotation;
    }

}  // namespace utilities
}  // namespace state_predictor
}  // namespace iav