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

#pragma once
namespace iav{ namespace state_predictor {

  /**
   * These are constants that can be used everywhere under state_predictor namespace
   */
  static constexpr double PI = 3.141592653589793;
  static  constexpr double TAU = 6.283185307179587;

  /**
   * These are types definitions that can be used everywhere under state_predictor namespace
   */
  using tTime = double;

}  // namespace state_predictor
}  // namespace iav

namespace iav{ namespace state_predictor { namespace utilities {

  /**
  * @brief Utility method keeping RPY angles in the range [-pi, pi].
  * @param[in] rotation - The rotation to bind
  * @return the bounded value
  */
  double clamp_rotation(double rotation);

  /**
  * @brief Utility method that transforms standard datatypes to string
  * @param[in] x - Variable of certain standard datatyp
  * @return corresponding string
  */
#define SSTR( x ) static_cast< std::ostringstream & >( \
        ( std::ostringstream() << std::dec << x ) ).str()

}  // namespace utilities
}  // namespace state_predictor
}  // namespace iav


