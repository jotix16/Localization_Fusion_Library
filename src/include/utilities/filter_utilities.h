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

#include <chrono>

namespace iav{ namespace state_predictor {

  /**
   * These are constants that can be used everywhere under state_predictor namespace
   */
  static constexpr double PI = 3.141592653589793;
  static  constexpr double TAU = 6.283185307179587;

  /**
   * These are types definitions that can be used everywhere under state_predictor namespace
   */
  using uint = unsigned short int;
  using tTime = double;
  // using tTime = float32_t;

		/**
		 * @brief Struct that defines the WallTime. It starts a timer when FilterWrapper is initialized. 
     * All global times are then returned relative to this time. 
		 */
  struct Clock{
    using clock_t = std::chrono::high_resolution_clock;
    clock_t::time_point init_time;

		/**
		 * @brief Initializes Wall Time(i.e startes the timer)
		 */
    Clock(): init_time(clock_t::now()) {};

		/**
		 * @brief Used to return actual time.
		 * @return time in seconds passed from init_time.
		 */
    inline tTime now() const
    {
      return std::chrono::duration<tTime>(clock_t::now() - init_time).count();
    }
  };

  /** 
   * Pose and twist messages' parameters
  */
  constexpr uint STATE_SIZE = 15;
  constexpr uint POSE_SIZE = 6;
  constexpr uint TWIST_SIZE = 6;
  constexpr uint POSITION_SIZE = 3;
  constexpr uint ORIENTATION_SIZE = 3;
  constexpr uint LINEAR_VELOCITY_SIZE = 3;
  constexpr uint ACCELERATION_SIZE = 3;

  constexpr uint STATE_X = 0U;       ///< index of x position
  constexpr uint STATE_Y = 1U;       ///< index of y position
  constexpr uint STATE_Z = 2U;       ///< index of z position
  constexpr uint STATE_ROLL= 3U;     ///< index of roll position
  constexpr uint STATE_PITCH= 4U;    ///< index of pitch position
  constexpr uint STATE_YAW= 5U;      ///< index of yaw position
  constexpr uint STATE_V_X = 6U;     ///< index of x velocity
  constexpr uint STATE_V_Y = 7U;     ///< index of y velocity
  constexpr uint STATE_V_Z = 8U;     ///< index of z velocity
  constexpr uint STATE_V_ROLL = 9U;  ///< index of roll velocity
  constexpr uint STATE_V_PITCH= 10U; ///< index of pitch velocity
  constexpr uint STATE_V_YAW= 11U;   ///< index of yaw velocity
  constexpr uint STATE_A_X = 12U;    ///< index of x acceleration
  constexpr uint STATE_A_Y = 13U;    ///< index of y acceleration
  constexpr uint STATE_A_Z = 14U;    ///< index of z acceleratio

}  // namespace state_predictor
}  // namespace iav

namespace iav{ namespace state_predictor{ namespace utilities{

  /**
  * @brief Utility method keeping RPY angles in the range [-pi, pi].
  * @param[in] rotation - The rotation to bind
  * @return the bounded value
  */
  double clamp_rotation(tTime rotation)
  {
      while (rotation > PI){rotation -= TAU;};
      while (rotation < -PI){rotation += TAU;};
      return rotation;
  }

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


