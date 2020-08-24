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

#include <array>
#include <iostream>
#include<Eigen/Dense>

#include<utilities/filter_utilities.h>


//TO_DO: No AngleWrapping done yet, with the motivation that 
// 	 	 it is better done in KalmanWrapper after both observation_step and prediction_step
namespace iav{ namespace state_predictor { namespace motion_model {
	using namespace Eigen;
	
	// @brief Type trait base class -> called primary template class
	template<int num_states, typename T = double>
	class MotionModel
	{
	public:		
		static constexpr int number_states = num_states;
		using StateVector = Matrix<T, num_states, 1>;
		using StateMatrix = Matrix<T, num_states, num_states>;
	};

	// @brief ctra2D: Constant turning rate and acceleration
	template<typename T = double>
	class MotionModelCtra2D : public MotionModel<8, T>
	{
	public:		
		/**
		* This state gives named handles for state indexing
		*/
		struct States
		{
			static constexpr int X = 0U;  ///< index of x position
			static constexpr int Y = 1U;  ///< index of y position
			static constexpr int YAW= 2U;  ///< index of yaw position
			static constexpr int V_X = 3U;  ///< index of x velocity
			static constexpr int V_Y = 4U;  ///< index of y velocity
			static constexpr int V_YAW = 5U;  ///< index of yaw velocity
			static constexpr int A_X = 6U;  ///< index of x acceleration
			static constexpr int A_Y = 7U;  ///< index of y acceleration

		};  // struct States

	public:		
		using StateVector = typename MotionModel::StateVector;
		using StateMatrix = typename MotionModel::StateMatrix;
		
		/**
		 * @brief Ctra2D: Method that computes the Jacobian and performs a predict step
		 *		  in the right order. It leverages the computed Jacobian to save some computations too.
		 * @param[inout] jacobi - Matrix to save Jacobian into
		 * @param[inout] state - The actual state to be used to compute the Jacobian and be updated later
		 * @param[in] dt - The time-difference for which the prediction has to be performed
		 * @return the bounded value
		 */
		static void compute_jacobian_and_predict(StateMatrix& jacobi, StateVector& state, const tTime& dt)
		{
			compute_jacobian(jacobi, state, dt);
			predict(state, jacobi, dt);
		}

		/**
		 * @brief Ctra2D: Method that computes the Jacobian
		 * 		  in the right order. It leverages the computed Jacobian to save some computations too.
		 * @param[inout] jacobi - Matrix to save Jacobian into
		 * @param[in] state - The actual state to be used to compute the Jacobian
		 * @param[in] dt - The time-difference for which the prediction has to be performed
		 * @return the bounded value
		 */
		static void compute_jacobian(StateMatrix& jacobi, const StateVector& state, const tTime& dt)
		{
			T dt_c_yaw = dt*cos(state[States::YAW]);
			T dt_s_yaw = dt*sin(state[States::YAW]);
			T dt_dt_c_yaw_05 = dt_c_yaw * dt * 0.5;
			T dt_dt_s_yaw_05 = dt_s_yaw * dt * 0.5;

			jacobi.setIdentity();
			jacobi(States::X, States::V_X) = dt_c_yaw;
			jacobi(States::X, States::V_Y) = dt_s_yaw;
			jacobi(States::X, States::YAW) = -dt_s_yaw * state[States::V_X] + dt_c_yaw * state[States::V_Y]
			 								- dt_dt_s_yaw_05 * state[States::A_X] + dt_dt_c_yaw_05 * state[States::A_Y];
			jacobi(States::X, States::A_X) = dt_dt_c_yaw_05;
			jacobi(States::X, States::A_Y) = dt_dt_s_yaw_05;
			jacobi(States::V_X, States::A_X) = dt;

			jacobi(States::Y, States::V_X) = dt_s_yaw;
			jacobi(States::Y, States::V_Y) = dt_c_yaw;
			jacobi(States::Y, States::YAW) = dt_c_yaw * state[States::V_X] - dt_s_yaw * state[States::V_Y]
			 								+ dt_dt_c_yaw_05 * state[States::A_X] - dt_dt_s_yaw_05 * state[States::A_Y];
			jacobi(States::Y, States::A_X) = dt_dt_s_yaw_05;
			jacobi(States::Y, States::A_Y) = dt_dt_c_yaw_05;
			jacobi(States::V_Y, States::A_Y) = dt;

			jacobi(States::YAW, States::V_YAW) = dt;
		}

		/**
		 * @brief Ctra2D: Method that updates the current state with a given motion. It is called after
		 * 		  compute_jacobian() in order to make sure that the state is not updated before 
		 * 		  the Jacobian is computed. It is normally followed by an observation update.
		 * 		  It is faster than the other predict function as it leverages the computed Jacobian 
		 * 		  to save some computations!
		 * @param[inout] jacobi - Matrix to save Jacobian into
		 * @param[in] state - The actual state to be used to compute the Jacobian
		 * @param[in] dt - The time-difference for which the prediction has to be performed
		 * @return the bounded value
		 */
		static void predict(StateVector& state, const StateMatrix& jacobi, const tTime& dt)
		{
			StateMatrix transform_matrix = jacobi;
			transform_matrix(States::X, States::YAW) = 0;
			transform_matrix(States::Y, States::YAW) = 0;
			state = transform_matrix * state;
		}

		/**
		 * @brief Ctra2D: Method that updates the current state with a given motion. It is called after
		 * 		  compute_jacobian() in order to make sure that the state is not updated before 
		 * 		  the Jacobian is computed. It is normally followed by an observation update.
		 * @param[inout] jacobi - Matrix to save Jacobian into
		 * @param[in] state - The actual state to be used to compute the Jacobian
		 * @param[in] dt - The time-difference for which the prediction has to be performed
		 * @return the bounded value
		 */
		static void predict(StateVector& state, const tTime& dt)
		{
			T dt_2 = 0.5 * dt;
			T dt_c_yaw = dt * cos(state[States::YAW]);
			T dt_s_yaw = dt * sin(state[States::YAW]);

			StateMatrix transform_matrix;
			transform_matrix.setIdentity();
			transform_matrix(States::X, States::V_X) = dt_c_yaw;
			transform_matrix(States::X, States::V_Y) = dt_s_yaw;
			transform_matrix(States::X, States::A_X) = dt_2 * dt_c_yaw;
			transform_matrix(States::X, States::A_Y) = dt_2 * dt_s_yaw;

			transform_matrix(States::Y, States::V_X) = dt_s_yaw;
			transform_matrix(States::Y, States::V_Y) = dt_c_yaw;
			transform_matrix(States::Y, States::A_X) = transform_matrix(States::X, States::A_Y);
			transform_matrix(States::Y, States::A_Y) = transform_matrix(States::X, States::A_X);

			transform_matrix(States::V_X, States::A_X) = dt;
			transform_matrix(States::V_Y, States::A_Y) = dt;

			transform_matrix(States::YAW, States::V_YAW) = dt;

			state = transform_matrix * state;
		}
	};

	// @brief ctrv2D: Constant turning rate and velocity
	template<typename T = double>
	class MotionModelCtrv2D : public MotionModel<6, T>
	{
	public:
		/**
		* This state gives named handles for state indexing
		*/
		struct States
		{
			static constexpr int X = 0U;  ///< index of x position
			static constexpr int Y = 1U;  ///< index of y position
			static constexpr int YAW= 2U;  ///< index of yaw position
			static constexpr int V_X = 3U;  ///< index of x velocity
			static constexpr int V_Y = 4U;  ///< index of y velocity
			static constexpr int V_YAW = 5U;  ///< index of yaw velocity

		};  // struct States

	public:
		using StateVector = typename MotionModel::StateVector;
		using StateMatrix = typename MotionModel::StateMatrix;


		/**
		 * @brief Ctrv2D: Method that computes the Jacobian and performs a predict step
		 *		  in the right order. It leverages the computed Jacobian to save some computations too.
		 * @param[inout] jacobi - Matrix to save Jacobian into
		 * @param[inout] state - The actual state to be used to compute the Jacobian and be updated later
		 * @param[in] dt - The time-difference for which the prediction has to be performed
		 * @return the bounded value
		 */
		static void compute_jacobian_and_predict(StateMatrix& jacobi, StateVector& state, const tTime& dt)
		{
			compute_jacobian(jacobi, state, dt);
			predict(state, jacobi, dt);
		}


		/**
		 * @brief Ctrv2D: Method that computes the Jacobian
		 * 		  in the right order. It leverages the computed Jacobian to save some computations too.
		 * @param[inout] jacobi - Matrix to save Jacobian into
		 * @param[in] state - The actual state to be used to compute the Jacobian
		 * @param[in] dt - The time-difference for which the prediction has to be performed
		 * @return the bounded value
		 */
		static void compute_jacobian(StateMatrix& jacobi, const StateVector& state, const tTime& dt)
		{
			T dt_c_yaw = dt*cos(state[States::YAW]);
			T dt_s_yaw = dt*sin(state[States::YAW]);

			jacobi.setIdentity();
			jacobi(States::X, States::V_X) = dt_c_yaw;
			jacobi(States::X, States::V_Y) = dt_s_yaw;
			jacobi(States::X, States::YAW) = -dt_s_yaw * state[States::V_X] + dt_c_yaw * state[States::V_Y];
			jacobi(States::Y, States::V_X) = dt_s_yaw;
			jacobi(States::Y, States::V_Y) = dt_c_yaw;
			jacobi(States::Y, States::YAW) = dt_c_yaw * state[States::V_X] - dt_s_yaw * state[States::V_Y];
			jacobi(States::YAW, States::V_YAW) = dt;
		}


		/**
		 * @brief Ctra2D: Method that computes the Jacobian
		 * 		  in the right order. It leverages the computed Jacobian to save some computations too.
		 * @param[inout] jacobi - Matrix to save Jacobian into
		 * @param[in] state - The actual state to be used to compute the Jacobian
		 * @param[in] dt - The time-difference for which the prediction has to be performed
		 * @return the bounded value
		 */
		static void predict(StateVector& state, const StateMatrix& jacobi, const tTime& dt)
		{
			StateMatrix transform_matrix = jacobi;
			transform_matrix(States::X, States::YAW) = 0;
			transform_matrix(States::Y, States::YAW) = 0;
			state = transform_matrix * state; 
		}


		 /** @brief Ctrv2D: Method that updates the current state with a given motion. It is called after
		 * 		  compute_jacobian() in order to make sure that the state is not updated before 
		 * 		  the Jacobian is computed. It is normally followed by an observation update.
		 * @param[inout] jacobi - Matrix to save Jacobian into
		 * @param[in] state - The actual state to be used to compute the Jacobian
		 * @param[in] dt - The time-difference for which the prediction has to be performed
		 * @return the bounded value
		 */
		static void predict(StateVector& state, const tTime& dt)
		{
			T dt_c_yaw = dt*cos(state[States::YAW]);
			T dt_s_yaw = dt*sin(state[States::YAW]);

			StateMatrix transform_matrix;
			transform_matrix.setIdentity();
			transform_matrix(States::X, States::V_X) = dt_c_yaw;
			transform_matrix(States::X, States::V_Y) = dt_s_yaw;
			transform_matrix(States::Y, States::V_X) = dt_s_yaw;
			transform_matrix(States::Y, States::V_Y) = dt_c_yaw;
			transform_matrix(States::YAW, States::V_YAW) = dt;
			state = transform_matrix * state;
		}
	};

	using Ctrv2D = MotionModelCtrv2D<double>;
	using Ctra2D = MotionModelCtra2D<double>;
} // end namespace motion_model 
} // end namespace state_predictor
} // end namespace iav 
     