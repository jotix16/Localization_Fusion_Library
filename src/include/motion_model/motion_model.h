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
#include <Eigen/Dense>

#include <utilities/filter_utilities.h>


//TO_DO: No AngleWrapping done yet, with the motivation that
// 	 	 it is better done in KalmanWrapper after both observation_step and prediction_step
namespace iav{ namespace state_predictor { namespace motion_model {

	/**
	* @brief Type trait base class -> called primary template class
	*/
	template<uint num_states, typename T = double>
	class MotionModel
	{
	public:
		static constexpr uint number_states = num_states;
		using ValueType = T;
		using StateVector = Eigen::Matrix<T, num_states, 1>;
		using StateMatrix = Eigen::Matrix<T, num_states, num_states>;
	};

	/**
	* @brief ctra2D: Constant turning rate and acceleration
	*/
	template<typename T = double>
	class MotionModelCtra2D : public MotionModel<8, T>
	{
	public:
		/**
		* @brief This state gives named handles for state indexing
		*/
		struct States
		{
			using Array = std::array<std::uint32_t, static_cast<std::size_t>(STATE_SIZE)>;
			static constexpr uint X = 0U;  ///< index of x position
			static constexpr uint Y = 1U;  ///< index of y position
			static constexpr uint YAW= 2U;  ///< index of yaw position
			static constexpr uint V_X = 3U;  ///< index of x velocity
			static constexpr uint V_Y = 4U;  ///< index of y velocity
			static constexpr uint V_YAW = 5U;  ///< index of yaw velocity
			static constexpr uint A_X = 6U;  ///< index of x acceleration
			static constexpr uint A_Y = 7U;  ///< index of y acceleration

			static constexpr Array full_state_to_estimated_state = {0,1,15,15,15,2,3,4,15,15,15,5,6,7,15};

			static constexpr uint STATE_SIZE_M = 8U;
			static constexpr uint POSITION_OFFSET_M = 0U;
			static constexpr uint ORIENTATION_OFFSET_M = 2U;
			static constexpr uint POSITION_V_OFFSET_M = 3U;
			static constexpr uint ORIENTATION_V_OFFSET_M = 5U;
			static constexpr uint POSITION_A_OFFSET_M = 6U;

		};  // struct States

	public:
		static constexpr uint num_states = MotionModel<8, T>::number_states;
		using StateVector = typename MotionModel<8, T>::StateVector;
		using StateMatrix = typename MotionModel<8, T>::StateMatrix;

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
			jacobi(States::X, States::V_Y) = -dt_s_yaw;
			jacobi(States::X, States::YAW) = -dt_s_yaw * state[States::V_X] - dt_c_yaw * state[States::V_Y]
			 								- dt_dt_s_yaw_05 * state[States::A_X] - dt_dt_c_yaw_05 * state[States::A_Y];
			jacobi(States::X, States::A_X) = dt_dt_c_yaw_05;
			jacobi(States::X, States::A_Y) = -dt_dt_s_yaw_05;
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
			transform_matrix(States::X, States::V_Y) = -dt_s_yaw;
			transform_matrix(States::X, States::A_X) = dt_2 * dt_c_yaw;
			transform_matrix(States::X, States::A_Y) = -dt_2 * dt_s_yaw;

			transform_matrix(States::Y, States::V_X) = dt_s_yaw;
			transform_matrix(States::Y, States::V_Y) = dt_c_yaw;
			transform_matrix(States::Y, States::A_X) = -transform_matrix(States::X, States::A_Y);
			transform_matrix(States::Y, States::A_Y) = transform_matrix(States::X, States::A_X);

			transform_matrix(States::V_X, States::A_X) = dt;
			transform_matrix(States::V_Y, States::A_Y) = dt;

			transform_matrix(States::YAW, States::V_YAW) = dt;

			state = transform_matrix * state;
		}
	};

	/**
	* @brief ctrv2D: Constant turning rate and velocity
	*/
	template<typename T = double>
	class MotionModelCtrv2D : public MotionModel<6, T>
	{
	public:
		/**
		* @brief This state gives named handles for state indexing
		*/
		struct States
		{
			using Array = std::array<std::uint32_t, static_cast<std::size_t>(STATE_SIZE)>;
			static constexpr uint X = 0U;  ///< index of x position
			static constexpr uint Y = 1U;  ///< index of y position
			static constexpr uint YAW= 2U;  ///< index of yaw position
			static constexpr uint V_X = 3U;  ///< index of x velocity
			static constexpr uint V_Y = 4U;  ///< index of y velocity
			static constexpr uint V_YAW = 5U;  ///< index of yaw velocity
			static constexpr Array full_state_to_estimated_state = {0,1,15,15,15,2,3,4,15,15,15,5,15,15,15};

			static constexpr uint STATE_SIZE_M = 6U;
			static constexpr uint POSITION_OFFSET_M = 0U;
			static constexpr uint ORIENTATION_OFFSET_M = 2U;
			static constexpr uint POSITION_V_OFFSET_M = 3U;
			static constexpr uint ORIENTATION_V_OFFSET_M = 5U;
			static constexpr uint POSITION_A_OFFSET_M = 6U;
		};  // struct States

	public:
		static constexpr uint num_states = MotionModel<6, T>::number_states;
		using StateVector = typename MotionModel<6, T>::StateVector;
		using StateMatrix = typename MotionModel<6, T>::StateMatrix;


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
			jacobi(States::X, States::V_Y) = -dt_s_yaw;
			jacobi(States::X, States::YAW) = -dt_s_yaw * state[States::V_X] - dt_c_yaw * state[States::V_Y];
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
			transform_matrix(States::X, States::V_Y) = -dt_s_yaw;
			transform_matrix(States::Y, States::V_X) = dt_s_yaw;
			transform_matrix(States::Y, States::V_Y) = dt_c_yaw;
			transform_matrix(States::YAW, States::V_YAW) = dt;
			state = transform_matrix * state;
		}
	};

	/**
	* @brief ctra3D: Constant turning rate and acceleration
	*/
	template<typename T = double>
	class MotionModelCtra3D : public MotionModel<15, T>
	{
	public:
		/**
		* @brief This state gives named handles for state indexing
		*/
		struct States
		{
			using Array = std::array<std::uint32_t, static_cast<std::size_t>(STATE_SIZE)>;
			static constexpr uint X = 0U;  ///< index of x position
			static constexpr uint Y = 1U;  ///< index of y position
			static constexpr uint Z = 2U;  ///< index of z position
			static constexpr uint ROLL= 3U;  ///< index of roll position
			static constexpr uint PITCH= 4U;  ///< index of pitch position
			static constexpr uint YAW= 5U;  ///< index of yaw position
			static constexpr uint V_X = 6U;  ///< index of x velocity
			static constexpr uint V_Y = 7U;  ///< index of y velocity
			static constexpr uint V_Z = 8U;  ///< index of z velocity
			static constexpr uint V_ROLL = 9U;  ///< index of roll velocity
			static constexpr uint V_PITCH= 10U;  ///< index of pitch velocity
			static constexpr uint V_YAW= 11U;  ///< index of yaw velocity
			static constexpr uint A_X = 12U;  ///< index of x acceleration
			static constexpr uint A_Y = 13U;  ///< index of y acceleration
			static constexpr uint A_Z = 14U;  ///< index of z acceleration
			static constexpr Array full_state_to_estimated_state = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14};

			static constexpr uint STATE_SIZE_M = 15U;
			static constexpr uint POSITION_OFFSET_M = 0U;
			static constexpr uint ORIENTATION_OFFSET_M = 3U;
			static constexpr uint POSITION_V_OFFSET_M = 6U;
			static constexpr uint ORIENTATION_V_OFFSET_M = 9U;
			static constexpr uint POSITION_A_OFFSET_M = 12U;

		};  // struct States

	public:
		static constexpr uint num_states = MotionModel<15, T>::number_states;
		using StateVector = typename MotionModel<15, T>::StateVector;
		using StateMatrix = typename MotionModel<15, T>::StateMatrix;

		/**
		 * @brief Ctra3D: Method that computes the Jacobian and performs a predict step
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
		 * @brief Ctra3D: Method that computes the Jacobian
		 * 		  in the right order. It leverages the computed Jacobian to save some computations too.
		 * @param[inout] jacobi - Matrix to save Jacobian into
		 * @param[in] state - The actual state to be used to compute the Jacobian
		 * @param[in] dt - The time-difference for which the prediction has to be performed
		 * @return the bounded value
		 */
		//TO_DO
		static void compute_jacobian(StateMatrix& jacobi, const StateVector& state, const tTime& dt)
		{
			T dt_2 = 0.5 * dt;
			T cr = cos(state[States::ROLL]);
			T sr = sin(state[States::ROLL]);

			T cp = cos(state[States::PITCH]);
			T sp = sin(state[States::PITCH]);
			T cpi = 1.0f/cp;
			T tp = sp * cpi;

			T cy = cos(state[States::YAW]);
			T sy = sin(state[States::YAW]);

			T x_coef = 0;
			T y_coef = 0;
			T z_coef = 0;

			jacobi.setIdentity();

			// The Linear part
			x_coef = dt * cy * cp;
			y_coef = dt * (cy * sp * sr - sy * cr);
			z_coef = dt * (cy * sp * cr + sy * sr);
			jacobi(States::X, States::V_X) = x_coef;
			jacobi(States::X, States::V_Y) = y_coef;
			jacobi(States::X, States::V_Z) = z_coef;
			jacobi(States::X, States::A_X) = dt_2 * x_coef;
			jacobi(States::X, States::A_Y) = dt_2 * y_coef;
			jacobi(States::X, States::A_Z) = dt_2 * z_coef;

			x_coef = dt * sy * cp;
			y_coef = dt * (sy * sp * sr + cy * cr);
			z_coef = dt * (sy * sp * cr - cy * sr);
			jacobi(States::Y, States::V_X) = x_coef;
			jacobi(States::Y, States::V_Y) = y_coef;
			jacobi(States::Y, States::V_Z) = z_coef;
			jacobi(States::Y, States::A_X) = dt_2 * x_coef;
			jacobi(States::Y, States::A_Y) = dt_2 * y_coef;
			jacobi(States::Y, States::A_Z) = dt_2 * z_coef;

			x_coef = -dt * sp;
			y_coef = dt * cp * sr;
			z_coef = dt * cp * cr;
			jacobi(States::Z, States::V_X) = x_coef;
			jacobi(States::Z, States::V_Y) = y_coef;
			jacobi(States::Z, States::V_Z) = z_coef;
			jacobi(States::Z, States::A_X) = dt_2 * x_coef;
			jacobi(States::Z, States::A_Y) = dt_2 * y_coef;
			jacobi(States::Z, States::A_Z) = dt_2 * z_coef;

			jacobi(States::ROLL, States::V_ROLL) = dt;
			jacobi(States::ROLL, States::V_PITCH) = dt * sr * tp;
			jacobi(States::ROLL, States::V_YAW) = dt * cr * tp;
			jacobi(States::PITCH, States::V_PITCH) = dt * cr;
			jacobi(States::PITCH, States::V_YAW) = -dt * sr;
			jacobi(States::YAW, States::V_PITCH) = dt * sr * cpi;
			jacobi(States::YAW, States::V_YAW) = dt * cr * cpi;

			jacobi(States::V_X, States::A_X) = dt;
			jacobi(States::V_Y, States::A_Y) = dt;
			jacobi(States::V_Z, States::A_Y) = dt;

			//TO_DO
			//The nonlinear part
			T v_x = dt * state[States::V_X];
			T v_y = dt * state[States::V_Y];
			T v_z = dt * state[States::V_Z];
			T a_x = dt_2 * dt * state[States::A_X];
			T a_y = dt_2 * dt * state[States::A_Y];
			T a_z = dt_2 * dt * state[States::A_Z];
			T v_roll = dt * state[States::V_ROLL];
			T v_pitch = dt * state[States::V_PITCH];
			T v_yaw = dt * state[States::V_YAW];

			// X
			x_coef = 0;
			y_coef = cy * sp * cr + sy * sr;
			z_coef = -cy * sp * sr + sy * cr;
			T dJx_dR = v_y * y_coef + v_z * z_coef + a_y * y_coef + a_z * z_coef;

			x_coef = -cy * sp;
			y_coef = cy * cp * sr;
			z_coef = cy * cp * cr;
			T dJx_dP = v_x * x_coef + v_y * y_coef + v_z * z_coef + a_x * x_coef + a_y * y_coef + a_z * z_coef;

			x_coef = -sy * cp;
			y_coef = -sy * sp * sr - cy * cr;
			z_coef = -sy * sp * cr + cy * sr;
			T dJx_dY = v_x * x_coef + v_y * y_coef + v_z * z_coef + a_x * x_coef + a_y * y_coef + a_z * z_coef;

			// Y
			x_coef = 0;
			y_coef = sy * sp * cr - cy * sr;
			z_coef = -sy * sp * sr - cy * cr;
			T dJy_dR = v_y * y_coef + v_z * z_coef + a_y * y_coef + a_z * z_coef;

			x_coef = -sy * sp;
			y_coef = sy * cp * sr;
			z_coef = sy * cp * cr;
			T dJy_dP = v_x * x_coef + v_y * y_coef + v_z * z_coef + a_x * x_coef + a_y * y_coef + a_z * z_coef;

			x_coef = cy * cp;
			y_coef = (cy * sp * sr - sy * cr);
			z_coef = (cy * sp * cr + sy * sr);
			T dJy_dY = v_x * x_coef + v_y * y_coef + v_z * z_coef + a_x * x_coef + a_y * y_coef + a_z * z_coef;

			// Z
			x_coef = 0;
			y_coef = cp * cr;
			z_coef = -cp * sr;
			T dJz_dR = v_y * y_coef + v_z * z_coef + a_y * y_coef + a_z * z_coef;

			x_coef = -cp;
			y_coef = -sp * sr;
			z_coef = -sp * cr;
			T dJz_dP = v_x * x_coef + v_y * y_coef + v_z * z_coef + a_x * x_coef + a_y * y_coef + a_z * z_coef;

			// ROLL PITCH YAW
			T cpi_2 = cpi * cpi;
			T dJR_dR = 1.0 + cr * tp * v_pitch - sr * tp * v_yaw;
			T dJR_dP = sr * cpi_2 * v_pitch + cr * cpi_2 * v_yaw;
			T dJP_dR = -sr * v_pitch - cr * v_yaw ;
			T dJY_dR = cr * cpi * v_pitch - sr * cpi * v_yaw;
			T dJY_dP = sr * tp * cpi * v_pitch + cr * tp * cpi * v_yaw;

			jacobi(States::X, States::ROLL) = dJx_dR;
			jacobi(States::X, States::PITCH) = dJx_dP;
			jacobi(States::X, States::YAW) = dJx_dY;
			jacobi(States::Y, States::ROLL) = dJy_dR;
			jacobi(States::Y, States::PITCH) = dJy_dP;
			jacobi(States::Y, States::YAW) = dJy_dY;
			jacobi(States::Z, States::ROLL) = dJz_dR;
			jacobi(States::Z, States::PITCH) = dJz_dP;
			jacobi(States::ROLL, States::ROLL) = dJR_dR;
			jacobi(States::ROLL, States::PITCH) = dJR_dP;
			jacobi(States::PITCH, States::ROLL) = dJP_dR;
			jacobi(States::YAW, States::ROLL) = dJY_dR;
			jacobi(States::YAW, States::PITCH) = dJY_dP;
		}

		/**
		 * @brief Ctra3D: Method that updates the current state with a given motion. It is called after
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
			transform_matrix(States::X, States::ROLL) = 0;
			transform_matrix(States::X, States::PITCH) = 0;
			transform_matrix(States::X, States::YAW) = 0;
			transform_matrix(States::Y, States::ROLL) = 0;
			transform_matrix(States::Y, States::PITCH) = 0;
			transform_matrix(States::Y, States::YAW) = 0;
			transform_matrix(States::ROLL, States::ROLL) = 0;
			transform_matrix(States::ROLL, States::PITCH) = 0;
			transform_matrix(States::PITCH, States::PITCH) = 0;
			transform_matrix(States::YAW, States::ROLL) = 0;
			transform_matrix(States::YAW, States::PITCH) = 0;
			state = transform_matrix * state;
		}

		/**
		 * @brief Ctra3D: Method that updates the current state with a given motion. It is called after
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
			T cr = cos(state[States::ROLL]);
			T sr = sin(state[States::ROLL]);

			T cp = cos(state[States::PITCH]);
			T sp = sin(state[States::PITCH]);
			T cpi = 1.0f/cp;
			T tp = sp * cpi;

			T cy = cos(state[States::YAW]);
			T sy = sin(state[States::YAW]);

			T x_coef = 0;
			T y_coef = 0;
			T z_coef = 0;

			StateMatrix transform_matrix;
			transform_matrix.setIdentity();

			// The Linear part
			x_coef = dt * cy * cp;
			y_coef = dt * (cy * sp * sr - sy * cr);
			z_coef = dt * (cy * sp * cr + sy * sr);
			transform_matrix(States::X, States::V_X) = x_coef;
			transform_matrix(States::X, States::V_Y) = y_coef;
			transform_matrix(States::X, States::V_Z) = z_coef;
			transform_matrix(States::X, States::A_X) = dt_2 * x_coef;
			transform_matrix(States::X, States::A_Y) = dt_2 * y_coef;
			transform_matrix(States::X, States::A_Z) = dt_2 * z_coef;

			x_coef = dt * sy * cp;
			y_coef = dt * (sy * sp * sr + cy * cr);
			z_coef = dt * (sy * sp * cr - cy * sr);
			transform_matrix(States::Y, States::V_X) = x_coef;
			transform_matrix(States::Y, States::V_Y) = y_coef;
			transform_matrix(States::Y, States::V_Z) = z_coef;
			transform_matrix(States::Y, States::A_X) = dt_2 * x_coef;
			transform_matrix(States::Y, States::A_Y) = dt_2 * y_coef;
			transform_matrix(States::Y, States::A_Z) = dt_2 * z_coef;

			x_coef = -dt * sp;
			y_coef = dt * cp * sr;
			z_coef = dt * cp * cr;
			transform_matrix(States::Z, States::V_X) = x_coef;
			transform_matrix(States::Z, States::V_Y) = y_coef;
			transform_matrix(States::Z, States::V_Z) = z_coef;
			transform_matrix(States::Z, States::A_X) = dt_2 * x_coef;
			transform_matrix(States::Z, States::A_Y) = dt_2 * y_coef;
			transform_matrix(States::Z, States::A_Z) = dt_2 * z_coef;

			transform_matrix(States::ROLL, States::V_ROLL) = dt;
			transform_matrix(States::ROLL, States::V_PITCH) = dt * sr * tp;
			transform_matrix(States::ROLL, States::V_YAW) = dt * cr * tp;
			transform_matrix(States::PITCH, States::V_PITCH) = dt * cr;
			transform_matrix(States::PITCH, States::V_YAW) = -dt * sr;
			transform_matrix(States::YAW, States::V_PITCH) = dt * sr * cpi;
			transform_matrix(States::YAW, States::V_YAW) = dt * cr * cpi;

			transform_matrix(States::V_X, States::A_X) = dt;
			transform_matrix(States::V_Y, States::A_Y) = dt;
			transform_matrix(States::V_Z, States::A_Y) = dt;
			state = transform_matrix * state;
		}
	};

	template<typename T>
	constexpr typename MotionModelCtra2D<T>::States::Array
	MotionModelCtra2D<T>::States::full_state_to_estimated_state;

	template<typename T>
	constexpr uint MotionModelCtra2D<T>::num_states;

	template<typename T>
	constexpr typename MotionModelCtrv2D<T>::States::Array
	MotionModelCtrv2D<T>::States::full_state_to_estimated_state;
	template<typename T>
	constexpr uint MotionModelCtrv2D<T>::num_states;

	template<typename T>
	constexpr typename MotionModelCtra3D<T>::States::Array
	MotionModelCtra3D<T>::States::full_state_to_estimated_state;
	template<typename T>
	constexpr uint MotionModelCtra3D<T>::num_states;

	using Ctrv2D = MotionModelCtrv2D<double>;
	using Ctra2D = MotionModelCtra2D<double>;
	using Ctra3D = MotionModelCtra3D<double>;

} // end namespace motion_model
} // end namespace state_predictor
} // end namespace iav