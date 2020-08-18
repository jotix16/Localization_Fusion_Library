#include <array>
#include<iostream>
namespace iav{ namespace state_predictor { namespace motion_model {
	

	// @brief Type trait base class -> called primary template class
	template<int num_states, typename ValueT = double>
	class MotionModel
	{
	public:		
		static constexpr int number_states = num_states;
		using StateVector = std::array<double, number_states>;
		using StateMatrix = std::array<std::array<double, number_states>, number_states >;
	};

	// ctra
	template<int num_states, typename ValueT = double>
	class MotionModelCtra : public MotionModel<num_states, ValueT>
	{
	public:		
		using MM = MotionModel<num_states, ValueT>;
		using StateVector = typename MM::StateVector;
		using StateMatrix = typename MM::StateMatrix;

		static StateMatrix compute_jacoby(const StateVector& state) {
			// implement 			
			return StateMatrix();
		}

		static StateVector predict(const StateVector& state) {
			return StateVector{ 1,1,1,1,1,1 };
		}
		
	};

	// ctrv
	template<int num_states, typename ValueT = double>
	class MotionModelCtrv : public MotionModel<num_states, ValueT>
	{
	public:
		using MM = MotionModel<num_states, ValueT>;
		using StateVector = typename MM::StateVector;
		using StateMatrix = typename MM::StateMatrix;

		static StateMatrix compute_jacoby(const StateVector& state) {
			// implement 			
			return StateMatrix();
		}

		static StateVector predict(const StateVector& state) {
			return StateVector{ 10,10,10,10,10,10, 10,10 };
		}
		static void call() {
			std::cout<<"CALLING MotionModelCtrv"<<std::endl;
		}
	};

	using MyCtrv = MotionModelCtrv<3, double>;
} // end namespace motion_model 
} // end namespace state_predictor
} // end namespace iav