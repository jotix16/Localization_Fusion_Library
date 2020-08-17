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
		using MotionModel = MotionModel<num_states, ValueT>;
		using StateVector = typename MotionModel::StateVector;
		using StateMatrix = typename MotionModel::StateMatrix;

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
		using MotionModel = MotionModel<num_states, ValueT>;
		using StateVector = typename MotionModel::StateVector;
		using StateMatrix = typename MotionModel::StateMatrix;

		static StateMatrix compute_jacoby(const StateVector& state) {
			// implement 			
			return StateMatrix();
		}

		static StateVector predict(const StateVector& state) {
			return StateVector{ 10,10,10,10,10,10, 10,10 };
		}
	};


} // end namespace motion_model 
} // end namespace state_predictor
} // end namespace iav