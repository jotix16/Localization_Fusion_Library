#pragma once
namespace iav{ namespace state_predictor { namespace utilities {

  //! @brief Utility method keeping RPY angles in the range [-pi, pi]
  //! @param[in] rotation - The rotation to bind
  //! @return the bounded value
  //!
  double clampRotation(double rotation);

}  // namespace iav
}  // namespace state_predictor
}  // namespace utilities
