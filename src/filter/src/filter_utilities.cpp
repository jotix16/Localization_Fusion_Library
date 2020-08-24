#include <filter/filter_utilities.h>


namespace iav{ namespace state_predictor {
  const double PI = 3.141592653589793;
  const double TAU = 6.283185307179587;
 namespace motion_model {
     
    double clampRotation(double rotation)
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

}  // namespace iav
}  // namespace state_predictor
}  // namespace utilities