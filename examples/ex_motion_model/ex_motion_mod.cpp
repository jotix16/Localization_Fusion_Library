#include<motion_model/motion_model.h>
#include<iostream>
#include<Eigen/Dense>

using namespace Eigen;

int main()
{
    std::cout<< "MOTION MODEL"<<std::endl;
    iav::state_predictor::motion_model::Ctra3D my_ctra;
    using State = iav::state_predictor::motion_model::Ctra3D::StateVector;
    using Matris = iav::state_predictor::motion_model::Ctra3D::StateMatrix;
    Matris x;
    x.setIdentity();
    x(1,2) = 2;
    x(1,3) = 3;

    State s,s1;
    s << 1,1,1,1,1,1,1,1,1,1,1,1,1,1,1;
    s1 = s;

    Matris jacobi;
    my_ctra.compute_jacobian(jacobi, s, 0.1);
    my_ctra.predict(s,1);
    std::cout<<"Your State1 is: \n"<< s << "\n";
    std::cout<<"Your Jacobian1 is: \n"<< jacobi << "\n";

    my_ctra.compute_jacobian_and_predict(jacobi, s1, 0.1);
    std::cout<<"Your State2 is: \n"<< s1 << "\n";
    std::cout<<"Your Jacobian2 is: \n"<< jacobi << "\n";

}